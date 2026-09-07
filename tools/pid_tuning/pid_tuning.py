#!/usr/bin/env python3
"""
pid_tuning.py -- identificacion y simulacion del PID de RIO-DSL, sin MATLAB.

Ver README.md seccion 9 del repo para el procedimiento completo y las
formulas usadas aqui. Resumen del flujo:

  1. capture   -- (opcional) grabar el log del puerto serie a un archivo.
  2. identify  -- a partir de UN escalon real de SET_RPM (en lazo cerrado,
                  con Kp conocido) calcula el modelo de planta FOPDT
                  (K, tau, L) del motor+servo real.
  3. tune      -- sobre ESE MODELO SIMULADO (no el motor real), busca la
                  ganancia ultima (Ku/Tu, metodo Ziegler-Nichols en lazo
                  cerrado) y da las ganancias P/PI/PID sugeridas.
  4. simulate  -- previsualiza la respuesta de una combinacion de
                  ganancias sobre el modelo, antes de cargarla al motor.

Las ganancias que salgan de "tune"/"simulate" son un punto de partida,
no un resultado final -- SIEMPRE hay que validarlas en el motor real
repitiendo el mismo escalon de SET_RPM (ver README seccion 9).
"""

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PID_TEST_PREFIX = "PID_TEST,"


# ============================================================================
# 1. Captura / parseo del log del puerto serie
# ============================================================================

@dataclass
class LogData:
    t_s: np.ndarray        # segundos relativos al primer dato (desde HAL_GetTick() ms)
    set_rpm: np.ndarray
    rpm: np.ndarray
    pulso_us: np.ndarray


def parse_pid_test_log(path):
    """Extrae las lineas 'PID_TEST,<ms>,<SET_RPM>,<RPM_filtrada>,<pulso_us>'
    de un log capturado, ignorando cualquier otra linea intercalada
    (RAK3172, GPS, eco del mando manual, etc.) -- ver README seccion 9."""
    t_ms, set_rpm, rpm, pulso = [], [], [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith(PID_TEST_PREFIX):
                continue
            partes = linea.split(",")
            if len(partes) != 4 + 1:
                continue
            try:
                t_ms.append(int(partes[1]))
                set_rpm.append(float(partes[2]))
                rpm.append(float(partes[3]))
                pulso.append(float(partes[4]))
            except ValueError:
                continue

    if not t_ms:
        raise ValueError(f"No se encontraron lineas '{PID_TEST_PREFIX}' validas en {path}")

    t_ms_arr = np.asarray(t_ms, dtype=float)
    return LogData(
        t_s=(t_ms_arr - t_ms_arr[0]) / 1000.0,
        set_rpm=np.asarray(set_rpm, dtype=float),
        rpm=np.asarray(rpm, dtype=float),
        pulso_us=np.asarray(pulso, dtype=float),
    )


def cmd_capture(args):
    import serial  # import local -- pyserial solo hace falta para este subcomando

    print(f"Capturando {args.port} @ {args.baud} baud -> {args.out}  (Ctrl+C para terminar)")
    contador_pid_test = 0
    with serial.Serial(args.port, args.baud, timeout=1) as ser, \
         open(args.out, "w", encoding="utf-8") as f:
        try:
            while True:
                linea = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
                if not linea:
                    continue
                print(linea)
                f.write(linea + "\n")
                f.flush()
                if linea.startswith(PID_TEST_PREFIX):
                    contador_pid_test += 1
        except KeyboardInterrupt:
            print(f"\nCapturado {contador_pid_test} lineas PID_TEST en: {args.out}")


# ============================================================================
# 2. Identificacion del modelo de planta (FOPDT) a partir de un escalon
#    en LAZO CERRADO -- ver README seccion 9 para la derivacion completa.
# ============================================================================

@dataclass
class PlantModel:
    K: float    # ganancia de la planta: RPM en estado estable por microsegundo de correccion
    tau: float  # constante de tiempo de la planta, segundos
    L: float    # tiempo muerto de la planta, segundos


@dataclass
class StepIdentification:
    step_time_s: float
    delta_set_rpm: float
    baseline_rpm: float
    final_rpm: float
    delta_rpm_ss: float
    G_cl: float
    L_cl: float
    tau_cl: float
    kp_used: float
    plant: PlantModel


def _detectar_escalon(data: LogData, salto_minimo_rpm=20.0):
    diffs = np.diff(data.set_rpm)
    if len(diffs) == 0:
        raise ValueError("El log no tiene suficientes muestras para detectar un escalon.")
    idx = int(np.argmax(np.abs(diffs)))
    if abs(diffs[idx]) < salto_minimo_rpm:
        raise ValueError(
            f"No se detecto un escalon claro en SET_RPM (mayor salto encontrado: "
            f"{diffs[idx]:.1f} RPM, minimo esperado {salto_minimo_rpm}) -- "
            "especificar --step-time-s a mano si el escalon es mas chico."
        )
    return idx + 1  # indice de la primera muestra DESPUES del escalon


def identify_step(data: LogData, kp_used, step_time_s=None,
                   pre_window_s=2.0, settle_window_s=3.0,
                   umbral_movimiento_rpm=5.0):
    """Identifica (K, tau, L) de la planta real a partir de un escalon de
    SET_RPM capturado en lazo cerrado con ganancia Kp=kp_used conocida
    (Ki=Kd=0 en la prueba, ver README seccion 9)."""
    if kp_used <= 0:
        raise ValueError("kp_used debe ser > 0 (es la Kp usada durante la prueba real).")

    if step_time_s is None:
        step_idx = _detectar_escalon(data)
        step_time_s = float(data.t_s[step_idx])

    pre_mask = (data.t_s >= step_time_s - pre_window_s) & (data.t_s < step_time_s)
    if not np.any(pre_mask):
        raise ValueError(
            "No hay suficientes datos ANTES del escalon -- capturar mas tiempo "
            "en reposo antes de mandar SET_RPM (pre_window_s="
            f"{pre_window_s}s)."
        )
    baseline_rpm = float(np.mean(data.rpm[pre_mask]))
    baseline_set = float(np.mean(data.set_rpm[pre_mask]))

    post_mask = data.t_s >= step_time_s
    settle_mask = post_mask & (data.t_s >= data.t_s[-1] - settle_window_s)
    if not np.any(settle_mask):
        raise ValueError(
            "No hay suficientes datos AL FINAL del log -- capturar hasta que "
            f"la RPM se estabilice (settle_window_s={settle_window_s}s)."
        )
    final_rpm = float(np.mean(data.rpm[settle_mask]))
    final_set = float(np.mean(data.set_rpm[settle_mask]))

    delta_set_rpm = final_set - baseline_set
    delta_rpm_ss = final_rpm - baseline_rpm

    if delta_set_rpm == 0:
        raise ValueError("delta_SET_RPM salio 0 -- no se detecto un cambio real de setpoint.")

    G_cl = delta_rpm_ss / delta_set_rpm
    if not (0.0 < G_cl < 0.999):
        raise ValueError(
            f"G_cl={G_cl:.3f} fuera del rango esperado (0,1) para un lazo P puro "
            "estable -- revisar los datos (¿la RPM realmente subio hacia el "
            "nuevo setpoint, sin haber saturado el servo?)."
        )

    moved = post_mask & (np.abs(data.rpm - baseline_rpm) > umbral_movimiento_rpm)
    if not np.any(moved):
        raise ValueError(
            "La RPM nunca se movio mas de umbral_movimiento_rpm tras el escalon "
            "-- revisar los datos o bajar ese umbral."
        )
    t_movio = float(data.t_s[moved][0])
    L_cl = max(t_movio - step_time_s, 0.0)

    objetivo_63 = baseline_rpm + 0.632 * delta_rpm_ss
    despues_de_moverse = data.t_s >= t_movio
    if delta_rpm_ss > 0:
        cruzo = despues_de_moverse & (data.rpm >= objetivo_63)
    else:
        cruzo = despues_de_moverse & (data.rpm <= objetivo_63)
    if not np.any(cruzo):
        raise ValueError(
            "La RPM nunca llego al 63.2% del cambio total -- ¿la captura "
            "termino antes de que se estabilizara del todo?"
        )
    t_63 = float(data.t_s[cruzo][0])
    tau_cl = max(t_63 - t_movio, 1e-3)

    K = G_cl / (kp_used * (1.0 - G_cl))
    tau = tau_cl / (1.0 - G_cl)
    L = L_cl

    return StepIdentification(
        step_time_s=step_time_s, delta_set_rpm=delta_set_rpm,
        baseline_rpm=baseline_rpm, final_rpm=final_rpm,
        delta_rpm_ss=delta_rpm_ss, G_cl=G_cl, L_cl=L_cl, tau_cl=tau_cl,
        kp_used=kp_used, plant=PlantModel(K=K, tau=tau, L=L),
    )


def cmd_identify(args):
    data = parse_pid_test_log(args.log)
    resultado = identify_step(data, kp_used=args.kp, step_time_s=args.step_time_s)

    print(f"Escalon detectado en t={resultado.step_time_s:.2f}s (log tiene {len(data.t_s)} muestras)")
    print(f"  SET_RPM:      {resultado.baseline_rpm + resultado.delta_set_rpm - resultado.delta_set_rpm:.1f} "
          f"-> delta={resultado.delta_set_rpm:.1f} RPM")
    print(f"  RPM medida:   {resultado.baseline_rpm:.1f} -> {resultado.final_rpm:.1f} "
          f"(delta_ss={resultado.delta_rpm_ss:.1f})")
    print(f"  G_cl (lazo cerrado) = {resultado.G_cl:.4f}")
    print(f"  L_cl (tiempo muerto observado)   = {resultado.L_cl:.3f} s")
    print(f"  tau_cl (constante de tiempo, lazo cerrado) = {resultado.tau_cl:.3f} s")
    print()
    print(f"Modelo de planta identificado (con Kp={resultado.kp_used} durante la prueba):")
    print(f"  K   = {resultado.plant.K:.6f}  RPM por microsegundo de correccion")
    print(f"  tau = {resultado.plant.tau:.3f} s")
    print(f"  L   = {resultado.plant.L:.3f} s")
    print()
    print("Siguiente paso:")
    print(f"  python pid_tuning.py tune --K {resultado.plant.K:.6f} "
          f"--tau {resultado.plant.tau:.3f} --L {resultado.plant.L:.3f}")


# ============================================================================
# 3. Replica exacta de PID_CalcularSalidaUs() (pid.c) -- si pid.c cambia,
#    actualizar esto tambien para que la simulacion siga siendo fiel.
# ============================================================================

@dataclass
class PidState:
    integral: float = 0.0
    prev_error: float = 0.0
    first_call: bool = True


def pid_calcular_salida_us(setpoint_rpm, rpm_medida, dt_s, kp, ki, kd,
                            minimo_us, maximo_us, state: PidState):
    error = setpoint_rpm - rpm_medida
    rango = maximo_us - minimo_us

    derivada = 0.0
    if not state.first_call and dt_s > 0.0:
        derivada = (error - state.prev_error) / dt_s
    state.prev_error = error
    state.first_call = False

    if ki > 0.0:
        integral_tentativa = state.integral + error * dt_s
        salida_tentativa = kp * error + ki * integral_tentativa + kd * derivada
        satura_alto = salida_tentativa > rango
        satura_bajo = salida_tentativa < 0.0
        if not (satura_alto and error > 0.0) and not (satura_bajo and error < 0.0):
            state.integral = integral_tentativa
    else:
        state.integral = 0.0

    salida = kp * error + ki * state.integral + kd * derivada
    salida = max(0.0, min(rango, salida))

    return minimo_us + salida


# ============================================================================
# 4. Simulacion del lazo cerrado sobre el modelo FOPDT identificado
# ============================================================================

def simulate_closed_loop(plant: PlantModel, kp, ki, kd, servo_min_us, servo_max_us,
                          sp_before, sp_after, step_time_s=None,
                          sim_time_s=None, dt_s=None):
    """Simula PID_CalcularSalidaUs() (replicado arriba) cerrando el lazo
    sobre un modelo de planta de primer orden con tiempo muerto (FOPDT):
        d(delta_rpm)/dt = (K * correccion(t-L) - delta_rpm) / tau
    donde 'correccion' es el pulso aplicado por encima de servo_min_us.
    """
    if dt_s is None:
        dt_s = max(1e-3, min(0.05, plant.tau / 20.0, max(plant.L, 0.01) / 5.0))
    if step_time_s is None:
        step_time_s = max(10.0, 5.0 * plant.tau)
    if sim_time_s is None:
        sim_time_s = step_time_s + max(40.0, 10.0 * plant.tau)

    n = int(sim_time_s / dt_s)
    retraso_muestras = max(1, int(round(plant.L / dt_s)))

    t = np.arange(n) * dt_s
    setpoint = np.where(t < step_time_s, sp_before, sp_after)

    rpm = np.zeros(n)
    pulso = np.zeros(n)
    correccion_hist = np.zeros(n + retraso_muestras)

    delta_rpm = 0.0
    rpm_actual = sp_before
    state = PidState()

    for i in range(n):
        pulso_us = pid_calcular_salida_us(
            setpoint[i], rpm_actual, dt_s, kp, ki, kd, servo_min_us, servo_max_us, state)
        correccion = pulso_us - servo_min_us
        correccion_hist[i + retraso_muestras] = correccion
        correccion_retrasada = correccion_hist[i]

        delta_rpm += dt_s * (plant.K * correccion_retrasada - delta_rpm) / plant.tau
        rpm_actual = sp_before + delta_rpm

        rpm[i] = rpm_actual
        pulso[i] = pulso_us

    return t, rpm, pulso, setpoint


def cmd_simulate(args):
    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)
    t, rpm, pulso, setpoint = simulate_closed_loop(
        plant, args.kp, args.ki, args.kd, args.servo_min, args.servo_max,
        args.sp_before, args.sp_before + args.step_size)

    paso = max(1, len(t) // 40)
    for i in range(0, len(t), paso):
        print(f"t={t[i]:7.2f}s  SET_RPM={setpoint[i]:7.1f}  RPM={rpm[i]:7.1f}  pulso={pulso[i]:7.1f}us")

    if args.plot_out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        ax1.plot(t, setpoint, "--", label="SET_RPM")
        ax1.plot(t, rpm, label="RPM simulada")
        ax1.set_ylabel("RPM")
        ax1.legend()
        ax1.grid(True)
        ax2.plot(t, pulso, label="pulso servo (us)")
        ax2.set_ylabel("us")
        ax2.set_xlabel("tiempo (s)")
        ax2.legend()
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot_out, dpi=120)
        print(f"\nGrafico guardado en {args.plot_out}")


# ============================================================================
# 5. Ziegler-Nichols en lazo cerrado, SOBRE EL MODELO SIMULADO (no el motor
#    real) -- ver README seccion 9. Heuristica de "relacion de decaimiento"
#    entre los dos primeros sobre-impulsos tras un escalon, para ubicar la
#    ganancia a partir de la cual la oscilacion deja de amortiguarse.
# ============================================================================

def _picos_locales(y):
    picos = []
    for i in range(2, len(y) - 2):
        if y[i] > y[i - 1] and y[i] >= y[i + 1] and y[i] > y[i - 2] and y[i] >= y[i + 2]:
            picos.append(i)
    return picos


def _relacion_decaimiento(plant, kp, servo_min_us, servo_max_us, sp_before, step_size):
    t, rpm, _, _ = simulate_closed_loop(
        plant, kp, 0.0, 0.0, servo_min_us, servo_max_us, sp_before, sp_before + step_size)

    idx_escalon = int(np.searchsorted(t, max(10.0, 5.0 * plant.tau)))
    post = rpm[idx_escalon:]
    post_t = t[idx_escalon:]
    valor_final = post[-1]

    picos = _picos_locales(post)
    if len(picos) < 2:
        return None, None

    desviaciones = [abs(post[p] - valor_final) for p in picos]
    if desviaciones[0] < 1e-6:
        return None, None

    relacion = desviaciones[1] / desviaciones[0]
    periodo = post_t[picos[1]] - post_t[picos[0]]
    return relacion, periodo


def find_ultimate_gain(plant: PlantModel, servo_min_us, servo_max_us,
                        sp_before=1000.0, step_size=200.0,
                        kp_min=0.01, kp_max=1000.0, tol=1e-3, max_iter=60,
                        umbral_sostenida=0.9):
    """Busqueda binaria (en escala log) de la ganancia ultima Ku sobre el
    modelo SIMULADO: el Kp mas chico a partir del cual dos sobre-impulsos
    consecutivos ya no decaen (relacion >= umbral_sostenida)."""
    relacion_hi, periodo_hi = _relacion_decaimiento(plant, kp_max, servo_min_us, servo_max_us, sp_before, step_size)
    if relacion_hi is None or relacion_hi < umbral_sostenida:
        raise ValueError(
            f"Ni con Kp={kp_max} se observa oscilacion sostenida en la simulacion "
            "-- el modelo de planta puede tener demasiado retraso/ganancia baja, "
            "probar con --kp-max mas alto."
        )

    lo, hi = kp_min, kp_max
    for _ in range(max_iter):
        mid = math.sqrt(lo * hi)
        relacion, periodo = _relacion_decaimiento(plant, mid, servo_min_us, servo_max_us, sp_before, step_size)
        if relacion is not None and relacion >= umbral_sostenida:
            hi, periodo_hi = mid, periodo
        else:
            lo = mid
        if hi / lo - 1.0 < tol:
            break

    return hi, periodo_hi


def ziegler_nichols_gains(ku, tu):
    return {
        "P":   {"Kp": 0.50 * ku},
        "PI":  {"Kp": 0.45 * ku, "Ki": 0.45 * ku / (tu / 1.2)},
        "PID": {"Kp": 0.60 * ku, "Ki": 0.60 * ku / (tu / 2.0), "Kd": 0.60 * ku * (tu / 8.0)},
    }


def cmd_tune(args):
    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)
    ku, tu = find_ultimate_gain(
        plant, args.servo_min, args.servo_max,
        sp_before=args.sp_before, step_size=args.step_size, kp_max=args.kp_max)

    print(f"Ganancia ultima (simulada, NO en el motor real): Ku = {ku:.4f}   Tu = {tu:.3f}s")
    print()
    for tipo, ganancias in ziegler_nichols_gains(ku, tu).items():
        partes = "  ".join(f"{nombre}={valor:.4f}" for nombre, valor in ganancias.items())
        print(f"  {tipo:4s}: {partes}")
    print()
    print("Recomendado para este acelerador (ver README seccion 9): empezar por PI")
    print("(el ruido de medicion conocido en ralenti hace que Kd amplifique ruido).")
    print()
    print("Estas ganancias son un PUNTO DE PARTIDA de la simulacion -- validar")
    print("SIEMPRE en el motor real repitiendo el mismo escalon de SET_RPM antes")
    print("de confiar en ellas.")


# ============================================================================
# 6. SIMC (Skogestad, 2003) -- alternativa a Ziegler-Nichols pensada para
#    plantas con tiempo muerto significativo respecto a su constante de
#    tiempo (L/tau alto), como esta. A diferencia de Z-N (que persigue la
#    respuesta mas rapida posible, y por eso tiende a dar ganancias con
#    sobre-impulso justo en este tipo de planta), SIMC deja elegir a
#    proposito que tan conservador ser via tau_c (constante de tiempo
#    deseada del lazo cerrado) -- no hace falta oscilar nada para usarla,
#    solo el modelo (K, tau, L) ya identificado con `identify`.
# ============================================================================

def simc_pi_gains(K, tau, L, tau_c):
    """Formulas SIMC para PI (Skogestad 2003):
        Kc  = (1/K) * tau / (tau_c + L)
        Ti  = min(tau, 4*(tau_c + L))
    tau_c mas chico (tipicamente >= L) da un lazo mas rapido pero con
    menos margen; mas grande da un lazo mas lento pero mas robusto ante
    un modelo de planta impreciso -- justo el control que Z-N no da."""
    kc = (1.0 / K) * (tau / (tau_c + L))
    ti = min(tau, 4.0 * (tau_c + L))
    ki = kc / ti
    return kc, ki


def cmd_simc(args):
    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)

    # Tres puntos sobre el espectro agresivo <-> conservador, todos
    # centrados en el tiempo muerto identificado (tau_c >= L siempre,
    # regla practica de Skogestad).
    opciones = [
        ("Agresivo (tau_c = L)",        plant.L),
        ("Medio (tau_c = 2L)",          2.0 * plant.L),
        ("Conservador (tau_c = tau)",   plant.tau),
    ]

    print(f"Modelo: K={plant.K:.6f}  tau={plant.tau:.3f}s  L={plant.L:.3f}s")
    print()
    for etiqueta, tau_c in opciones:
        kc, ki = simc_pi_gains(plant.K, plant.tau, plant.L, tau_c)
        print(f"  {etiqueta:24s} (tau_c={tau_c:.3f}s): Kp={kc:.4f}  Ki={ki:.4f}")
    print()
    print("A diferencia de `tune` (Ziegler-Nichols), estas ganancias NO buscan")
    print("la respuesta mas rapida posible -- estan pensadas para no oscilar")
    print("ni siquiera cerca del setpoint, en plantas con bastante tiempo")
    print("muerto como esta. Empezar por la fila 'Medio' y validar con")
    print("`simulate` antes de probar en el motor real; si simulate ya se ve")
    print("con sobre-impulso, probar la fila 'Conservador'.")


# ============================================================================
# CLI
# ============================================================================

def build_parser():
    p = argparse.ArgumentParser(
        description="Identificacion y simulacion del PID de RIO-DSL (ver README seccion 9).")
    sub = p.add_subparsers(dest="comando", required=True)

    p_cap = sub.add_parser("capture", help="Capturar el log del puerto serie a un archivo (requiere pyserial).")
    p_cap.add_argument("--port", required=True, help="Puerto serie, ej. COM5")
    p_cap.add_argument("--baud", type=int, default=115200)
    p_cap.add_argument("--out", required=True, help="Archivo donde guardar el log capturado")
    p_cap.set_defaults(func=cmd_capture)

    p_id = sub.add_parser("identify", help="Identificar el modelo de planta (K, tau, L) a partir de un escalon real.")
    p_id.add_argument("--log", required=True, help="Archivo de log capturado (o su recorte)")
    p_id.add_argument("--kp", type=float, required=True, help="PID_KP que estaba activo durante la prueba")
    p_id.add_argument("--step-time-s", type=float, default=None,
                       help="Instante del escalon en segundos relativos al log (autodetectado si se omite)")
    p_id.set_defaults(func=cmd_identify)

    p_tune = sub.add_parser("tune", help="Buscar Ku/Tu (Ziegler-Nichols) sobre el modelo simulado y sugerir ganancias.")
    p_tune.add_argument("--K", type=float, required=True)
    p_tune.add_argument("--tau", type=float, required=True)
    p_tune.add_argument("--L", type=float, required=True)
    p_tune.add_argument("--servo-min", type=float, default=1000.0, help="SERVO_PULSO_MIN actual (us)")
    p_tune.add_argument("--servo-max", type=float, default=2000.0, help="SERVO_PULSO_MAX actual (us)")
    p_tune.add_argument("--sp-before", type=float, default=1000.0, help="RPM base simulada antes del escalon")
    p_tune.add_argument("--step-size", type=float, default=200.0, help="Tamano del escalon de SET_RPM simulado")
    p_tune.add_argument("--kp-max", type=float, default=1000.0, help="Techo de busqueda de Ku")
    p_tune.set_defaults(func=cmd_tune)

    p_simc = sub.add_parser("simc", help="Ganancias PI via SIMC (Skogestad) -- alternativa a Ziegler-Nichols para plantas con bastante tiempo muerto.")
    p_simc.add_argument("--K", type=float, required=True)
    p_simc.add_argument("--tau", type=float, required=True)
    p_simc.add_argument("--L", type=float, required=True)
    p_simc.set_defaults(func=cmd_simc)

    p_sim = sub.add_parser("simulate", help="Previsualizar la respuesta de una combinacion de ganancias sobre el modelo.")
    p_sim.add_argument("--K", type=float, required=True)
    p_sim.add_argument("--tau", type=float, required=True)
    p_sim.add_argument("--L", type=float, required=True)
    p_sim.add_argument("--kp", type=float, required=True)
    p_sim.add_argument("--ki", type=float, default=0.0)
    p_sim.add_argument("--kd", type=float, default=0.0)
    p_sim.add_argument("--servo-min", type=float, default=1000.0)
    p_sim.add_argument("--servo-max", type=float, default=2000.0)
    p_sim.add_argument("--sp-before", type=float, default=1000.0)
    p_sim.add_argument("--step-size", type=float, default=200.0)
    p_sim.add_argument("--plot-out", default=None, help="Si se da, guarda un PNG con la respuesta (requiere matplotlib)")
    p_sim.set_defaults(func=cmd_simulate)

    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.func(args)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
