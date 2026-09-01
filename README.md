# TID — Firmware del Nodo Motor Diésel (RIO-DSL)

Firmware para STM32G431KBT6U (Nucleo-32) que mide RPM de un motor diésel
vía el alternador, controla el acelerador con un lazo PID sobre un
servo, y se comunica por LoRaWAN (RAK3172), con calibración/configuración
remota mediante un protocolo de parámetros propio. Pendiente: ganancias
reales del PID (sintonización empírica en campo, ver sección 8), la
máquina de modos del gobernador (0/1/2), y el sensor de presión de
entrada 4-20mA.

---

## 1. Hardware y mapa de pines

MCU: **STM32G431KBT6U**, LQFP32, Nucleo-32 (NUCLEO-G431KB).

| Pin | Función | Periférico | Notas |
|---|---|---|---|
| PA0 | Tacómetro (Input Capture) | TIM2_CH1 | Señal ya acondicionada (PC817 + diodo serie) |
| PA9 | RAK3172 TX | USART1_TX | — |
| PA10 | RAK3172 RX | USART1_RX | — |
| PA2 | Debug TX | LPUART1_TX | Vía adaptador USB-TTL (CP2102) |
| PA3 | Debug RX | LPUART1_RX | — |
| PB3 | GPS (SIM7600X) TX | USART2_TX | Va al pin RX del módulo |
| PB4 | GPS (SIM7600X) RX | USART2_RX | Va al pin TX del módulo — pull-up habilitado |
| PB0 | Servo (PWM) | TIM3_CH3 | **No usar PA6/TIM3_CH1** — el header físico de esa posición en el Nucleo-32 está enrutado a PA15 por el solder bridge SB3 de fábrica (ver UM2397, Tabla 9), PA6 no llega al header |
| PA1 | Sensor de presión (ADC) | ADC1_IN2 | Pendiente de implementar (`presion.c/h`) |

⚠️ **El debug se movió de USART2 a LPUART1** (2026-08-18) para liberar
USART2 para el GPS. `USART1` (RAK) se quedó exactamente donde estaba.
Si CubeMX vuelve a reasignar pines al agregar/quitar un periférico
(pasó varias veces durante esta migración, incluyendo una petición de
DMA huérfana que quedó apuntando a un periférico ya deshabilitado),
revisar siempre `Mcu.IPx`/`Dma.Request*` en el `.ioc` contra lo que se
espera, no confiar solo en que "compiló".

**RTC**: el RTC usa el reloj interno **LSI (32kHz nominal)**, sin
cristal LSE ni pila de respaldo (VBAT) — ver sección 2.5 para el porqué
y sus implicancias.

---

## 2. Módulos del firmware

```
tacometro.c/h          -> Medicion de RPM (Input Capture + filtro EMA)
rak3172.c/h             -> Comunicacion con el modulo LoRaWAN + sincronizacion de hora
calibracion_flash.c/h   -> Persistencia de parametros + dispatcher de downlinks
servo.c/h               -> Control PWM del servo del acelerador
pid.c/h                 -> Lazo de control PID de RPM -> pulso del servo
rtc_reloj.c/h           -> Reloj interno (RTC sobre LSI) + conversion epoch<->calendario
gps.c/h                 -> Posicion GPS/GNSS (SIM7600X) + disciplina del RTC
comando_serial.c/h      -> Mando manual TEMPORAL por serial (sustituto de LoRa en banco)
main.c                  -> Orquestacion general
```

### 2.1 `tacometro.c/h`

Mide la frecuencia del alternador con **TIM2 en modo PWM Input** (Slave
Mode Reset en TI1FP1): el canal directo captura el período completo en
cada flanco de subida, el canal indirecto captura el ancho de pulso en
el mismo instante — sin condición de carrera entre dos capturas
separadas.

- **Filtro de ruido en 2 capas**: filtro de hardware del timer
  (`ICFilter=8`) + guarda de período mínimo por software
  (`TACOMETRO_PERIODO_MINIMO_US`).
- **Filtro de duty cycle** (`TACOMETRO_DUTY_MINIMO/MAXIMO`): descarta
  picos angostos que no representan un semiciclo real de la señal.
- **Detección de motor detenido**: timeout sin capturas nuevas
  (`TACOMETRO_TIMEOUT_DETENIDO_MS`) → `Tacometro_EstaDetenido()`. Este
  es el criterio único para saber si el motor opera — **no depende de
  ningún switch físico de encendido**.
- **Filtro EMA** (media móvil exponencial) sobre la RPM instantánea,
  coeficiente configurable en caliente (`ALPHA`, ver protocolo de
  parámetros).

Circuito de entrada: **H11AA1** (optoacoplador de 2 LEDs anti-paralelos,
detecta ambos semiciclos de la señal AC por sí solo — a diferencia del
PC817/JC817, que solo tienen 1 LED y requerirían un diodo en serie o
puente rectificador adicional si se quisiera sustituir), alimentado
desde el terminal W del alternador vía R1 (680Ω/3W) + clamp zener
(D1/D2) + R3 (2.2kΩ) + D3 + C1. Factor de calibración de campo
confirmado: **`SET_RATIO = 17.5`** pulsos/revolución (incluye relación
de poleas alternador:motor, con 1 pulso/ciclo AC confirmado en campo
con el motor real).

### 2.2 `rak3172.c/h`

Comunicación con el módulo LoRaWAN por USART1, DMA en modo Normal.

- **Recepción**: buffer circular propio, ensamblado de líneas por
  `\n`, cola de líneas pendientes para procesar fuera de contexto de
  interrupción (`RAK3172_Update()`).
- **Envío de comandos AT**: bloqueante (`HAL_UART_Transmit`), con
  timeout configurable.
- **Application ACK**: si el canal AT está ocupado al momento de
  mandar un ACK, se encola para reintento automático (hasta
  `RAK3172_ACK_REINTENTO_TIMEOUT_MS`, 5s).
- **Parser de eventos `+EVT:`**: formato real confirmado en campo
  (firmware RUI_4.2.4_RAK3172-E):
  ```
  +EVT:RX_1:-28:8:UNICAST:2:00000710
       |    |   |    |    |    +-- payload en hex: [ID][valor]
       |    |   |    |    +------- FPort
       |    |   |    +------------ tipo (UNICAST)
       |    |   +----------------- SNR
       |    +--------------------- RSSI
       +-------------------------- ventana de recepcion
  ```

**Configuración crítica que resolvió un bug de cuelgue en reset**
(con el RAK3172 conectado): USART1 (no LPUART1), DMA modo Normal (no
Circular), `Overrun`/`DMA on RX Error` deshabilitados en el `.ioc`,
pull-up en el pin RX, y limpieza de flags de error antes de rearmar la
recepción en cada `RAK3172_Init()`.

**FPorts usados**:

| FPort | Uso |
|---|---|
| 1 | Uplink LIVE (27 bytes: RPM + presión + estado + timestamps + lat/lon) |
| 2 | Downlink de parámetro (ver protocolo abajo) |
| 3 | Application ACK |

**Estado LoRaWAN**: Clase C, join OTAA. ⚠️ El RAK3172 **no cambia de
clase automáticamente** solo porque el Device Profile en el servidor
diga "admite Clase C" — hay que mandarle explícitamente `AT+CLASS=C`
(default de fábrica es Clase A).

⚠️ **`AutoJoin` en `0`, no `1`, como prueba temporal en curso**
(`RAK3172_Join()` manda `AT+JOIN=1:0:10:8`): se cambió para aislar si
ese parámetro específico causaba el `AT_ERROR` de join visto en campo.
La causa real resultó ser el bug de sub-banda de abajo (`AT+MASK`), ya
resuelto — falta decidir si `AutoJoin` vuelve a `1` en producción
(reintento automático interno del propio módulo tras un power-cycle,
sin que el host tenga que pedir el join de nuevo) o si conviene dejarlo
en `0` y manejar todos los reintentos desde el host, como ya se hace
ahora (ver más abajo). Ver pendientes, sección 8.

#### Orden de arranque, join, y recuperación ante caídas — LoRa como prioridad

Requisito explícito del proyecto: **el join LoRaWAN debe intentarse
antes que cualquier otra cosa en el arranque**, y el nodo debe
reconectarse solo si pierde el join en cualquier momento, sin necesidad
de un reinicio manual del STM32. Así quedó implementado en `main.c` y
`rak3172.c`:

**1. Arranque — LoRa antes que el GPS.** `RAK3172_Init()` corre
inmediatamente después de `Tacometro_Init()`, seguido de `AT+MASK=0002`
(fix de sub-banda, ver 2.2 más abajo) y `RAK3172_Join()` — **todo esto
antes de `GPS_Init()`**. Antes, el GPS se inicializaba primero y sus
~2.5s de `HAL_Delay()` bloqueantes retrasaban el primer intento de join
sin necesidad; ahora el join sale lo antes posible y el GPS se
inicializa después, sin competir por tiempo de arranque.

**2. El propio módulo reintenta el join solo, 8 veces cada 10s**
(`AT+JOIN=1:0:10:8`, sin que el host tenga que hacer nada) antes de
declarar la primera ronda fallida. Un evento
**`+EVT:JOIN_FAILED_RX_TIMEOUT`** aislado es normal (una ventana de
recepción perdida) — no es una falla del sistema mientras algún
intento de esos 8 termine en `+EVT:JOINED`, que es justo lo que se ve
en campo la mayoría de las veces.

**3. Confirmación**: `+EVT:JOINED` pone `RAK3172_EstaUnido()` en
`true`. A partir de ahí se habilita el envío periódico del uplink LIVE
(cada 30s) y la solicitud de hora por red (ver sincronización de hora
más abajo).

**4. Detección de caída a mitad de sesión (agregado 2026-08-29, tras un
caso real en campo).** El primer diseño no tenía ninguna forma de
notarlo: `RAK3172_EstaUnido()` solo se ponía en `false` cuando el
propio host llamaba a `RAK3172_Join()`, así que si el módulo perdía el
join por su cuenta (ej. un reinicio espontáneo — ver hallazgo de
hardware abajo), el firmware seguía creyendo que todo estaba bien para
siempre, mandando uplinks al vacío sin parar. Ahora `rak3172.c`
reconoce la respuesta **`AT_NO_NETWORK_JOINED`** (la que da el módulo
cuando se le pide enviar sin estar unido) como prueba definitiva de que
ya no hay join, y de inmediato (sin esperar los 2s del timeout) pone
`RAK3172_EstaUnido()` en `false` y lo reporta por log.

**5. Reintento automático, sin bloquear el resto del loop.** Con
`RAK3172_EstaUnido()` en `false`, el loop principal dispara una máquina
de 2 pasos no bloqueante: reafirma `AT+MASK=0002` (por si ese comando
había fallado silenciosamente en el arranque original) y, en cuanto
responde, manda `AT+JOIN` de nuevo. Si no se logra unir, este ciclo se
repite cada `INTERVALO_REINTENTO_JOIN_MS` (2 minutos, con margen sobre
los ~80s que tarda el módulo en agotar sus propios 8 intentos) de forma
indefinida, hasta que el join se recupere. Antes de este cambio, un
fallo de join al arrancar (o una caída a mitad de sesión) dejaba al
nodo sin LoRa **para siempre**, sin ningún reintento — este era
justamente el reporte original que motivó todo este trabajo.

**6. Recuperación de errores de UART (`HAL_UART_ErrorCallback`).**
Aparte de la lógica de arriba, se encontró que un solo error de UART
(overrun/framing/ruido — ej. el ruido del cable USB-a-PC ya documentado
en 2.6) dejaba la recepción por DMA de USART1 o USART2 muerta para el
resto de la sesión: HAL aborta la recepción internamente al entrar en
error, y sin un `HAL_UART_ErrorCallback()` que la rearme, el callback
normal de datos nunca vuelve a dispararse — todo comando futuro por esa
UART expira por `TIMEOUT` sin que el host jamás vea la respuesta real,
aunque el módulo del otro lado siga funcionando bien. Se agregaron
`RAK3172_ErrorCallback()`/`GPS_ErrorCallback()` (limpian los flags de
error y rearman la recepción) y un `HAL_UART_ErrorCallback()` en
`main.c` que las despacha según la instancia — mismo patrón que
`HAL_UARTEx_RxEventCallback()`. ⚠️ Actualmente instrumentado con prints
de diagnóstico temporal (`DIAGNOSTICO TEMPORAL` en el código) para
confirmar en campo si este camino llega a dispararse; quitar una vez
confirmado.

**7. Logging de resultados AT: por flanco, no por cambio de valor.**
El reporte `RAK3172: resultado=...` del loop principal originalmente
solo se imprimía cuando el resultado **cambiaba** respecto al último
mostrado. Eso ocultaba fallos reales: si un comando fallaba con
`TIMEOUT` dos veces seguidas, la segunda vez no se imprimía nada (mismo
valor que la anterior), y en el log parecía que el nodo "dejó de
intentar" cuando en realidad seguía intentando y fallando cada vez
igual. Corregido para detectar el **flanco** de "comando recién
terminado" (en curso → listo), que reporta cada finalización real sin
importar si el resultado se repite.

⚠️ **Caso de campo abierto (2026-08-29): reinicios espontáneos del
RAK3172.** Se observó el módulo reiniciándose solo repetidamente
(reaparece su banner de arranque `RAKwireless RAK3172-E...` en medio de
la sesión, cada 10-30s en el peor caso, formando un ciclo de reinicios)
— la lógica de los puntos 4-5 de arriba maneja esto correctamente
(detecta la caída y reintenta sin quedarse trabado), pero la causa raíz
de por qué el módulo se reinicia **no es de software**: se sospecha de
la conexión de la antena o de la alimentación (picos de corriente de
TX de LoRa hundiendo el rail de alimentación — ver hallazgos de
hardware de este proyecto sobre márgenes ajustados de energía). En
investigación, ver pendientes (sección 8).

#### Sincronización de hora: GPS (primaria) + LoRaWAN DeviceTimeReq (respaldo)

El RTC corre del LSI interno (ver 2.5), sin cristal LSE — deriva
**medida en campo (2026-08-26): ~0.84%** (equivale a ~12 min/día sin
corrección). Por eso la hora no se sincroniza una sola vez al
arrancar, sino que se resincroniza durante toda la operación contra
**dos fuentes redundantes**, ambas llamando a `Reloj_SetHoraUtc()`:

1. **GPS** (`GPS_GetFechaHoraUtc()`, primaria): `main.c` la consulta
   cada `INTERVALO_RESYNC_GPS_MS` (30s, igual al envío de RPM) si hay
   fix vivo (`GPS_TieneFix()`). No compite por el canal AT del
   RAK3172 ni gasta duty cycle — el único límite real es que el propio
   SIM7600X solo refresca su reporte `+CGPSINFO:` cada 10s (ver 2.6),
   así que preguntar más seguido no trae dato más fresco.
2. **Red LoRaWAN** (`DeviceTimeReq`, respaldo): antes del primer sync
   (de cualquier fuente) compite libremente con el GPS — el que llegue
   primero gana. Una vez que ya hubo un sync exitoso, se limita a
   activarse solo si pasan `INTERVALO_FALLBACK_LORA_MS` (30 min) sin
   ningún resync nuevo — típicamente porque el GPS no tiene vista al
   cielo por un rato largo. Ver `necesitaFallbackLora` en `main.c`.

El flujo del `DeviceTimeReq` en sí (RUI3: `AT+TIMEREQ`/`AT+LTIME`) no
cambia:

1. `RAK3172_SolicitarHoraRed()` manda `AT+TIMEREQ=1` — a diferencia
   del diseño original, se puede (y se debe) llamar repetidas veces,
   no solo una vez por arranque; cada llamada limpia internamente el
   flag del ciclo anterior (`RAK3172_HoraDeRedDisponible()`) para no
   confundirlo con el evento de un ciclo previo.
2. La hora **no llega de inmediato** — viaja "montada" como MAC
   command en el `FOpts` del próximo uplink exitoso (no es un uplink
   dedicado, sin costo de duty cycle extra). Cuando eso pasa, el
   módulo emite el evento **`+EVT:TIMEREQ_OK`**, detectado en
   `RAK3172_ProcesarLinea()` y expuesto como
   `RAK3172_HoraDeRedDisponible()`.
3. Con la hora ya disponible, se consulta el valor con `AT+LTIME=?`
   (`RAK3172_ConsultarHoraRed()`), y la respuesta se lee por el
   mecanismo genérico `RAK3172_GetUltimaRespuesta()`.
4. `main.c` parsea esa respuesta y llama a `Reloj_SetHoraUtc()`.

Formato de `AT+LTIME=?` **confirmado en campo** (2026-08-17): texto
legible, no un epoch plano —
`"AT+LTIME=15h08m55s on 08/17/2026"` (`MM/DD/YYYY`), parseado con
`sscanf(respuesta, "AT+LTIME=%2uh%2um%2us on %2u/%2u/%4u", ...)` en
`main.c`.

⚠️ El uplink LIVE (`RAK3172_EnviarUplinkLive()`) **NO espera** a que el
reloj sincronice — sale desde el primer ciclo con la mejor hora
disponible (la persistida en flash de un arranque anterior, o el
default de `MX_RTC_Init()` si es el primerísimo arranque). Esto evita
el ciclo circular "sin uplink no hay hora, sin hora no hay uplink" —
ver `CalibFlash_GetUltimaHoraUtcConocida()`/`Reloj_CargarHoraAproximada()`
en 2.5.

**Preservación del contador de estado al (re)sincronizar** (bug
corregido 2026-08-18, generalizado a todo resync 2026-08-26): cada vez
que el reloj se corrige — el primer sync desde el placeholder de
flash, o cualquier resync periódico posterior, por GPS o por red — el
código en `main.c` **desplaza** el ancla de `inicio_operacion`
(`inicioEstadoLocal`) por el mismo salto (bandera
`relojFueCorregidoEsteCiclo`), en vez de reiniciarla a "ahora". Si no,
cada corrección (incluido un resync rutinario cada 30s por GPS)
borraría el tiempo ya transcurrido en el estado actual, aunque el
motor llevara horas sin cambiar de estado.

⚠️ **No confundir latencia de pipeline con imprecisión del RTC**
(observado en campo, 2026-08-26): comparar `fecha_hora` del payload
(la pone el dispositivo al armar el uplink) contra la marca de tiempo
de recepción en AWS/MQTT casi siempre muestra un desfase de varios
segundos — eso es el tiempo real que tarda el mensaje en viajar
LoRaWAN → gateway → network server → Lambda decoder → MQTT, **no**
error del reloj. La señal de que sí es el RTC es que ese desfase
**crezca** con el tiempo entre mensajes sucesivos; si se mantiene
plano (aunque no sea exactamente 0s), el reloj está bien.

### 2.3 `calibracion_flash.c/h`

Almacena los parámetros configurables en la **última página de flash**
del STM32G431KB (2KB, dirección `0x0801F800`, página 63), y centraliza
la lógica de aplicar cualquier downlink de configuración.

**⚠️ Regla crítica**: cada vez que se agregue, quite o reordene un
campo de `CalibFlash_Datos_t`, hay que **subir `CALIB_FLASH_MAGIC`**.
Si no se hace, `CalibFlash_Init()` copia bytes de flash borrada (0xFF)
hacia los campos nuevos, dando valores basura (típicamente 65535) —
esto ya causó un bug real de servo atascado en el máximo. Magic actual:
`"CALE"` (subido desde `"CALD"` al agregar
`ultimaLatitudConocida`/`ultimaLongitudConocida`, ver 2.6).

**Punto de entrada único**: `CalibFlash_ProcesarParametroConEstado()`
recibe `(id, datos, longitud, motorOperando)`, valida, aplica (o
rechaza), persiste si corresponde, y devuelve el `STATUS` + el valor
que quedó realmente vigente (en 2 bytes, listo para el ACK).

**Categorías de parámetro** (`CalibFlash_CategoriaDe()`), determinan si
se puede cambiar con el motor operando:

| Categoría | Se permite con el motor operando | Parámetros |
|---|---|---|
| CALIBRACION | Sí, siempre | `SET_RATIO`, `ALPHA`, `PID_KP`, `PID_KI`, `PID_KD` |
| PROCESO | Sí, siempre (es su función) | `SET_RPM`, `PRESION` |
| COMANDO | Evaluado aparte | `FORZAR_REPORTE`, `RESTAURAR_DEFAULTS`, `RESET_REMOTO` |
| CONFIGURACION | **No** — se rechaza | Todos los demás (default conservador) |

### 2.4 `servo.c/h`

Control PWM del servo del acelerador vía **TIM3_CH3 (PB0)**, 50Hz
(20ms), resolución de 1µs por tick (Prescaler=169, Period=19999).

`Servo_SetPulsoUs(microsegundos)` recorta el valor contra
`SERVO_PULSO_MIN/MAX` antes de aplicarlo — nunca se manda un pulso
fuera de esos límites, sin importar qué tan fuera de rango venga la
solicitud (ni del downlink, ni de un futuro PID).

`Servo_MoverHacia(destinoUs)` mueve el servo gradualmente hacia
`destinoUs` sin bloquear, limitando la velocidad a
`SERVO_VELOCIDAD_MAX_US_S` (µs/segundo, constante fija en `servo.h`,
no configurable remoto — protección mecánica contra saltos bruscos
del pulso, ej. ante una corrección fuerte del futuro PID). Calcula el
paso por tiempo real transcurrido (`HAL_GetTick()`), no por conteo de
llamadas, así que no depende de la cadencia exacta del loop principal.

**Modo calibración** (`main.c`): con el motor detenido, un downlink
`CONTROL_HABILITADO=1` activa un barrido continuo del servo entre
`SERVO_PULSO_MIN` y `SERVO_PULSO_MAX` (vía `Servo_MoverHacia()`) — para
observar en banco el recorrido físico real mientras se ajustan esos
dos límites por downlink, con efecto inmediato en el barrido en curso
(se leen en cada vuelta del loop, no se cachean). Al llegar a cada
extremo se hace una pausa de `SERVO_BARRIDO_PAUSA_EXTREMOS_MS` (1000ms,
`servo.h` — fija, no configurable por downlink; ajustar y reflashear si
hace falta) antes de invertir la dirección — el pulso PWM comandado
llega exacto al límite en cuanto se cumple `SERVO_VELOCIDAD_MAX_US_S`,
pero el servo físico tiene su propio tiempo de asentamiento; sin esa
pausa, el firmware invertía la marcha en la misma vuelta del loop en
que el pulso llegaba al extremo, y el brazo real nunca alcanzaba a
terminar de llegar al tope mecánico antes de que se le pidiera ir al
otro lado (los modos `0`/`2` no tienen este problema porque el destino
se queda fijo indefinidamente, dándole tiempo de sobra al servo).
`CONTROL_HABILITADO=2`
es el modo manual: el servo se queda quieto en su posición actual (no
salta a ningún límite al entrar) y solo se mueve cuando llega un
downlink de `SERVO_PULSO_MIN` o `SERVO_PULSO_MAX`, yendo directo a ese
valor — útil para verificar un punto exacto del recorrido sin esperar
una vuelta completa del barrido. El "llegó un downlink" se marca con
una bandera en el instante en que `calibracion_flash.c` aplica el
parámetro con éxito
(`CalibFlash_HayObjetivoManualServo()`/`GetObjetivoManualServoUs()`/
`LimpiarObjetivoManualServo()`), **no** comparando contra el valor
anterior — si se comparara, un downlink que repite el valor ya guardado
(ej. los defaults de fábrica, `SERVO_PULSO_MIN=1000`/`MAX=2000`) nunca
se vería como "cambio" y el servo no se movería, aunque el downlink sí
haya llegado. Ambos (`1` y `2`) son el único
momento en que `SERVO_PULSO_MIN/MAX` se pueden cambiar (fuera de modo
calibración, se rechazan con `APPLY_ERROR` aunque el motor esté
apagado). Medida de seguridad: si el motor arranca mientras
`CONTROL_HABILITADO` es `1` o `2`, el firmware lo fuerza a `0`
localmente en el mismo ciclo (vía `Tacometro_EstaDetenido()`), sin
esperar ningún downlink — nunca se deja el servo bajo control remoto de
calibración con el motor operando.

**Log `SERVO_CAL`**: mientras `CONTROL_HABILITADO` sea `1` o `2`, cada
1s se imprime `SERVO_CAL,min=<us>,max=<us>,pulso_actual=<us>` — antes
de esto, la única forma de ver los límites vigentes en banco era el ACK
de cada downlink de `SERVO_PULSO_MIN/MAX` (`valor vigente=X`); esta
línea muestra en vivo, mientras se observa el servo moverse, dónde
están los límites configurados y qué pulso se le está aplicando en ese
instante. Fuera de esos dos modos no imprime nada (a diferencia de
`PID_TEST`, sección 9, esto es solo para lectura humana en banco, no
pensado para analizarse después con un script).

Este bloque de barrido queda como el mecanismo permanente para
recalibrar `SERVO_PULSO_MIN/MAX` en banco (ej. si se cambia de servo o
de geometría de la varilla) — no se retira al existir el PID real
(`pid.c/h`, ver abajo), simplemente son mutuamente excluyentes: uno
corre en `CONTROL_HABILITADO=1`, el otro con el motor operando y
`CONTROL_HABILITADO=0` (ver 4.4).

**Posición segura = `SERVO_PULSO_MIN`** (confirmado en el montaje
físico real: ese extremo deja el acelerador sin aceleración/ralentí,
no es un valor arbitrario). Con `CONTROL_HABILITADO=0` y el motor
detenido (arranque, o justo después del apagado de seguridad de
arriba), el servo siempre va o se mantiene ahí — al arrancar se manda
de una vez (no se conoce la posición previa, quedó como estuviera al
perder alimentación); al salir del barrido por el apagado de
seguridad, regresa gradualmente vía `Servo_MoverHacia()` (respetando
`SERVO_VELOCIDAD_MAX_US_S`, no de un salto), sin importar en qué punto
del recorrido se haya quedado. Si en cambio el motor está operando
(y no se está calibrando), el servo lo maneja el PID (ver abajo), no
esta posición fija.

**PID (`pid.c/h`)**: lazo de control real, activo en `main.c` solo
cuando el motor opera, `CONTROL_HABILITADO=0` o `=3` (no se está
calibrando el servo -- `=3` es la sesión de sintonización del PID en
sí, ver 4.4, y el servo se comporta exactamente igual que en `=0`)
**y** se comandó explícitamente un `SET_RPM` por encima de `RPM_MIN`
— mutuamente excluyente con el barrido de banco por diseño (ver 4.4).
`PID_CalcularSalidaUs(setpointRpm, rpmMedida)` devuelve el pulso en µs
directos que se le pasa a `Servo_MoverHacia()`, usando
`SERVO_PULSO_MIN` como línea base (la corrección del PID se suma sobre
ella, no la reemplaza) y recortando contra `SERVO_PULSO_MIN/MAX`.
Anti-windup por integración condicional (la integral se congela si ya
está saturando en la misma dirección del error) y se resetea sola
mientras `PID_KI=0`, para que no acumule en silencio y aparezca de
golpe cuando se active esa ganancia. `PID_Init()` se llama una sola vez
en el flanco de entrada a este modo, para que el primer cálculo no use
un `dt` inflado por el tiempo inactivo.

**`RPM_MIN` es el umbral de "¿debe intervenir el control?", no solo un
límite duro** — `RPM_MIN` ya está documentado como "límite duro /
ralentí" (tabla de parámetros), y el ralentí real es distinto en cada
motor. Modo 0 (ralentí: el motor sube solo de 0Hz a su ralentí natural,
sección 4.3) es deliberadamente **sin control**: con `SET_RPM` en su
default sin comandar (`0`, no persiste en flash, ver
`CalibFlash_Init()`) o en cualquier valor por debajo de `RPM_MIN`, el
PID no corre y el servo se queda en `SERVO_PULSO_MIN` — nunca se
recorta `SET_RPM` hacia arriba hasta `RPM_MIN` como si fuera un
setpoint válido. El lazo solo se activa cuando `SET_RPM > RPM_MIN`
(equivale en la práctica a pedir "Modo 1" antes de que exista la
máquina Modo 0/1/2 real); una vez activo, el setpoint sí se recorta
hacia abajo contra `RPM_MAX` si se pide algo más alto.

El setpoint hoy es `SET_RPM` (downlink directo) — uso de prueba hasta
que exista la máquina Modo 0/1/2 (sección 4.3), que decidirá el
setpoint real según la presión sin que este módulo tenga que cambiar.
Ganancias (`PID_KP/KI/KD`) siguen en sus defaults (`Kp=1.0, Ki=0,
Kd=0`) a la espera de sintonización con el motor real (Ziegler-Nichols
en lazo cerrado, ver sección 9; solo se pueden tocar en
`CONTROL_HABILITADO=3`) — suficientes para validar que el lazo mueve
el servo en la dirección correcta, no para un control ya afinado.

**Alimentación del servo**: fuente externa 5-6V, GND común con el
G431, señal PWM a 3.3V (compatible con la mayoría de servos de RC sin
level shifter).

### 2.5 `rtc_reloj.c/h`

Reloj interno del STM32G431 sobre el periférico **RTC**, con **LSI**
(~32kHz nominal, confirmado en el datasheet del G431 y en el diagrama
de árbol de reloj de CubeMX) como fuente — **no hay cristal LSE
poblado** en este Nucleo-32, ni pila de respaldo (VBAT) dedicada.

**Consecuencia práctica**: la hora se pierde en cada corte real de
energía (no en un `NVIC_SystemReset()` mientras VDD no se interrumpa)
— por eso el firmware **busca resincronizarse (por GPS o por red
LoRaWAN, ver 2.2) en cada arranque**, en vez de asumir que el RTC
"recuerda" la hora entre encendidos. Además, al no tener cristal LSE,
el LSI deriva con el tiempo (**~0.84% medido en campo**, ver 2.2) — por
eso la resincronización tampoco es única al arrancar, sino periódica
durante toda la operación.

- El RTC guarda la hora en **UTC** (la misma que entrega
  `DeviceTimeReq`) — el offset de **-6h de Guatemala se aplica solo al
  leer la hora para el payload de salida** (`Reloj_GetUnixTimeLocal()`),
  nunca se le resta al RTC en sí.
- Conversión epoch↔calendario implementada **a mano** (sin
  `time.h`/`gmtime` de la libc), para no depender de que la
  newlib-nano del proyecto tenga esas funciones enlazadas. Verificada
  contra `datetime` de Python en varios casos, incluyendo bordes de fin
  de año.
- **Prescalers del RTC**: `AsynchPrediv=99` / `SynchPrediv=319`
  (`100 × 320 = 32000`), calculados para el LSI de 32kHz nominal del
  G431 — **no usar el default de CubeMX (127/255, pensado para un LSE
  de 32.768kHz)**, da un error de base de ~2.3% además de la
  imprecisión propia del LSI. Configurado directamente en el `.ioc`
  (panel RTC), no hardcodeado en código.
- `Reloj_EstaSincronizado()` es `false` hasta el primer
  `Reloj_SetUnixTimeUtc()`/`Reloj_SetHoraUtc()` exitoso (por GPS o por
  red, ver 2.2) — el uplink LIVE **no** espera este flag para empezar a
  mandarse (ver 2.2), solo lo usa para saber si vale la pena reportar
  `RelojSync=1` en el log de depuración.

### 2.6 `gps.c/h`

Posición GPS/GNSS del módulo **SIM7600X (Waveshare 4G HAT)** sobre
USART2, DMA en modo Circular + detección de línea inactiva (IDLE) —
mismo patrón de recepción que `rak3172.c`, pero para un flujo periódico
en vez de comando/respuesta.

⚠️ **Cambio de diseño importante (confirmado en campo, 2026-08-18)**:
este módulo **NO transmite NMEA crudo espontáneamente** por esta UART
(probado a mano vía USB-TTL directo). Lo que sí soporta, documentado en
el *AT Command Manual* de SIMCom, es `AT+CGPSINFO=<1-255>`: configurado
una sola vez en `main.c`, el módulo empieza a mandar una línea
`+CGPSINFO: ...` por su cuenta cada N segundos (URC, igual que los
`+EVT:` del RAK3172) — no es necesario volver a pedirla.

Formato de `+CGPSINFO:` (sin checksum, a diferencia de NMEA):
```
+CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<fecha ddmmyy>,<hora hhmmss.s>,<alt>,<vel>,<rumbo>
```
Sin fix, todos los campos vienen vacíos: `+CGPSINFO: ,,,,,,,,`.

**Secuencia de inicio** (`main.c`): `GPS_Init(&huart2)` arranca la
recepción, una pausa de 500ms, `AT+CGPS=1` (enciende el motor GNSS;
responde `ERROR` —inofensivo— si el módulo ya estaba encendido de una
sesión previa), una pausa de 1.5s, y por último `AT+CGPSINFO=10`
(habilita el auto-reporte cada 10s). Simple, sin lógica adicional —
el bug de abajo **no era esto**.

✅ **RESUELTO (2026-08-25) — causa raíz real: el cable USB del Nucleo
a la PC, no el firmware.** El GPS nunca daba fix tras un arranque en
frío real **mientras el cable USB del Nucleo a la computadora estaba
conectado** (el mismo que se usa para ver el monitor serie de debug).
Confirmado con una prueba A/B limpia, mismo hardware, un solo cable
como única variable: con el USB del Nucleo conectado a la PC, el GPS
se queda en `+CGPSINFO: ,,,,,,,,,` (vacío) indefinidamente; al
desconectarlo, engancha fix casi de inmediato. Reproducido de forma
consistente. Hipótesis del mecanismo: con el Nucleo alimentado por
batería **y** por USB al mismo tiempo, las dos tierras (batería/PC)
meten ruido eléctrico al plano de tierra compartido — un UART digital
lo tolera sin problema (por eso los comandos AT siempre respondían
bien), pero el front-end de RF del receptor GNSS, tratando de
detectar señales de satélite muy débiles, es mucho más sensible a ese
ruido.

**Implicación importante**: en campo, el Nucleo va a estar alimentado
solo por batería, sin ninguna laptop conectada por USB — es probable
que este bug **nunca ocurra en operación real**, solo se manifestaba
por estar viendo el log de depuración en el banco mientras el sistema
corría con alimentación externa simultánea. Pendiente de una
confirmación final: un ciclo de encendido con USB desconectado desde
el principio (no solo reconectado después) para verificar que el GPS
ya tiene fix cuando finalmente se conecta el monitor.

**Vías investigadas y descartadas antes de encontrar la causa real**
(no volver a intentarlas sin evidencia nueva — todas con evidencia
real de campo):
- Timing de `AT+CGPS=1`/`AT+CGPSINFO=10` (delays de 0.5s a 10s, espera
  event-driven del `'RDY'`) — el comando siempre llegaba y se
  ejecutaba bien; probado también con delay largo y corto vía USB-TTL
  aislado, ambos funcionaron — no era timing.
- `AT+CGPSCOLD` (cold start explícito) en vez de `AT+CGPS=1` — devolvió
  `ERROR`, no soportado en esta versión de firmware del módulo.
- Interferencia RF del RAK3172 (LoRaWAN transmitiendo durante la
  adquisición GNSS) — descartado con el RAK3172 desconectado de
  energía, el GPS seguía sin dar fix mientras el USB del Nucleo
  estuviera conectado.
- Capacitor electrolítico local en los pines de alimentación del
  módulo GPS — probado, no cambió nada.
- Regulador compartido con el servo — descartado, ya están en ramas
  separadas.
- "El STM32 no reflashea bien" — descartado: un simple reset por botón
  (sin cargar código nuevo) ya funcionaba, así que nunca fue cuestión
  de código nuevo.

**Prioridad de posición para el uplink LIVE** (`main.c`): GPS con fix
vivo (`GPS_TieneFix()`) → última posición conocida persistida en flash
(`CalibFlash_GetUltimaLatitudConocida()`/`...Longitud...()`, se guarda
una sola vez por arranque, la primera vez que hay fix — no en cada
reporte, para no desgastar la flash) → `LATITUD_FIJA`/`LONGITUD_FIJA`
como último respaldo si el GPS nunca ha conseguido fix (ni en este
arranque ni en ninguno anterior).

**Disciplina del RTC**: `GPS_GetFechaHoraUtc()` expone la fecha/hora
UTC del último `+CGPSINFO:` con fix (campos `fecha ddmmyy`/`hora
hhmmss.s` del formato de arriba — antes descartados, solo se usaba
lat/lon). Es la fuente **primaria** de sincronización del RTC, con la
red LoRaWAN como respaldo — ver sección 2.2 para el mecanismo completo.

**⚠️ Hallazgos de hardware del Waveshare SIM7600X-H 4G HAT** (ninguno
obvio desde el firmware, costaron varias horas de campo):

- **Alimentación**: el módulo necesita una fuente de 5V **separada**
  del STM32 (picos de hasta ~2A) — alimentarlo desde el pin 3.3V/5V del
  Nucleo no le alcanza para arrancar el módem por completo (el LED
  "PWR" enciende igual, pero el chip nunca llega a responder ningún
  comando AT). GND sí debe ser compartido con el STM32.
- **Jumper "PWR"**: debe estar en **`PWR—3V3`** (arranque automático al
  detectar alimentación), no en `PWR—D6` (que espera una señal de
  encendido de un GPIO de Raspberry Pi que este proyecto no tiene —
  sin ella, el módem nunca arranca, aunque el LED de alimentación esté
  encendido).
- **Jumper "UART JMP"** (3 posiciones, cada una puentea un par
  adyacente en la fila CP2102—PI—SIM7600X): **posición B** (PI↔SIM) es
  la que conecta el módem directo a los pines del header (donde está
  cableado el STM32, haciendo de "PI"). Posición A (CP2102↔PI) y C
  (SIM↔CP2102) enrutan hacia el chip USB-serial integrado, no hacia el
  header — con el jumper en la posición equivocada, el STM32 no recibe
  absolutamente nada aunque el cableado esté perfecto.
- **Antena**: la de GPS va específicamente en el conector marcado
  **GNSS** (no en `MAIN` ni `AUX`, que son para la antena celular/LTE).

**Bug corregido (2026-08-18)**: el ciclo de consumo del buffer circular
en `GPS_RxEventCallback()` se colgaba indefinidamente cuando el DMA
completaba una vuelta exacta del buffer (HAL reporta `Size==tamaño del
buffer` en vez de `0` en ese caso) — congelaba el `while(1)` completo
del `main()` (incluida la impresión de `Frecuencia`), no solo al GPS.
Corregido acotando `Size` a `0` cuando llega igual al tamaño del
buffer.

**Reintento automático tras reportes vacíos seguidos**: si llegan
`GPS_REPORTES_VACIOS_ANTES_DE_REENVIAR` (10, `gps.h`) reportes
`+CGPSINFO: ,,,,,,,,` **seguidos** (~100s sin fix a razón de uno cada
10s), `GPS_ProcesarCGPSInfo()` reenvía `AT+CGPS=1` por su cuenta y
reinicia el contador. Es una red de seguridad barata, no una solución
al bug de arriba (ese ya se resolvió y no era del módulo) — si el GPS
está sano y solo tarda en conseguir fix (mal clima, sin vista al
cielo), el reenvío es inofensivo: con el GPS ya encendido, `AT+CGPS=1`
solo contesta `ERROR` sin interrumpir la adquisición en curso. El
contador se reinicia también en cuanto llega un fix real.

### 2.7 `comando_serial.c/h` — mando manual TEMPORAL por serial

Mientras no hay red LoRa disponible en campo para probar downlinks
reales, este módulo permite escribir a mano un "downlink" por el mismo
puerto de debug (LPUART1) que ya se usa para ver los logs. Sondea la
bandera RXNE del UART sin bloquear (sin IT/DMA — un operador tipeando
nunca lo satura), arma la línea con eco local, y al recibir Enter la
pasa por `CalibFlash_ProcesarParametroConEstado()` — **el mismo punto
de entrada que usa `rak3172.c` para un downlink real** (misma
validación de rango, mismo bloqueo por categoría con el motor
operando, misma persistencia en flash). La única diferencia real es el
transporte: no manda el Application ACK por LoRaWAN
(`RAK3172_EnviarAck`), el resultado se imprime directo al mismo
puerto, como si fuera el ACK.

Formato de línea: `NOMBRE_PARAMETRO [VALOR]`, con el mismo nombre y el
mismo valor "humano" (sin escalar) que se manda por MQTT vía
`RIO-DSL-SendDownlink` (ver sección 6) — ej. `CONTROL_HABILITADO 1`,
`SET_RPM 900`, `SERVO_PULSO_MIN 1050`. Los comandos
(`RESTAURAR_DEFAULTS`, `FORZAR_REPORTE`, `RESET_REMOTO`) no llevan
VALOR, el módulo manda el byte de confirmación `0xA5` automáticamente.

Este módulo es un mando temporal — quitar su llamada en `main.c`
(`ComandoSerial_Init()`/`ComandoSerial_Update()`) cuando exista un
mando local real (pantalla/botonera) o ya no se necesite probar sin
red LoRa.

### Formato del downlink (FPort 2)

```
[PARAMETER_ID: 1 byte][VALUE_H: 1 byte][VALUE_L: 1 byte]   (3 bytes)
```

### Formato del Application ACK (FPort 3)

```
[PARAMETER_ID: 1 byte][STATUS: 1 byte][VALUE_H: 1 byte][VALUE_L: 1 byte]   (4 bytes)
```

`VALUE` en el ACK es el valor que **realmente quedó vigente** — si el
downlink fue rechazado, es el valor anterior, no el solicitado.

### Códigos de STATUS

| Código | Nombre | Significado |
|---|---|---|
| 0 | `OK` | Se aplicó correctamente |
| 1 | `OUT_OF_RANGE` | Valor fuera del rango válido |
| 2 | `UNKNOWN_PARAMETER_ID` | ID no reconocido, o longitud de datos insuficiente |
| 3 | `STORAGE_ERROR` | Falló la escritura en flash (hardware) |
| 4 | `APPLY_ERROR` | Precondición no cumplida (ej. `SERVO_PULSO_MIN/MAX` con `CONTROL_HABILITADO=0`) |
| 5 | `REJECTED_ENGINE_RUNNING` | Parámetro de categoría CONFIGURACION, motor operando |

### Tabla completa de parámetros

| ID | Nombre | Escala | Ancho | Categoría | Notas |
|---|---|---|---|---|---|
| 1 | `SET_RATIO` | x100 | 2B uint16 | Calibración | Pulsos/revolución |
| 2 | `ALPHA` | x1000 | 2B uint16 | Calibración | Coef. filtro EMA (0-1) |
| 3 | `SET_RPM` | x10 | 2B uint16 | Proceso | Setpoint directo (uso interno/pruebas, MODO=0) |
| 4 | `RPM_MAX` | x10 | 2B uint16 | Configuración | Límite duro superior |
| 5 | `RPM_MIN` | x10 | 2B uint16 | Configuración | Límite duro / ralentí |
| 6 | `PID_KP` | x100 | 2B int16 | Calibración | Con signo. Solo se acepta con `CONTROL_HABILITADO=3` (modo sintonización PID) — `APPLY_ERROR` en cualquier otro modo, incluida la operación normal (`=0`). Se permite con el motor operando (necesario para sintonizar en lazo cerrado, ver sección 9) |
| 7 | `PID_KI` | x1000 | 2B int16 | Calibración | Con signo. Igual que `PID_KP` |
| 8 | `PID_KD` | x1000 | 2B int16 | Calibración | Con signo. Igual que `PID_KP` |
| 9 | `SERVO_PULSO_MIN` | directo (µs) | 2B uint16 | Configuración | Límite mecánico. Además requiere `CONTROL_HABILITADO=1` o `=2` (modo calibración del servo, NO `=3`) — si no, `APPLY_ERROR` aunque el motor esté apagado |
| 10 | `SERVO_PULSO_MAX` | directo (µs) | 2B uint16 | Configuración | Igual que `SERVO_PULSO_MIN` |
| 11 | `TIMEOUT_SIN_COMANDO_S` | directo (s) | 2B uint16 | Configuración | 60-3600s |
| 12 | `TASA_MAX_CAMBIO_RPM_S` | x10 | 2B uint16 | Configuración | Rampa normal |
| 13 | `CONTROL_HABILITADO` | 0/1/2/3 | 1B | Configuración | Enable/disable lazo de control + **modos de calibración** (con el motor ya apagado, por la regla general de categoría Configuración, para entrar a `1`/`2`/`3`): `0` desactivado (lazo PID normal si aplica), `1` barrido automático continuo entre `SERVO_PULSO_MIN/MAX`, `2` manual -- el servo se mantiene quieto en su posición, y cada downlink de `SERVO_PULSO_MIN` o `SERVO_PULSO_MAX` lo mueve directo a ese valor, `3` **sintonización de PID** -- el servo lo maneja el PID normal exactamente igual que en `0`, pero es el único modo en que `PID_KP/KI/KD` se aceptan (y activa el log `PID_TEST`, ver sección 9); a diferencia de `1`/`2`, el motor SÍ puede seguir operando en `3` sin que se fuerce de vuelta a `0` (la sintonización lo requiere). En `1` o `2` se habilita cambiar `SERVO_PULSO_MIN/MAX`. El firmware fuerza `CONTROL_HABILITADO=0` localmente en el instante que el motor arranca **solo si estaba en `1` o `2`** (nunca en `3`), y también fuerza a `0` en cada arranque del firmware (`CalibFlash_Init()`), sin importar el valor que haya quedado guardado en flash de una sesión anterior — nunca reanuda ningún modo de calibración solo. **Medida de seguridad adicional**: al entrar a `1`, `2` o `3` (downlink aceptado con valor != 0), `SET_RPM` se limpia a `0` (su estado "sin comandar") — evita que un `SET_RPM` que haya quedado de una operación anterior active el PID solo con el motor arrancando en modo `3`, sin que el operador lo haya vuelto a pedir explícitamente para esa sesión. Mismo criterio en el camino de regreso: cuando el apagado de seguridad de `main.c` fuerza `CONTROL_HABILITADO` de `1`/`2` de vuelta a `0` (motor arrancando durante calibración del servo), también limpia `SET_RPM` a `0` — por si se había mandado un `SET_RPM` mientras se calibraba (categoría Proceso, siempre se acepta, sin importar el modo), que no quede activando el PID solo al volver a operación normal |
| 14 | `INTERVALO_ENVIO_OPERATIVO_S` | directo (s) | 2B uint16 | Configuración | Uplink en operación |
| 15 | `INTERVALO_ENVIO_STANDBY_S` | directo (s) | 2B uint16 | Configuración | Uplink en standby |
| 16 | `MODO` | — | 1B | Configuración | 0=Ralentí, 1=Local, 2=Remoto |
| 17 | `PRESION` | x10 | 2B uint16 | Proceso | Presión del aspersor (remota) |
| 18 | `NODE_ID` | — | 1B | Configuración | Uso futuro (multicast) |
| 19 | `RESTAURAR_DEFAULTS` | — | 1B | Comando | Requiere byte confirmación `0xA5` |
| 20 | `FORZAR_REPORTE` | — | 1B | Comando | Requiere byte confirmación `0xA5` |
| 21 | `HISTERESIS_MODO_S` | directo (s) | 2B uint16 | Configuración | Anti-parpadeo intervalo envío |
| 22 | `RESET_REMOTO` | — | 1B | Comando | ⚠️ No conectado aún, ver pendientes |
| 23 | `PRESION_OBJETIVO` | x10 | 2B uint16 | Configuración | Umbral fase llenado→régimen |
| 24 | `TASA_MAX_CAMBIO_RPM_LLENADO_S` | x10 | 2B uint16 | Configuración | Rampa conservadora (llenado tubería) |

**Ganancias PID con signo**: se codifican como `int16` (complemento a
2), no `uint16` — un consumidor debe reinterpretar valores > 32767
restando 65536 antes de dividir por la escala.

---

## 4. Máquinas de estado

### 4.1 Bloqueo de configuración por operación (implementada)

```
Motor detenido  -> cualquier parametro se puede cambiar
Motor operando  -> solo CALIBRACION y PROCESO se aceptan;
                   CONFIGURACION se rechaza (STATUS=5)
```

Determinado por `!Tacometro_EstaDetenido()`, consultado en
`rak3172.c` y pasado a `CalibFlash_ProcesarParametroConEstado()`.

### 4.2 Estado del motor para telemetría (ENCENDIDO/APAGADO, parcialmente implementada)

Distinto de la máquina de estados del gobernador (4.3, sin
implementar) — este es el campo `estado` que viaja en el uplink LIVE
(ver sección 6), calculado en `main.c`:

```
ESTADO_APAGADO   (2): rpm == 0
ESTADO_ENCENDIDO (3): rpm > 0, placeholder hasta que exista presión real
ESTADO_ACTIVO    (1): rpm > 0 Y con presión de salida del motor -- NO
                       alcanzable aún, falta presion.c (ver sección 5)
```

`inicio_operacion`/`segundos_transcurridos` se recalculan cada vez que
`estado` cambia, usando `Reloj_GetUnixTimeLocal()` (ver el detalle del
"desplazamiento en vez de reinicio" al sincronizar, sección 2.2).

### 4.3 Modos de operación del motor (MODO 0/1/2) — **pendiente de implementar**

Terminología real del gobernador que se está reemplazando:

```
Modo 0: Ralenti       - del encendido al ralenti
Modo 1: Local         - gobernado por presion de la motobomba (sensor propio, 4-20mA)
Modo 2: Remoto        - gobernado por presion del aspersor (downlink PRESION)
```

- Transición Modo 0→1: pendiente de confirmar con cliente (¿por
  tiempo fijo, o estabilización de RPM?).
- Transición Modo 1→2: automática cuando `PRESION >= PRESION_OBJETIVO`,
  o por selector manual (a definir si existe físicamente).
- Posible regreso Modo 2→1 si se pierde comunicación con el aspersor
  (usar `TIMEOUT_SIN_COMANDO_S`) — pendiente de definir.
- Dentro de Modo 2 (remoto): sub-fase de llenado vs. régimen, ya
  resuelta a nivel de parámetros (`TASA_MAX_CAMBIO_RPM_LLENADO_S` vs.
  `TASA_MAX_CAMBIO_RPM_S`), falta la lógica de conmutación en tiempo
  de ejecución.

### 4.4 Modo calibración del servo (`CONTROL_HABILITADO`, implementada)

Independiente de 4.1-4.3 — no es un modo de operación del motor, es el
mecanismo para ajustar en banco los topes mecánicos del acelerador
(`SERVO_PULSO_MIN/MAX`) sin reflashear, descrito en detalle en la
sección 2.4. `CONTROL_HABILITADO=0` en realidad cubre dos sub-casos,
resueltos en `main.c` cada vuelta del loop según el motor **y** si se
comandó control (`SET_RPM > RPM_MIN`, ver 2.4):

```
CALIBRACIÓN-BARRIDO (CONTROL_HABILITADO=1): servo en barrido continuo
                                     MIN <-> MAX. Requiere motor
                                     detenido para entrar y para
                                     permanecer -- se sale solo si el
                                     motor arranca.
CALIBRACIÓN-MANUAL  (CONTROL_HABILITADO=2): servo quieto en su
                                     posición actual; cada downlink de
                                     SERVO_PULSO_MIN o SERVO_PULSO_MAX
                                     lo mueve directo a ese valor.
                                     Requiere motor detenido para
                                     entrar y para permanecer, igual
                                     que el barrido.
SINTONIZACIÓN-PID   (CONTROL_HABILITADO=3): el servo lo maneja el PID
                                     normal, EXACTAMENTE igual que en
                                     SEGURO/PID ACTIVO -- lo único que
                                     cambia es que PID_KP/KI/KD se
                                     desbloquean y se activa el log
                                     PID_TEST (ver sección 9). Requiere
                                     motor detenido para ENTRAR, pero a
                                     diferencia de los dos modos de
                                     arriba, el motor SÍ puede arrancar
                                     y seguir operando sin que se
                                     fuerce la salida -- la
                                     sintonización en lazo cerrado
                                     necesita el motor corriendo todo
                                     el tiempo (README sección 9).
SEGURO      (CONTROL_HABILITADO=0 o =3, motor detenido, O motor
             operando en su ralentí natural sin SET_RPM > RPM_MIN
             comandado):             servo fijo/regresando a
                                     SERVO_PULSO_MIN (sin
                                     aceleración). Estado por defecto.
PID ACTIVO  (CONTROL_HABILITADO=0 o =3, motor operando, Y SET_RPM >
             RPM_MIN comandado explícitamente): servo controlado por
                                     pid.c/h (ver 2.4), no por una
                                     posición fija.
```

- `* -> CALIBRACIÓN-BARRIDO/MANUAL/SINTONIZACIÓN-PID`: downlink
  `CONTROL_HABILITADO=1`, `=2` o `=3`, solo se acepta con el motor
  detenido (regla general de 4.1, categoría Configuración) — con el
  motor operando se rechaza con `REJECTED_ENGINE_RUNNING` (STATUS=5).
  Esto aplica igual a `=3`, aunque su propósito sea usarse con el motor
  corriendo — hay que entrar **antes** de arrancarlo.
- `CALIBRACIÓN-BARRIDO ⇄ CALIBRACIÓN-MANUAL ⇄ SINTONIZACIÓN-PID`: por
  downlink directo entre `1`, `2` y `3` (los tres requieren motor
  detenido para el cambio, igual que entrar desde `SEGURO`). Al entrar
  a `CALIBRACIÓN-MANUAL` el servo arranca quieto en la posición en la
  que estaba (no salta a `SERVO_PULSO_MIN` ni `MAX`) hasta el primer
  downlink de esos dos parámetros.
- `CALIBRACIÓN-BARRIDO/MANUAL -> SEGURO`: por downlink
  `CONTROL_HABILITADO=0`, **o** automáticamente en el firmware (sin
  downlink) si el motor arranca mientras se está en uno de esos dos
  modos — medida de seguridad, nunca se deja el servo bajo barrido o
  posición manual con el motor operando. Si además ya había un
  `SET_RPM > RPM_MIN` comandado desde antes, ese mismo arranque cae
  directo en `PID ACTIVO` en vez de `SEGURO`. **`SINTONIZACIÓN-PID` no
  tiene esta salida automática** — el motor arrancando ahí es
  justamente lo esperado, no una condición de falla.
- `SEGURO ⇄ PID ACTIVO` (dentro de `CONTROL_HABILITADO=0` **o** `=3`):
  automático según `Tacometro_EstaDetenido()` y si `SET_RPM` supera
  `RPM_MIN`, sin downlink de por medio para el motor (arrancar/detener
  basta) — el ralentí (Modo 0, sección 4.3) es deliberadamente sin
  control, nunca se recorta `SET_RPM` hacia arriba hasta `RPM_MIN` para
  forzar el lazo. Al entrar a `PID ACTIVO` se llama `PID_Init()` una
  sola vez (flanco de entrada), para no arrastrar estado de una
  activación anterior. El comportamiento del servo es idéntico en `0`
  y en `3` — la diferencia entre ambos está solo en qué parámetros se
  pueden tocar y si se ve el log `PID_TEST`, no en el control en sí.
- `SERVO_PULSO_MIN/MAX` solo se pueden cambiar por downlink estando en
  `CALIBRACIÓN-BARRIDO` o `CALIBRACIÓN-MANUAL` (NO en
  `SINTONIZACIÓN-PID`); fuera de esos dos modos se rechazan con
  `APPLY_ERROR` (STATUS=4) aunque el motor esté apagado. En
  `CALIBRACIÓN-BARRIDO` el cambio tiene efecto inmediato sobre el
  barrido en curso; en `CALIBRACIÓN-MANUAL` mueve el servo directo al
  valor recién configurado.
- `PID_KP/PID_KI/PID_KD` solo se pueden cambiar estando en
  `SINTONIZACIÓN-PID` (`CONTROL_HABILITADO=3`); en cualquier otro modo
  (incluida la operación normal, `=0`) se rechazan con `APPLY_ERROR`,
  motor operando o no — evita que las ganancias del PID cambien fuera
  de una sesión deliberada de sintonización.

---

## 5. Sensor de presión (motobomba, 4-20mA) — **diseñado, no construido**

Sensor: **PCM300-1M-G-B1-C15-J4** (0-1MPa, salida 4-20mA, alimentación
12-30VDC).

Circuito de acondicionamiento (reemplaza intento inicial con
RAK5801, descartado por requerir modificación de hardware no
autorizada):

```
Lazo 4-20mA -> R_burden (49.9 ohm, mismo valor que usa el RAK5801 de
              referencia) -> GND
                    |
              LM358 (config. no inversora, ganancia 3)
              R1=10k ohm, R2=20k ohm (Ganancia = 1 + R2/R1 = 3)
                    |
              Filtro RC (1k ohm + 100nF) -> PA1 (ADC1_IN2 del G431)
```

Rango de salida esperado: 0.6V (4mA) a 3.0V (20mA) — dentro del rango
0-3.3V del ADC, sin necesitar ADC externo (ADS1115 descartado, la
resolución de 12 bits del ADC interno ya da ~0.336 kPa/paso, más que
suficiente).

Canal B del LM358 (no usado): ambas entradas a GND, para evitar
oscilación/ruido acoplado desde el canal flotante.

**Pendiente**: construir el circuito, escribir `presion.c/h`
(lectura ADC + conversión a mA + conversión a presión real según el
rango del sensor), y confirmar el voltaje de alimentación del lazo
(12-24VDC, fuente separada del circuito de 3.3V). Hasta que esto
exista, el uplink LIVE manda `presion = 0.0` como placeholder y el
estado `ACTIVO` (que depende de esta lectura) no es alcanzable.

---

## 6. Integración con AWS IoT Core for LoRaWAN

- **Dispositivo**: `DSL-0001`, Device Profile `RIO-DSL` (US915, Clase
  C, LoRaWAN 1.0.3), Service Profile `RIO` (compartido con el nodo
  aspersor del equipo).
- **Regla de IoT**: `RIO_DSL_Lambda`, tópico de entrada `RIO/DSL`.
- **Lambda de uplinks**: `RIO-DSL-DecodeUplink` (Python 3.12,
  `lambda_function.py` + `decoder.py`).
- **Lambda de downlinks**: `RIO-DSL-SendDownlink`
  (`send_downlink.py` + `encoder.py`), dispara
  `iotwireless:SendDataToWirelessDevice` a partir de un mensaje MQTT
  con el nombre del parámetro (ej. `{"SET_RPM": 450}`), sin exponer
  el ID numérico al usuario.

### Forma real del evento que recibe la Lambda de uplinks (confirmada con tráfico real)

```json
{ "PayloadData": "<base64>", "Fport": 1 }
```

⚠️ **No** viene anidado bajo `WirelessMetadata.LoRaWAN`, y **no**
incluye `DevEui` en absoluto (solo `FCnt` y `FPort`) — confirmado
capturando un mensaje real en el cliente de prueba MQTT, no asumido de
la documentación.

### Consulta SQL de la regla (uplinks)

```sql
SELECT VALUE
  aws_lambda(
    "arn:aws:lambda:us-east-1:271551172366:function:RIO-DSL-DecodeUplink",
    { "PayloadData": PayloadData, "Fport": WirelessMetadata.LoRaWAN.FPort }
  )
FROM 'RIO/DSL'
```

`SELECT VALUE` (en vez de listar campos con `as`) hace que el
resultado de la Lambda salga **plano** en el tópico de salida, sin
envolverlo en una propiedad extra.

Acción: **Republish** a `RIO/DSL/DECODED`, rol IAM `GIO_role`
(compartido con la regla del nodo aspersor).

### Layout de bytes del uplink LIVE (FPort 1, 27 bytes)

Debe coincidir byte a byte entre `RAK3172_EnviarUplinkLive()`
(firmware) y `decoder.py::_decodificar_live()` (Lambda):

| Offset | Bytes | Campo | Formato |
|---|---|---|---|
| 0–1 | 2 | `motorIdNumeric` | uint16 BE (ej. 1 → "DSL-0001") |
| 2–3 | 2 | `rpm` | uint16 BE, x10 |
| 4–5 | 2 | `presion` (salida del motor) | uint16 BE, x10 — placeholder 0 hasta `presion.c` |
| 6 | 1 | `estado` | uint8 — 1=ACTIVO, 2=APAGADO, 3=ENCENDIDO |
| 7–10 | 4 | `fecha_hora` (unix, ya con -6h aplicado en el STM32) | uint32 BE |
| 11–14 | 4 | `inicio_operacion` (unix, ya local) | uint32 BE |
| 15–18 | 4 | `segundos_transcurridos` | uint32 BE |
| 19–22 | 4 | `latitud` | int32 BE, x10,000,000 |
| 23–26 | 4 | `longitud` | int32 BE, x10,000,000 |

`latitud`/`longitud` vienen del GPS en vivo cuando hay fix, con
respaldo a la última posición conocida persistida en flash, y a las
coordenadas fijas `14.27387764641955, -91.09260397779028` como último
recurso si el GPS nunca ha conseguido fix — ver prioridad completa en
2.6.

### Formato de salida de la Lambda

```python
# LIVE (FPort 1) -- registro plano estilo GIOSoftware:
{
  "fecha_hora": "2026-08-14 14:45:41",
  "codigo_maquinaria": "DSL-0001",
  "tipo": "Motor Diesel",
  "area": "Riegos",
  "presion": 88.5,
  "nombre_operacion": "ACTIVO",
  "codigo_operacion": 1,
  "clasificacion_operacion": "Productivo",
  "inicio_operacion": "2026-08-14T14:44:31",
  "tiempo_transcurrido": "0 dias, 00:01:10",
  "segundos_transcurridos": 70,
  "longitud": -91.092375,
  "latitud": 14.2740023
}

# ACK (FPort 3):
{"parameter": "SET_RATIO", "status": "OK", "valorAplicado": 17.5}
```

⚠️ **SUPUESTOS pendientes de confirmar contra lo que espera
GIOSoftware** (marcados en `decoder.py`): `tipo`/`area` fijos como
`"Motor Diesel"`/`"Riegos"`; `codigo_operacion` reutiliza 1=ACTIVO y
2=APAGADO (mismos números que el nodo aspersor, para consistencia
entre tipos de máquina) y asigna 5 a `ENCENDIDO` (estado nuevo, sin
equivalente en el aspersor).

El `valorAplicado` del ACK ya viene **convertido a valor real** (no
crudo) — la Lambda aplica la escala correcta según el parámetro
(`ESCALA_PARAMETRO` en `decoder.py`, debe coincidir exactamente con la
escala del firmware).

---

## 7. Cómo probar un downlink de configuración manualmente

1. Calcular el payload (Python):
   ```python
   import base64
   id_byte = 1  # SET_RATIO
   valor_escalado = round(17.5 * 100)  # 1750
   payload = bytes([id_byte, (valor_escalado >> 8) & 0xFF, valor_escalado & 0xFF])
   print(base64.b64encode(payload).decode())  # AQbW
   ```
2. AWS IoT Core → Conectividad inalámbrica → Dispositivos → `DSL-0001`
   → pestaña "Cola" → encolar con `FPort=2`, payload en base64.
   (O, con `RIO-DSL-SendDownlink` ya desplegado, simplemente publicar
   `{"SET_RATIO": 17.5}` en el tópico de downlink correspondiente.)
3. Confirmar en el monitor serie del G431 (`Downlink ID=... STATUS=...`)
   y en el cliente MQTT (`RIO/DSL/DECODED`).

**Sin red LoRa disponible** (ej. en banco, o en campo antes de tener
cobertura): usar el mando manual por serial en su lugar, escribiendo
directo en el mismo monitor de depuración — ver `comando_serial.c/h`,
sección 2.7. Mismo nombre y mismo valor humano que en el paso 2, ej.
escribir `SET_RATIO 17.5` y Enter.

---

## 8. Pendientes generales

- [x] Lazo PID (`pid.c/h`) implementado y activo en `main.c` (motor
      operando, sin calibración) — ver secciones 2.4 y 4.4.
- [ ] Ganancias reales `PID_KP/KI/KD` — siguen en default (`1.0/0/0`),
      pendientes de sintonizar en el motor real. Ver sección 9 para el
      procedimiento (Ziegler-Nichols en lazo cerrado, sin MATLAB) — solo
      se pueden tocar en `CONTROL_HABILITADO=3`.
- [x] **AWS**: `send_downlink.py`'s `PARAMETER_TABLE["CONTROL_HABILITADO"]["max"]`
      subido de `2` a `3` para aceptar el nuevo modo de sintonización de
      PID (mismo ajuste que ya se había hecho cuando se agregó el valor
      `2`).
- [ ] Setpoint real del PID — hoy usa `SET_RPM` (prueba manual); falta
      conectarlo a la máquina Modo 0/1/2 (sección 4.3) para que la
      presión decida el objetivo de RPM.
- [ ] Posible parámetro 25, `INTEGRAL_MAX` (anti-windup configurable
      del PID) — hoy el anti-windup es fijo en `pid.c` (integración
      condicional, sin límite configurable por downlink).
- [ ] Máquina de estados Modo 0/1/2 (sección 4.3) — sin implementar en
      firmware. Mientras tanto, la distinción "Modo 0 (ralentí, sin
      control) vs. control activo" ya existe pero **implícita**: se
      infiere solo de `SET_RPM > RPM_MIN` (ver 2.4/4.4), sin ninguna
      señal explícita de modo.
- [ ] `MODO` (ID 16) — existe en el protocolo (persistido, con
      getter/setter) pero **nadie lo lee** en `main.c`, igual que
      estaba `CONTROL_HABILITADO` antes de cablearlo. Además su enum
      en código (`CALIB_MODO_MANUAL/AUTOMATICO/MANTENIMIENTO`,
      `calibracion_flash.h`) **no coincide** con los nombres que usa
      el README (`Ralentí/Local/Remoto`, tabla de parámetros y sección
      4.3) — hay que decidir la terminología real y corregir el enum
      antes de conectarlo, no después.
- [ ] Construcción física y prueba del circuito de presión (LM358).
- [ ] `presion.c/h` — módulo de lectura/conversión, sin escribir aún.
      Una vez que exista, conectar el estado `ACTIVO` (sección 4.2) a
      una lectura real en vez del placeholder `0.0`.
- [ ] `RESET_REMOTO` (ID 22) — recibido y validado, pero
      deliberadamente **no conectado** a ninguna acción real hasta
      definir qué hace el servo durante un reinicio (mantener última
      posición vs. quedar sin control unos segundos).
- [ ] Valores reales (no placeholder) de `RPM_MAX/MIN`,
      `TIMEOUT_SIN_COMANDO_S`, `HISTERESIS_MODO_S` — pendientes de
      definir con datos del motor/cliente real.
- [ ] Confirmar con el equipo si los IDs de parámetro son un espacio
      compartido entre tipos de nodo, o independientes por tipo.
- [ ] Verificar si `AT+TIMEREQ=1` requiere mandarse después del primer
      uplink exitoso (no solo después del join) — reporte de la
      comunidad de RAK sugiere que puede fallar si se manda demasiado
      pronto.
- [ ] `MOTOR_ID_NUMERIC` está hardcodeado a `1` en `main.c` — cuando
      exista más de un nodo motor, resolverlo desde
      `CalibFlash_GetNodeId()` (ID 18, `NODE_ID`) en vez de una
      constante.
- [ ] GPS (sección 2.6) — funcionando en campo, con fix real
      confirmado. Pendiente: confirmar si `AT+CGPS=1` devolviendo
      `ERROR` ocasionalmente (ej. si el GPS ya estaba encendido de un
      arranque anterior) necesita manejarse distinto, o si es
      inofensivo como parece hasta ahora.
- [ ] **Reinicios espontáneos del RAK3172 en campo (2026-08-29)** — ver
      sección 2.2. El módulo se reinicia solo (banner de arranque
      reaparece en medio de la sesión), a veces en ciclo continuo.
      Sospecha actual: conexión de antena o alimentación (picos de
      corriente de TX de LoRa), no software — el firmware ya detecta y
      reintenta el join correctamente cuando pasa, pero la causa raíz
      del reinicio en sí sigue sin confirmarse. Revisar cable/conector
      de antena y estabilidad del rail de alimentación del RAK3172
      durante una transmisión (osciloscopio o multímetro en modo
      min/max), y considerar más capacitancia de desacople local.
- [ ] Decidir si `AutoJoin` en `RAK3172_Join()` (`AT+JOIN=1:0:10:8`)
      vuelve a `1` para producción, o se queda en `0` (prueba temporal
      actual, ver 2.2) manejando todos los reintentos desde el host.
- [ ] Quitar los prints de `DIAGNOSTICO TEMPORAL` en `rak3172.c`
      (`RAK3172_ProcesarLinea()` imprimiendo toda línea cruda, y
      `RAK3172_ErrorCallback()`) y `gps.c` (`GPS_ErrorCallback()`) una
      vez confirmada en campo la causa raíz de los reinicios del
      RAK3172 — agregados el 2026-08-29 solo para diagnóstico, ver 2.2.

---

## 9. Sintonización del PID en el motor real (sin MATLAB)

No se usa MATLAB para este proyecto — la sintonización se hace
empírica, directo en el motor real, con los mismos mandos que ya
existen (downlink MQTT o el mando manual por serial de la sección
2.7). No hace falta ningún software externo para lograrlo, aunque un
script de Python puede ayudar a analizar los datos (ver más abajo).

### Por qué hace falta sintonizar (y no solo probar `Kp=1` a ojo)

El PID no "sabe" nada del motor — solo reacciona al error entre
`SET_RPM` y la RPM medida. Las ganancias determinan qué tan agresiva
es esa reacción:

- **`Kp` muy alto**: el servo sobre-corrige, la RPM se pasa del
  setpoint, vuelve a corregir de más en la otra dirección → oscilación
  (el motor "bombea" RPM arriba y abajo). Además de ser inútil para
  operación real, es estrés mecánico repetido sobre la varilla del
  acelerador.
- **`Kp` muy bajo**: reacciona lento y se queda con bastante error de
  estado estable (lo que ya viste con el default `Kp=1.0` en modo
  puramente proporcional).
- **`Ki`**: elimina el error de estado estable que deja un `Kp` por sí
  solo, pero mal puesto agrega sobre-impulso (overshoot) y puede
  tardar en "soltar" la corrección (aunque ya hay anti-windup por
  integración condicional en `pid.c`, ver sección 2.4).
- **`Kd`**: amortigua el sobre-impulso que puede dejar `Ki`, pero
  reacciona a qué tan rápido *cambia* el error — con una medición de
  RPM que ya de por sí "caza" en ralentí (ver la conversación sobre la
  aguja/ralentí inestable, sección 2.1), un `Kd` puesto a la ligera
  amplifica ese ruido en vez de suavizar el control, y el servo se
  pone nervioso. Si se usa, hacerlo con `Kd` pequeño y considerar bajar
  `ALPHA` primero para llegar con una RPM ya más filtrada.

El objetivo de "simular" (en MATLAB, Python, o cualquier otra
herramienta) es siempre el mismo: **probar combinaciones de ganancias
sin gastar tiempo de motor real ni arriesgar una oscilación fuerte en
la varilla física** — se prueba en una computadora primero, y solo se
valida en el motor la combinación que ya se ve razonable en papel. Acá
se salta ese paso intermedio y se sintoniza directo en el motor, con
más cuidado y de forma incremental.

### Restricción importante del firmware: no hay "lazo abierto" con el motor operando

`CONTROL_HABILITADO=1`/`=2` (barrido/manual, sección 2.4) — que sí
permitirían mandar un pulso fijo al servo sin que el PID reaccione —
**requieren el motor detenido para entrar y para permanecer**, y si el
motor arranca estando en cualquiera de esos dos modos, el firmware los
apaga solo de inmediato (medida de seguridad, ver 4.4). Esto es
intencional y no se debe evadir: significa que **no se puede hacer una
prueba de "curva de reacción" en lazo abierto** (mandar un escalón de
pulso fijo y medir cómo responde la RPM) con el motor corriendo —
cualquier prueba en el motor real va a ser necesariamente **en lazo
cerrado**, con el PID ya corriendo. Por suerte, hay un método de
sintonización estándar pensado exactamente para esto: **Ziegler-Nichols
en lazo cerrado (ganancia última)**.

### Procedimiento (Ziegler-Nichols en lazo cerrado)

1. **Con el motor detenido**, mandar `CONTROL_HABILITADO=3` (modo
   sintonización PID, ver sección 4.4) — es el único modo en el que
   `PID_KP/KI/KD` se aceptan; en cualquier otro modo (incluida la
   operación normal, `=0`) se rechazan con `APPLY_ERROR`, por medida de
   seguridad (que no se puedan tocar las ganancias por accidente fuera
   de una sesión deliberada). A diferencia de `=1`/`=2`, este modo
   **no** se apaga solo cuando el motor arranca — hace falta que siga
   operando toda la sesión. Arrancar el motor y dejarlo estabilizar en
   ralentí caliente.
2. Confirmar `PID_KI=0` y `PID_KD=0` (default de fábrica) — se
   sintoniza `Kp` solo primero.
3. Mandar un `SET_RPM` por encima de `RPM_MIN` (un escalón moderado,
   no extremo — ej. 200-300 RPM sobre el ralentí, no el máximo del
   motor) y observar la respuesta en el log de debug (`RPM filtrada`,
   cada 1s) o mejor, capturando el log completo a un archivo (log de
   PuTTY/Tera Term, o un script de Python leyendo el puerto COM y
   guardando CSV con marca de tiempo) para poder medirlo con precisión
   después en vez de a ojo.
4. Ir subiendo `Kp` en pasos pequeños (ej. `PID_KP 1.5`, `PID_KP 2.0`,
   ...) repitiendo el mismo escalón de `SET_RPM` cada vez, hasta que la
   RPM empiece a **oscilar de forma sostenida** (ni crece sin control
   ni se apaga, se mantiene oscilando con amplitud pareja alrededor del
   setpoint). Ese valor de `Kp` es la **ganancia última (`Ku`)**; medí
   el período de esa oscilación (tiempo entre dos picos consecutivos)
   — es el **período último (`Tu`)**.
   ⚠️ Al acercarte a `Ku` la RPM va a oscilar de verdad — tené a mano
   `SET_RPM 0` (por serial o MQTT) para cortar el lazo de inmediato si
   se ve demasiado agresivo, y no dejes esta prueba desatendida.
5. Con `Ku` y `Tu` medidos, aplicar las fórmulas clásicas de
   Ziegler-Nichols (elegir según qué tan agresivo se quiera el control
   final):
   ```
   Solo P:    Kp = 0.50 * Ku
   PI:        Kp = 0.45 * Ku          Ki = Kp / (Tu / 1.2)
   PID:       Kp = 0.60 * Ku          Ki = Kp / (Tu / 2)      Kd = Kp * (Tu / 8)
   ```
   Para un acelerador diésel, conviene arrancar por la fila **PI** (sin
   `Kd`) dado el ruido de medición ya conocido en el ralentí — agregar
   `Kd` solo si de verdad hace falta amortiguar más el sobre-impulso, y
   con cautela.
6. Cargar esas ganancias (`PID_KP`/`PID_KI`/`PID_KD` por downlink o
   serial) y repetir el escalón de `SET_RPM` del paso 3 — buscar una
   respuesta que llegue al setpoint sin oscilar más de una vez y sin
   error de estado estable notorio. Ajustar a mano desde ahí si hace
   falta (bajar un poco `Kp`/`Ki` si todavía se pasa de largo).

### Dónde sí ayuda un script de Python

No hace falta para ejecutar el procedimiento de arriba (es 100% de
campo, con los mandos que ya existen), pero sí ayuda para analizarlo
mejor que a ojo sobre el log crudo:

- Parsear el log capturado (serial o CSV) a una serie de tiempo real,
  y graficarla (`matplotlib`) — ver la oscilación sostenida del paso 4
  y medir `Tu` con precisión (distancia entre picos) es mucho más
  confiable en un gráfico que contando segundos en la terminal.
- Automatizar el cálculo de las fórmulas de Ziegler-Nichols del paso 5
  a partir de `Ku`/`Tu` medidos, para no hacerlo a mano.
- Si en algún momento se quiere ir más allá de Ziegler-Nichols (ajustar
  fino con un modelo real del sistema), se puede ajustar una curva a
  la respuesta capturada (ej. `scipy.optimize.curve_fit` contra un
  modelo de segundo orden) y usar ese modelo para probar más
  combinaciones de ganancias en la computadora antes de volver a tocar
  el motor — es exactamente el rol que iba a cumplir MATLAB, solo que
  con datos reales del motor en vez de un modelo teórico.

**Ya existe una implementación de esto** en [`tools/pid_tuning/`](../tools/pid_tuning/)
(`pid_tuning.py`): parsea el log `PID_TEST` capturado, identifica el
modelo de planta FOPDT (`K`/`tau`/`L`) de un escalón real en lazo
cerrado, busca `Ku`/`Tu` con Ziegler-Nichols **sobre el modelo
simulado** (sin oscilar el motor real), y sugiere ganancias P/PI/PID —
ver el `README.md` de esa carpeta para el flujo completo con ejemplos
de comandos. Replica a mano la lógica exacta de `PID_CalcularSalidaUs()`
(`pid.c`) para que la simulación sea fiel al firmware real; si `pid.c`
cambia, esa réplica en Python hay que actualizarla también.

### Alternativa más segura: identificar el modelo sin llevar el motor a oscilar

El paso 4 de arriba (subir `Kp` hasta oscilación sostenida) funciona,
pero implica hacer oscilar el motor real a propósito. Se puede evitar
por completo si lo que se quiere es simular en Python antes de tocar
ganancias agresivas en el motor: alcanza con **una sola prueba segura**
con las ganancias de fábrica (`Kp=1, Ki=0, Kd=0` — ya sabemos que esto
no oscila, solo deja error de estado estable).

1. Motor estable, mandar un escalón moderado de `SET_RPM` (ej.
   +200-300 RPM) y capturar el log `PID_TEST` (ver más abajo) hasta que
   se estabilice.
2. De la curva capturada, medir:
   - `ΔSET_RPM`: tamaño del escalón mandado.
   - `ΔRPM_ss`: cuánto subió la RPM en estado estable (menor que
     `ΔSET_RPM`, por el error de P puro).
   - `L`: retraso (s) entre el escalón y que la RPM empiece a moverse
     (incluye el slew-rate del servo y el retraso mecánico del motor).
   - `τ_cl`: tiempo desde que empieza a moverse hasta llegar al 63.2%
     del cambio total.
3. Como el lazo es proporcional puro con `Kp` conocido, se puede
   despejar la planta en lazo abierto (realimentación unitaria sobre
   una planta de primer orden):
   ```
   G_cl = ΔRPM_ss / ΔSET_RPM

   K = G_cl / (Kp * (1 - G_cl))     -- ganancia de la planta (RPM/µs)
   τ = τ_cl / (1 - G_cl)            -- constante de tiempo de la planta
   L ≈ L_cl                          -- el retraso no cambia con la realimentación
   ```
4. Con `K`/`τ`/`L` ya hay un modelo FOPDT (first-order-plus-dead-time)
   simulable en Python. Replicar ahí **la lógica exacta de `pid.c`**
   (mismo clamp `[0, rango]`, mismo anti-windup, mismo reset de
   integral con `Ki=0`) sobre ese modelo, y recién ahí sí buscar `Ku`/
   `Tu` subiendo `Kp` **en la simulación**, sin riesgo real. Validar la
   combinación ganadora en el motor real repitiendo el escalón del
   paso 1.
5. Este modelo es una aproximación lineal válida cerca del punto de
   operación donde se hizo la prueba — repetir con 1-2 escalones de
   tamaño distinto para confirmar que `K`/`τ`/`L` salen parecidos antes
   de confiar en él.

### Log `PID_TEST` para capturar los escalones (agregado en `main.c`)

Línea de log dedicada, en formato CSV, para no tener que parsear el
resto del texto intercalado en el monitor serie (RAK3172, GPS, eco del
mando manual, etc.):

```
PID_TEST,<ms_desde_arranque>,<SET_RPM>,<RPM_filtrada>,<pulso_servo_us>
```

- Se imprime cada `200ms` (5Hz) — mucho más seguido que el log general
  de 1s, para tener suficientes puntos por constante de tiempo al
  ajustar la curva.
- **Solo se activa en `CONTROL_HABILITADO=3`** (modo sintonización PID,
  ver sección 4.4) — la misma condición que desbloquea
  `PID_KP/KI/KD`, una sola fuente de verdad para "¿estamos en una
  sesión de sintonización?". Fuera de ese modo el monitor se ve
  exactamente como antes de que existiera este log (solo la línea de
  1s). Se apaga solo en cuanto se sale del modo `3` (downlink
  `CONTROL_HABILITADO=0`).
- Usa `HAL_GetTick()` (ms desde el arranque), **no** la hora del RTC —
  el RTC se resincroniza cada ~30s por GPS/LoRaWAN (sección 2.2) y esas
  correcciones contaminarían el tiempo relativo de un escalón en
  curso; `HAL_GetTick()` es monótono y no depende de si el reloj ya
  sincronizó.
- Incluye el **pulso real aplicado al servo** (`Servo_GetPulsoActualUs()`),
  no solo el setpoint — importante porque `SERVO_VELOCIDAD_MAX_US_S`
  (sección 2.4) limita qué tan rápido puede moverse el pulso real, así
  que el valor efectivamente aplicado puede ir por detrás de lo que el
  PID "pidió" ese ciclo.
- Para capturarlo: log del terminal (PuTTY/Tera Term) o un script
  leyendo el puerto COM, filtrando las líneas que empiecen con
  `PID_TEST,` antes de armar el CSV para Python.