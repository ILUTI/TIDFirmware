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
| CALIBRACION | Sí, siempre por categoría; candado propio adentro del `case` si hace falta uno más fino | `SET_RATIO`, `ALPHA`, `PID_KP`, `PID_KI`, `PID_KD`, `CONTROL_HABILITADO` (este último rechaza `1`/`2` con el motor operando desde adentro del `case`, no por la categoría — ver ID 13 más abajo) |
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
cuando el motor opera, `CONTROL_HABILITADO=0` o `=7` (no se está
calibrando el servo -- `=7` es la sesión de sintonización del PID en
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
motor. `MODO=0`/Ralentí (el motor sube solo de 0Hz a su ralentí
natural, sección 4.3) es deliberadamente **sin control**: el setpoint
que produce ese modo es siempre `0`, por debajo de `RPM_MIN`, así que
el PID no corre y el servo se queda en `SERVO_PULSO_MIN` — nunca se
recorta ningún setpoint hacia arriba hasta `RPM_MIN` como si fuera
válido. El lazo solo se activa cuando el setpoint elegido según `MODO`
(sección 4.3: `SET_RPM` en `MODO=1`/Local, fórmula presión→RPM en
`MODO=2`/Remoto) supera `RPM_MIN`; una vez activo, se recorta hacia
abajo contra `RPM_MAX` si se pide algo más alto.

El setpoint sale de `CalibFlash_GetSetRpm()` (`MODO=1`) o de la fórmula
lineal presión→RPM (`MODO=2`) según corresponda -- ver la máquina
`MODO` completa en sección 4.3. Ganancias (`PID_KP/KI/KD`) siguen en
sus defaults (`Kp=1.0, Ki=0,
Kd=0`) a la espera de sintonización con el motor real (Ziegler-Nichols
en lazo cerrado, ver sección 9; solo se pueden tocar en
`CONTROL_HABILITADO=7`) — suficientes para validar que el lazo mueve
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
| 6 | `PID_KP` | x100 | 2B int16 | Calibración | Con signo. Solo se acepta con `CONTROL_HABILITADO=7` (modo sintonización PID) — `APPLY_ERROR` en cualquier otro modo, incluida la operación normal (`=0`). Se permite con el motor operando (necesario para sintonizar en lazo cerrado, ver sección 9) |
| 7 | `PID_KI` | x1000 | 2B int16 | Calibración | Con signo. Igual que `PID_KP` |
| 8 | `PID_KD` | x1000 | 2B int16 | Calibración | Con signo. Igual que `PID_KP` |
| 9 | `SERVO_PULSO_MIN` | directo (µs) | 2B uint16 | Configuración | Límite mecánico. Además requiere `CONTROL_HABILITADO=1` o `=2` (modo calibración del servo, NO `=7`) — si no, `APPLY_ERROR` aunque el motor esté apagado |
| 10 | `SERVO_PULSO_MAX` | directo (µs) | 2B uint16 | Configuración | Igual que `SERVO_PULSO_MIN` |
| 11 | `TIMEOUT_SIN_COMANDO_S` | directo (s) | 2B uint16 | Configuración | 60-3600s |
| 12 | `TASA_MAX_CAMBIO_RPM_S` | x10 | 2B uint16 | Configuración | Rampa normal |
| 13 | `CONTROL_HABILITADO` | 0/1/2/3/4/5/6/7 | 1B | Calibración* | Enable/disable lazo de control + **modos de calibración**. Tres grupos, por seguridad: **apagado** (`1`/`2`, calibración de servo — mueven el servo directo, sin ninguna realimentación de RPM, así que exigen el motor detenido para entrar, con rechazo explícito `REJECTED_ENGINE_RUNNING` si no), **encendido** (`0`/`3`/`5`/`7` — no cambian cómo se maneja el servo respecto a la operación normal, así que no exigen detener el motor para entrar; de hecho `3`/`5`/`7` solo tienen sentido con el motor ya operando) y `4`/`6` (calibración de `SERVO_PULSO_MIN` — mueven el servo directo como `1`/`2`, pero SÍ miran la RPM en cada paso, así que tampoco exigen el motor operando para entrar, simplemente esperan). Por eso este parámetro ya NO cae en la categoría Configuración genérica (que bloquearía *cualquier* valor con el motor andando) — tiene su propio candado adentro del `case`, igual que `PID_KP/KI/KD`. Valores: `0` desactivado/operación normal (que además ES el "ralentí" cuando `SET_RPM` no supera `RPM_MIN` — no hay un modo ralentí aparte, sería redundante), `1` barrido automático continuo entre `SERVO_PULSO_MIN/MAX`, `2` manual -- el servo se mantiene quieto en su posición, y cada downlink de `SERVO_PULSO_MIN` o `SERVO_PULSO_MAX` lo mueve directo a ese valor, `3` **calibración de ralentí** -- ver sección 10, `4` **calibración de `SERVO_PULSO_MIN`** (umbral de aceleración) -- ver sección 11, `5` **calibración de `ALPHA`** (mide el ruido real de RPM y calcula el filtro) -- ver sección 12, `6` **mapeo de curva de ganancia** (barre el pulso en lazo abierto desde `SERVO_PULSO_MIN` hasta una RPM techo fija, logueando pulso vs RPM real en cada paso -- por ahora solo mide/loguea, no aplica ninguna corrección todavía) -- ver sección 12, `7` **sintonización de PID** -- el servo lo maneja el PID normal exactamente igual que en `0`, pero es el único modo en que `PID_KP/KI/KD` se aceptan (y activa el log `PID_TEST`, ver sección 9); `3`, `4`, `5` y `6` solo se pueden pedir viniendo de modo `0` (rechazado con `OUT_OF_RANGE` si se piden desde `1`/`2`/`7`, o entre sí). En `1`/`2`/`3`/`5`/`7` el servo nunca se comporta distinto a "sin control activo" salvo que `pidActivoAhora` esté armado (solo en `0`/`7` con un `SET_RPM` real) — en `1`/`2`/`3`/`5` siempre cae en la posición segura `SERVO_PULSO_MIN`; `4` y `6` son las excepciones que sí mueven el servo por encima de `SERVO_PULSO_MIN` de forma automática, en pasos chicos y acotados (ver sección 11 y 12). En `1` o `2` se habilita cambiar `SERVO_PULSO_MIN/MAX`. El firmware fuerza `CONTROL_HABILITADO=0` localmente en el instante que el motor arranca **solo si estaba en `1` o `2`** (nunca en `3`/`4`/`5`/`6`/`7`, que necesitan que el motor siga operando), y también fuerza a `0` en cada arranque del firmware (`CalibFlash_Init()`), sin importar el valor que haya quedado guardado en flash de una sesión anterior — nunca reanuda ningún modo de calibración solo; los modos `3`, `4`, `5` y `6` además se auto-completan solos (vuelven a `0` sin downlink al terminar, ver secciones 10, 11 y 12). **Medida de seguridad adicional**: al entrar a `1`-`7` (downlink aceptado con valor != 0), `SET_RPM` se limpia a `0` (su estado "sin comandar") — evita que un `SET_RPM` que haya quedado de una operación anterior active el PID solo al entrar a `7`, sin que el operador lo haya vuelto a pedir explícitamente para esa sesión. Mismo criterio en el camino de regreso: cuando el apagado de seguridad de `main.c` fuerza `CONTROL_HABILITADO` de `1`/`2` de vuelta a `0` (motor arrancando durante calibración del servo), también limpia `SET_RPM` a `0` — por si se había mandado un `SET_RPM` mientras se calibraba (categoría Proceso, siempre se acepta, sin importar el modo), que no quede activando el PID solo al volver a operación normal |
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
| 25 | `SET_RATIO_AUTO` | x10 (entrada) | 2B uint16 | Calibración | El valor recibido es el RPM que marca un tacómetro de referencia externo en ese instante, NO el ratio — el firmware calcula `SET_RATIO = frecuenciaHz_actual × 60 / RPM_recibido` y lo aplica (misma validación de rango que `SET_RATIO`, mismo campo persistido). El valor vigente devuelto en el ACK es el ratio resultante, codificado x100 como `SET_RATIO` (no eco del RPM recibido). `OUT_OF_RANGE` si se manda `0` o si el motor no tiene lectura de tacómetro válida (`frecuenciaHz == 0`) — no hay nada que calcular sin motor girando. Ver sección 12 |
| 26 | `MECANISMO_MANIVELA_CM` | x100 | 2B uint16 | Calibración | Radio de la manivela (brazo del servo) del mecanismo biela-manivela actual, en cm. Usado por la corrección de linealización geométrica del PID — ver sección 12. Default `5.5` (medido en campo 2026-09-03) |
| 27 | `MECANISMO_VARILLA_CM` | x100 | 2B uint16 | Calibración | Largo de la varilla rígida (biela) del mecanismo, en cm. Default `30.0` (medido en campo 2026-09-03) |
| 28 | `MECANISMO_OFFSET_GRADOS` | x100 | 2B uint16 | Calibración | Ángulo de la manivela en `SERVO_PULSO_MIN`, medido desde el punto muerto (manivela alineada con la varilla). `0` = montada justo en el punto muerto (el caso medido en campo). Rango `0-179` |
| 29 | `MECANISMO_CORRECCION_ACTIVA` | directo (0/1) | 2B uint16 | Calibración | Prende/apaga la corrección de linealización sin perder los valores de manivela/varilla cargados. Default `0` (apagada) — hay que cargar la geometría y encenderla a propósito. Interina: pensada para descartarse si se reemplaza el mecanismo (piñón-cremallera, o montaje directo del servo sobre el eje de la palanca de la bomba) |
| 30 | `PRESION_GANANCIA_RPM` | x100 | 2B int16 | Calibración | Con signo. Fórmula lineal presión→RPM de `MODO=2` (sección 4.3) — solo se acepta con `CONTROL_HABILITADO=7`, igual que `PID_KP/KI/KD`. Default `0` (sin calibrar, `MODO=2` no hace nada hasta que se calibre) |
| 31 | `PRESION_OFFSET_RPM` | x10 | 2B uint16 | Calibración | RPM base de la fórmula de `MODO=2`. Igual gate que `PRESION_GANANCIA_RPM`. Default `0` |

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

### 4.3 Modos de operación del motor (`MODO` 0/1/2) — **implementado 2026-09-07 (interino en `MODO=1`)**

`MODO` (ID 16) ahora sí se lee en el ciclo de control real (`main.c`) --
antes se recibía/persistía/ACKeaba pero nada lo consultaba, así que
**todo nodo se comportaba siempre como si estuviera en `MODO=1`, sin
importar su valor real** (esto se descubrió revisando el código al
evaluar qué pasaría si otra Lambda empezara a mandar `PRESION`
automáticamente desde los aspersores -- ver hallazgo 2026-09-07 más
abajo).

```
MODO=0 (Ralentí): sin control activo, sin importar SET_RPM/PRESION --
                  el servo cae a SERVO_PULSO_MIN (ralentí natural).
MODO=1 (Local):   setpoint = SET_RPM (downlink directo).
MODO=2 (Remoto):  setpoint = PRESION_OFFSET_RPM + PRESION_GANANCIA_RPM
                  * PRESION -- fórmula lineal, ver abajo.
```

⚠️ **`MODO=1` (Local) es una simplificación interina, no la visión
original de esta sección** (que era "gobernado por presión de la
motobomba, sensor propio 4-20mA") -- esa lectura de presión propia
(`presion.c`, sección 5) todavía no existe, así que hoy `MODO=1` usa
`SET_RPM` directo, que es exactamente el comportamiento que **ya tenía
de facto** todo nodo antes de este cambio. El día que exista
`presion.c`, `MODO=1` debería pasar a derivarse de esa lectura en vez
de `SET_RPM` -- pendiente, ver sección 8.

⚠️ **MIGRACIÓN para nodos ya desplegados**: como `MODO` nunca se leía,
cualquier nodo que ya esté operando con `SET_RPM` (sin haber tocado
`MODO` jamás -- lo más probable en todos los nodos activos hoy) va a
arrancar en `MODO=0` (Ralentí, sigue siendo el default) al actualizar
este firmware, y **va a dejar de responder a `SET_RPM`** hasta que se
mande `MODO 1` explícito. Mandar ese downlink (o por serial,
`MODO 1`) a cada nodo activo antes o justo después de actualizar.

**`MODO=2` (Remoto) — fórmula presión→RPM calibrable en campo, igual
que el PID.** No existe (todavía) una relación presión→RPM conocida de
antemano -- depende del sistema hidráulico real (motor + aspersores) --
así que en vez de inventar una fórmula fija, se agregaron dos
parámetros lineales que se calibran **exactamente como `PID_KP/KI/KD`**
(solo se aceptan con `CONTROL_HABILITADO=7`, con el motor operando,
observando la RPM real contra la presión real):

| ID | Nombre | Escala | Notas |
|---|---|---|---|
| 30 | `PRESION_GANANCIA_RPM` | x100, con signo | RPM por unidad de presión |
| 31 | `PRESION_OFFSET_RPM` | x10 | RPM base (presión=0) |

`setpoint = PRESION_OFFSET_RPM + PRESION_GANANCIA_RPM * PRESION`,
recortado igual que el resto (`RPM_MIN`/`RPM_MAX`). Default de ambos:
`0` -- con ganancia `0` el setpoint de `MODO=2` sin calibrar es `0`
(sin comandar), o sea **`MODO=2` no hace nada hasta que se calibre
deliberadamente**, nunca acelera el motor con valores sin sentido.

**Watchdog de `TIMEOUT_SIN_COMANDO_S` (ID 11) — implementado, solo
aplica en `MODO=2`.** Si no llega una `PRESION` válida en más de ese
tiempo (cuenta desde el arranque si nunca llegó ninguna), el firmware
fuerza el setpoint a "sin comandar" -- el motor cae a ralentí natural
(misma rama física que `MODO=0`) en vez de sostener indefinidamente el
último valor de presión, ya viejo (ej. si la Lambda que reenvía
`PRESION` se cae, o los aspersores se quedan sin batería). Se loguea
por flanco (`MODO_REMOTO: sin PRESION valida...`/`...reanudando control
por presion`), no en cada vuelta del loop.

- Transición Modo 0→1: pendiente de confirmar con cliente (¿por
  tiempo fijo, o estabilización de RPM?) -- hoy es siempre manual, por
  downlink de `MODO`.
- Transición Modo 1→2: pendiente de definir si es automática (ej.
  cuando `PRESION >= PRESION_OBJETIVO`) o por selector manual -- hoy es
  siempre manual, por downlink de `MODO`.
- Dentro de Modo 2 (remoto): sub-fase de llenado vs. régimen, ya
  resuelta a nivel de parámetros (`TASA_MAX_CAMBIO_RPM_LLENADO_S` vs.
  `TASA_MAX_CAMBIO_RPM_S`), falta la lógica de conmutación en tiempo
  de ejecución.

### 4.4 Modo calibración del servo (`CONTROL_HABILITADO`, implementada)

Independiente de 4.1-4.3 — no es un modo de operación del motor, es el
mecanismo para ajustar en banco los topes mecánicos del acelerador
(`SERVO_PULSO_MIN/MAX`), sintonizar el PID, y auto-calibrar el
ralentí y el propio `SERVO_PULSO_MIN`, sin reflashear. Tres grupos,
por seguridad (ver también la tabla de categorías en la sección 2.3):

```
APAGADO   (CONTROL_HABILITADO=1 o =2): mueven el servo directo, sin
          ninguna realimentación de RPM -- exigen el motor detenido
          para ENTRAR (rechazo REJECTED_ENGINE_RUNNING si no), y se
          fuerza la salida a 0 si el motor arranca mientras se está en
          cualquiera de los dos.
ENCENDIDO (CONTROL_HABILITADO=0, =3 o =7): no cambian cómo se maneja
          el servo respecto a la operación normal -- no exigen
          detener el motor para entrar; de hecho 3/7 solo tienen
          sentido con el motor ya operando.
CALIBRACIÓN-SERVO-MIN (CONTROL_HABILITADO=4): también mueve el servo
          directo, como 1/2 -- pero a diferencia de esos dos, SÍ mira
          la RPM en cada paso (ver sección 11), así que tampoco exige
          el motor operando para entrar, igual que 4.
```

`CONTROL_HABILITADO=0` en realidad cubre dos sub-casos, resueltos en
`main.c` cada vuelta del loop según el motor **y** si se comandó
control (`SET_RPM > RPM_MIN`, ver 2.4):

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
SINTONIZACIÓN-PID   (CONTROL_HABILITADO=7): el servo lo maneja el PID
                                     normal, EXACTAMENTE igual que en
                                     SEGURO/PID ACTIVO -- lo único que
                                     cambia es que PID_KP/KI/KD se
                                     desbloquean y se activa el log
                                     PID_TEST (ver sección 9). NO exige
                                     motor detenido ni para entrar ni
                                     para permanecer -- la
                                     sintonización en lazo cerrado solo
                                     tiene sentido con el motor ya
                                     corriendo (README sección 9).
CALIBRACIÓN-RALENTÍ (CONTROL_HABILITADO=3): servo fijo en
                                     SERVO_PULSO_MIN mientras dura la
                                     ventana de medición (ver sección
                                     10). Tampoco exige motor detenido
                                     para entrar -- si se activa sin el
                                     motor operando, simplemente espera
                                     a que arranque antes de empezar a
                                     contar la ventana. Solo se puede
                                     pedir viniendo de SEGURO/PID
                                     ACTIVO (CONTROL_HABILITADO=0), no
                                     directo desde 1/2/7.
CALIBRACIÓN-SERVO-MIN (CONTROL_HABILITADO=4): barre el pulso hacia
                                     arriba en pasos chicos desde
                                     SERVO_PULSO_MIN, buscando dónde el
                                     motor empieza a acelerar de verdad
                                     (ver sección 11). Tampoco exige
                                     motor detenido para entrar -- si
                                     se activa sin el motor operando,
                                     espera a que arranque antes de
                                     medir la línea base y barrer.
                                     Solo se puede pedir viniendo de
                                     SEGURO/PID ACTIVO
                                     (CONTROL_HABILITADO=0), no directo
                                     desde 1/2/3/7.
SEGURO      (CONTROL_HABILITADO=0 o =7, motor detenido, O motor
             operando en su ralentí natural sin SET_RPM > RPM_MIN
             comandado):             servo fijo/regresando a
                                     SERVO_PULSO_MIN (sin
                                     aceleración). Estado por defecto.
PID ACTIVO  (CONTROL_HABILITADO=0 o =7, motor operando, Y SET_RPM >
             RPM_MIN comandado explícitamente): servo controlado por
                                     pid.c/h (ver 2.4), no por una
                                     posición fija.
```

- `* -> CALIBRACIÓN-BARRIDO/MANUAL`: downlink `CONTROL_HABILITADO=1`
  o `=2`, solo se acepta con el motor detenido — con el motor operando
  se rechaza con `REJECTED_ENGINE_RUNNING` (STATUS=5).
- `SEGURO/PID ACTIVO (0) -> SINTONIZACIÓN-PID (7), CALIBRACIÓN-RALENTÍ
  (3) o CALIBRACIÓN-SERVO-MIN (4)`: downlink directo, con el motor
  operando o detenido indistinto — `3`/`7` no mueven el servo de forma
  distinta a la operación normal, así que no hace falta detener el
  motor primero; `4` sí mueve el servo directo, pero como mira la RPM
  en cada paso tampoco exige detenerlo, solo espera si hace falta.
  `3` y `4` además solo se aceptan viniendo de `0` (rechazo
  `OUT_OF_RANGE` si se piden desde `1`, `2`, `7`, o uno desde el otro).
- `CALIBRACIÓN-BARRIDO ⇄ CALIBRACIÓN-MANUAL`: por downlink directo
  entre `1` y `2` (ambos requieren motor detenido para el cambio). Al
  entrar a `CALIBRACIÓN-MANUAL` el servo arranca quieto en la posición
  en la que estaba (no salta a `SERVO_PULSO_MIN` ni `MAX`) hasta el
  primer downlink de esos dos parámetros.
- `CALIBRACIÓN-BARRIDO/MANUAL -> SEGURO`: por downlink
  `CONTROL_HABILITADO=0`, **o** automáticamente en el firmware (sin
  downlink) si el motor arranca mientras se está en uno de esos dos
  modos — medida de seguridad, nunca se deja el servo bajo barrido o
  posición manual con el motor operando. Si además ya había un
  `SET_RPM > RPM_MIN` comandado desde antes, ese mismo arranque cae
  directo en `PID ACTIVO` en vez de `SEGURO`. **`SINTONIZACIÓN-PID`,
  `CALIBRACIÓN-RALENTÍ` y `CALIBRACIÓN-SERVO-MIN` no tienen esta salida
  automática** — el motor arrancando (o ya estando operando) ahí es
  justamente lo esperado, no una condición de falla.
- `CALIBRACIÓN-RALENTÍ -> SEGURO`: automático, sin downlink, en cuanto
  se completa la ventana de medición de 2 minutos (aplica el nuevo
  `RPM_MIN` y vuelve a `CONTROL_HABILITADO=0` solo, ver sección 10). Si
  el motor se detiene a mitad de la ventana, la medición en curso se
  descarta pero se permanece en `CALIBRACIÓN-RALENTÍ`, esperando a que
  el motor vuelva a arrancar, sin necesidad de reenviar el downlink.
- `CALIBRACIÓN-SERVO-MIN -> SEGURO`: automático, sin downlink, al
  detectar el umbral de aceleración (aplica el nuevo `SERVO_PULSO_MIN`
  y vuelve a `CONTROL_HABILITADO=0` solo) o al llegar al techo de
  seguridad sin detectar nada (ver sección 11 — en ese caso no se
  aplica ningún cambio, es un error, no un resultado). Igual que en
  `CALIBRACIÓN-RALENTÍ`, si el motor se detiene a mitad del barrido, se
  descarta el progreso pero se permanece en el modo esperando a que
  vuelva a arrancar.
- `SEGURO ⇄ PID ACTIVO` (dentro de `CONTROL_HABILITADO=0` **o** `=7`):
  automático según `Tacometro_EstaDetenido()` y si `SET_RPM` supera
  `RPM_MIN`, sin downlink de por medio para el motor (arrancar/detener
  basta) — el ralentí (Modo 0, sección 4.3) es deliberadamente sin
  control, nunca se recorta `SET_RPM` hacia arriba hasta `RPM_MIN` para
  forzar el lazo. Al entrar a `PID ACTIVO` se llama `PID_Init()` una
  sola vez (flanco de entrada), para no arrastrar estado de una
  activación anterior. El comportamiento del servo es idéntico en `0`
  y en `7` — la diferencia entre ambos está solo en qué parámetros se
  pueden tocar y si se ve el log `PID_TEST`, no en el control en sí. En
  `CALIBRACIÓN-RALENTÍ` (`3`) el servo siempre está en
  `SERVO_PULSO_MIN`, no existe un "PID ACTIVO" propio de este modo. En
  `CALIBRACIÓN-SERVO-MIN` (`4`) el servo va en pasos automáticos por
  encima de `SERVO_PULSO_MIN` mientras dura el barrido (ver sección
  11) — es el único modo, aparte de `1`/`2`, donde el servo se mueve
  fuera de esa posición sin ser por el PID.
- `SERVO_PULSO_MIN/MAX` solo se pueden cambiar **por downlink**
  estando en `CALIBRACIÓN-BARRIDO` o `CALIBRACIÓN-MANUAL` (NO en
  `SINTONIZACIÓN-PID`, `CALIBRACIÓN-RALENTÍ` ni `CALIBRACIÓN-SERVO-MIN`);
  fuera de esos dos modos se rechazan con `APPLY_ERROR` (STATUS=4)
  aunque el motor esté apagado. En `CALIBRACIÓN-BARRIDO` el cambio
  tiene efecto inmediato sobre el barrido en curso; en
  `CALIBRACIÓN-MANUAL` mueve el servo directo al valor recién
  configurado. `CALIBRACIÓN-SERVO-MIN` (`4`) es la única forma de que
  `SERVO_PULSO_MIN` cambie sin un downlink explícito de ese parámetro
  — lo hace `main.c` directo, al completar el barrido (ver sección 11).
- `PID_KP/PID_KI/PID_KD` solo se pueden cambiar estando en
  `SINTONIZACIÓN-PID` (`CONTROL_HABILITADO=7`); en cualquier otro modo
  (incluida la operación normal, `=0`, y `CALIBRACIÓN-RALENTÍ`, `=3`)
  se rechazan con `APPLY_ERROR`, motor operando o no — evita que las
  ganancias del PID cambien fuera de una sesión deliberada de
  sintonización.

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
      se pueden tocar en `CONTROL_HABILITADO=7`.
- [x] **AWS**: `send_downlink.py`'s `PARAMETER_TABLE["CONTROL_HABILITADO"]["max"]`
      subido de `2` a `3` para aceptar el nuevo modo de sintonización de
      PID (mismo ajuste que ya se había hecho cuando se agregó el valor
      `2`).
- [ ] **AWS, pendientes acumulados (ir agregando acá cada vez que el
      firmware agregue/cambie algo del lado de `send_downlink.py`, para
      no perder el hilo entre sesiones)**:
      - `PARAMETER_TABLE["CONTROL_HABILITADO"]["max"]` subir a `7`
        (quedó en `3` la última vez que se confirmó hecho — cubre de
        una vez los modos `3`, `4`, `5` y `6` agregados después, ver
        secciones 10/11/12).
      - `PARAMETER_TABLE` necesita una entrada nueva para
        `SET_RATIO_AUTO` (ID `25`, escala `x10`, sin la cual ese
        parámetro solo funciona por serial, no por downlink LoRa —
        ver sección 12).
      - `ALPHA_AUTO` NO necesita entrada propia — quedó como
        `CONTROL_HABILITADO=5`, cubierto por el bump de arriba.
      - `PARAMETER_TABLE` necesita 4 entradas nuevas para
        `MECANISMO_MANIVELA_CM`/`MECANISMO_VARILLA_CM` (IDs `26`/`27`,
        escala x100), `MECANISMO_OFFSET_GRADOS` (ID `28`, escala x100),
        y `MECANISMO_CORRECCION_ACTIVA` (ID `29`, directo 0/1) — ver
        sección 12.
      - **Nuevo 2026-09-07**: `PARAMETER_TABLE` necesita 2 entradas
        nuevas para `PRESION_GANANCIA_RPM` (ID `30`, escala x100, con
        signo) y `PRESION_OFFSET_RPM` (ID `31`, escala x10) — sin ellas
        `MODO=2` no se puede calibrar por downlink LoRa, solo por
        serial. Ver sección 4.3.
      - **Aviso importante, no es un cambio de tabla**: `MODO` (ID 16)
        ahora sí tiene efecto real en el firmware (antes era un no-op
        aceptado y ACKeado sin hacer nada) — si alguna Lambda/servicio
        ya manda `MODO=2` + `PRESION` esperando que el motor reaccione,
        confirmar primero que `PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM`
        ya se calibraron en ese nodo (ver sección 4.3) y que el nodo no
        se quedó en `MODO=0` tras la actualización (ver aviso de
        migración en esa misma sección) — si no, `MODO=2` no va a mover
        el motor aunque el downlink de `PRESION` llegue y se ACKee OK.
- [ ] Posible parámetro 25, `INTEGRAL_MAX` (anti-windup configurable
      del PID) — hoy el anti-windup es fijo en `pid.c` (integración
      condicional, sin límite configurable por downlink).
- [x] **Puente aspersor → motor (`MODO`/`PRESION`/`TIMEOUT_SIN_COMANDO_S`)
      — implementado 2026-09-07** (encontrado al revisar qué pasaría si
      otra Lambda empezara a mandar `PRESION` como downlink automático
      desde los aspersores; ver sección 4.3 para el detalle completo):
      `MODO` ahora rama el ciclo de control real (antes era un no-op);
      `MODO=2` deriva el setpoint de `PRESION` vía una fórmula lineal
      calibrable en campo (`PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM`,
      IDs 30/31, mismo gate `CONTROL_HABILITADO=7` que `PID_KP/KI/KD`);
      el enum `CalibFlash_Modo_t` se renombró a
      `CALIB_MODO_RALENTI/LOCAL/REMOTO` para coincidir con el README; y
      el watchdog de `TIMEOUT_SIN_COMANDO_S` fuerza ralentí si `MODO=2`
      deja de recibir `PRESION` fresca. **Pendientes reales que quedan**
      (no de diseño, de calibración/AWS): `PRESION_GANANCIA_RPM`/
      `PRESION_OFFSET_RPM` siguen en `0` (sin calibrar) hasta que se
      corra una sesión real con el motor y el sistema hidráulico
      completo; `PARAMETER_TABLE` de `send_downlink.py` necesita las
      entradas para IDs 30/31 (ver checklist de AWS arriba); y el día
      que exista `presion.c` (sección 5), decidir si `MODO=1` debería
      pasar a usar esa lectura propia en vez de `SET_RPM` (ver aviso en
      sección 4.3).
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
- [ ] **Idea planteada 2026-09-03, no diseñada aún**: un mecanismo para
      que el backend pueda *pedir* (no solo recibir el ACK del momento
      en que se aplicó) los valores vigentes de los parámetros de
      calibración de un nodo — útil para auditar/resincronizar la base
      de datos si se sospecha que un ACK se perdió. Hoy el único camino
      es el Application ACK de cada downlink individual (FPort 3, en el
      momento en que se manda ese parámetro) — el uplink estándar
      (FPort 1, sección 6) NO incluye `SET_RATIO`/`ALPHA`/etc., solo
      telemetría operativa. No confundir con `FORZAR_REPORTE`
      (dispara ese mismo uplink estándar, no un volcado de
      configuración).
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
apaga solo de inmediato (medida de seguridad, ver 4.4). `=7`
(sintonización PID) no tiene esa restricción — de hecho solo tiene
sentido con el motor ya operando — pero tampoco da un lazo abierto: el
servo lo sigue manejando el PID igual que en `=0`. Esto es
intencional y no se debe evadir: significa que **no se puede hacer una
prueba de "curva de reacción" en lazo abierto** (mandar un escalón de
pulso fijo y medir cómo responde la RPM) con el motor corriendo —
cualquier prueba en el motor real va a ser necesariamente **en lazo
cerrado**, con el PID ya corriendo. Por suerte, hay un método de
sintonización estándar pensado exactamente para esto: **Ziegler-Nichols
en lazo cerrado (ganancia última)**.

### Procedimiento (Ziegler-Nichols en lazo cerrado)

1. Mandar `CONTROL_HABILITADO=7` (modo sintonización PID, ver sección
   4.4) — es el único modo en el que `PID_KP/KI/KD` se aceptan; en
   cualquier otro modo (incluida la operación normal, `=0`) se
   rechazan con `APPLY_ERROR`, por medida de seguridad (que no se
   puedan tocar las ganancias por accidente fuera de una sesión
   deliberada). A diferencia de `=1`/`=2`, este modo no exige el motor
   detenido para entrar ni se apaga solo cuando arranca — se puede
   activar con el motor ya corriendo, o antes de arrancarlo, indistinto.
   Con el motor ya estabilizado en ralentí caliente, seguir al paso 2.
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

### ⚠️ Confirmado en campo (2026-09-01): Ziegler-Nichols oscila cerca del setpoint en este motor — usar SIMC en su lugar

Ziegler-Nichols está diseñado para dar la respuesta **más rápida
posible**, sin ponderar mucho el retraso de la planta — y en un motor
con tiempo muerto significativo respecto a su constante de tiempo
(como resultó ser este), eso se traduce en ganancias que **oscilan
justo al acercarse al setpoint**, sin importar qué tan chico se ponga
`Ki` (se confirmó con `Ki=0.05`, `0.02` y hasta `0.01` — todos terminan
oscilando, solo cambia cuánto tardan). No es un problema de encontrar
el valor correcto dentro de este método — es una limitación del método
en sí para este tipo de planta.

**Alternativa: [SIMC](https://folk.ntnu.no/skoge/publications/2003/tuningPID/) (Skogestad, 2003)**
— mismas entradas (`K`, `tau`, `L` del modelo identificado con
`identify`), pero deja elegir a propósito qué tan conservador ser en
vez de perseguir siempre la respuesta más rápida:
```
Kc = (1/K) × τ / (τc + L)          Ti = min(τ, 4×(τc + L))     Ki = Kc/Ti
```
`τc` (constante de tiempo deseada del lazo cerrado) se elige a mano —
más grande = más lento pero más robusto. Implementado en
`tools/pid_tuning/pid_tuning.py simc`. Confirmado en simulación:
con una planta de `K=0.5, tau=3.0s, L=1.5s`, las ganancias PI de
Ziegler-Nichols (`Kp=5.69, Ki=1.22`) oscilan sin asentar ni en 54s
simulados, mientras que SIMC con `τc=2L` (`Kp=1.33, Ki=0.44`) converge
suave y sin sobre-impulso en ~40s — mismo modelo de planta, mismo
punto de partida, solo cambia la fórmula de tuneo.

**Procedimiento revisado**: en vez del paso 4 de arriba (subir `Kp` en
el motor real hasta oscilación), alcanza con:
1. Un solo escalón de `SET_RPM` con `Kp=1, Ki=0, Kd=0` (ya lo tenés).
2. `python pid_tuning.py identify --log captura.log --kp 1.0` para
   sacar `K`/`tau`/`L`.
3. `python pid_tuning.py simc --K <K> --tau <tau> --L <L>` — da tres
   filas (agresivo/medio/conservador). Empezar por "Medio".
4. `python pid_tuning.py simulate --K ... --kp ... --ki ...` para
   confirmar que no oscila en la simulación antes de cargarlo.
5. Cargar esas ganancias y validar en el motor real — recién ahí, con
   mucho menos riesgo de toparse con la oscilación que costó tanto
   tiempo de motor real diagnosticar.

### ✅ Ganancias confirmadas en el motor real (2026-09-02): `Kp=0.3, Ki=0.02, Kd=0`

Validadas sobre el rango real de operación del motor (ralentí hasta
1200-1500 RPM, el máximo habitual de trabajo). Sostenidas más de 100s
en `SET_RPM=1200` sin oscilar y sin error de estado estable notorio
(RPM se mantiene en banda ~1195-1205, solo ruido normal de medición).

**Por qué no se llegó a esto por el camino de `identify`/`simc` de
arriba**: el motor resultó tener una ganancia de planta fuertemente no
lineal según el punto de operación — un escalón chico cerca de ralentí
(870→885 RPM) identificó `K=0.014`, mientras que un escalón hasta 1500
RPM identificó `K=0.224`, dieciséis veces más grande. Un modelo FOPDT
ajustado en un punto de operación no se puede extrapolar de forma
confiable a otro punto lejano — de hecho, al simular las ganancias que
SIMC calculó a partir del modelo de `K=0.224` contra un objetivo de
1500 RPM, el servo se saturaba al máximo sin llegar nunca al setpoint,
señal de que el modelo no era válido tan lejos de donde se identificó.

También se confirmó que el problema de oscilación cerca del setpoint
(sección anterior) no era solo cuestión de `Ki`: `Kp=0.6` y `Kp=1.0`,
que parecían limpios con P puro cerca de ralentí, resultaron oscilar
sosteniblemente (sin amortiguarse) al mandar un escalón directo a
`SET_RPM=1300` — el `Kp` en sí ya era demasiado agresivo para la zona
de mayor ganancia de la planta, lejos de ralentí.

**Lo que sí funcionó**: bajar a `Kp=0.3` (confirmado estable con P puro
en 1300 y 1500) y agregar un `Ki=0.02` chico de forma incremental,
igual que el resto de esta sección — validado directamente en el
motor real en vez de confiar en la extrapolación del modelo lineal.

**Sostenido largo en 1500 también confirmado limpio y asentado del
todo (2026-09-02)**: se repitió el escalón ralentí→1500, esta vez
sostenido ~170s completos — la RPM termina asentada en una banda
angustada justo alrededor de 1500 (~1485-1509, solo el ruido normal de
medición, ±10-15 RPM), sin un solo ciclo de oscilación en toda la
subida ni en el asentamiento final. `Kp=0.3, Ki=0.02, Kd=0` queda
**confirmado por completo** en todo el rango real de operación
(ralentí hasta 1500), en ambos setpoints probados (1200 y 1500).

**Bajada de setpoint confirmada limpia (2026-09-02)**: escalón directo
1400→1200 (mismas ganancias) desciende suave y monótono, sin ninguna
oscilación sostenida, convergiendo en ~85s — mismo comportamiento
simétrico que la subida.

**`Kd` probado y descartado (2026-09-02): la respuesta final es
`Kd=0`.** Se probó `Kd=0.005` sobre el `Kp=0.3, Ki=0.02` ya confirmado
(escalón ralentí→1200): no trajo ningún problema (`pulso_us` se
mantuvo igual de suave, sin el temblor por ruido que se temía) pero
tampoco ninguna mejora medible (tiempo de asentamiento prácticamente
igual, ~90-100s). Como un `Kd` que no demuestra beneficio es solo una
variable más para mantener, se optó por dejar `Kd=0`.

**`Ki=0.03` probado y descartado (2026-09-02) — confirma que `Ki=0.02`
queda como definitivo.** Se probó `Ki=0.03` buscando un asentamiento
más rápido. La primera prueba salió confundida (`Kd` seguía en
`0.005`, no en `0`) y mostró ráfagas cortas de saltos grandes entre
muestras (±30-90 RPM) durante la subida — parecía ruido amplificado
por la derivada. Se repitió con `Kd=0` confirmado explícitamente, y
**la misma ráfaga volvió a aparecer** — prueba que es `Ki=0.03` por sí
solo el causante, no `Kd`. La ráfaga es transitoria (el motor se
recupera solo y asienta limpio después, ~1195-1200 con ruido normal),
pero nunca apareció en ninguna prueba con `Ki=0.02` — señal de que
`0.03` ya está más cerca del límite de estabilidad de este motor. El
tiempo de asentamiento tampoco mejoró de forma medible (~90-130s en
ambos casos). Conclusión: sin beneficio de velocidad y con un riesgo
nuevo, se vuelve a `Ki=0.02`. Y dado que la ráfaga no es el patrón
clásico de sobre-impulso que `Kd` corrige, agregar `Kd` acá tampoco
tenía sentido — se descartó por la misma razón que antes.

**GANANCIAS FINALES DEL PID RIO-DSL: `Kp=0.3, Ki=0.02, Kd=0`** —
confirmadas limpias (sin oscilación, sin windup, sin ráfagas
transitorias, error de estado estable prácticamente nulo) en todo el
rango real de operación, en ambos sentidos (ralentí↔1200, ralentí↔1500,
bajada 1500→1200), y confirmadas como el mejor compromiso después de
probar y descartar tanto `Ki=0.03` como `Kd=0.005`.

### Re-sintonización para el montaje directo del servo (branch `servo-directo`, 2026-09-08)

Con el servo montado directo sobre el eje de la palanca del acelerador
(ver sección 12 y la memoria de rediseño mecánico) la planta cambió
por completo — no se puede reusar `Kp=0.3, Ki=0.02` de arriba, tunadas
contra la biela-manivela. Re-sintonización completa, con
`SERVO_PULSO_MIN=1124` / `SERVO_PULSO_MAX=1650` ya calibrados.

**`identify` dio `L=0.000s`** — tiempo muerto no detectable, confirma
que el montaje directo eliminó el falso tiempo muerto que tenía el
punto muerto de la biela-manivela (`K=0.246`, `tau=0.430s`, a partir de
un escalón cerrado con `Kp=0.3`). Pero ese `K` salió de una señal
débil y ruidosa (el escalón nunca llegó cerca del setpoint con
`Kp=0.3` solo), así que no es confiable por sí solo — el `K` real se
tomó del mapeo de curva de ganancia en lazo abierto
(`CONTROL_HABILITADO=6`, sección 12), que varía entre **1.9 y 6.3
RPM/µs** según la zona del rango (~3.4x de variación, menos que las
~8.8x de la biela-manivela, pero todavía significativo — sigue siendo
una planta con ganancia no uniforme, ver "gain-scheduling" en
[[project-rio-dsl-mechanism-redesign]] y sección 8).

**Ziegler-Nichols tampoco aplicó esta vez, pero por una razón
distinta**: con `L≈0`, `pid_tuning.py tune` no encuentra ninguna
ganancia última (`Ku`) ni con `Kp=1000` en la simulación — un sistema
de primer orden puro (sin tiempo muerto) bajo control proporcional es
incondicionalmente estable en la teoría, así que el método de
ganancia última de Z-N literalmente no tiene nada que buscar. La
oscilación real que sí se observó en el motor viene de retardos más
chicos que el modelo FOPDT no captura (muestreo de 200ms, filtro
`ALPHA`, límite de velocidad del servo).

**Bug encontrado en `pid_tuning.py simc`**: con `L=0` exacto, la
opción "Agresivo" (`tau_c=L`) divide entre cero
(`ZeroDivisionError` en `simc_pi_gains`). Se recalculó a mano con un
piso conservador `L=0.15s` (del orden del período de muestreo) en vez
de confiar en el `L=0` literal — pendiente de agregar ese piso mínimo
directamente al script.

**Validación en el motor real, iterativa**:
1. Con `K=3.0` (zona de ganancia media del mapeo) SIMC dio
   `Kp=0.25, Ki=0.57` — funcionó limpio en `SET_RPM=1050`, pero
   **osciló sostenido** en `SET_RPM=1300` (zona de mayor ganancia del
   mapeo) — confirma en campo el problema de ganancia no uniforme.
2. Recalculado con el `K=6.3` peor caso del mapeo → `Kp=0.12, Ki=0.27`
   — mucho mejor en 1300, pero quedó una oscilación lenta residual
   (~5-10s de período, ~90-130 RPM de amplitud).
3. Más conservador todavía (`tau_c=3×tau`) → `Kp=0.047, Ki=0.11` — la
   oscilación residual **casi no cambió** en tamaño ni período pese a
   bajar la ganancia proporcional/integral más de 2.5x, señal de que
   ya no era el lazo (más ganancia lo habría agravado, no una
   oscilación que se mantiene igual). Confirmado sin carga real (motor
   sin tubería conectada) — no es un efecto de purga de aire.
4. Dejado correr más tiempo en `SET_RPM=1300`: **asienta limpio
   después de un rato largo**, en banda ~1290-1310 (ruido normal),
   sin volver a oscilar — confirma que era un transitorio, no una
   inestabilidad sostenida.

**GANANCIAS FINALES (montaje directo): `Kp=0.047, Ki=0.11, Kd=0`** —
sintonizadas con SIMC (no Ziegler-Nichols, que no aplica a esta planta
por falta de tiempo muerto detectable) sobre el `K` peor caso medido
en el mapeo de ganancia, con margen extra de conservadurismo
(`tau_c=3×tau`) para tolerar la variación de ganancia entre zonas sin
gain-scheduling real todavía implementado. Asentamiento lento
**a propósito** — aceptado para esta aplicación (motobomba, necesita
tiempo igual para llenar/purgar la tubería). Pendiente: validar con
carga real de agua, y considerar gain-scheduling real si se necesita
una respuesta más rápida en el futuro.

**✅ Confirmado en banco (2026-09-08): estas ganancias funcionan bien
en un barrido de setpoints** por todo el rango real de operación
(ralentí hasta 1400+ RPM), sin oscilación sostenida en ningún punto,
con el motor sin carga (sin tubería conectada). Validación completa
con agua real todavía pendiente.

**⚠️ Incidente de campo (2026-09-08): un cambio de estructura en
`calibracion_flash.c` (fuera de esta sesión de tuneo, para agregar
`PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM`) subió el "magic" de
validación de la flash (`"CALF"`→`"CALG"`).** En el siguiente reflash,
esto borró TODA la calibración guardada (`SET_RATIO`, `SERVO_PULSO_MIN/MAX`,
`PID_KP/KI/KD`, todo) y la volvió a los valores de fábrica del código,
sin ningún aviso visible más que el pulso quedandose fijo en el
`SERVO_PULSO_MIN` de fábrica (`1000`, no el `1124` calibrado) sin
reaccionar a ningún `SET_RPM`. Hay que estar atento a esto cada vez
que se toque la estructura `CalibFlash_Datos_t` — el próximo reflash
después de ese cambio borra silenciosamente todo lo calibrado en
campo. Ver también el nuevo log `PID_GANANCIAS` más abajo, agregado
justamente para que esto sea más fácil de detectar la próxima vez.

**Efecto colateral del mismo incidente: `MODO` (ID 16) recuperó su
comportamiento por defecto (`RALENTÍ`), que ahora sí es funcional**
(antes de que existiera `MODO_REMOTO`/presión, `MODO` no se leía en
ningún lado — ver el bloque "MODO: fuente del setpoint de RPM" en
`main.c`). Con `MODO=0` (RALENTÍ), el firmware ignora `SET_RPM` por
completo, sin importar qué tan bien esté sintonizado el PID —
síntoma: el servo se queda fijo en `SERVO_PULSO_MIN` pase lo que pase.
Hace falta `MODO=1` (LOCAL) para que `SET_RPM` vuelva a comandar el
lazo, como se venía usando durante toda esta sesión. **`MODO` se
reclasificó de `CONFIGURACION` a `CALIBRACION`** (2026-09-08) para
poder cambiarlo sin detener el motor — antes de que tuviera un efecto
real, no importaba en qué categoría cayera.

**Log `PID_GANANCIAS` (agregado 2026-09-08)**: mientras se está en
`CONTROL_HABILITADO=7`, el firmware imprime
`PID_GANANCIAS,Kp=...,Ki=...,Kd=...` apenas se entra al modo, y de
nuevo cada vez que `PID_KP`/`PID_KI`/`PID_KD` cambian (por serial o
por downlink LoRa, los dos pasan por el mismo dispatcher) — para que
nunca más quede la duda de qué ganancias están realmente vigentes en
una sesión de sintonización, después del incidente de arriba.

**Investigación de discrepancia RPM vs. tacómetro analógico
(2026-09-08) — resuelta, no era el TID.** En campo se armó una tabla
comparando `SET_RPM` contra la lectura del tacómetro analógico de la
máquina, mostrando un desvío creciente con la RPM (bien hasta
~1150 RPM, hasta +100 RPM de diferencia en 1300-1400). Antes de tocar
nada del PID o del circuito de lectura, se comparó la frecuencia cruda
medida por el TID (monitor serial) contra la frecuencia real medida
con un osciloscopio directo en la señal W del alternador, en todo el
rango — **coincidieron casi exactas en cada punto** (ej. 383 Hz vs
383 Hz en 1300 RPM, 409 Hz vs 409 Hz en 1400 RPM). Esto descarta por
completo el circuito de lectura y la conversión frecuencia→RPM del
firmware (una sola fórmula lineal en `tacometro.c`, sin ninguna
compensación oculta) como causa. La razón frecuencia/RPM que implica
la lectura del tacómetro analógico, en cambio, **no es constante**
(~0.29 a baja RPM, ~0.273 a alta RPM, ~6% de no linealidad) — la
desviación está en el tacómetro analógico (su propio circuito interno
de conversión, o una calibración de fábrica para otro alternador), no
en el TID. **Conclusión: la lectura de RPM del TID es la correcta**,
no hace falta ni conviene replicar la no linealidad del instrumento
analógico en el firmware.

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
- **Solo se activa en `CONTROL_HABILITADO=7`** (modo sintonización PID,
  ver sección 4.4) — la misma condición que desbloquea
  `PID_KP/KI/KD`, una sola fuente de verdad para "¿estamos en una
  sesión de sintonización?". Fuera de ese modo el monitor se ve
  exactamente como antes de que existiera este log (solo la línea de
  1s). Se apaga solo en cuanto se sale del modo `7` (downlink
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

## 10. Auto-calibración de ralentí (`CONTROL_HABILITADO=3`)

Primera de las rutinas de auto-calibración planteadas para sacar el
ojo humano de la calibración inicial de cada unidad (la segunda,
detectar automáticamente dónde el servo empieza a acelerar el motor
para fijar `SERVO_PULSO_MIN`, queda pendiente de diseñar — a
diferencia de esta, sí necesita mover el servo directo con el motor
operando, así que le hace falta su propia cerca de seguridad antes de
construirse, no es un ajuste chico).

**Qué hace**: mide el ralentí real del motor (promediando
`RPM_filtrada`) y actualiza `RPM_MIN` automáticamente, en vez de que
alguien tenga que mirar el log y copiar un número a mano.

**Cómo usarla**: estando en operación normal (`CONTROL_HABILITADO=0`),
mandar
```
CONTROL_HABILITADO 3
```
(por downlink o por el mando manual de serial, sección 7). A
diferencia de los modos `1`/`2` (calibración de servo), **no hace
falta detener el motor** para entrar — de hecho la rutina solo tiene
sentido con el motor operando. Se puede mandar con el motor ya
corriendo, o antes de arrancarlo (en ese caso, la rutina simplemente
espera a que arranque antes de empezar a contar la ventana de 2
minutos). Ver sección 4.4 para el resto de la máquina de estados de
`CONTROL_HABILITADO`.

**Condiciones para que se acepte el downlink** (si no se cumplen, se
rechaza de una vez, sin quedar pendiente):
- Modo actual `CONTROL_HABILITADO=0` — no se puede pedir `3` directo
  desde `1`, `2` o `7` (rechazo `OUT_OF_RANGE`).

Una vez dentro del modo `3`, el servo queda fijo en `SERVO_PULSO_MIN`
todo el tiempo (igual que "sin control activo" en operación normal),
sin importar en qué punto esté la ventana de medición.

**Por qué dura 2 minutos en vez de unos segundos**: un motor frío
puede arrancar bien por debajo de su ralentí real y tardar en subir
(observado en campo: arranca ~500 y sube a ~800 ya caliente). Un
promedio corto agarraría esa rampa de calentamiento y calcularía un
`RPM_MIN` demasiado bajo. La rutina corre **2 minutos en total** desde
que el motor está operando, pero solo promedia los **últimos 30
segundos** de esa ventana — los primeros ~90s sirven de tiempo de
calentamiento/estabilización y se descartan.

**Si el motor se detiene a mitad de la ventana**: la medición en curso
se descarta (no tiene sentido promediar con el motor parado), pero se
permanece en modo `3` esperando a que vuelva a arrancar — no hace
falta reenviar el downlink, la ventana de 2 minutos arranca de nuevo
sola apenas el motor esté operando otra vez.

**Resultado**: `RPM_MIN` = promedio de los últimos 30s + margen fijo
de 30 RPM (para que el ruido normal de medición, ±10-15 RPM, no cruce
el umbral por accidente y dispare el PID solo por ruido). Al aplicar
el resultado, el firmware vuelve solo a `CONTROL_HABILITADO=0` — no
hace falta un downlink extra para salir del modo `3`.

**"ACK" por LoRa al terminar**: además del log serial, al completar la
medición el firmware fuerza un uplink inmediato (`CalibFlash_ForzarReporte()`,
misma bandera interna que dispara `FORZAR_REPORTE` por downlink) en vez
de esperar al próximo intervalo periódico de 30s — así se sabe por
LoRaWAN, sin mirar el log serial, que la calibración terminó. Es el
mismo formato de uplink estándar (RPM/estado/posición, sección 6), no
lleva el valor nuevo de `RPM_MIN` como campo propio — si hiciera falta
que el uplink cargue ese valor explícito habría que extender el
formato de 27 bytes (fuera de este repo, en el decoder de AWS).

**Log de progreso** (formato CSV, igual criterio que `PID_TEST`):
```
RALENTI_CAL,INICIO,esperando_motor=<0/1>
RALENTI_CAL,MIDIENDO,duracion_total_s=120,ventana_promedio_s=30
RALENTI_CAL,PROGRESO,transcurrido_s=<s>,rpm_actual=<rpm>            -- cada 5s
RALENTI_CAL,COMPLETO,promedio=<rpm>,RPM_MIN_nuevo=<rpm>,muestras=<n>,ok=<0/1>
RALENTI_CAL,ABORTADO,motivo=motor_se_detuvo                         -- vuelve a esperar, sigue en modo 3
```

**Pendiente**: `send_downlink.py` (Lambda AWS, fuera de este repo)
necesita `PARAMETER_TABLE["CONTROL_HABILITADO"]["max"]` bumped `3`→`5`
(cubre este modo y el de la sección 11) — mismo patrón que cuando se
agregaron `CONTROL_HABILITADO=2` y `=7`.

## 11. Auto-calibración de `SERVO_PULSO_MIN` (`CONTROL_HABILITADO=4`)

Segunda rutina de auto-calibración (la primera es la sección 10).
Encuentra el pulso exacto donde el acelerador empieza a mover la RPM
de verdad — motivado por una observación de campo: moviendo el brazo
del servo a mano, unos grados de recorrido no hacen nada (una **zona
muerta** mecánica del acelerador) antes de que el motor empiece a
reaccionar. Si `SERVO_PULSO_MIN` queda apenas *dentro* de esa zona
muerta, las correcciones chicas del PID cerca de ralentí caen a veces
adentro (sin efecto) y a veces la cruzan (respuesta brusca) — un
posible motivo de la oscilación intermitente observada a RPM bajas.

A diferencia de `CONTROL_HABILITADO=1`/`=2` (que también mueven el
servo directo, pero a ciegas, sin mirar la RPM — por eso exigen el
motor detenido), este modo sí mira la RPM en cada paso, así que puede
correr con el motor operando de forma seguro y acotada.

**Cómo usarla**: estando en operación normal (`CONTROL_HABILITADO=0`),
mandar
```
CONTROL_HABILITADO 4
```
Igual que el modo `3`: no hace falta detener el motor primero (si se
manda sin el motor operando, la rutina espera a que arranque). Solo se
acepta viniendo de modo `0` (rechazo `OUT_OF_RANGE` si se pide desde
`1`/`2`/`3`/`7`).

**Algoritmo:**
1. **Línea base fresca**: al arrancar (con el motor ya operando),
   promedia `RPM_filtrada` durante 12s con el servo quieto en el
   `SERVO_PULSO_MIN` actual — no confía en `RPM_MIN` guardado, mide de
   nuevo en el momento.
2. **Pasos chicos hacia arriba**: incrementa el pulso de a 8µs por
   vez, empezando en `SERVO_PULSO_MIN`.
3. **Espera de asentamiento**: 2.5s en cada paso antes de leer la RPM.
4. **Detección**: si `RPM_filtrada` sube 30 RPM o más sobre la línea
   base (bien por encima del ruido normal de ±10-15 RPM) durante 2
   lecturas seguidas **en el mismo pulso** (no dispara con un pico de
   una sola muestra) → ahí "empieza a acelerar".
5. **Margen de seguridad**: el nuevo `SERVO_PULSO_MIN` = el pulso
   donde se confirmó la detección, menos 6 pasos (48µs, subido de 3
   pasos/24µs el 2026-09-03 — ver nota de campo más abajo) de colchón —
   para no quedar pegado justo al borde de la zona muerta.
6. **Techo duro**: si se suben 180µs sobre el `SERVO_PULSO_MIN` de
   entrada sin detectar nada, se aborta con error — no sigue subiendo
   a ciegas si algo no encaja.
7. **Aborto inmediato** si: el motor se detiene a mitad del barrido
   (se descarta el progreso, pero se sigue en el modo esperando a que
   vuelva a arrancar — no hace falta reenviar el downlink), o si un
   solo paso sube la RPM 120 o más de golpe (un salto mucho más grande
   de lo esperado — señal de que algo salió distinto a lo previsto, se
   frena ya en vez de aplicar cualquier resultado automático).
8. **Al completar** (éxito o error): el servo vuelve a
   `SERVO_PULSO_MIN` (el nuevo si se encontró, el de entrada si no), y
   `CONTROL_HABILITADO` vuelve solo a `0`. Si se aplicó un valor nuevo,
   dispara el mismo "ACK" por LoRa que la calibración de ralentí
   (`CalibFlash_ForzarReporte()`, sección 10).

**Análisis de riesgo** (por qué estos números son de bajo riesgo):
- El techo (180µs) es una fracción chica del recorrido total del
  servo (~1075µs típico, `925→2000`) — menos del 20%. El pulso nunca
  puede terminar más allá de ese techo, sin importar qué pase.
- El movimiento es autolimitado por diseño: en cuanto se confirma la
  detección, deja de subir. No sigue empujando "para confirmar más" —
  el peor caso es quedarse justo en el borde de la zona muerta, nunca
  mucho más allá.
- Duración acotada: en el peor caso (llega al techo sin detectar
  nada), son `180/8 ≈ 23` pasos × 2.5s ≈ 1 minuto — motor cerca del
  ralentí ese rato, nada distinto a dejarlo en marcha mínima.
- El riesgo real no es mecánico, es de precisión: si el tiempo de
  asentamiento (2.5s) fuera corto para el motor real, un paso podría
  "pasarse de largo" unos µs antes de que se note la subida de RPM.
  Pero el margen de seguridad (retroceder 6 pasos al aplicar el
  resultado) ya absorbe justo ese tipo de error — no hace falta que la
  detección sea perfecta al µs.

**Log de progreso** (formato CSV):
```
SERVOMIN_CAL,INICIO,esperando_motor=<0/1>
SERVOMIN_CAL,BASELINE,pulso_base=<us>,duracion_s=12
SERVOMIN_CAL,BARRIENDO,baseline_rpm=<rpm>,pulso_inicial=<us>
SERVOMIN_CAL,PASO,pulso=<us>,rpm=<rpm>,subida=<rpm>,confirmaciones=<0-2>
SERVOMIN_CAL,COMPLETO,pulso_umbral=<us>,SERVO_PULSO_MIN_nuevo=<us>,ok=<0/1>
SERVOMIN_CAL,ABORTADO,motivo=motor_se_detuvo        -- vuelve a esperar, sigue en modo 4
SERVOMIN_CAL,ABORTADO,motivo=salto_rpm_anormal      -- sale del modo, no aplica nada
SERVOMIN_CAL,ERROR,motivo=techo_alcanzado_sin_deteccion  -- sale del modo, no aplica nada
```

**✅ Confirmado en campo (2026-09-03)**: en el motor de pruebas se
encontró una zona muerta mecánica real de ~136µs (`925→1061`, mucho
más grande que el techo de 180µs presupuestado — quedó justo). Se
aplicó `SERVO_PULSO_MIN=1037` (1061 − 3 pasos de 8µs, margen original).

**⚠️ Corrección de campo el mismo día**: al volver a correr esta
rutina más tarde (motor ya con `SET_RATIO` recalibrado), el umbral
detectado salió en `1093`, no `1061` — y la curva paso a paso mostró
que la transición **no siempre es un escalón limpio**: hubo subida
sostenida y real (`14.8`/`7.3`/`26.8`/`26.1`/`27.0` RPM) durante ~6
pasos antes de confirmar la detección. Con el margen original de 3
pasos, el `SERVO_PULSO_MIN` resultante (`1069`) quedó a mitad de esa
rampa, no en zona plana — el motor en reposo terminó rondando
`845-865` RPM en vez del relentí real (~826). Diagnosticado comparando
la curva `PASO` completa contra el barrido anterior (más abrupto) —
confirma que la zona muerta de este acelerador tiene un "codo" de
transición gradual, no un salto limpio. **Margen subido de 3 a 6
pasos (24µs→48µs)** en base a estos dos barridos reales (cubre la
rampa observada sin perjudicar el caso abrupto). Corregido en el
sitio manualmente ese mismo día (`SERVO_PULSO_MIN=950`, confirmado sin
acelerar el relentí) mientras se aplicaba el fix. Sigue pendiente
hacer el bump de AWS Lambda mencionado arriba. También sigue pendiente
(no construida, apenas discutida) una tercera medida de seguridad: si
el PID entra en oscilación sostenida por más de ~30s, desactivarlo
automáticamente (bajar a `SET_RPM=0`, no de a poco — ya se confirmó en
campo que bajar gradual no corta el *windup*) y reintentar el setpoint
original después de una pausa corta.

## 12. Secuencia de puesta en marcha en una máquina nueva

Orden recomendado al instalar el nodo en un motor por primera vez.
Cada paso depende de que el/los anteriores ya estén bien, así que no
conviene saltarlos ni cambiar el orden.

**Fase A — motor apagado (mecánico):**
1. `CONTROL_HABILITADO=2` — verificación visual manual de mínimos y
   máximos (sección 2.4/4.4). Sirve también como chequeo de seguridad:
   si a simple vista el servo ya se ve "abierto" más de la cuenta en
   la posición de reposo, es señal de que el `SERVO_PULSO_MIN` heredado
   (de otra máquina o calibración anterior) no aplica a este montaje
   mecánico nuevo, y hay que ajustarlo a mano antes de automatizar nada
   con el modo `4`.
2. `CONTROL_HABILITADO=1` — barrido automático sin restricción, para
   confirmar que el servo se mueve libre en todo el rango sin
   resistencia mecánica anómala.

**Fase B — motor encendido (sensores):**
3. `SET_RATIO` (o `SET_RATIO_AUTO`, ver abajo) — calibrar el tacómetro
   contra una referencia externa (tacómetro de mano). No basta una sola
   lectura en relentí: conviene tomar 2-3 lecturas repartidas en el
   rango de operación (relentí, un punto medio, uno alto si es seguro)
   y comparar el ratio implícito por cada una
   (`ratio = frecuenciaHz × 60 / RPM_referencia`). Si salen dentro de
   ~1-2% entre sí es solo ruido de la comparación — quedarse con
   cualquiera de ellas (o la última que se mandó, que es la que queda
   aplicada). Si hay una tendencia clara y consistente entre RPM bajas
   y altas, hay algo mecánico real (holgura/deslizamiento en la fuente
   de la señal) que un solo número fijo no puede corregir del todo; en
   ese caso priorizar el punto donde el PID va a operar la mayor parte
   del tiempo. La medición de `tacometro.c` usa Input Capture (período
   real en µs, no conteo de pulsos en ventana fija), así que no debería
   haber no-linealidad propia de la medición del firmware entre
   relentí y RPM altas.

   **`SET_RATIO_AUTO` (parámetro 25, agregado 2026-09-03)** hace el
   cálculo por vos: en vez de mandar el ratio ya despejado a mano, se
   manda el RPM que marca el tacómetro de referencia en ese instante
   (`x10`, mismo formato que `SET_RPM`) y el firmware calcula y aplica
   `SET_RATIO` directo, usando su propia `frecuenciaHz` del momento
   (`tacometro.c:109`, la misma fórmula pero despejada). El ACK
   devuelve el ratio resultante (no el RPM que mandaste), así que se ve
   directo si quedó razonable. Repetir el comando en 2-3 puntos de RPM
   durante la puesta en marcha sigue siendo la forma de detectar si hay
   un problema mecánico real: si el ratio resultante cambia bastante
   entre un punto y otro, no es la medición del firmware (ver arriba)
   — es la fuente de la señal físicamente. `OUT_OF_RANGE` si se manda
   `0` o si el motor no tiene lectura válida en ese momento. Comando
   serial equivalente: `SET_RATIO_AUTO <rpm>`. **Pendiente**: falta el
   alta en `send_downlink.py`'s `PARAMETER_TABLE` (ID 25, escala x10)
   para poder mandarlo por downlink LoRa, no solo por serial.
4. `ALPHA` (o `ALPHA_AUTO`, ver abajo) — fijar el filtro EMA de RPM.
   Debe quedar fijo **antes** de los modos `3`/`4`/`7`: cambiarlo
   después de sintonizar el PID le cambia la dinámica que el PID ya
   aprendió a manejar (más o menos lag en la señal), obligando a
   re-sintonizar. En la práctica esto pesa poco: el filtro se actualiza
   en cada pulso del tacómetro (cada pocos ms a las RPM típicas de este
   motor), no cada 200ms como el log `PID_TEST` — con `ALPHA=0.35` el
   retraso que agrega el filtro es del orden de milisegundos,
   insignificante frente a la dinámica real del lazo (cientos de ms).
   `ALPHA` no tiene un valor "correcto" único como `SET_RATIO` — es un
   compromiso de diseño entre ruido y velocidad de respuesta, así que
   no hace falta recalcularlo en cada instalación si el default ya
   funciona bien.

   **`CONTROL_HABILITADO=5` (agregado 2026-09-03) — calibración
   automática de `ALPHA`.** A diferencia de `SET_RATIO_AUTO`, acá no
   hay ninguna referencia externa que darle — el firmware mide su
   propio ruido, así que no necesita cargar ningún valor: se manda
   igual que los modos `3`/`4` (`CONTROL_HABILITADO 5`, solo aceptado
   viniendo de modo `0`, con o sin el motor operando — si no está
   operando, la rutina espera). Con el motor ya operando, toma ~8s de
   muestras de `RPM_instantánea` sin filtrar (cada 200ms), calcula su
   desviación estándar (algoritmo de Welford, evita el error numérico
   de restar cuadrados de RPM que ya son grandes), y despeja el `ALPHA`
   que dejaría la señal filtrada dentro de un objetivo fijo de `±10
   RPM` (`ALPHA_CAL_OBJETIVO_DESVIACION_RPM` en `main.c`), usando la
   atenuación de ruido de un filtro EMA: `alpha = 2r²/(1+r²)` con
   `r = objetivo/desviación_cruda`. Se aplica y persiste directo (mismo
   `CalibFlash_SetAlphaFiltro()` que usa `ALPHA`), sin que el operador
   tenga que copiar ningún número, y al terminar vuelve sola a
   `CONTROL_HABILITADO=0` (igual que `3`/`4`), disparando el mismo
   "ACK" por LoRa (`CalibFlash_ForzarReporte()`). **Techo de seguridad
   (`ALPHA_CAL_TECHO_ALPHA=0.5`, agregado 2026-09-03 tras un caso real
   en campo)**: si el ruido crudo medido en la ventana de 8s ya sale en
   o por debajo del objetivo, la fórmula tiende a `alpha≈1.0`
   (prácticamente sin filtro) — pasó en la primera prueba real
   (`desviación_cruda=10.08`, casi igual al objetivo de `10`, dio
   `alpha=0.99` antes de este fix). El riesgo es que una ventana de 8s
   puede simplemente no capturar alguna de las ráfagas de perturbación
   periódicas (~15-25s) ya conocidas en este motor — "se vio limpio en
   esos 8 segundos" no es garantía de que no haga falta nada de
   suavizado. El techo evita aplicar un `alpha` más agresivo que `0.5`
   sin importar qué tan bajo salga medido el ruido. Progreso en el log
   `ALPHA_CAL,...` (`INICIO`/`MIDIENDO`/`COMPLETO`/`ABORTADO` si el
   motor se detiene a mitad de la ventana — se descarta el progreso
   pero se sigue en modo `5` esperando a que arranque de nuevo, mismo
   criterio que `3`/`4` — /`ERROR` si no hubo muestras suficientes). No
   reemplaza a `ALPHA` directo — sigue haciendo falta para
   cargar/restaurar un valor ya conocido desde la base de datos sin
   tener que correr la medición de nuevo (mismo argumento que
   `SET_RATIO` vs `SET_RATIO_AUTO`). **Pendiente**: falta el bump de
   `send_downlink.py`'s `PARAMETER_TABLE["CONTROL_HABILITADO"]["max"]`
   a `5` (mismo patrón que las subidas anteriores) y una prueba real en
   campo (por ahora solo compila limpio).

**Fase C — motor encendido (calibración automática):**
5. `CONTROL_HABILITADO=3` — calibración automática de relentí
   (`RPM_MIN`, sección 10). No depende de que el modo `4` ya haya
   corrido: mientras el `SERVO_PULSO_MIN` actual esté realmente dentro
   de la zona muerta (confirmado a ojo en el paso 1), el relentí medido
   va a ser el real sin importar el punto exacto adentro de esa zona.
6. `CONTROL_HABILITADO=4` — calibración automática de zona muerta del
   servo (`SERVO_PULSO_MIN`, sección 11).
7. **`CONTROL_HABILITADO=6` (opcional, diagnóstico) — mapeo de curva de
   ganancia.** Agregado 2026-09-03 a raíz de una discusión sobre la
   geometría del mecanismo brazo-varilla que mueve la cremallera de la
   bomba: el brazo del servo gira, pero la cremallera se mueve
   linealmente (o casi) — esa conversión ángulo→posición no es
   constante en todo el recorrido (parecido a un mecanismo biela-
   manivela), así que la ganancia real del sistema (cuánta RPM sube por
   cada µs de pulso) varía según en qué parte del recorrido esté el
   servo, no solo por la zona muerta ya conocida. Esta rutina barre el
   pulso hacia arriba en pasos de `8µs` (mismo tamaño que la zona
   muerta, a propósito — no uno más grande, para no arriesgarse a
   pasarse del techo de RPM de un salto grande en la zona de más
   ganancia) desde `SERVO_PULSO_MIN`, en **lazo abierto** (sin PID),
   con una espera de asentamiento de `2.5s` en cada paso, registrando
   el par `(pulso, RPM real)` — hasta llegar a una RPM techo fija
   (`GANANCIA_CAL_RPM_TECHO`, elegida por el usuario según lo que
   considere seguro para su motor, no el máximo del servo) o hasta
   `SERVO_PULSO_MAX`. Aborta si el motor se detiene a mitad del
   barrido, o si un solo paso sube la RPM más de
   `GANANCIA_CAL_SALTO_ANORMAL_RPM` de golpe (protege contra pasarse
   del techo de un salto grande, además de detectar lecturas
   anómalas). **Por ahora esta rutina SOLO mide y loguea la curva — no
   aplica ninguna corrección al PID todavía.** La idea es usar estos
   datos más adelante para compensar la ganancia variable (ej.
   escalando la salida del PID según en qué parte del recorrido esté),
   pero eso es un diseño aparte, pendiente. Log: `GANANCIA_CAL,INICIO`
   / `BARRIENDO` / `PASO,pulso=<us>,rpm=<rpm>,salto=<rpm>` /
   `COMPLETO,motivo=techo_rpm_alcanzado,...` /
   `ABORTADO,motivo=motor_se_detuvo` /
   `ABORTADO,motivo=salto_rpm_anormal` /
   `ERROR,motivo=servo_pulso_max_alcanzado_sin_llegar_al_techo`.
   **Pendiente**: compilado limpio (0 errores/0 warnings), no probado
   en campo todavía.
8. `CONTROL_HABILITADO=7` — sintonización de PID (sección 9).

**`SERVO_PULSO_MAX` queda fuera de esta secuencia automática a
propósito**: a diferencia del mínimo, no hay forma segura de detectar
"llegó al tope mecánico y generó resistencia" solo mirando RPM — haría
falta retroalimentación de corriente/torque del servo, que el hardware
actual no tiene. Se sigue calibrando a mano (`CONTROL_HABILITADO=2`).
Si en el futuro se agrega sensado de corriente al servo, ahí sí valdría
la pena automatizarlo.

### Corrección de linealización geométrica del mecanismo (`MECANISMO_*`, interina, agregada 2026-09-03)

El actuador actual es un mecanismo biela-manivela: el brazo del servo
(la manivela) gira y empuja una varilla rígida (la biela) hacia la
palanca de la bomba de inyección. Este tipo de mecanismo tiene una
relación **no lineal** entre el ángulo del servo y la posición real
que alcanza la varilla — la misma raíz que explica varios hallazgos de
esta sesión: la zona muerta más ancha de lo esperado en
`CONTROL_HABILITADO=4`, la rampa gradual (no un escalón limpio) en esa
misma calibración, y la ganancia del motor variando ~16-25x según el
punto de operación durante la sintonización de PID.

**Geometría real, confirmada en campo 2026-09-03**: manivela
`r=5.5cm`, varilla `L=30cm`, montada exactamente en el **punto
muerto** (manivela y varilla alineadas, una continuación de la otra)
en `SERVO_PULSO_MIN` — la posición de menor ganancia posible del
mecanismo. La posición real de la varilla en función del ángulo de la
manivela sigue la ecuación clásica de biela-manivela (la misma de un
pistón de motor):

```
x(θ) = r·cos(θ) + √(L² − r²·sin²(θ))
```

Con estos números, en el rango real de operación (`~60°`), el avance
por cada `10°` de giro crece de forma casi constante desde `~1mm`
(cerca del punto muerto) hasta `~8.7mm` (al final del recorrido) — casi
9 veces más ganancia al final que al principio, para el mismo `Kp` del
PID.

**La corrección**: en vez de ignorar esta curva, se invierte
matemáticamente. Se trata el pulso que ya calculó el PID (con
`Kp/Ki/Kd`, sin tocar esas ganancias) como si representara linealmente
una posición `x` deseada entre los dos extremos reales del mecanismo
(en `SERVO_PULSO_MIN` y `SERVO_PULSO_MAX`), y se despeja el ángulo real
que efectivamente logra esa `x`, usando la inversa exacta de la
ecuación de arriba — que resulta ser la ley de cosenos aplicada al
mismo triángulo (manivela-varilla-posición):

```
cos(θ) = (r² + x² − L²) / (2·r·x)
```

Ese `θ` real se convierte de vuelta a un pulso (usando
`MECANISMO_US_POR_GRADO`, una constante fija en `main.c` que asume la
calibración típica `500-2500µs ↔ 180°` de un servo de hobby estándar
como el MG996R usado — revisar si se cambia de modelo de servo), y ese
es el pulso que realmente se le manda al servo. Implementado en
`Mecanismo_CorregirPulso()` (`main.c`), aplicado solo en la rama activa
del PID (no afecta los modos `1`/`2`/`3`/`4`/`5`/`6`, que mueven el
servo directo sin pasar por el PID).

**Parámetros nuevos** (`MECANISMO_MANIVELA_CM`, `MECANISMO_VARILLA_CM`,
`MECANISMO_OFFSET_GRADOS`, `MECANISMO_CORRECCION_ACTIVA` — IDs 26-29,
ver tabla de parámetros): `MECANISMO_OFFSET_GRADOS` generaliza la
fórmula para el caso de remontar la manivela lejos del punto muerto en
el futuro (ver discusión de campo: alejar el montaje `15-20°` del
punto muerto suaviza bastante la curva, aunque no la elimina — solo un
mecanismo genuinamente lineal, como piñón-cremallera, o el servo
montado directo sobre el eje de la palanca de la bomba, elimina la
no-linealidad de raíz en vez de compensarla). `MECANISMO_CORRECCION_ACTIVA`
queda apagada por defecto (`0`) — hay que cargar la geometría real y
encenderla a propósito; si la geometría cargada es inválida (radio o
varilla en `0`, o varilla ≤ radio), la corrección se salta sola y el
pulso pasa sin corregir, para no romper el control por una
configuración incompleta.

**Por qué es "interina"**: esta corrección compensa el mecanismo
biela-manivela *actual* — no reemplaza un rediseño mecánico. Está
pensada explícitamente para descartarse (dejar `MECANISMO_CORRECCION_ACTIVA=0`,
sin necesidad de quitar el código) si se reemplaza el actuador por un
mecanismo genuinamente lineal — piñón-cremallera, o la idea explorada
en paralelo de montar el servo directo sobre el eje de la palanca de
la bomba (sin brazo ni varilla intermedios, eliminando la geometría
por completo en vez de corregirla). Ver la memoria del proyecto para
la discusión completa de ambas alternativas y el análisis de ventaja
mecánica (torque) que también depende de esta misma geometría.

**Pendiente**: compilado limpio (0 errores/0 warnings), no probado en
campo todavía.