# TID — Firmware del Nodo Motor Diésel (RIO-DSL)

Firmware para STM32G431KBT6U (Nucleo-32) que mide RPM de un motor diésel
vía el alternador, se comunica por LoRaWAN (RAK3172), y permite
calibración/configuración remota mediante un protocolo de parámetros
propio. Pendiente: control PID del acelerador vía servo, con entrada de
presión 4-20mA.

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
rtc_reloj.c/h           -> Reloj interno (RTC sobre LSI) + conversion epoch<->calendario
gps.c/h                 -> Posicion GPS/GNSS (SIM7600X) + disciplina del RTC
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

**Estado LoRaWAN**: Clase C, join OTAA con `AutoJoin=1` (persistente).
⚠️ El RAK3172 **no cambia de clase automáticamente** solo porque el
Device Profile en el servidor diga "admite Clase C" — hay que mandarle
explícitamente `AT+CLASS=C` (default de fábrica es Clase A).

#### Sincronización de hora (GPS + DeviceTimeReq)

Desde que se integró el GPS (ver 2.6), la hora real (unix epoch) tiene
**dos fuentes independientes**, ambas llaman a `Reloj_SetHoraUtc()`:

1. **GPS** (`gps.c`, más rápida): cada `+CGPSINFO:` con fix trae fecha
   y hora UTC — se aplica al RTC en cada reporte (cada 10s), lo que de
   paso disciplina continuamente el LSI (impreciso, deriva con
   temperatura) contra una fuente precisa.
2. **Red LoRaWAN** (`DeviceTimeReq`, respaldo): si el GPS nunca
   consigue fix (sin vista al cielo), este mecanismo sigue funcionando
   igual que siempre. Como está protegido por
   `!Reloj_EstaSincronizado()`, si el GPS ya sincronizó primero, este
   bloque simplemente no hace nada — no hay conflicto entre las dos
   fuentes.

El flujo completo de `DeviceTimeReq` (RUI3: `AT+TIMEREQ`/`AT+LTIME`):

1. Una vez unido (`RAK3172_EstaUnido()`), se manda `AT+TIMEREQ=1`
   (`RAK3172_SolicitarHoraRed()`).
2. La hora **no llega de inmediato** — viaja "montada" en el próximo
   uplink exitoso. Cuando eso pasa, el módulo emite el evento
   `+EVT:TIMEREQ`, detectado en `RAK3172_ProcesarLinea()` y expuesto
   como `RAK3172_HoraDeRedDisponible()`.
3. Con la hora ya disponible, se consulta el valor con `AT+LTIME=?`
   (`RAK3172_ConsultarHoraRed()`), y la respuesta se lee por el
   mecanismo genérico `RAK3172_GetUltimaRespuesta()`.
4. `main.c` parsea esa respuesta como epoch UTC y llama a
   `Reloj_SetUnixTimeUtc()` (ver sección 2.5).

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

**Cuidado con el reset del contador de estado al sincronizar** (bug
corregido 2026-08-18): cuando el reloj salta de un valor
aproximado/placeholder al real (por GPS o por red), el código en
`main.c` **desplaza** el ancla de `inicio_operacion`
(`inicioEstadoLocal`) por el mismo salto, en vez de reiniciarla a
"ahora" — si no, cada sincronización borraba el tiempo ya transcurrido
en el estado actual (`estado` sin cambiar), aunque el motor llevara
horas en el mismo estado.

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
| CALIBRACION | Sí, siempre | `SET_RATIO`, `ALPHA` |
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
— por eso el firmware **resincroniza contra la red LoRaWAN
(DeviceTimeReq) en cada arranque** (ver sección 2.2), en vez de asumir
que el RTC "recuerda" la hora entre encendidos.

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
recepción, luego `AT+CGPS=1` (enciende el motor GNSS) y
`AT+CGPSINFO=10` (habilita el auto-reporte cada 10s).

**Prioridad de posición para el uplink LIVE** (`main.c`): GPS con fix
vivo (`GPS_TieneFix()`) → última posición conocida persistida en flash
(`CalibFlash_GetUltimaLatitudConocida()`/`...Longitud...()`, se guarda
una sola vez por arranque, la primera vez que hay fix — no en cada
reporte, para no desgastar la flash) → `LATITUD_FIJA`/`LONGITUD_FIJA`
como último respaldo si el GPS nunca ha conseguido fix (ni en este
arranque ni en ninguno anterior).

**Disciplina del RTC**: cada `+CGPSINFO:` con fix también llama a
`Reloj_SetHoraUtc()` con la fecha/hora UTC del GPS — ver sección 2.2.

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
| 4 | `APPLY_ERROR` | Reservado, sin uso actual |
| 5 | `REJECTED_ENGINE_RUNNING` | Parámetro de categoría CONFIGURACION, motor operando |

### Tabla completa de parámetros

| ID | Nombre | Escala | Ancho | Categoría | Notas |
|---|---|---|---|---|---|
| 1 | `SET_RATIO` | x100 | 2B uint16 | Calibración | Pulsos/revolución |
| 2 | `ALPHA` | x1000 | 2B uint16 | Calibración | Coef. filtro EMA (0-1) |
| 3 | `SET_RPM` | x10 | 2B uint16 | Proceso | Setpoint directo (uso interno/pruebas, MODO=0) |
| 4 | `RPM_MAX` | x10 | 2B uint16 | Configuración | Límite duro superior |
| 5 | `RPM_MIN` | x10 | 2B uint16 | Configuración | Límite duro / ralentí |
| 6 | `PID_KP` | x100 | 2B int16 | Configuración | Con signo |
| 7 | `PID_KI` | x1000 | 2B int16 | Configuración | Con signo |
| 8 | `PID_KD` | x1000 | 2B int16 | Configuración | Con signo |
| 9 | `SERVO_PULSO_MIN` | directo (µs) | 2B uint16 | Configuración | Límite mecánico |
| 10 | `SERVO_PULSO_MAX` | directo (µs) | 2B uint16 | Configuración | Límite mecánico |
| 11 | `TIMEOUT_SIN_COMANDO_S` | directo (s) | 2B uint16 | Configuración | 60-3600s |
| 12 | `TASA_MAX_CAMBIO_RPM_S` | x10 | 2B uint16 | Configuración | Rampa normal |
| 13 | `CONTROL_HABILITADO` | flag | 1B | Configuración | Enable/disable lazo de control |
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

---

## 8. Pendientes generales

- [ ] PID (`Kp/Ki/Kd`) — a la espera de simulación en MATLAB. Salida
      del PID debe ser en microsegundos directos (mismo dominio que
      `Servo_SetPulsoUs()`).
- [ ] Posible parámetro 25, `INTEGRAL_MAX` (anti-windup del PID).
- [ ] Máquina de estados Modo 0/1/2 (sección 4.3) — sin implementar en
      firmware.
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