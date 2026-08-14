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
| PA2 | Debug TX | USART2_TX | Vía adaptador USB-TTL (CP2102) |
| PA3 | Debug RX | USART2_RX | — |
| PB0 | Servo (PWM) | TIM3_CH3 | **No usar PA6/TIM3_CH1** — el header físico de esa posición en el Nucleo-32 está enrutado a PA15 por el solder bridge SB3 de fábrica (ver UM2397, Tabla 9), PA6 no llega al header |
| PA1 | Sensor de presión (ADC) | ADC1_IN2 | Pendiente de implementar (`presion.c/h`) |

**Advertencia sobre SWD/depuración**: PA13/PA14 son SWDIO/SWCLK. Ningún
periférico de este proyecto los usa, así que el debugger (ST-Link,
Connect Under Reset) funciona con normalidad.

---

## 2. Módulos del firmware

```
tacometro.c/h          -> Medicion de RPM (Input Capture + filtro EMA)
rak3172.c/h             -> Comunicacion con el modulo LoRaWAN
calibracion_flash.c/h   -> Persistencia de parametros + dispatcher de downlinks
servo.c/h               -> Control PWM del servo del acelerador
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
| 1 | Uplink LIVE (RPM en vivo) |
| 2 | Downlink de parámetro (ver protocolo abajo) |
| 3 | Application ACK |

**Estado LoRaWAN**: Clase C, join OTAA con `AutoJoin=1` (persistente).
⚠️ El RAK3172 **no cambia de clase automáticamente** solo porque el
Device Profile en el servidor diga "admite Clase C" — hay que mandarle
explícitamente `AT+CLASS=C` (default de fábrica es Clase A).

### 2.3 `calibracion_flash.c/h`

Almacena los parámetros configurables en la **última página de flash**
del STM32G431KB (2KB, dirección `0x0801F800`, página 63), y centraliza
la lógica de aplicar cualquier downlink de configuración.

**⚠️ Regla crítica**: cada vez que se agregue, quite o reordene un
campo de `CalibFlash_Datos_t`, hay que **subir `CALIB_FLASH_MAGIC`**.
Si no se hace, `CalibFlash_Init()` copia bytes de flash borrada (0xFF)
hacia los campos nuevos, dando valores basura (típicamente 65535) —
esto ya causó un bug real de servo atascado en el máximo.

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

---

## 3. Protocolo de parámetros (downlink de configuración)

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

### 4.2 Modos de operación del motor (MODO 0/1/2) — **pendiente de implementar**

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
(12-24VDC, fuente separada del circuito de 3.3V).

---

## 6. Integración con AWS IoT Core for LoRaWAN

- **Dispositivo**: `DSL-0001`, Device Profile `RIO-DSL` (US915, Clase
  C, LoRaWAN 1.0.3), Service Profile `RIO` (compartido con el nodo
  aspersor del equipo).
- **Regla de IoT**: `RIO_DSL_Lambda`, tópico de entrada `RIO/DSL`.
- **Lambda**: `RIO-DSL-DecodeUplink` (Python 3.12,
  `lambda_function.py` + `decoder.py`).

### Forma real del evento que recibe la Lambda (confirmada con tráfico real)

```json
{ "PayloadData": "<base64>", "Fport": 1 }
```

⚠️ **No** viene anidado bajo `WirelessMetadata.LoRaWAN`, y **no**
incluye `DevEui` en absoluto (solo `FCnt` y `FPort`) — confirmado
capturando un mensaje real en el cliente de prueba MQTT, no asumido de
la documentación.

### Consulta SQL de la regla

```sql
SELECT
  WirelessDeviceId,
  WirelessMetadata.LoRaWAN.FPort as FPort,
  aws_lambda(
    "arn:aws:lambda:us-east-1:271551172366:function:RIO-DSL-DecodeUplink",
    { "PayloadData": PayloadData, "Fport": WirelessMetadata.LoRaWAN.FPort }
  ) as decodingoutput
FROM 'RIO/DSL'
```

Acción: **Republish** a `RIO/DSL/DECODED`, rol IAM `GIO_role`
(compartido con la regla del nodo aspersor).

### Formato de salida de la Lambda

```python
# LIVE (FPort 1):
{"rpm": 1027.8}

# ACK (FPort 3):
{"parameter": "SET_RATIO", "status": "OK", "valorAplicado": 17.5}
```

El `valorAplicado` ya viene **convertido a valor real** (no crudo) —
la Lambda aplica la escala correcta según el parámetro
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
3. Confirmar en el monitor serie del G431 (`Downlink ID=... STATUS=...`)
   y en el cliente MQTT (`RIO/DSL/DECODED`).

---

## 8. Pendientes generales

- [ ] PID (`Kp/Ki/Kd`) — a la espera de simulación en MATLAB. Salida
      del PID debe ser en microsegundos directos (mismo dominio que
      `Servo_SetPulsoUs()`).
- [ ] Posible parámetro 25, `INTEGRAL_MAX` (anti-windup del PID).
- [ ] Máquina de estados Modo 0/1/2 — sin implementar en firmware.
- [ ] Construcción física y prueba del circuito de presión (LM358).
- [ ] `presion.c/h` — módulo de lectura/conversión, sin escribir aún.
- [ ] `RESET_REMOTO` (ID 22) — recibido y validado, pero
      deliberadamente **no conectado** a ninguna acción real hasta
      definir qué hace el servo durante un reinicio (mantener última
      posición vs. quedar sin control unos segundos).
- [ ] Valores reales (no placeholder) de `RPM_MAX/MIN`,
      `TIMEOUT_SIN_COMANDO_S`, `HISTERESIS_MODO_S` — pendientes de
      definir con datos del motor/cliente real.
- [ ] Confirmar con el equipo si los IDs de parámetro son un espacio
      compartido entre tipos de nodo, o independientes por tipo.