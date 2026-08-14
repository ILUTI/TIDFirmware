/**
 * @file    calibracion_flash.h
 * @brief   Almacena en la última página de flash del G431 los
 *          parámetros de configuración remota del nodo motor
 *          (ver "Tabla de Parametros de Configuracion Remota"),
 *          y centraliza el punto de entrada para aplicar cualquier
 *          parámetro recibido por downlink (LoRaWAN, sin importar si
 *          viene de Chirpstack o de una Lambda en AWS IoT Core).
 *
 * Categorías de parámetros:
 *   - PERSISTENTES: se guardan en flash, sobreviven resets. Ej.
 *     SET_RATIO, RPM_MAX, PID_KP, etc.
 *   - NO PERSISTENTES (solo RAM): SET_RPM y PRESION -- llegan con
 *     frecuencia y no vale la pena desgastar flash con ellos.
 *   - COMANDOS: RESTAURAR_DEFAULTS, FORZAR_REPORTE, RESET_REMOTO --
 *     no son valores que se guardan, son acciones que se ejecutan al
 *     recibir el downlink. Requieren un byte de confirmación
 *     específico (CALIB_BYTE_CONFIRMACION) para reducir el riesgo de
 *     ejecución accidental por un downlink corrupto.
 *
 * Uso típico en main.c:
 *   CalibFlash_Init();                          // antes de todo lo demás
 *   Tacometro_Init(&htim2, TIM_CHANNEL_1);
 *   ...
 *   while (1) {
 *       if (CalibFlash_HayReporteForzado()) {
 *           ... mandar uplink inmediato ...
 *           CalibFlash_LimpiarReporteForzado();
 *       }
 *       if (CalibFlash_HayResetPendiente()) {
 *           HAL_Delay(100); // dar tiempo a que salga cualquier log pendiente
 *           NVIC_SystemReset();
 *       }
 *   }
 *
 * Y en rak3172.c, al procesar un downlink ya separado en ID + bytes:
 *   CalibFlash_ProcesarParametro(idRecibido, payloadBytes, longitudPayload);
 */

#ifndef CALIBRACION_FLASH_H
#define CALIBRACION_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==================== IDs DE PARÁMETROS (ver tabla Excel) ==================== */

#define CALIB_ID_SET_RATIO                       1U
#define CALIB_ID_ALPHA                            2U
#define CALIB_ID_SET_RPM                          3U
#define CALIB_ID_RPM_MAX                          4U
#define CALIB_ID_RPM_MIN                          5U
#define CALIB_ID_PID_KP                           6U
#define CALIB_ID_PID_KI                           7U
#define CALIB_ID_PID_KD                           8U
#define CALIB_ID_SERVO_PULSO_MIN                  9U
#define CALIB_ID_SERVO_PULSO_MAX                  10U
#define CALIB_ID_TIMEOUT_SIN_COMANDO_S             11U
#define CALIB_ID_TASA_MAX_CAMBIO_RPM_S             12U
#define CALIB_ID_CONTROL_HABILITADO                13U
#define CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S       14U
#define CALIB_ID_INTERVALO_ENVIO_STANDBY_S         15U
#define CALIB_ID_MODO                              16U
#define CALIB_ID_PRESION                           17U
#define CALIB_ID_NODE_ID                           18U
#define CALIB_ID_RESTAURAR_DEFAULTS                19U
#define CALIB_ID_FORZAR_REPORTE                    20U
#define CALIB_ID_HISTERESIS_MODO_S                 21U
#define CALIB_ID_RESET_REMOTO                      22U
#define CALIB_ID_PRESION_OBJETIVO                  23U
#define CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S     24U

/* Byte de confirmación requerido para ejecutar los comandos críticos
 * (RESTAURAR_DEFAULTS, RESET_REMOTO). Cambiar aquí si se requiere
 * otro valor -- debe coincidir con lo que mande la Lambda/Chirpstack. */
#define CALIB_BYTE_CONFIRMACION   0xA5U

/* Categorías de parámetro, para decidir si se permite cambiar mientras
 * el motor está operando (ver CalibFlash_ProcesarParametroConEstado):
 *   CALIBRACION    -- siempre permitido, incluso operando (SET_RATIO,
 *                      ALPHA -- se necesitan ajustar en caliente para
 *                      calibrar contra un tacómetro externo real).
 *   CONFIGURACION  -- solo permitido con el motor detenido (límites de
 *                      seguridad, ganancias del PID, límites del servo,
 *                      etc.) -- cambiarlos en caliente podría causar un
 *                      comportamiento impredecible del lazo de control.
 *   PROCESO        -- siempre permitido, es su función normal (SET_RPM,
 *                      PRESION llegan constantemente mientras opera).
 *   COMANDO        -- casos especiales (RESTAURAR_DEFAULTS, etc.), se
 *                      evalúan aparte, no bloqueados por esta regla.
 */
typedef enum {
    CALIB_CATEGORIA_CALIBRACION = 0,
    CALIB_CATEGORIA_CONFIGURACION = 1,
    CALIB_CATEGORIA_PROCESO = 2,
    CALIB_CATEGORIA_COMANDO = 3
} CalibFlash_Categoria_t;

/* Valores válidos de MODO */
typedef enum {
    CALIB_MODO_MANUAL        = 0,
    CALIB_MODO_AUTOMATICO    = 1,
    CALIB_MODO_MANTENIMIENTO = 2
} CalibFlash_Modo_t;

/* Códigos de STATUS del protocolo Quick-Set (ver Tabla 2 acordada con
 * el equipo). Se devuelven en el ACK de aplicación, junto con el
 * valor que quedó realmente vigente (no necesariamente el solicitado,
 * si fue rechazado). */
typedef enum {
    CALIB_STATUS_OK                     = 0,
    CALIB_STATUS_OUT_OF_RANGE           = 1,
    CALIB_STATUS_UNKNOWN_PARAMETER_ID   = 2,
    CALIB_STATUS_STORAGE_ERROR          = 3,
    CALIB_STATUS_APPLY_ERROR            = 4,  /* reservado, sin uso actual */
    CALIB_STATUS_REJECTED_ENGINE_RUNNING = 5  /* motor operando -- parametro
                                                 * de configuracion/seguridad
                                                 * no se puede cambiar hasta
                                                 * que el motor se detenga */
} CalibFlash_ProtocoloStatus_t;

/* ==================== API DE INICIALIZACIÓN ==================== */

/** Carga la calibración guardada en flash, o valores por defecto si
 * no hay ninguna guardada todavía (primera vez que corre el firmware
 * en este chip, o flash corrupta/borrada). */
void CalibFlash_Init(void);

/* ==================== PUNTO DE ENTRADA CENTRALIZADO PARA DOWNLINKS ==================== */

/**
 * Versión detallada de CalibFlash_ProcesarParametro(), pensada para
 * armar el Application ACK del protocolo Quick-Set (4 bytes:
 * [PARAMETER_ID][STATUS][VALUE_H][VALUE_L]).
 *
 * A diferencia de la versión simple (bool), esta:
 *   - Devuelve el STATUS exacto (ver CalibFlash_ProtocoloStatus_t).
 *   - Entrega en 'valorAplicadoRaw' el valor de 2 bytes que quedó
 *     REALMENTE vigente tras procesar el downlink -- si se rechazó,
 *     es el valor anterior (el que ya tenía); si se aplicó, es el
 *     nuevo valor. Ya viene codificado en la misma escala/formato de
 *     2 bytes que usa el protocolo (big-endian, listo para partir en
 *     VALUE_H/VALUE_L).
 *
 * @param id                 Uno de los CALIB_ID_*.
 * @param datos              Bytes del valor recibido (2 bytes, big-endian).
 * @param longitudDatos      Cantidad de bytes disponibles en 'datos'.
 * @param motorOperando      true si el motor está operando (no detenido).
 *                           Los parámetros de categoría CONFIGURACION se
 *                           rechazan con CALIB_STATUS_REJECTED_ENGINE_RUNNING
 *                           si este valor es true -- normalmente se pasa
 *                           !Tacometro_EstaDetenido() desde el llamador
 *                           (rak3172.c), sin que este módulo dependa
 *                           directamente de tacometro.h.
 * @param valorAplicadoRaw   [salida] valor vigente, codificado en 2 bytes.
 * @return El STATUS correspondiente.
 */
CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id,
                                                                     const uint8_t *datos,
                                                                     uint8_t longitudDatos,
                                                                     bool motorOperando,
                                                                     uint16_t *valorAplicadoRaw);

/**
 * Procesa un parámetro recibido por downlink, ya separado en ID +
 * bytes de payload (sin el byte de ID). Decodifica según el tipo de
 * cada parámetro (ver tabla), valida rango, y aplica (persistiendo en
 * flash si corresponde). Es el único punto que rak3172.c necesita
 * llamar -- no requiere que rak3172.c conozca los detalles de escala
 * o validación de cada parámetro.
 *
 * Wrapper simple sobre CalibFlash_ProcesarParametroConEstado(), para
 * quien solo necesite saber si se aplicó o no, sin armar un ACK.
 *
 * @param id             Uno de los CALIB_ID_* de arriba.
 * @param datos          Puntero a los bytes del valor (big-endian).
 * @param longitudDatos  Cantidad de bytes disponibles en 'datos'.
 * @return true si el ID fue reconocido Y el valor se aplicó
 *         correctamente (rango válido, o comando con confirmación
 *         correcta). false si el ID no existe, el valor está fuera de
 *         rango, o un comando llegó sin la confirmación esperada.
 */
bool CalibFlash_ProcesarParametro(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando);

/* ==================== GETTERS DE PARÁMETROS PERSISTENTES ==================== */

float    CalibFlash_GetPulsosPorRevolucion(void);
float    CalibFlash_GetAlphaFiltro(void);
float    CalibFlash_GetRpmMax(void);
float    CalibFlash_GetRpmMin(void);
float    CalibFlash_GetPidKp(void);
float    CalibFlash_GetPidKi(void);
float    CalibFlash_GetPidKd(void);
uint16_t CalibFlash_GetServoPulsoMinUs(void);
uint16_t CalibFlash_GetServoPulsoMaxUs(void);
uint32_t CalibFlash_GetTimeoutSinComandoS(void);
float    CalibFlash_GetTasaMaxCambioRpmS(void);
bool     CalibFlash_GetControlHabilitado(void);
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void);
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void);
CalibFlash_Modo_t CalibFlash_GetModo(void);
uint8_t  CalibFlash_GetNodeId(void);
uint16_t CalibFlash_GetHisteresisModoS(void);
float    CalibFlash_GetPresionObjetivo(void);
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void);

/* ==================== SETTERS INDIVIDUALES (validan + persisten) ==================== */
/* Se exponen también individualmente por si se necesitan llamar
 * directo (ej. desde una futura pantalla local, o pruebas), aunque el
 * camino normal desde un downlink es CalibFlash_ProcesarParametro(). */

bool CalibFlash_SetPulsosPorRevolucion(float nuevoValor);
bool CalibFlash_SetAlphaFiltro(float nuevoValor);
bool CalibFlash_SetRpmMax(float nuevoValor);
bool CalibFlash_SetRpmMin(float nuevoValor);
bool CalibFlash_SetPidKp(float nuevoValor);
bool CalibFlash_SetPidKi(float nuevoValor);
bool CalibFlash_SetPidKd(float nuevoValor);
bool CalibFlash_SetServoPulsoMinUs(uint16_t nuevoValor);
bool CalibFlash_SetServoPulsoMaxUs(uint16_t nuevoValor);
bool CalibFlash_SetTimeoutSinComandoS(uint32_t nuevoValor);
bool CalibFlash_SetTasaMaxCambioRpmS(float nuevoValor);
bool CalibFlash_SetControlHabilitado(bool habilitado);
bool CalibFlash_SetIntervaloEnvioOperativoS(uint16_t nuevoValor);
bool CalibFlash_SetIntervaloEnvioStandbyS(uint16_t nuevoValor);
bool CalibFlash_SetModo(CalibFlash_Modo_t nuevoModo);
bool CalibFlash_SetNodeId(uint8_t nuevoValor);
bool CalibFlash_SetHisteresisModoS(uint16_t nuevoValor);
bool CalibFlash_SetPresionObjetivo(float nuevoValor);
bool CalibFlash_SetTasaMaxCambioRpmLlenadoS(float nuevoValor);

/* ==================== PARÁMETROS NO PERSISTENTES (solo RAM) ==================== */

float CalibFlash_GetSetRpm(void);
void  CalibFlash_SetSetRpm(float nuevoValor); /* sin validación de rango propia --
                                                 * el módulo de control debe recortarla
                                                 * contra RPM_MIN/RPM_MAX antes de usarla */

float CalibFlash_GetPresion(void);
void  CalibFlash_SetPresion(float nuevoValor);

/* ==================== COMANDOS ==================== */

/**
 * true si llegó un FORZAR_REPORTE pendiente de atender. El loop
 * principal debe consultarlo, mandar el uplink correspondiente, y
 * llamar CalibFlash_LimpiarReporteForzado().
 */
bool CalibFlash_HayReporteForzado(void);
void CalibFlash_LimpiarReporteForzado(void);

/**
 * true si llegó un RESET_REMOTO válido (con confirmación correcta).
 * El loop principal debe consultarlo y, cuando sea seguro hacerlo
 * (ej. servo en una posición conocida/segura), ejecutar el reset
 * real (NVIC_SystemReset()).
 */
bool CalibFlash_HayResetPendiente(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRACION_FLASH_H */
