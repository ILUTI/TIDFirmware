/**
 * @file    calibracion_flash.c
 * @brief   Implementación del almacenamiento de calibración en flash
 *          y del dispatcher centralizado de parámetros por downlink.
 *
 * Usa la ÚLTIMA página de flash del STM32G431KB (128 Kbytes, páginas
 * de 2 Kbytes -- 64 páginas, página 63 es la última, dirección
 * 0x0801F800). El STM32G4 requiere programar en "double word" (64
 * bits) alineados -- de ahí que la estructura se rellene a múltiplo
 * de 8 bytes.
 *
 * Codificación asumida para cada parámetro recibido por downlink
 * (payload después del byte de ID, big-endian):
 *   - SET_RATIO, ALPHA, PID_KP/KI/KD, TASA_MAX_CAMBIO_*,
 *     PRESION_OBJETIVO: 4 bytes, int32 con signo, escala según tabla.
 *   - SET_RPM, RPM_MAX, RPM_MIN, PRESION: 2 bytes, uint16, x10.
 *   - SERVO_PULSO_MIN/MAX: 2 bytes, uint16, en µs directo (sin escala).
 *   - TIMEOUT_SIN_COMANDO_S: 4 bytes, uint32, en segundos directo.
 *   - INTERVALO_*, HISTERESIS_MODO_S: 2 bytes, uint16, en segundos.
 *   - CONTROL_HABILITADO, MODO, NODE_ID, y los 3 comandos: 1 byte.
 */

#include "calibracion_flash.h"
#include "main.h"
#include <string.h>

/* ==================== CONFIGURACIÓN DE UBICACIÓN EN FLASH ==================== */

#define CALIB_FLASH_PAGE_NUMBER   63U
#define CALIB_FLASH_ADDRESS       0x0801F800UL
#define CALIB_FLASH_MAGIC         0x43414C45UL  /* "CALE" -- subido desde
                                                   * "CALD" porque se agregaron
                                                   * ultimaLatitudConocida/
                                                   * ultimaLongitudConocida
                                                   * a CalibFlash_Datos_t. */

/* ==================== VALORES POR DEFECTO ==================== */
/* Los mismos que ya tenías validados en campo para SET_RATIO/ALPHA;
 * el resto son placeholders razonables -- AJUSTAR antes de operar
 * con el motor/servo real. */

#define DEFAULT_PULSOS_POR_REVOLUCION     17.5f
#define DEFAULT_ALPHA_FILTRO              0.35f
#define DEFAULT_RPM_MAX                   2500.0f
#define DEFAULT_RPM_MIN                   700.0f
#define DEFAULT_PID_KP                    1.0f
#define DEFAULT_PID_KI                    0.0f
#define DEFAULT_PID_KD                    0.0f
#define DEFAULT_SERVO_PULSO_MIN_US        1000U
#define DEFAULT_SERVO_PULSO_MAX_US        2000U
#define DEFAULT_TIMEOUT_SIN_COMANDO_S     1800U   /* 30 min, mayor al Estado 1 del aspersor (15-20 min) */
#define DEFAULT_TASA_MAX_CAMBIO_RPM_S     50.0f
#define DEFAULT_CONTROL_HABILITADO        false
#define DEFAULT_INTERVALO_OPERATIVO_S     30U
#define DEFAULT_INTERVALO_STANDBY_S       300U
#define DEFAULT_MODO                      CALIB_MODO_MANUAL
#define DEFAULT_NODE_ID                   0U
#define DEFAULT_HISTERESIS_MODO_S         30U
#define DEFAULT_PRESION_OBJETIVO          40.0f
#define DEFAULT_TASA_MAX_CAMBIO_LLENADO_S 10.0f  /* mas lenta que la normal, a proposito */

/* ==================== RANGOS VÁLIDOS ==================== */

#define RANGO_PULSOS_MIN         0.1f
#define RANGO_PULSOS_MAX         200.0f
#define RANGO_ALPHA_MAX          1.0f
#define RANGO_RPM_MAX_TECHO      6000.0f
#define RANGO_TASA_CAMBIO_MAX    2000.0f
#define RANGO_PRESION_MAX        500.0f

/* ==================== ESTRUCTURA GUARDADA EN FLASH ==================== */
/* Debe quedar en múltiplo de 8 bytes (double word) para el
 * STM32G4. Se agrega 'relleno' al final si hace falta. */

typedef struct {
    uint32_t magic;

    float    pulsosPorRevolucion;      /* SET_RATIO */
    float    alphaFiltro;              /* ALPHA */
    float    rpmMax;                   /* RPM_MAX */
    float    rpmMin;                   /* RPM_MIN */
    float    pidKp;                    /* PID_KP */
    float    pidKi;                    /* PID_KI */
    float    pidKd;                    /* PID_KD */
    float    tasaMaxCambioRpmS;        /* TASA_MAX_CAMBIO_RPM_S */
    float    presionObjetivo;          /* PRESION_OBJETIVO */
    float    tasaMaxCambioRpmLlenadoS; /* TASA_MAX_CAMBIO_RPM_LLENADO_S */
    float    ultimaLatitudConocida;    /* GPS, 0.0f = nunca hubo fix */
    float    ultimaLongitudConocida;   /* GPS, 0.0f = nunca hubo fix */

    uint32_t timeoutSinComandoS;       /* TIMEOUT_SIN_COMANDO_S */
    uint32_t ultimaHoraUtcConocida;    /* epoch UTC, 0 = nunca sincronizado */

    uint16_t servoPulsoMinUs;          /* SERVO_PULSO_MIN */
    uint16_t servoPulsoMaxUs;          /* SERVO_PULSO_MAX */
    uint16_t intervaloOperativoS;      /* INTERVALO_ENVIO_OPERATIVO_S */
    uint16_t intervaloStandbyS;        /* INTERVALO_ENVIO_STANDBY_S */
    uint16_t histeresisModoS;          /* HISTERESIS_MODO_S */

    uint8_t  controlHabilitado;        /* CONTROL_HABILITADO (0/1) */
    uint8_t  modo;                     /* MODO (0/1/2) */
    uint8_t  nodeId;                   /* NODE_ID */
    uint8_t  relleno[7];               /* completa a múltiplo de 8 bytes -- ajustar si
                                         * CalibFlash_VerificarTamano rompe la compilación */
} CalibFlash_Datos_t;

/* Verificación en tiempo de compilación de que el tamaño es múltiplo
 * de 8 -- si esto rompe la compilación, ajustar 'relleno'. */
typedef char CalibFlash_VerificarTamano[
    (sizeof(CalibFlash_Datos_t) % 8U == 0U) ? 1 : -1
];

/* ==================== ESTADO EN RAM ==================== */

static CalibFlash_Datos_t s_datos;

/* Parámetros NO persistentes (se reciben seguido, solo RAM) */
static float s_setRpm = 0.0f;
static float s_presion = 0.0f;

/* Banderas de comandos */
static volatile bool s_reporteForzadoPendiente = false;
static volatile bool s_resetPendiente = false;

/* Se pone en true si la última llamada a CalibFlash_EscribirEnFlash()
 * falló por un error real de hardware (borrado/programación), en vez
 * de un rechazo de validación de rango. El dispatcher la consulta
 * justo después de un Set fallido para distinguir STORAGE_ERROR de
 * OUT_OF_RANGE en el ACK. */
static volatile bool s_ultimaEscrituraFallo = false;

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static bool CalibFlash_EscribirEnFlash(void);
static uint16_t CalibFlash_ValorActualRaw(uint8_t id);
static uint16_t LeerUint16BigEndian(const uint8_t *datos);
static int16_t LeerInt16BigEndian(const uint8_t *datos);
static uint16_t CodificarUint16(float valorReal, float escala);
static uint16_t CodificarInt16ComoRaw(float valorReal, float escala);

/* ==================== API DE INICIALIZACIÓN ==================== */

void CalibFlash_Init(void)
{
    const CalibFlash_Datos_t *datosEnFlash = (const CalibFlash_Datos_t *)CALIB_FLASH_ADDRESS;

    if (datosEnFlash->magic == CALIB_FLASH_MAGIC) {
        memcpy(&s_datos, datosEnFlash, sizeof(CalibFlash_Datos_t));
        /* ultimaHoraUtcConocida/ultimaLatitudConocida/ultimaLongitudConocida
         * vienen tal cual estaban en flash -- NO se tocan aca, son
         * justamente los valores que queremos preservar entre arranques. */
    } else {
        memset(&s_datos, 0, sizeof(s_datos));
        s_datos.magic = CALIB_FLASH_MAGIC;
        s_datos.pulsosPorRevolucion      = DEFAULT_PULSOS_POR_REVOLUCION;
        s_datos.alphaFiltro              = DEFAULT_ALPHA_FILTRO;
        s_datos.rpmMax                   = DEFAULT_RPM_MAX;
        s_datos.rpmMin                   = DEFAULT_RPM_MIN;
        s_datos.pidKp                    = DEFAULT_PID_KP;
        s_datos.pidKi                    = DEFAULT_PID_KI;
        s_datos.pidKd                    = DEFAULT_PID_KD;
        s_datos.tasaMaxCambioRpmS        = DEFAULT_TASA_MAX_CAMBIO_RPM_S;
        s_datos.presionObjetivo          = DEFAULT_PRESION_OBJETIVO;
        s_datos.tasaMaxCambioRpmLlenadoS = DEFAULT_TASA_MAX_CAMBIO_LLENADO_S;
        s_datos.timeoutSinComandoS       = DEFAULT_TIMEOUT_SIN_COMANDO_S;
        s_datos.ultimaHoraUtcConocida    = 0U;  /* 0 = nunca sincronizado con la red --
                                                   * SOLO se resetea aca (primera vez /
                                                   * flash invalida), no en cada arranque. */
        s_datos.ultimaLatitudConocida    = 0.0f; /* 0.0f = nunca hubo fix GPS -- mismo
                                                     * criterio que ultimaHoraUtcConocida. */
        s_datos.ultimaLongitudConocida   = 0.0f;
        s_datos.servoPulsoMinUs          = DEFAULT_SERVO_PULSO_MIN_US;
        s_datos.servoPulsoMaxUs          = DEFAULT_SERVO_PULSO_MAX_US;
        s_datos.intervaloOperativoS      = DEFAULT_INTERVALO_OPERATIVO_S;
        s_datos.intervaloStandbyS        = DEFAULT_INTERVALO_STANDBY_S;
        s_datos.histeresisModoS          = DEFAULT_HISTERESIS_MODO_S;
        s_datos.controlHabilitado        = DEFAULT_CONTROL_HABILITADO ? 1U : 0U;
        s_datos.modo                     = (uint8_t)DEFAULT_MODO;
        s_datos.nodeId                   = DEFAULT_NODE_ID;
        /* No se escribe a flash aquí -- solo al primer Set explícito. */
    }

    s_setRpm = 0.0f;
    s_presion = 0.0f;
    s_reporteForzadoPendiente = false;
    s_resetPendiente = false;
}

/* ==================== DISPATCHER CENTRALIZADO ==================== */

/* Categoría de cada parámetro -- ver CalibFlash_Categoria_t en el header
 * para el criterio completo. IDs no listados aquí se tratan como
 * CONFIGURACION por defecto (más conservador: si se agrega un parámetro
 * nuevo y se olvida clasificarlo, queda bloqueado durante operación en
 * vez de quedar accidentalmente siempre permitido). */
static CalibFlash_Categoria_t CalibFlash_CategoriaDe(uint8_t id)
{
    switch (id) {
        case CALIB_ID_SET_RATIO:
        case CALIB_ID_ALPHA:
            return CALIB_CATEGORIA_CALIBRACION;

        case CALIB_ID_SET_RPM:
        case CALIB_ID_PRESION:
            return CALIB_CATEGORIA_PROCESO;

        case CALIB_ID_FORZAR_REPORTE:
        case CALIB_ID_RESTAURAR_DEFAULTS:
        case CALIB_ID_RESET_REMOTO:
            return CALIB_CATEGORIA_COMANDO;

        /* Todo lo demas (RPM_MAX/MIN, PID_KP/KI/KD, SERVO_PULSO_MIN/MAX,
         * TIMEOUT_SIN_COMANDO_S, TASA_MAX_CAMBIO_*, CONTROL_HABILITADO,
         * INTERVALO_*, MODO, NODE_ID, HISTERESIS_MODO_S,
         * PRESION_OBJETIVO) cae en CONFIGURACION por el default. */
        default:
            return CALIB_CATEGORIA_CONFIGURACION;
    }
}


CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, uint16_t *valorAplicadoRaw)
{
    *valorAplicadoRaw = 0U;
    s_ultimaEscrituraFallo = false;

    /* Bloqueo de parámetros de CONFIGURACION mientras el motor opera --
     * se evalúa ANTES que cualquier otra validación (rango, flash),
     * y antes incluso de saber si el ID es válido o no, porque un ID
     * desconocido de todas formas cae en CATEGORIA_CONFIGURACION por
     * el default conservador de CalibFlash_CategoriaDe(). */
    if (motorOperando && CalibFlash_CategoriaDe(id) == CALIB_CATEGORIA_CONFIGURACION) {
        *valorAplicadoRaw = CalibFlash_ValorActualRaw(id);
        return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
    }

    /* El protocolo Quick-Set acordado manda siempre 2 bytes de valor
     * (VALUE_H+VALUE_L), incluso para parámetros de 1 byte lógico
     * (se usa solo VALUE_L, VALUE_H debe ir en 0). Si llega menos de
     * 2 bytes, es un downlink malformado -- se reporta como ID
     * desconocido, ya que no hay un código de protocolo específico
     * para "longitud inválida". */
    if (longitudDatos < 2) {
        return CALIB_STATUS_UNKNOWN_PARAMETER_ID;
    }

    switch (id) {

        case CALIB_ID_SET_RATIO: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetPulsosPorRevolucion(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_ALPHA: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetAlphaFiltro(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetAlphaFiltro(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SET_RPM: {
            /* No persistente, sin validación de rango propia -- el
             * módulo de control debe recortarla contra RPM_MIN/MAX.
             * Siempre se considera aplicado. */
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            CalibFlash_SetSetRpm(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetRpm(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_RPM_MAX: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMax(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMax(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RPM_MIN: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMin(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_KP: {
            /* Escala x100 -- AJUSTAR según resultados de MATLAB. */
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetPidKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKp(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_KI: {
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKi(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_KD: {
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidKd(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKd(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SERVO_PULSO_MIN: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetServoPulsoMinUs(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetServoPulsoMinUs();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SERVO_PULSO_MAX: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetServoPulsoMaxUs(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetServoPulsoMaxUs();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TIMEOUT_SIN_COMANDO_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetTimeoutSinComandoS((uint32_t)nuevoValor);
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetTimeoutSinComandoS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TASA_MAX_CAMBIO_RPM_S: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetTasaMaxCambioRpmS(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaMaxCambioRpmS(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_CONTROL_HABILITADO: {
            bool ok = CalibFlash_SetControlHabilitado(datos[1] != 0U);
            *valorAplicadoRaw = CalibFlash_GetControlHabilitado() ? 1U : 0U;
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetIntervaloEnvioOperativoS(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetIntervaloEnvioOperativoS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_INTERVALO_ENVIO_STANDBY_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetIntervaloEnvioStandbyS(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetIntervaloEnvioStandbyS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_MODO: {
            bool ok = CalibFlash_SetModo((CalibFlash_Modo_t)datos[1]);
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetModo();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            CalibFlash_SetPresion(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresion(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_NODE_ID: {
            bool ok = CalibFlash_SetNodeId(datos[1]);
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetNodeId();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RESTAURAR_DEFAULTS: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                *valorAplicadoRaw = 0U;
                return CALIB_STATUS_OUT_OF_RANGE; /* confirmación inválida */
            }
            memset(&s_datos, 0, sizeof(s_datos));
            CalibFlash_Init(); /* magic queda inválido -> Init reconstruye defaults en RAM */
            bool ok = CalibFlash_EscribirEnFlash();
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return ok ? CALIB_STATUS_OK : CALIB_STATUS_STORAGE_ERROR;
        }

        case CALIB_ID_FORZAR_REPORTE: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                *valorAplicadoRaw = 0U;
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            s_reporteForzadoPendiente = true;
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_HISTERESIS_MODO_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetHisteresisModoS(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetHisteresisModoS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RESET_REMOTO: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                *valorAplicadoRaw = 0U;
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            s_resetPendiente = true;
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_PRESION_OBJETIVO: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetPresionObjetivo(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionObjetivo(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetTasaMaxCambioRpmLlenadoS(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaMaxCambioRpmLlenadoS(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        default:
            *valorAplicadoRaw = 0U;
            return CALIB_STATUS_UNKNOWN_PARAMETER_ID;
    }
}

bool CalibFlash_ProcesarParametro(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando)
{
    uint16_t valorAplicado;
    CalibFlash_ProtocoloStatus_t status = CalibFlash_ProcesarParametroConEstado(id, datos, longitudDatos, motorOperando, &valorAplicado);
    return (status == CALIB_STATUS_OK);
}

/* ==================== GETTERS ==================== */

float    CalibFlash_GetPulsosPorRevolucion(void)      { return s_datos.pulsosPorRevolucion; }
float    CalibFlash_GetAlphaFiltro(void)              { return s_datos.alphaFiltro; }
float    CalibFlash_GetRpmMax(void)                   { return s_datos.rpmMax; }
float    CalibFlash_GetRpmMin(void)                   { return s_datos.rpmMin; }
float    CalibFlash_GetPidKp(void)                    { return s_datos.pidKp; }
float    CalibFlash_GetPidKi(void)                    { return s_datos.pidKi; }
float    CalibFlash_GetPidKd(void)                    { return s_datos.pidKd; }
uint16_t CalibFlash_GetServoPulsoMinUs(void)          { return s_datos.servoPulsoMinUs; }
uint16_t CalibFlash_GetServoPulsoMaxUs(void)          { return s_datos.servoPulsoMaxUs; }
uint32_t CalibFlash_GetTimeoutSinComandoS(void)       { return s_datos.timeoutSinComandoS; }
float    CalibFlash_GetTasaMaxCambioRpmS(void)        { return s_datos.tasaMaxCambioRpmS; }
bool     CalibFlash_GetControlHabilitado(void)        { return s_datos.controlHabilitado != 0U; }
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void) { return s_datos.intervaloOperativoS; }
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void)   { return s_datos.intervaloStandbyS; }
CalibFlash_Modo_t CalibFlash_GetModo(void)            { return (CalibFlash_Modo_t)s_datos.modo; }
uint8_t  CalibFlash_GetNodeId(void)                   { return s_datos.nodeId; }
uint16_t CalibFlash_GetHisteresisModoS(void)          { return s_datos.histeresisModoS; }
float    CalibFlash_GetPresionObjetivo(void)          { return s_datos.presionObjetivo; }
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void) { return s_datos.tasaMaxCambioRpmLlenadoS; }

uint32_t CalibFlash_GetUltimaHoraUtcConocida(void)
{
    return s_datos.ultimaHoraUtcConocida;
}

bool CalibFlash_SetUltimaHoraUtcConocida(uint32_t epochUtc)
{
    s_datos.ultimaHoraUtcConocida = epochUtc;
    return CalibFlash_EscribirEnFlash();
}

float CalibFlash_GetUltimaLatitudConocida(void)
{
    return s_datos.ultimaLatitudConocida;
}

float CalibFlash_GetUltimaLongitudConocida(void)
{
    return s_datos.ultimaLongitudConocida;
}

bool CalibFlash_SetUltimaPosicionConocida(float latitud, float longitud)
{
    /* Un solo borrado/escritura para los dos campos juntos -- se
     * actualizan siempre a la vez (ver GPS_TieneFix() en main.c), no
     * tiene sentido separarlos en dos llamadas y gastar flash doble. */
    s_datos.ultimaLatitudConocida = latitud;
    s_datos.ultimaLongitudConocida = longitud;
    return CalibFlash_EscribirEnFlash();
}

/* ==================== SETTERS (validan rango + persisten) ==================== */

bool CalibFlash_SetPulsosPorRevolucion(float v)
{
    if (v < RANGO_PULSOS_MIN || v > RANGO_PULSOS_MAX) return false;
    s_datos.pulsosPorRevolucion = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetAlphaFiltro(float v)
{
    if (v <= 0.0f || v > RANGO_ALPHA_MAX) return false;
    s_datos.alphaFiltro = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetRpmMax(float v)
{
    if (v <= s_datos.rpmMin || v > RANGO_RPM_MAX_TECHO) return false;
    s_datos.rpmMax = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetRpmMin(float v)
{
    if (v < 0.0f || v >= s_datos.rpmMax) return false;
    s_datos.rpmMin = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidKp(float v)
{
    s_datos.pidKp = v; /* ganancias PID: se permite cualquier valor con signo,
                         * la sintonización decide qué es razonable */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidKi(float v)
{
    s_datos.pidKi = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidKd(float v)
{
    s_datos.pidKd = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetServoPulsoMinUs(uint16_t v)
{
    if (v < 500U || v >= s_datos.servoPulsoMaxUs) return false;
    s_datos.servoPulsoMinUs = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetServoPulsoMaxUs(uint16_t v)
{
    if (v <= s_datos.servoPulsoMinUs || v > 2500U) return false;
    s_datos.servoPulsoMaxUs = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTimeoutSinComandoS(uint32_t v)
{
    if (v < 60U || v > 3600U) return false; /* 1 min a 60 min */
    s_datos.timeoutSinComandoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTasaMaxCambioRpmS(float v)
{
    if (v <= 0.0f || v > RANGO_TASA_CAMBIO_MAX) return false;
    s_datos.tasaMaxCambioRpmS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetControlHabilitado(bool habilitado)
{
    s_datos.controlHabilitado = habilitado ? 1U : 0U;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetIntervaloEnvioOperativoS(uint16_t v)
{
    if (v < 5U || v > 3600U) return false;
    s_datos.intervaloOperativoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetIntervaloEnvioStandbyS(uint16_t v)
{
    if (v < 5U || v > 3600U) return false;
    s_datos.intervaloStandbyS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetModo(CalibFlash_Modo_t v)
{
    if (v != CALIB_MODO_MANUAL && v != CALIB_MODO_AUTOMATICO && v != CALIB_MODO_MANTENIMIENTO) {
        return false;
    }
    s_datos.modo = (uint8_t)v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetNodeId(uint8_t v)
{
    s_datos.nodeId = v; /* 0-255, cualquier valor es válido */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetHisteresisModoS(uint16_t v)
{
    if (v > 600U) return false;
    s_datos.histeresisModoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionObjetivo(float v)
{
    if (v <= 0.0f || v > RANGO_PRESION_MAX) return false;
    s_datos.presionObjetivo = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTasaMaxCambioRpmLlenadoS(float v)
{
    if (v <= 0.0f || v > RANGO_TASA_CAMBIO_MAX) return false;
    /* Advertencia de diseño: se acepta aunque sea mayor a
     * tasaMaxCambioRpmS -- si eso pasa, el llamador (módulo de
     * control) debería tratarlo como configuración inconsistente y
     * usar el menor de los dos por seguridad. */
    s_datos.tasaMaxCambioRpmLlenadoS = v;
    return CalibFlash_EscribirEnFlash();
}

/* ==================== NO PERSISTENTES ==================== */

float CalibFlash_GetSetRpm(void)  { return s_setRpm; }
void  CalibFlash_SetSetRpm(float v) { s_setRpm = v; }

float CalibFlash_GetPresion(void) { return s_presion; }
void  CalibFlash_SetPresion(float v) { s_presion = v; }

/* ==================== COMANDOS ==================== */

bool CalibFlash_HayReporteForzado(void)
{
    return s_reporteForzadoPendiente;
}

void CalibFlash_LimpiarReporteForzado(void)
{
    s_reporteForzadoPendiente = false;
}

bool CalibFlash_HayResetPendiente(void)
{
    return s_resetPendiente;
}

/* ==================== FUNCIONES PRIVADAS ==================== */

static uint16_t LeerUint16BigEndian(const uint8_t *datos)
{
    return (uint16_t)(((uint16_t)datos[0] << 8) | (uint16_t)datos[1]);
}

static int16_t LeerInt16BigEndian(const uint8_t *datos)
{
    uint16_t valor = (uint16_t)(((uint16_t)datos[0] << 8) | (uint16_t)datos[1]);
    return (int16_t)valor;
}

/* Codifica un valor real a uint16, aplicando la escala inversa y
 * redondeando. Se usa para parámetros sin signo (RPM, presión,
 * segundos, etc.). Se satura al rango de uint16 por seguridad, en
 * vez de desbordarse silenciosamente. */

/* Consulta el valor vigente de un parámetro (sin intentar cambiarlo),
 * ya codificado en 2 bytes -- usado para reportar el valor correcto en
 * el ACK cuando un cambio se rechaza (ej. por CALIB_STATUS_REJECTED_
 * ENGINE_RUNNING), en vez de reportar 0. */
static uint16_t CalibFlash_ValorActualRaw(uint8_t id)
{
    switch (id) {
        case CALIB_ID_SET_RATIO:
            return CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
        case CALIB_ID_ALPHA:
            return CodificarUint16(CalibFlash_GetAlphaFiltro(), 1000.0f);
        case CALIB_ID_RPM_MAX:
            return CodificarUint16(CalibFlash_GetRpmMax(), 10.0f);
        case CALIB_ID_RPM_MIN:
            return CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
        case CALIB_ID_PID_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPidKp(), 100.0f);
        case CALIB_ID_PID_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPidKi(), 1000.0f);
        case CALIB_ID_PID_KD:
            return CodificarInt16ComoRaw(CalibFlash_GetPidKd(), 1000.0f);
        case CALIB_ID_SERVO_PULSO_MIN:
            return CalibFlash_GetServoPulsoMinUs();
        case CALIB_ID_SERVO_PULSO_MAX:
            return CalibFlash_GetServoPulsoMaxUs();
        case CALIB_ID_TIMEOUT_SIN_COMANDO_S:
            return (uint16_t)CalibFlash_GetTimeoutSinComandoS();
        case CALIB_ID_TASA_MAX_CAMBIO_RPM_S:
            return CodificarUint16(CalibFlash_GetTasaMaxCambioRpmS(), 10.0f);
        case CALIB_ID_CONTROL_HABILITADO:
            return CalibFlash_GetControlHabilitado() ? 1U : 0U;
        case CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S:
            return CalibFlash_GetIntervaloEnvioOperativoS();
        case CALIB_ID_INTERVALO_ENVIO_STANDBY_S:
            return CalibFlash_GetIntervaloEnvioStandbyS();
        case CALIB_ID_MODO:
            return (uint16_t)CalibFlash_GetModo();
        case CALIB_ID_NODE_ID:
            return (uint16_t)CalibFlash_GetNodeId();
        case CALIB_ID_HISTERESIS_MODO_S:
            return CalibFlash_GetHisteresisModoS();
        case CALIB_ID_PRESION_OBJETIVO:
            return CodificarUint16(CalibFlash_GetPresionObjetivo(), 10.0f);
        case CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S:
            return CodificarUint16(CalibFlash_GetTasaMaxCambioRpmLlenadoS(), 10.0f);
        default:
            return 0U;
    }
}

static uint16_t CodificarUint16(float valorReal, float escala)
{
    float escalado = valorReal * escala;
    if (escalado < 0.0f) {
        escalado = 0.0f;
    }
    if (escalado > 65535.0f) {
        escalado = 65535.0f;
    }
    return (uint16_t)(escalado + 0.5f);
}

/* Codifica un valor real con signo a un patrón de bits de 16 bits
 * (int16 reinterpretado como uint16 para el transporte), usado para
 * las ganancias del PID. Se satura al rango de int16. */
static uint16_t CodificarInt16ComoRaw(float valorReal, float escala)
{
    float escalado = valorReal * escala;
    if (escalado < -32768.0f) {
        escalado = -32768.0f;
    }
    if (escalado > 32767.0f) {
        escalado = 32767.0f;
    }
    int16_t valorEntero = (int16_t)(escalado >= 0.0f ? (escalado + 0.5f) : (escalado - 0.5f));
    return (uint16_t)valorEntero; /* reinterpretación de bits, no conversión numérica */
}

static bool CalibFlash_EscribirEnFlash(void)
{
    HAL_StatusTypeDef estado;
    FLASH_EraseInitTypeDef borrado = {0};
    uint32_t paginaConError = 0;

    s_datos.magic = CALIB_FLASH_MAGIC;
    s_ultimaEscrituraFallo = false;

    HAL_FLASH_Unlock();

    borrado.TypeErase = FLASH_TYPEERASE_PAGES;
    borrado.Banks = FLASH_BANK_1;
    borrado.Page = CALIB_FLASH_PAGE_NUMBER;
    borrado.NbPages = 1;

    estado = HAL_FLASHEx_Erase(&borrado, &paginaConError);
    if (estado != HAL_OK) {
        HAL_FLASH_Lock();
        s_ultimaEscrituraFallo = true;
        return false;
    }

    uint64_t *origen = (uint64_t *)&s_datos;
    uint32_t direccion = CALIB_FLASH_ADDRESS;
    uint32_t cantidadDoubleWords = sizeof(CalibFlash_Datos_t) / sizeof(uint64_t);

    for (uint32_t i = 0; i < cantidadDoubleWords; i++) {
        estado = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, direccion, origen[i]);
        if (estado != HAL_OK) {
            HAL_FLASH_Lock();
            s_ultimaEscrituraFallo = true;
            return false;
        }
        direccion += sizeof(uint64_t);
    }

    HAL_FLASH_Lock();
    return true;
}
