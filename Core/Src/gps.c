/**
 * @file    gps.c
 * @brief   Implementación del módulo de lectura de posición GPS/GNSS
 *          (SIM7600X) vía auto-reporte periódico de AT+CGPSINFO.
 *
 * ⚠️ Cambio de diseño (confirmado en campo, 2026-08-18): este módulo
 * NO transmite sentencias NMEA por su cuenta en esta UART -- probado
 * a mano vía USB-TTL directo, sin ninguna sentencia '$...' espontánea.
 * Lo que SÍ soporta (documentado en el AT Command Manual de SIMCom) es
 * "AT+CGPSINFO=<1-255>": configurado una sola vez, el módulo empieza a
 * mandar una línea "+CGPSINFO: ..." por su cuenta cada N segundos, sin
 * que el host tenga que volver a pedirla -- un URC, igual que los
 * "+EVT:" del RAK3172, no una respuesta a comando en curso.
 *
 * Diseño de recepción (modo Circular): el buffer de DMA se rearma
 * solo -- este módulo lleva su propio índice de "última posición
 * leída" (s_ultimaPosLeida) y en cada callback consume desde ahí hasta
 * la posición absoluta que reporta HAL ('Size'), con aritmética
 * módulo el tamaño del buffer para manejar el "dar la vuelta". Las
 * líneas completas (terminadas en '\n') se encolan para que
 * GPS_Update() las parsee fuera de contexto de interrupción -- mismo
 * patrón que rak3172.c.
 */

#include "gps.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ==================== ESTADO INTERNO ==================== */

static UART_HandleTypeDef *s_huart = NULL;

/* Buffer de recepción, llenado continuamente por DMA en modo Circular. */
static uint8_t  s_rxDmaBuffer[GPS_RX_BUFFER_SIZE];
static uint16_t s_ultimaPosLeida = 0;

/* Ensamblador de línea (acumula bytes hasta '\n'). */
static char     s_lineaEnConstruccion[GPS_LINEA_MAX_LEN];
static uint16_t s_lineaLen = 0;

/* Cola circular de sentencias NMEA completas pendientes de procesar. */
#define GPS_MAX_LINEAS_PENDIENTES  8U
static char     s_colaLineas[GPS_MAX_LINEAS_PENDIENTES][GPS_LINEA_MAX_LEN];
static volatile uint8_t s_colaHead = 0; /* siguiente a leer */
static volatile uint8_t s_colaTail = 0; /* siguiente a escribir */
static volatile uint8_t s_colaCount = 0;

/* Última posición conocida. */
static volatile bool  s_tieneFix = false;
static float           s_latitud = 0.0f;
static float           s_longitud = 0.0f;
static volatile uint32_t s_ultimoFixTickMs = 0;

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static void GPS_EncolarLinea(const char *linea, uint16_t len);
static void GPS_ProcesarSentencia(const char *linea);
static void GPS_ProcesarCGPSInfo(char *linea);
static uint8_t GPS_DividirCampos(char *sentencia, char *campos[], uint8_t maxCampos);
static float GPS_NmeaACoordenadaDecimal(const char *campo, uint8_t digitosGrados);

/* ==================== API PÚBLICA ==================== */

void GPS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_ultimaPosLeida = 0;
    s_lineaLen = 0;
    memset(s_lineaEnConstruccion, 0, sizeof(s_lineaEnConstruccion));

    s_colaHead = 0;
    s_colaTail = 0;
    s_colaCount = 0;

    s_tieneFix = false;
    s_latitud = 0.0f;
    s_longitud = 0.0f;
    s_ultimoFixTickMs = 0;

    /* Igual que en RAK3172_Init(): limpiar flags de error y vaciar el
     * registro de datos antes de arrancar, por si quedó algo colgado
     * de un reset previo del host mientras el módulo seguía activo. */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    volatile uint32_t dummy = s_huart->Instance->RDR;
    (void)dummy;

    /* Arranca la recepción continua por DMA en modo Circular (definido
     * en el .ioc) con detección de línea inactiva (IDLE). El callback
     * HAL_UARTEx_RxEventCallback debe reenviar el evento a
     * GPS_RxEventCallback() cuando huart->Instance == USART2 (ver
     * ejemplo en gps.h). A diferencia de RAK3172_Init(), acá NO hace
     * falta rearmar nada en el callback -- el modo Circular se
     * rearma solo.
     *
     * Nota: NO se deshabilita el evento de medio buffer (DMA_IT_HT)
     * a propósito -- rak3172.c documenta que intentar hacerlo justo
     * después de esta llamada causó un HardFault (hdmarx llegaba
     * NULL en ese punto). No es necesario deshabilitarlo de todas
     * formas: GPS_RxEventCallback() procesa correctamente cualquier
     * evento (HT, TC o IDLE) sin importar cuál lo disparó. */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, GPS_RX_BUFFER_SIZE);
}

void GPS_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance) {
        return;
    }

    /* En modo Circular, 'Size' es la posición ABSOLUTA de escritura
     * del DMA dentro del buffer (0..GPS_RX_BUFFER_SIZE-1), no una
     * cantidad de bytes nuevos -- se consume byte a byte desde la
     * última posición leída hasta acá, con módulo para el "dar la
     * vuelta" del buffer circular.
     *
     * ⚠️ BUG CORREGIDO (2026-08-18, encontrado con el depurador -- se
     * quedaba colgado dentro de este while): cuando el DMA completa una
     * vuelta EXACTA del buffer, HAL puede reportar Size==GPS_RX_BUFFER_SIZE
     * (512) en vez de 0. s_ultimaPosLeida jamas puede valer 512 porque
     * el modulo lo mantiene siempre en 0..511 -- sin este ajuste, la
     * condicion del while nunca se cumple y el ciclo gira para siempre
     * consumiendo basura, congelando el loop principal completo (todo
     * lo que dependa de HAL_UART_Transmit/printf sobre hlpuart1 se
     * queda sin correr, incluida la Frecuencia del tacometro). */
    uint16_t posActual = Size;
    if (posActual >= GPS_RX_BUFFER_SIZE) {
        posActual = 0;
    }

    while (s_ultimaPosLeida != posActual) {
        char c = (char)s_rxDmaBuffer[s_ultimaPosLeida];
        s_ultimaPosLeida = (uint16_t)((s_ultimaPosLeida + 1U) % GPS_RX_BUFFER_SIZE);

        if (c == '\r') {
            continue; /* ignorar CR, solo nos importa el LF como fin de sentencia */
        }

        if (c == '\n') {
            if (s_lineaLen > 0) {
                GPS_EncolarLinea(s_lineaEnConstruccion, s_lineaLen);
                s_lineaLen = 0;
            }
            continue;
        }

        if (s_lineaLen < (GPS_LINEA_MAX_LEN - 1U)) {
            s_lineaEnConstruccion[s_lineaLen++] = c;
        } else {
            /* Sentencia más larga que el buffer de línea (corrupta o
             * el parser se desincronizó) -- se descarta y se espera
             * al próximo '\n' en vez de partirla a la mitad. */
            s_lineaLen = 0;
        }
    }

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
}

void GPS_Update(void)
{
    while (s_colaCount > 0) {
        __disable_irq();
        char linea[GPS_LINEA_MAX_LEN];
        strncpy(linea, s_colaLineas[s_colaHead], GPS_LINEA_MAX_LEN);
        s_colaHead = (s_colaHead + 1) % GPS_MAX_LINEAS_PENDIENTES;
        s_colaCount--;
        __enable_irq();

        GPS_ProcesarSentencia(linea);
    }
}

bool GPS_EnviarComandoAT(const char *comando)
{
    char buffer[GPS_TX_BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "%s\r\n", comando);
    if (len <= 0 || (uint32_t)len >= sizeof(buffer)) {
        return false; /* comando demasiado largo */
    }

    HAL_UART_Transmit(s_huart, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    return true;
}

bool GPS_TieneFix(void)
{
    return s_tieneFix;
}

float GPS_GetLatitud(void)
{
    return s_latitud;
}

float GPS_GetLongitud(void)
{
    return s_longitud;
}

uint32_t GPS_GetUltimoFixTickMs(void)
{
    return s_ultimoFixTickMs;
}

/* ==================== FUNCIONES PRIVADAS ==================== */

static void GPS_EncolarLinea(const char *linea, uint16_t len)
{
    if (s_colaCount >= GPS_MAX_LINEAS_PENDIENTES) {
        return; /* cola llena -- se descarta la sentencia más antigua no leída */
    }

    if (len == 0) {
        return;
    }

    /* No se filtra por prefijo acá -- el volumen de líneas es bajo (un
     * "OK" ocasional a los comandos de arranque, más un "+CGPSINFO:"
     * cada N segundos), así que no vale la pena descartar nada antes
     * de la cola. GPS_ProcesarSentencia() decide qué le importa. */

    uint16_t copiar = (len < (GPS_LINEA_MAX_LEN - 1U)) ? len : (GPS_LINEA_MAX_LEN - 1U);
    memcpy(s_colaLineas[s_colaTail], linea, copiar);
    s_colaLineas[s_colaTail][copiar] = '\0';

    s_colaTail = (s_colaTail + 1) % GPS_MAX_LINEAS_PENDIENTES;
    s_colaCount++;
}

static void GPS_ProcesarSentencia(const char *linea)
{
    /* ⚠️ DIAGNOSTICO TEMPORAL -- util para confirmar en el monitor que
     * los "+CGPSINFO:" siguen llegando cada N segundos. Quitar cuando
     * ya no haga falta ver el crudo. */
    printf("GPS RX crudo: '%s'\r\n", linea);

    if (strncmp(linea, "+CGPSINFO:", 10) == 0) {
        /* Saltar el prefijo "+CGPSINFO:" y cualquier espacio despues
         * (el modulo manda "+CGPSINFO: 1416.43...", con un espacio) --
         * si no se salta, ese espacio se cuela como primer caracter del
         * campo de latitud y desalinea los "digitosGrados" que espera
         * GPS_NmeaACoordenadaDecimal(). */
        const char *datos = linea + 10;
        while (*datos == ' ') {
            datos++;
        }
        char copia[GPS_LINEA_MAX_LEN];
        strncpy(copia, datos, sizeof(copia) - 1);
        copia[sizeof(copia) - 1] = '\0';
        GPS_ProcesarCGPSInfo(copia);
    }
    /* Otras líneas ("OK" de arranque, banners, etc.) se ignoran. */
}

static void GPS_ProcesarCGPSInfo(char *linea)
{
    /* Formato de respuesta/URC de SIMCom (SIM7500_SIM7600 Series AT
     * Command Manual), NO es NMEA -- sin checksum:
     *   +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<fecha ddmmyy>,<hora hhmmss.s>,<alt>,<vel>,<rumbo>
     * Sin fix, todos los campos vienen vacíos:
     *   +CGPSINFO: ,,,,,,,,
     */
    char *campos[10] = { 0 };
    uint8_t n = GPS_DividirCampos(linea, campos, 10);
    if (n < 5) {
        return; /* línea incompleta/corrupta */
    }

    /* campos[0] es "+CGPSINFO: <lat>" -- el prefijo se saltó al llamar
     * con linea+10 más abajo en GPS_ProcesarSentencia, así que campos[0]
     * ya es directamente el campo de latitud. */
    if (campos[0][0] == '\0') {
        s_tieneFix = false; /* sin fix -- se conserva la ultima lat/lon conocida */
        return;
    }

    float lat = GPS_NmeaACoordenadaDecimal(campos[0], 2);
    if (campos[1][0] == 'S') {
        lat = -lat;
    }

    float lon = GPS_NmeaACoordenadaDecimal(campos[2], 3);
    if (campos[3][0] == 'W') {
        lon = -lon;
    }

    s_latitud = lat;
    s_longitud = lon;
    s_tieneFix = true;
    s_ultimoFixTickMs = HAL_GetTick();
}

static uint8_t GPS_DividirCampos(char *sentencia, char *campos[], uint8_t maxCampos)
{
    /* Tokeniza a mano (en vez de sscanf) porque NMEA permite campos
     * vacíos entre comas consecutivas (ej. variación magnética casi
     * siempre vacía) -- sscanf con "%[^,]" no matchea un campo vacío
     * y desalinea todos los campos siguientes. */
    uint8_t n = 0;
    char *cursor = sentencia;
    campos[n++] = cursor;

    while (*cursor != '\0' && n < maxCampos) {
        if (*cursor == ',' || *cursor == '*') {
            *cursor = '\0';
            cursor++;
            campos[n++] = cursor;
        } else {
            cursor++;
        }
    }

    return n;
}

static float GPS_NmeaACoordenadaDecimal(const char *campo, uint8_t digitosGrados)
{
    if (campo == NULL || campo[0] == '\0') {
        return 0.0f;
    }

    /* Formato NMEA: los primeros 'digitosGrados' caracteres son los
     * grados (2 para latitud, 3 para longitud), el resto son minutos
     * decimales (mm.mmmm). */
    char gradosBuf[4] = { 0 };
    strncpy(gradosBuf, campo, digitosGrados);
    gradosBuf[digitosGrados] = '\0';

    int grados = atoi(gradosBuf);
    float minutos = (float)atof(campo + digitosGrados);

    return (float)grados + (minutos / 60.0f);
}
