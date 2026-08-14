/**
 * @file    rak3172.c
 * @brief   Implementación del módulo de comunicación con el RAK3172.
 *
 * Diseño de recepción (modo Normal, no circular):
 *   El módulo posee su propio buffer de recepción y arranca la
 *   captura por DMA (HAL_UARTEx_ReceiveToIdle_DMA) dentro de
 *   RAK3172_Init(). Cada evento de recepción (línea inactiva o buffer
 *   lleno) entrega en 'Size' la cantidad de bytes nuevos recibidos, y
 *   la recepción se rearma manualmente al final del callback -- a
 *   propósito, en vez de modo Circular, para tener un punto de
 *   control donde limpiar flags de error antes de rearmar, y evitar
 *   que un error de línea puntual (ruido, glitch) se vuelva un ciclo
 *   que se retroalimenta solo.
 *
 *   Las líneas completas (terminadas en '\n') se acumulan en una cola
 *   circular pequeña para que RAK3172_Update() las procese fuera de
 *   contexto de interrupción.
 */

#include "rak3172.h"
#include "calibracion_flash.h"
#include "tacometro.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ==================== ESTADO INTERNO ==================== */

static UART_HandleTypeDef *s_huart = NULL;

/* Buffer de recepción, llenado por DMA en modo Normal (no circular). */
static uint8_t  s_rxDmaBuffer[RAK3172_RX_BUFFER_SIZE];

/* Ensamblador de línea (acumula bytes hasta '\n'). */
static char     s_lineaEnConstruccion[RAK3172_RX_BUFFER_SIZE];
static uint16_t s_lineaLen = 0;

/* Cola circular de líneas completas pendientes de procesar. */
#define RAK3172_MAX_LINEAS_PENDIENTES  4U
static char     s_colaLineas[RAK3172_MAX_LINEAS_PENDIENTES][RAK3172_RX_BUFFER_SIZE];
static volatile uint8_t s_colaHead = 0; /* siguiente a leer */
static volatile uint8_t s_colaTail = 0; /* siguiente a escribir */
static volatile uint8_t s_colaCount = 0;

/* Estado del comando AT en curso. */
static volatile bool     s_comandoEnCurso   = false;
static volatile uint32_t s_comandoTickInicio = 0;
static volatile uint32_t s_comandoTimeoutMs  = RAK3172_TIMEOUT_DEFAULT_MS;
static RAK3172_Resultado_t s_ultimoResultado = RAK3172_OK;
static char     s_ultimaRespuesta[RAK3172_RX_BUFFER_SIZE];
static bool     s_hayRespuestaNueva = false;
static volatile bool s_estaUnido = false;

/* Reintento de Application ACK: si al momento de querer mandarlo el
 * canal AT está ocupado (otro comando en curso), se guarda aquí y
 * RAK3172_Update() lo reintenta en cuanto el canal se libere, hasta
 * un timeout máximo -- para no perder la confirmación en silencio
 * solo porque coincidió con, por ejemplo, el uplink periódico de RPM. */
#define RAK3172_ACK_REINTENTO_TIMEOUT_MS  5000U

static volatile bool     s_ackPendiente = false;
static uint8_t           s_ackId = 0;
static uint8_t           s_ackStatus = 0;
static uint16_t          s_ackValorRaw = 0;
static uint32_t          s_ackTickInicio = 0;

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static void RAK3172_EncolarLinea(const char *linea, uint16_t len);
static void RAK3172_ProcesarLinea(const char *linea);
static void RAK3172_ProcesarEventoDownlink(const char *linea);
static bool RAK3172_HexAAlBytes(const char *hex, uint8_t *bytesSalida, uint8_t maxBytes, uint8_t *cantidadBytes);
static bool RAK3172_EnviarAckInterno(uint8_t id, uint8_t status, uint16_t valorRaw);

/* ==================== API PÚBLICA ==================== */

void RAK3172_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_lineaLen = 0;
    memset(s_lineaEnConstruccion, 0, sizeof(s_lineaEnConstruccion));

    s_colaHead = 0;
    s_colaTail = 0;
    s_colaCount = 0;

    s_comandoEnCurso = false;
    s_ultimoResultado = RAK3172_OK;
    s_hayRespuestaNueva = false;
    memset(s_ultimaRespuesta, 0, sizeof(s_ultimaRespuesta));

    /* Antes de arrancar la recepción, limpiar cualquier flag de error
     * (Overrun/Framing/Noise) y vaciar el registro de datos. Esto es
     * importante porque el RAK3172 NO se resetea junto con el G431 --
     * si había actividad en la línea justo en el instante del reset,
     * puede quedar un error de recepción "colgado" que, combinado con
     * Overrun/DMA-on-RX-Error habilitados, entra en un ciclo de
     * reintento continuo antes de que la aplicación tenga oportunidad
     * de intervenir. */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    volatile uint32_t dummy = s_huart->Instance->RDR; /* vaciar el registro de datos */
    (void)dummy;

    /* Arranca la recepción continua por DMA con detección de línea
     * inactiva (IDLE). El callback HAL_UARTEx_RxEventCallback debe
     * reenviar el evento a RAK3172_RxEventCallback() cuando
     * huart->Instance == LPUART1 (ver ejemplo en rak3172.h). */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, RAK3172_RX_BUFFER_SIZE);
    /* __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT); -- causaba HardFault:
     * s_huart->hdmarx llegaba NULL en este punto. No es crítico (solo evita
     * el evento de "medio buffer" del DMA), así que se quita por ahora. */
}

void RAK3172_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != s_huart->Instance) {
        return;
    }

    /* En modo Normal (no circular), 'Size' SÍ es la cantidad de bytes
     * nuevos recibidos en esta transacción (a diferencia de modo
     * circular, donde era una posición absoluta). Se procesan desde
     * el inicio del buffer cada vez. */
    for (uint16_t i = 0; i < Size; i++) {
        char c = (char)s_rxDmaBuffer[i];

        if (c == '\r') {
            continue; /* ignorar CR, solo nos importa el LF como fin de línea */
        }

        if (c == '\n') {
            if (s_lineaLen > 0) {
                RAK3172_EncolarLinea(s_lineaEnConstruccion, s_lineaLen);
                s_lineaLen = 0;
            }
            continue;
        }

        if (s_lineaLen < (RAK3172_RX_BUFFER_SIZE - 1)) {
            s_lineaEnConstruccion[s_lineaLen++] = c;
        }
    }

    /* Limpiar posibles flags de error acumulados antes de rearmar,
     * para no heredar un estado de error de la transacción anterior. */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);

    /* Modo Normal: hay que rearmar la recepción explícitamente cada
     * vez, a diferencia del modo Circular que se rearma solo. */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, RAK3172_RX_BUFFER_SIZE);
}

void RAK3172_Update(void)
{
    /* --- 1. Procesar líneas completas encoladas por el callback --- */
    while (s_colaCount > 0) {
        __disable_irq();
        char linea[RAK3172_RX_BUFFER_SIZE];
        strncpy(linea, s_colaLineas[s_colaHead], RAK3172_RX_BUFFER_SIZE);
        s_colaHead = (s_colaHead + 1) % RAK3172_MAX_LINEAS_PENDIENTES;
        s_colaCount--;
        __enable_irq();

        RAK3172_ProcesarLinea(linea);
    }

    /* --- 2. Timeout de comando AT en curso --- */
    if (s_comandoEnCurso) {
        uint32_t transcurrido = HAL_GetTick() - s_comandoTickInicio;
        if (transcurrido > s_comandoTimeoutMs) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_TIMEOUT;
        }
    }

    /* --- 3. Reintento de Application ACK pendiente --- */
    if (s_ackPendiente && RAK3172_ComandoListo()) {
        if (RAK3172_EnviarAckInterno(s_ackId, s_ackStatus, s_ackValorRaw)) {
            s_ackPendiente = false;
        } else if ((HAL_GetTick() - s_ackTickInicio) > RAK3172_ACK_REINTENTO_TIMEOUT_MS) {
            /* Se agotó el tiempo de reintento -- se descarta el ACK
             * para no quedar atascado reintentando indefinidamente.
             * El parámetro en sí ya se aplicó correctamente; solo se
             * pierde esta confirmación puntual. */
            printf("RAK3172: Application ACK (ID=%u) descartado tras reintentar %lums sin éxito\r\n",
                   s_ackId, (unsigned long)RAK3172_ACK_REINTENTO_TIMEOUT_MS);
            s_ackPendiente = false;
        }
    }
}

bool RAK3172_EnviarComandoAT(const char *comando)
{
    if (s_comandoEnCurso) {
        return false; /* RAK3172_BUSY -- espera a que termine el anterior */
    }

    char buffer[RAK3172_TX_BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "%s\r\n", comando);
    if (len <= 0 || (uint32_t)len >= sizeof(buffer)) {
        return false; /* comando demasiado largo */
    }

    s_comandoEnCurso = true;
    s_comandoTickInicio = HAL_GetTick();
    s_comandoTimeoutMs = RAK3172_TIMEOUT_DEFAULT_MS;

    /* Transmisión bloqueante: los comandos AT son cortos (unas pocas
     * decenas de bytes), igual que el patrón ya usado en __io_putchar
     * para el printf de depuración. */
    HAL_UART_Transmit(s_huart, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);

    return true;
}

bool RAK3172_ComandoListo(void)
{
    return !s_comandoEnCurso;
}

RAK3172_Resultado_t RAK3172_GetUltimoResultado(void)
{
    return s_ultimoResultado;
}

bool RAK3172_GetUltimaRespuesta(char *destino, uint32_t tamDestino)
{
    if (!s_hayRespuestaNueva) {
        return false;
    }
    strncpy(destino, s_ultimaRespuesta, tamDestino - 1);
    destino[tamDestino - 1] = '\0';
    s_hayRespuestaNueva = false;
    return true;
}

bool RAK3172_EstaUnido(void)
{
    return s_estaUnido;
}

bool RAK3172_Join(void)
{
    s_estaUnido = false;
    /* Formato recomendado por RAK (ver AT Command Migration Guide):
     * join=1, auto-join=1 (el propio módulo reintenta unirse solo,
     * de forma persistente -- RUI3 guarda esta configuración en
     * memoria permanente, así que incluso ante un reset/pérdida de
     * energía el RAK3172 puede seguir intentando sin depender de que
     * el G431 vuelva a mandar este comando), intervalo de
     * reintento=10s, máximo 8 intentos. */
    return RAK3172_EnviarComandoAT("AT+JOIN=1:1:10:8");
}

static bool RAK3172_EnviarAckInterno(uint8_t id, uint8_t status, uint16_t valorRaw)
{
    /* Protocolo Quick-Set (Application ACK), 4 bytes:
     * [PARAMETER_ID][STATUS][VALUE_H][VALUE_L] */
    char comando[RAK3172_TX_BUFFER_SIZE];
    snprintf(comando, sizeof(comando), "AT+SEND=%u:%02X%02X%02X%02X",
              (unsigned int)RAK3172_FPORT_ACK,
              id, status,
              (unsigned int)((valorRaw >> 8) & 0xFFU),
              (unsigned int)(valorRaw & 0xFFU));

    return RAK3172_EnviarComandoAT(comando);
}

bool RAK3172_EnviarAck(uint8_t id, uint8_t status, uint16_t valorRaw)
{
    if (RAK3172_ComandoListo() && RAK3172_EnviarAckInterno(id, status, valorRaw)) {
        return true; /* se pudo mandar de inmediato */
    }

    /* Canal ocupado (u otro fallo puntual) -- se guarda para
     * reintentar automáticamente desde RAK3172_Update(). Si ya había
     * un ACK pendiente sin enviar, se sobreescribe con este (caso
     * raro de dos downlinks casi simultáneos); se prioriza no
     * bloquear el sistema sobre no perder ningún ACK en ese
     * escenario extremo. */
    s_ackPendiente = true;
    s_ackId = id;
    s_ackStatus = status;
    s_ackValorRaw = valorRaw;
    s_ackTickInicio = HAL_GetTick();

    return false;
}

bool RAK3172_EnviarRPM(float rpm)
{
    /* Codificación simple: RPM x10 como entero de 16 bits sin signo,
     * en hexadecimal de 4 dígitos. Ej: 2783.2 RPM -> 27832 -> "6CB8".
     * Ajustar si se define un formato de payload distinto del lado
     * del servidor/decoder de la aplicación LoRaWAN. */
    if (rpm < 0.0f) {
        rpm = 0.0f;
    }
    if (rpm > 6553.5f) {
        rpm = 6553.5f; /* techo del rango representable en 16 bits x10 */
    }

    uint16_t valorEscalado = (uint16_t)(rpm * 10.0f + 0.5f);

    char comando[RAK3172_TX_BUFFER_SIZE];
    snprintf(comando, sizeof(comando), "AT+SEND=%u:%04X",
              (unsigned int)RAK3172_FPORT_UPLINK_RPM, valorEscalado);

    return RAK3172_EnviarComandoAT(comando);
}

/* ==================== FUNCIONES PRIVADAS ==================== */

static void RAK3172_EncolarLinea(const char *linea, uint16_t len)
{
    if (s_colaCount >= RAK3172_MAX_LINEAS_PENDIENTES) {
        return; /* cola llena -- se descarta la línea más antigua no leída */
    }

    uint16_t copiar = (len < (RAK3172_RX_BUFFER_SIZE - 1)) ? len : (RAK3172_RX_BUFFER_SIZE - 1);
    memcpy(s_colaLineas[s_colaTail], linea, copiar);
    s_colaLineas[s_colaTail][copiar] = '\0';

    s_colaTail = (s_colaTail + 1) % RAK3172_MAX_LINEAS_PENDIENTES;
    s_colaCount++;
}

static void RAK3172_ProcesarLinea(const char *linea)
{
    if (strcmp(linea, "+EVT:JOINED") == 0) {
        s_estaUnido = true;
        return;
    }

    if (strncmp(linea, "+EVT:", 5) == 0) {
        RAK3172_ProcesarEventoDownlink(linea);
        return;
    }

    if (s_comandoEnCurso) {
        if (strcmp(linea, "OK") == 0) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_OK;
        } else if (strncmp(linea, "ERROR", 5) == 0) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_ERROR;
        } else if (linea[0] != '\0') {
            /* Línea de datos previa al OK, ej. respuesta de AT+VER=? */
            strncpy(s_ultimaRespuesta, linea, sizeof(s_ultimaRespuesta) - 1);
            s_ultimaRespuesta[sizeof(s_ultimaRespuesta) - 1] = '\0';
            s_hayRespuestaNueva = true;
        }
    }
    /* Líneas fuera de un comando en curso y que no son "+EVT:" se
     * ignoran (ej. banners de arranque del módulo). */
}

static void RAK3172_ProcesarEventoDownlink(const char *linea)
{
    /* Formato REAL confirmado en campo (firmware RUI_4.2.4_RAK3172-E):
     *
     *   +EVT:RX_1:-28:8:UNICAST:2:00000710
     *        │    │   │    │    │    │
     *        │    │   │    │    │    └─ payload en hex: [ID][valor...]
     *        │    │   │    │    └────── FPort (RAK3172_FPORT_PARAMETRO)
     *        │    │   │    └─────────── tipo (UNICAST)
     *        │    │   └──────────────── SNR
     *        │    └──────────────────── RSSI
     *        └───────────────────────── ventana de recepción (RX_1/RX_2)
     *
     * El payload ya NO se interpreta como "todo es un solo número" --
     * el PRIMER byte es el ID del parámetro (ver calibracion_flash.h,
     * CALIB_ID_*), y los bytes restantes son el valor de ese parámetro
     * en el formato/escala que le corresponda. Toda esa lógica vive
     * en CalibFlash_ProcesarParametro(), este módulo solo separa los
     * bytes y se los pasa.
     */
    char ventanaRx[16];
    int rssi;
    int snr;
    char tipo[16];
    unsigned int fport = 0;
    char payloadHex[RAK3172_RX_BUFFER_SIZE];

    int camposLeidos = sscanf(linea, "+EVT:%15[^:]:%d:%d:%15[^:]:%u:%63s",
                               ventanaRx, &rssi, &snr, tipo, &fport, payloadHex);

    if (camposLeidos != 6) {
        return; /* no es una línea de downlink con payload, se ignora */
    }

    if (fport != RAK3172_FPORT_PARAMETRO) {
        return; /* downlink en un FPort que no es el de parámetros -- se ignora */
    }

    uint8_t bytesPayload[16];
    uint8_t cantidadBytes = 0;

    if (!RAK3172_HexAAlBytes(payloadHex, bytesPayload, sizeof(bytesPayload), &cantidadBytes)) {
        return; /* payload no era hexadecimal válido */
    }

    if (cantidadBytes < 1) {
        return; /* no hay ni siquiera el byte de ID */
    }

    uint8_t idParametro = bytesPayload[0];
    const uint8_t *datosValor = &bytesPayload[1];
    uint8_t longitudValor = (uint8_t)(cantidadBytes - 1U);

    /* El motor "opera" si el tacómetro NO lo reporta detenido -- se
     * consulta aquí (en vez de que calibracion_flash.c dependa
     * directamente de tacometro.h) para mantener a calibracion_flash
     * desacoplado del módulo de medición de RPM. */
    bool motorOperando = !Tacometro_EstaDetenido();

    uint16_t valorAplicadoRaw = 0U;
    CalibFlash_ProtocoloStatus_t status = CalibFlash_ProcesarParametroConEstado(
        idParametro, datosValor, longitudValor, motorOperando, &valorAplicadoRaw);

    printf("Downlink ID=%u (%u bytes de valor) -> STATUS=%d, valor vigente=0x%04X\r\n",
           idParametro, longitudValor, (int)status, valorAplicadoRaw);

    /* Application ACK del protocolo Quick-Set: confirma al servidor
     * qué quedó realmente vigente, sin importar si se aplicó o se
     * rechazó el valor solicitado. */
    RAK3172_EnviarAck(idParametro, (uint8_t)status, valorAplicadoRaw);
}

static bool RAK3172_HexAAlBytes(const char *hex, uint8_t *bytesSalida, uint8_t maxBytes, uint8_t *cantidadBytes)
{
    size_t longitudHex = strlen(hex);

    /* Se espera un número par de caracteres hex (2 por byte). Si el
     * payload real trae longitud impar (poco común, pero posible si
     * el servidor no rellena con cero a la izquierda), se rechaza en
     * vez de intentar adivinar el alineamiento. */
    if (longitudHex == 0 || (longitudHex % 2) != 0) {
        return false;
    }

    uint8_t cantidad = (uint8_t)(longitudHex / 2);
    if (cantidad > maxBytes) {
        return false; /* payload más grande de lo que se esperaba */
    }

    for (uint8_t i = 0; i < cantidad; i++) {
        char parBytes[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
        char *fin = NULL;
        unsigned long valor = strtoul(parBytes, &fin, 16);
        if (fin != parBytes + 2) {
            return false; /* carácter no hexadecimal encontrado */
        }
        bytesSalida[i] = (uint8_t)valor;
    }

    *cantidadBytes = cantidad;
    return true;
}
