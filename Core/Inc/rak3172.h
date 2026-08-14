/**
 * @file    rak3172.h
 * @brief   Módulo de comunicación con el módulo RAK3172 (LoRaWAN) vía
 *          comandos AT, sobre LPUART1 (PA2=TX, PA3=RX).
 *
 * Responsabilidades:
 *   - Enviar comandos AT y esperar su respuesta ("OK", "ERROR", o el
 *     valor consultado) de forma no bloqueante, integrada al loop
 *     principal (Tacometro_Update()/Modbus_Update()).
 *   - Recibir y parsear eventos asíncronos "+EVT:" que el módulo manda
 *     por su cuenta (ej. downlink recibido), sin que el host los haya
 *     solicitado.
 *   - Exponer una función simple para mandar la RPM actual como uplink.
 *
 * Uso típico en main.c:
 *   RAK3172_Init(&hlpuart1);
 *   ...
 *   while (1) {
 *       RAK3172_Update();
 *       if (debe_mandar_uplink) {
 *           RAK3172_EnviarRPM(Tacometro_GetRPMFiltrada());
 *       }
 *   }
 *
 * Y en el callback global de HAL (en main.c, USER CODE):
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *       ...
 *       else if (huart->Instance == LPUART1) {
 *           RAK3172_RxEventCallback(huart, Size);
 *       }
 *   }
 */

#ifndef RAK3172_H
#define RAK3172_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACIÓN AJUSTABLE ==================== */

/* Tamaño del buffer circular de recepción por DMA (debe coincidir con
 * el tamaño de bufferRAKRx que se pasa a HAL_UARTEx_ReceiveToIdle_DMA
 * en main.c). */
#define RAK3172_RX_BUFFER_SIZE        128U

/* Tamaño máximo de un comando AT a transmitir (incluye "AT+...\r\n"). */
#define RAK3172_TX_BUFFER_SIZE        96U

/* Timeout por defecto esperando respuesta "OK"/"ERROR" a un comando AT
 * enviado (ms). Comandos de join a la red pueden tardar más -- se
 * puede pasar un timeout distinto por comando si se requiere. */
#define RAK3172_TIMEOUT_DEFAULT_MS    2000U

/* FPort usado para el uplink de RPM (elige un valor y mantenlo
 * consistente con lo que se configure del lado del servidor/decoder). */
#define RAK3172_FPORT_UPLINK_RPM      1U

/* FPort usado para downlinks de configuración de parámetros.
 * El payload en este FPort sigue el esquema [ID de 1 byte][valor],
 * donde ID es uno de los CALIB_ID_* definidos en calibracion_flash.h.
 * Toda la lógica de qué ID corresponde a qué parámetro vive en
 * calibracion_flash.c -- este módulo solo separa los bytes. */
#define RAK3172_FPORT_PARAMETRO       2U

/* FPort usado para el Application ACK que confirma el resultado de
 * procesar un downlink de parámetro (protocolo Quick-Set acordado).
 * Payload de 4 bytes: [PARAMETER_ID][STATUS][VALUE_H][VALUE_L]. */
#define RAK3172_FPORT_ACK             3U

/* ==================== TIPOS ==================== */

typedef enum {
    RAK3172_OK = 0,
    RAK3172_ERROR,
    RAK3172_TIMEOUT,
    RAK3172_BUSY   /* ya hay un comando en curso esperando respuesta */
} RAK3172_Resultado_t;

/* ==================== API PÚBLICA ==================== */

/**
 * true si se recibió el evento "+EVT:JOINED" (el módulo se unió
 * exitosamente a la red LoRaWAN). Se pone en false de nuevo al llamar
 * RAK3172_Join().
 */
bool RAK3172_EstaUnido(void);

/**
 * Inicia el proceso de join a la red LoRaWAN, usando el comando
 * recomendado por RAK (join=1, auto=0, intervalo=10s, 8 intentos).
 * No bloquea -- el resultado se refleja después en RAK3172_EstaUnido().
 */
bool RAK3172_Join(void);

/**
 * Inicializa el módulo. Llamar una vez en main(), después de que
 * MX_LPUART1_UART_Init() ya corrió y de haber arrancado la recepción
 * con HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, bufferRAKRx, ...).
 *
 * @param huart   Puntero al handle de LPUART1 (&hlpuart1 en main.c).
 */
void RAK3172_Init(UART_HandleTypeDef *huart);

/**
 * Debe llamarse periódicamente desde el loop principal. Atiende
 * timeouts de comandos AT pendientes de respuesta. No bloquea.
 */
void RAK3172_Update(void);

/**
 * Debe llamarse desde el callback global HAL_UARTEx_RxEventCallback()
 * en main.c, cuando el evento provenga de LPUART1. Es seguro llamarlo
 * solo cuando huart->Instance == LPUART1 (ver ejemplo de uso arriba).
 */
void RAK3172_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * Envía un comando AT crudo (sin "\r\n", se agrega internamente) y
 * regresa inmediatamente -- no bloquea esperando la respuesta.
 * La respuesta se procesa de forma asíncrona; usar
 * RAK3172_HayRespuestaPendiente()/RAK3172_GetUltimoResultado() si se
 * necesita conocer el resultado desde el loop principal.
 *
 * @param comando   Comando AT sin terminador, ej. "AT+VER=?".
 * @return true si se pudo encolar/transmitir, false si ya hay un
 *         comando en curso (RAK3172_BUSY) o el comando es muy largo.
 */
bool RAK3172_EnviarComandoAT(const char *comando);

/**
 * true si el último comando enviado con RAK3172_EnviarComandoAT() ya
 * recibió respuesta (OK/ERROR/timeout) y no hay uno en curso.
 */
bool RAK3172_ComandoListo(void);

/** Resultado del último comando AT completado. */
RAK3172_Resultado_t RAK3172_GetUltimoResultado(void);

/**
 * Copia la última línea de respuesta cruda (por ejemplo el valor
 * devuelto por "AT+VER=?") a un buffer del llamador.
 *
 * @param destino     Buffer donde copiar.
 * @param tamDestino  Tamaño del buffer destino.
 * @return true si había una respuesta disponible para copiar.
 */
bool RAK3172_GetUltimaRespuesta(char *destino, uint32_t tamDestino);

/**
 * Arma y envía el uplink de RPM actual usando AT+SEND, en el FPort
 * definido por RAK3172_FPORT_UPLINK_RPM. Codifica la RPM como entero
 * sin signo de 16 bits escalado x10 (ej. 2783.2 RPM -> 27832 -> hex
 * "6CB8"), para evitar la complejidad de mandar floats en hex.
 *
 * @param rpm   RPM a enviar (típicamente Tacometro_GetRPMFiltrada()).
 * @return true si se pudo encolar el comando AT+SEND correspondiente.
 */
bool RAK3172_EnviarRPM(float rpm);

/**
 * Arma y envía el Application ACK del protocolo Quick-Set (4 bytes:
 * [PARAMETER_ID][STATUS][VALUE_H][VALUE_L]) en el FPort
 * RAK3172_FPORT_ACK, confirmando el resultado de procesar un
 * downlink de parámetro.
 *
 * Si el canal AT está ocupado en el momento de la llamada (por
 * ejemplo, coincide con el uplink periódico de RPM), el ACK se
 * encola internamente y RAK3172_Update() lo reintenta automáticamente
 * en cuanto el canal se libere, hasta un máximo de
 * RAK3172_ACK_REINTENTO_TIMEOUT_MS -- no hace falta que el llamador
 * reintente por su cuenta.
 *
 * @param id            ID del parámetro procesado.
 * @param status        Uno de los CalibFlash_ProtocoloStatus_t.
 * @param valorRaw      Valor de 2 bytes (big-endian) que quedó
 *                       vigente, tal como lo entrega
 *                       CalibFlash_ProcesarParametroConEstado().
 * @return true si se pudo mandar de inmediato. false si quedó
 *         encolado para reintento (no es necesariamente un error).
 */
bool RAK3172_EnviarAck(uint8_t id, uint8_t status, uint16_t valorRaw);

#ifdef __cplusplus
}
#endif

#endif /* RAK3172_H */
