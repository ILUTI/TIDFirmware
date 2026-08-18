/**
 * @file    gps.h
 * @brief   Módulo de lectura de posición GPS/GNSS del módulo SIM7600X,
 *          vía sentencias NMEA en flujo continuo sobre USART2 (PA2=TX,
 *          PA3=RX).
 *
 * Diseño de recepción (modo Circular + IDLE):
 *   A diferencia de rak3172.c (modo Normal, comando/respuesta), acá se
 *   usa DMA en modo Circular -- el buffer se rearma solo, y el evento
 *   de línea inactiva (IDLE) dispara el procesamiento de las sentencias
 *   NMEA que el módulo manda por su cuenta de forma continua (no hay
 *   "comando" que esperar como con el RAK3172).
 *
 * Uso típico en main.c:
 *   GPS_Init(&huart2);
 *   // Habilitar salida NMEA continua en el módulo -- comando SIN
 *   // CONFIRMAR EN CAMPO todavía, ver advertencia en gps.c:
 *   GPS_EnviarComandoAT("AT+CGPS=1");
 *   ...
 *   while (1) {
 *       GPS_Update();
 *       if (GPS_TieneFix()) {
 *           latitud = GPS_GetLatitud();
 *           longitud = GPS_GetLongitud();
 *       }
 *   }
 *
 * Y en el callback global de HAL (en main.c, USER CODE):
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *       ...
 *       else if (huart->Instance == USART2) {
 *           GPS_RxEventCallback(huart, Size);
 *       }
 *   }
 *
 * ⚠️ USART2 hoy también es el destino de __io_putchar() (printf de
 * depuración, ver main.c). Mientras eso no se mueva a otro UART (la
 * idea planteada es la VCP del ST-Link), cualquier printf() de la
 * aplicación sale por el mismo cable que espera el SIM7600X y se
 * interpretaría como basura -- no habilitar GPS_Init() en main.c hasta
 * resolver ese conflicto.
 */

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACIÓN AJUSTABLE ==================== */

/* Tamaño del buffer circular de recepción por DMA. A 115200 baud, una
 * ráfaga típica de sentencias NMEA (RMC+GGA+GSA+3xGSV+VTG+GLL) de un
 * receptor GNSS puede rondar 400-500 bytes por ciclo de 1 Hz -- 512
 * deja margen sin desperdiciar demasiada RAM. Si se ven sentencias
 * truncadas/corruptas en GPS_Update(), subir este valor primero. */
#define GPS_RX_BUFFER_SIZE            512U

/* Longitud máxima de una sentencia NMEA reconstruida (el estándar NMEA
 * 0183 limita a 82 caracteres incluyendo '$' y checksum; 96 deja
 * margen). */
#define GPS_LINEA_MAX_LEN             96U

/* Tamaño máximo de un comando AT a transmitir hacia el módulo. */
#define GPS_TX_BUFFER_SIZE            64U

/* ==================== API PÚBLICA ==================== */

/**
 * Inicializa el módulo y arranca la recepción continua por DMA en modo
 * Circular con detección de línea inactiva (IDLE). Llamar una vez en
 * main(), después de que MX_USART2_UART_Init() ya corrió.
 *
 * No envía ningún comando AT por sí sola -- habilitar la salida NMEA
 * del módulo (ver advertencia arriba sobre el comando aún sin
 * confirmar) es responsabilidad del llamador, vía GPS_EnviarComandoAT().
 *
 * @param huart   Puntero al handle de USART2 (&huart2 en main.c).
 */
void GPS_Init(UART_HandleTypeDef *huart);

/**
 * Debe llamarse periódicamente desde el loop principal. Parsea las
 * sentencias NMEA completas encoladas por el callback (actualmente
 * solo $__RMC -- fix, latitud, longitud). No bloquea.
 */
void GPS_Update(void);

/**
 * Debe llamarse desde el callback global HAL_UARTEx_RxEventCallback()
 * en main.c, cuando el evento provenga de USART2. Es seguro llamarlo
 * siempre; internamente valida que la instancia coincida con la
 * pasada a GPS_Init().
 */
void GPS_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * Envía un comando AT crudo al módulo (se agrega "\r\n" internamente),
 * de forma bloqueante -- igual que RAK3172_EnviarComandoAT(), pero acá
 * no hay que esperar una respuesta "OK"/"ERROR" explícita porque el
 * flujo NMEA no sigue ese protocolo de comando/respuesta.
 *
 * @param comando   Comando AT sin terminador, ej. "AT+CGPS=1".
 * @return true si se pudo transmitir, false si el comando es muy largo.
 */
bool GPS_EnviarComandoAT(const char *comando);

/**
 * true si la última sentencia $__RMC procesada trae fix válido (campo
 * de estado 'A'). Se pone en false de nuevo si llega un RMC con
 * estado 'V' (sin fix) -- pero la última posición conocida (lat/lon)
 * se conserva, no se resetea a 0.
 */
bool GPS_TieneFix(void);

/** Última latitud válida conocida (grados decimales, + = Norte). 0.0f si nunca hubo fix. */
float GPS_GetLatitud(void);

/** Última longitud válida conocida (grados decimales, + = Este). 0.0f si nunca hubo fix. */
float GPS_GetLongitud(void);

/**
 * HAL_GetTick() del momento del último fix válido recibido. 0 si nunca
 * hubo fix -- el llamador puede compararlo contra HAL_GetTick() actual
 * para decidir si la posición ya es demasiado vieja y usar un
 * fallback (ver LATITUD_FIJA/LONGITUD_FIJA en main.c).
 */
uint32_t GPS_GetUltimoFixTickMs(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
