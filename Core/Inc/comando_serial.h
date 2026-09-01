/**
 * @file    comando_serial.h
 * @brief   Comandos de parámetro escritos a mano por el puerto de debug
 *          (LPUART1), como mando manual TEMPORAL mientras no hay red
 *          LoRa disponible en campo para probar downlinks reales.
 *
 * Reutiliza el mismo punto de entrada que un downlink real
 * (CalibFlash_ProcesarParametroConEstado(), el mismo que llama
 * rak3172.c en RAK3172_ProcesarEventoDownlink()) -- el comportamiento
 * (validación de rango, bloqueo por categoría con el motor operando,
 * persistencia en flash) es idéntico al de un downlink real, solo
 * cambia el transporte. A diferencia de un downlink real, esto NO
 * manda el Application ACK por LoRaWAN (RAK3172_EnviarAck) -- el
 * resultado se imprime directo al mismo puerto serial, como si fuera
 * el ACK.
 *
 * Formato de línea (escrita a mano en el monitor serie, termina al
 * mandar Enter):
 *   NOMBRE_PARAMETRO [VALOR]
 * ej.:
 *   CONTROL_HABILITADO 1
 *   SET_RPM 900
 *   RESTAURAR_DEFAULTS        (los comandos no llevan VALOR)
 *
 * Nombres válidos: los mismos (exactos, en mayúsculas) de la tabla de
 * parámetros del TID (README sección 3) y de PARAMETER_TABLE en
 * send_downlink.py -- mismo valor "humano" que se manda por MQTT (ej.
 * RPM directo, no el raw escalado).
 *
 * Quitar este módulo (y su llamada en main.c) el día que exista un
 * mando local real (pantalla/botonera), o cuando ya no se necesite
 * probar sin red LoRa en campo.
 */

#ifndef COMANDO_SERIAL_H
#define COMANDO_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/** Inicializa el módulo sobre el UART de debug ya inicializado
 * (MX_LPUART1_UART_Init(), generado por CubeMX) -- no reconfigura el
 * periférico, solo guarda el handle para sondear su bandera RXNE. */
void ComandoSerial_Init(UART_HandleTypeDef *huart);

/** Llamar en cada vuelta del loop principal. Sondea el registro de
 * datos del UART sin bloquear (no usa interrupciones ni DMA -- un
 * operador tipeando a mano nunca satura el UART); arma la línea
 * carácter por carácter con eco local, y al recibir '\r'/'\n' la
 * procesa como si fuera un downlink recién llegado. */
void ComandoSerial_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* COMANDO_SERIAL_H */
