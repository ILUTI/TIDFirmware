/**
 * @file    servo.h
 * @brief   Control del servo del acelerador (MG996R/análogo) vía PWM
 *          (TIM3_CH3, PB0).
 *
 * NOTA: se usa PB0 (TIM3_CH3) en vez de PA6 (TIM3_CH1) porque en el
 * Nucleo-32 (NUCLEO-G431KB), el header físico donde vive PA6 comparte
 * pin con PA15 mediante el solder bridge SB3, cuyo estado por defecto
 * de fábrica enruta ese header a PA15, no a PA6 (ver UM2397, Table 9).
 * PB0/TIM3_CH3 evita ese problema sin necesidad de resoldar nada.
 *
 * El timer ya está configurado (MX_TIM3_Init, generado por CubeMX) a
 * 1 tick = 1µs, periodo de 20,000 ticks = 20ms (50Hz) -- el estándar
 * de servos de RC. Este módulo solo escribe el registro de
 * comparación (ancho de pulso), en microsegundos directos, con
 * recorte de seguridad contra los límites mecánicos configurados
 * (SERVO_PULSO_MIN/MAX en calibracion_flash.h).
 *
 * Uso típico en main.c:
 *   Servo_Init(&htim3, TIM_CHANNEL_3);
 *   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
 *   ...
 *   Servo_SetPulsoUs(1500);   // punto medio, prueba en banco
 *
 * IMPORTANTE (prueba en banco antes de conectar a la varilla real):
 *   Antes de conectar el servo a la varilla del acelerador, pruébalo
 *   suelto, confirmando visualmente que los valores de
 *   SERVO_PULSO_MIN/MAX corresponden a los topes mecánicos deseados
 *   y que el servo no "tiembla" ni se sale de rango.
 */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ==================== API PÚBLICA ==================== */

/**
 * Inicializa el módulo. Debe llamarse una vez en main(), después de
 * que el timer ya fue inicializado por MX_TIM3_Init() (generado por
 * CubeMX). No arranca el PWM por sí solo -- llamar
 * HAL_TIM_PWM_Start(htim, canal) por separado, igual que se hace con
 * el Input Capture del tacómetro.
 */
void Servo_Init(TIM_HandleTypeDef *htim, uint32_t canal);

/**
 * Mueve el servo al ancho de pulso indicado, en microsegundos.
 * El valor se recorta (clamp) contra
 * CalibFlash_GetServoPulsoMinUs()/GetServoPulsoMaxUs() antes de
 * aplicarse -- nunca se manda un pulso fuera de esos límites, sin
 * importar qué tan fuera de rango venga el valor solicitado.
 *
 * @param microsegundos  Ancho de pulso deseado, en µs.
 * @return El valor REAL aplicado (después del recorte), útil para
 *         que el llamador sepa si su solicitud se ajustó.
 */
uint16_t Servo_SetPulsoUs(uint16_t microsegundos);

/** Último ancho de pulso realmente aplicado (después de cualquier
 * recorte por límites). Útil para telemetría/depuración. */
uint16_t Servo_GetPulsoActualUs(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
