/**
 * @file    servo.c
 * @brief   Implementación del control del servo vía PWM.
 */

#include "servo.h"
#include "calibracion_flash.h"

/* ==================== ESTADO INTERNO ==================== */

static TIM_HandleTypeDef *s_htim = NULL;
static uint32_t s_canal = 0;
static uint16_t s_pulsoActualUs = 0;
static uint32_t s_ultimoMovimientoMs = 0;

/* ==================== API PÚBLICA ==================== */

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t canal)
{
    s_htim = htim;
    s_canal = canal;
    s_pulsoActualUs = 0;
    s_ultimoMovimientoMs = HAL_GetTick();
}

uint16_t Servo_SetPulsoUs(uint16_t microsegundos)
{
    uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
    uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();

    uint16_t valorRecortado = microsegundos;
    if (valorRecortado < minimo) {
        valorRecortado = minimo;
    }
    if (valorRecortado > maximo) {
        valorRecortado = maximo;
    }

    __HAL_TIM_SET_COMPARE(s_htim, s_canal, valorRecortado);
    s_pulsoActualUs = valorRecortado;

    return valorRecortado;
}

uint16_t Servo_MoverHacia(uint16_t destinoUs)
{
    uint32_t ahora = HAL_GetTick();
    uint32_t transcurridoMs = ahora - s_ultimoMovimientoMs;
    s_ultimoMovimientoMs = ahora;

    int32_t pasoMaximo = (int32_t)((SERVO_VELOCIDAD_MAX_US_S * transcurridoMs) / 1000U);
    int32_t diferencia = (int32_t)destinoUs - (int32_t)s_pulsoActualUs;

    int32_t siguiente;
    if (diferencia > pasoMaximo) {
        siguiente = (int32_t)s_pulsoActualUs + pasoMaximo;
    } else if (diferencia < -pasoMaximo) {
        siguiente = (int32_t)s_pulsoActualUs - pasoMaximo;
    } else {
        siguiente = destinoUs;
    }

    return Servo_SetPulsoUs((uint16_t)siguiente);
}

uint16_t Servo_GetPulsoActualUs(void)
{
    return s_pulsoActualUs;
}
