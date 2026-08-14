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

/* ==================== API PÚBLICA ==================== */

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t canal)
{
    s_htim = htim;
    s_canal = canal;
    s_pulsoActualUs = 0;
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

uint16_t Servo_GetPulsoActualUs(void)
{
    return s_pulsoActualUs;
}
