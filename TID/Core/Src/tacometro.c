/**
 * @file    tacometro.c
 * @brief   Implementación del módulo de medición de RPM vía Input Capture.
 */

#include "tacometro.h"

/* ==================== ESTADO INTERNO (privado a este módulo) ==================== */

static TIM_HandleTypeDef *s_htim   = NULL;
static uint32_t            s_canal  = 0;

/* Todo lo que se comparte entre el contexto de interrupción (callback)
 * y el contexto normal (Update/getters) debe ser volatile. */
static volatile uint32_t s_ultimoPeriodoUs    = 0;     /* último período válido, en µs */
static volatile bool     s_primerFlancoHecho  = false; /* aún no hay período calculable */
static volatile bool     s_hayPeriodoNuevo    = false; /* flag: Update() debe procesarlo */
static volatile uint32_t s_contadorRuido      = 0;

static volatile uint32_t s_ultimoTickCapturaMs = 0; /* HAL_GetTick() de la última captura válida */

static bool  s_motorDetenido   = true;
static float s_rpmInstantanea  = 0.0f;
static float s_rpmFiltrada     = 0.0f;
static float s_frecuenciaHz    = 0.0f;

/* ==================== API PÚBLICA ==================== */

void Tacometro_Init(TIM_HandleTypeDef *htim, uint32_t canal)
{
    s_htim  = htim;
    s_canal = canal;

    s_ultimoPeriodoUs    = 0;
    s_primerFlancoHecho  = false;
    s_hayPeriodoNuevo    = false;
    s_contadorRuido      = 0;
    s_ultimoTickCapturaMs = HAL_GetTick();

    s_motorDetenido  = true;
    s_rpmInstantanea = 0.0f;
    s_rpmFiltrada    = 0.0f;
    s_frecuenciaHz   = 0.0f;
}

void Tacometro_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != s_htim->Instance) {
        return;
    }

    /* En PWM Input mode, s_canal (el canal directo, Rising+Reset) es el
     * que dispara esta interrupción. Su par indirecto en el mismo timer
     * ya tiene, en el mismo instante, el ancho del pulso alto capturado
     * por hardware — sincronizado al mismo evento, sin riesgo de
     * desemparejamiento entre interrupciones separadas. */
    uint32_t canalActivoEsperado, canalIndirecto;
    switch (s_canal) {
        case TIM_CHANNEL_1: canalActivoEsperado = HAL_TIM_ACTIVE_CHANNEL_1; canalIndirecto = TIM_CHANNEL_2; break;
        case TIM_CHANNEL_2: canalActivoEsperado = HAL_TIM_ACTIVE_CHANNEL_2; canalIndirecto = TIM_CHANNEL_1; break;
        case TIM_CHANNEL_3: canalActivoEsperado = HAL_TIM_ACTIVE_CHANNEL_3; canalIndirecto = TIM_CHANNEL_4; break;
        case TIM_CHANNEL_4: canalActivoEsperado = HAL_TIM_ACTIVE_CHANNEL_4; canalIndirecto = TIM_CHANNEL_3; break;
        default:             canalActivoEsperado = HAL_TIM_ACTIVE_CHANNEL_CLEARED; canalIndirecto = 0; break;
    }
    if (htim->Channel != canalActivoEsperado) {
        return;
    }

    /* Gracias al Slave Mode Reset (TI1FP1), esta captura YA ES el
     * período completo en ticks de 1µs — el contador se reinicia en
     * cada flanco de subida, así que no hay que restar contra una
     * captura anterior como en el modo Input Capture simple. */
    uint32_t periodoUs    = HAL_TIM_ReadCapturedValue(htim, s_canal);
    uint32_t anchoPulsoUs = HAL_TIM_ReadCapturedValue(htim, canalIndirecto);

    if (!s_primerFlancoHecho) {
        /* Descarta la primera captura tras Init/reanudación: el
         * contador puede no estar sincronizado con el reset todavía. */
        s_primerFlancoHecho = true;
        return;
    }

    if (periodoUs < TACOMETRO_PERIODO_MINIMO_US) {
        s_contadorRuido++;
        return;
    }

    float duty = (float)anchoPulsoUs / (float)periodoUs;
    if (duty < TACOMETRO_DUTY_MINIMO || duty > TACOMETRO_DUTY_MAXIMO) {
        /* Pico angosto de disparo simétrico del clamp, no un semiciclo
         * real del H11AA1 — descartar sin mover el estado de frecuencia. */
        s_contadorRuido++;
        return;
    }

    s_ultimoPeriodoUs     = periodoUs;
    s_hayPeriodoNuevo     = true;
    s_ultimoTickCapturaMs = HAL_GetTick();
}

void Tacometro_Update(void)
{
    /* --- 1. Procesar un período nuevo, si llegó desde el último Update --- */
    if (s_hayPeriodoNuevo) {
        /* Copia atómica del valor volatile antes de usarlo, para evitar
         * que el callback lo modifique a mitad de un cálculo. En este
         * MCU (32 bits, acceso alineado) la lectura de un uint32_t ya es
         * atómica por naturaleza, pero se deshabilitan interrupciones
         * brevemente por claridad y robustez ante refactors futuros. */
        __disable_irq();
        uint32_t periodoUs = s_ultimoPeriodoUs;
        s_hayPeriodoNuevo = false;
        __enable_irq();

        s_frecuenciaHz   = (float)TACOMETRO_TICKS_POR_SEGUNDO / (float)periodoUs;
        s_rpmInstantanea = (s_frecuenciaHz * 60.0f) / TACOMETRO_PULSOS_POR_REVOLUCION;

        /* Filtro de media móvil exponencial:
         *   filtrada_nueva = alpha*instantanea + (1-alpha)*filtrada_anterior */
        s_rpmFiltrada = (TACOMETRO_ALPHA_FILTRO * s_rpmInstantanea) +
                         ((1.0f - TACOMETRO_ALPHA_FILTRO) * s_rpmFiltrada);

        s_motorDetenido = false;
    }

    /* --- 2. Detección de motor detenido (timeout) --- */
    uint32_t ahora = HAL_GetTick();
    uint32_t transcurrido = ahora - s_ultimoTickCapturaMs; /* resta unsigned, segura ante wraparound de HAL_GetTick */

    if (!s_motorDetenido && transcurrido > TACOMETRO_TIMEOUT_DETENIDO_MS) {
        s_motorDetenido  = true;
        s_rpmInstantanea = 0.0f;
        s_rpmFiltrada    = 0.0f;
        s_frecuenciaHz   = 0.0f;
        s_primerFlancoHecho = false; /* forzar nueva referencia al reanudar */
    }
}

float Tacometro_GetRPMInstantanea(void)
{
    return s_rpmInstantanea;
}

float Tacometro_GetRPMFiltrada(void)
{
    return s_rpmFiltrada;
}

float Tacometro_GetFrecuenciaHz(void)
{
    return s_frecuenciaHz;
}

bool Tacometro_EstaDetenido(void)
{
    return s_motorDetenido;
}

uint32_t Tacometro_GetContadorRuidoFiltrado(void)
{
    return s_contadorRuido;
}
