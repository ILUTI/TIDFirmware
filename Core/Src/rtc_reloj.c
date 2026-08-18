/**
 * @file    rtc_reloj.c
 * @brief   Ver rtc_reloj.h para el diseño general.
 *
 * La conversión epoch<->calendario está implementada a mano (sin
 * time.h/gmtime de la libc) para no depender de que la newlib-nano de
 * este proyecto tenga esas funciones enlazadas -- es una decisión de
 * portabilidad, no de rendimiento (esto corre una vez cada tanto, no en
 * un lazo caliente).
 */

#include "rtc_reloj.h"

static RTC_HandleTypeDef *s_hrtc = NULL;
static bool s_sincronizado = false;

/* ==================== Conversión epoch <-> calendario (UTC puro) ==================== */

static bool EsAnioBisiesto(uint32_t anio)
{
    return (anio % 4U == 0U && (anio % 100U != 0U || anio % 400U == 0U));
}

static const uint8_t DIAS_POR_MES[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static void EpochAUtc(uint32_t epoch, uint16_t *anio, uint8_t *mes, uint8_t *dia,
                      uint8_t *hora, uint8_t *minuto, uint8_t *segundo, uint8_t *diaSemanaHal)
{
    uint32_t diasTotales = epoch / 86400UL;
    uint32_t segundosDelDia = epoch % 86400UL;

    *hora = (uint8_t)(segundosDelDia / 3600UL);
    *minuto = (uint8_t)((segundosDelDia % 3600UL) / 60UL);
    *segundo = (uint8_t)(segundosDelDia % 60UL);

    /* 1-Ene-1970 fue Jueves. HAL numera RTC_WEEKDAY_MONDAY=1 .. SUNDAY=7.
     * (diasTotales + 3) % 7 dá 0=Jueves..6=Miercoles con offset 0-based;
     * se remapea sumando 4 y llevando a rango 1-7 con base Jueves=4. */
    *diaSemanaHal = (uint8_t)(((diasTotales + 3UL) % 7UL) + 1UL);

    uint16_t anioActual = 1970;
    for (;;) {
        uint32_t diasEnEsteAnio = EsAnioBisiesto(anioActual) ? 366UL : 365UL;
        if (diasTotales < diasEnEsteAnio) break;
        diasTotales -= diasEnEsteAnio;
        anioActual++;
    }
    *anio = anioActual;

    uint8_t mesActual;
    for (mesActual = 0; mesActual < 12U; mesActual++) {
        uint32_t diasEnEsteMes = DIAS_POR_MES[mesActual];
        if (mesActual == 1U && EsAnioBisiesto(anioActual)) {
            diasEnEsteMes = 29U;
        }
        if (diasTotales < diasEnEsteMes) break;
        diasTotales -= diasEnEsteMes;
    }
    *mes = (uint8_t)(mesActual + 1U);
    *dia = (uint8_t)(diasTotales + 1U);
}

static uint32_t UtcAEpoch(uint16_t anio, uint8_t mes, uint8_t dia,
                          uint8_t hora, uint8_t minuto, uint8_t segundo)
{
    uint32_t dias = 0;
    for (uint16_t a = 1970; a < anio; a++) {
        dias += EsAnioBisiesto(a) ? 366UL : 365UL;
    }
    for (uint8_t m = 0; m < (uint8_t)(mes - 1U); m++) {
        uint32_t diasEnEsteMes = DIAS_POR_MES[m];
        if (m == 1U && EsAnioBisiesto(anio)) {
            diasEnEsteMes = 29U;
        }
        dias += diasEnEsteMes;
    }
    dias += (uint32_t)(dia - 1U);

    return (dias * 86400UL) + ((uint32_t)hora * 3600UL) + ((uint32_t)minuto * 60UL) + segundo;
}

/* ==================== API pública ==================== */

void Reloj_Init(RTC_HandleTypeDef *hrtc)
{
    s_hrtc = hrtc;
    s_sincronizado = false;
}

void Reloj_SetUnixTimeUtc(uint32_t epochUtc)
{
    if (s_hrtc == NULL) {
        return;
    }

    uint16_t anio;
    uint8_t mes, dia, hora, minuto, segundo, diaSemana;
    EpochAUtc(epochUtc, &anio, &mes, &dia, &hora, &minuto, &segundo, &diaSemana);

    RTC_TimeTypeDef sTime = {0};
    sTime.Hours = hora;
    sTime.Minutes = minuto;
    sTime.Seconds = segundo;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime(s_hrtc, &sTime, RTC_FORMAT_BIN);

    RTC_DateTypeDef sDate = {0};
    sDate.WeekDay = diaSemana;
    sDate.Month = mes;
    sDate.Date = dia;
    sDate.Year = (uint8_t)(anio - 2000U); /* RTC_DateTypeDef.Year es offset desde 2000 */
    HAL_RTC_SetDate(s_hrtc, &sDate, RTC_FORMAT_BIN);

    s_sincronizado = true;
}

void Reloj_SetHoraUtc(uint16_t anio, uint8_t mes, uint8_t dia,
                       uint8_t hora, uint8_t minuto, uint8_t segundo)
{
    if (s_hrtc == NULL) {
        return;
    }

    /* Mismo procedimiento que Reloj_SetUnixTimeUtc(), pero sin pasar
     * por EpochAUtc() -- los campos de calendario ya vienen sueltos
     * (ej. parseados de "AT+LTIME=15h08m55s on 08/17/2026"). */
    RTC_TimeTypeDef sTime = {0};
    sTime.Hours = hora;
    sTime.Minutes = minuto;
    sTime.Seconds = segundo;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime(s_hrtc, &sTime, RTC_FORMAT_BIN);

    RTC_DateTypeDef sDate = {0};
    /* WeekDay no se puede derivar sin convertir a epoch -- se deja en
     * lunes como placeholder. Ningun consumidor de este proyecto lee
     * el dia de la semana del RTC (Reloj_GetUnixTimeUtc() lo ignora
     * por completo), asi que no afecta nada real. */
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = mes;
    sDate.Date = dia;
    sDate.Year = (uint8_t)(anio - 2000U);
    HAL_RTC_SetDate(s_hrtc, &sDate, RTC_FORMAT_BIN);

    s_sincronizado = true;
}

bool Reloj_EstaSincronizado(void)
{
    return s_sincronizado;
}uint32_t Reloj_GetUnixTimeUtc(void)
{
    if (s_hrtc == NULL) {
        return 0;
    }

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    /* El HAL exige leer SIEMPRE Time y luego Date, en ese orden, para
     * destrabar el registro sombra del RTC -- si se omite GetDate justo
     * después de GetTime, la próxima lectura de Time puede quedar
     * congelada (comportamiento documentado del periférico RTC de
     * STM32, no un capricho de esta implementación). */
    HAL_RTC_GetTime(s_hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(s_hrtc, &sDate, RTC_FORMAT_BIN);

    return UtcAEpoch((uint16_t)(2000U + sDate.Year), sDate.Month, sDate.Date,
                      sTime.Hours, sTime.Minutes, sTime.Seconds);
}

uint32_t Reloj_GetUnixTimeLocal(void)
{
    return Reloj_GetUnixTimeUtc() + GUATEMALA_UTC_OFFSET_SEGUNDOS;
}

void Reloj_CargarHoraAproximada(uint32_t epochUtc)
{
    if (s_hrtc == NULL || epochUtc == 0U) {
        return;
    }

    bool sincronizadoPrevio = s_sincronizado; /* preservar: esto NO cuenta como sync real */
    Reloj_SetUnixTimeUtc(epochUtc);
    s_sincronizado = sincronizadoPrevio; /* deshacer el "true" que puso SetUnixTimeUtc */
}
