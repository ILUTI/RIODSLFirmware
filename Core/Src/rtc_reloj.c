/**
 * @file    rtc_reloj.c
 * @brief   Ver rtc_reloj.h para el diseño general.
 *
 * La conversión calendario->epoch está implementada a mano (sin
 * time.h/gmtime de la libc) para no depender de que la newlib-nano de
 * este proyecto tenga esas funciones enlazadas -- es una decisión de
 * portabilidad, no de rendimiento (esto corre una vez cada tanto, no en
 * un lazo caliente).
 */

#include "rtc_reloj.h"

static RTC_HandleTypeDef *s_hrtc = NULL;
static bool s_sincronizado = false;

/* ==================== Conversión calendario -> epoch (UTC puro) ==================== */

static bool EsAnioBisiesto(uint32_t anio)
{
    return (anio % 4U == 0U && (anio % 100U != 0U || anio % 400U == 0U));
}

static const uint8_t DIAS_POR_MES[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint32_t Reloj_CalendarioAEpoch(uint16_t anio, uint8_t mes, uint8_t dia,
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

/* Inversa de Reloj_CalendarioAEpoch(): epoch UTC -> campos de calendario. */
static void EpochACalendario(uint32_t epoch, uint16_t *anio, uint8_t *mes, uint8_t *dia,
                             uint8_t *hora, uint8_t *minuto, uint8_t *segundo)
{
    uint32_t dias = epoch / 86400UL;
    uint32_t resto = epoch % 86400UL;
    *hora = (uint8_t)(resto / 3600UL);
    *minuto = (uint8_t)((resto % 3600UL) / 60UL);
    *segundo = (uint8_t)(resto % 60UL);

    uint16_t a = 1970U;
    for (;;) {
        uint32_t diasAnio = EsAnioBisiesto(a) ? 366UL : 365UL;
        if (dias < diasAnio) break;
        dias -= diasAnio;
        a++;
    }
    uint8_t m = 0U;
    for (;;) {
        uint32_t diasMes = DIAS_POR_MES[m] + ((m == 1U && EsAnioBisiesto(a)) ? 1UL : 0UL);
        if (dias < diasMes) break;
        dias -= diasMes;
        m++;
    }
    *anio = a;
    *mes = (uint8_t)(m + 1U);
    *dia = (uint8_t)(dias + 1UL);
}

/* ==================== API pública ==================== */

void Reloj_Init(RTC_HandleTypeDef *hrtc)
{
    s_hrtc = hrtc;
    s_sincronizado = false;
}

void Reloj_SetHoraUtc(uint16_t anio, uint8_t mes, uint8_t dia,
                       uint8_t hora, uint8_t minuto, uint8_t segundo)
{
    if (s_hrtc == NULL) {
        return;
    }

    /* Los campos de calendario ya vienen sueltos (ej. parseados de
     * "AT+LTIME=15h08m55s on 08/17/2026" o del +CGPSINFO del GPS). */
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

void Reloj_SetUnixTimeUtc(uint32_t epochUtc)
{
    uint16_t anio;
    uint8_t mes, dia, hora, minuto, segundo;
    EpochACalendario(epochUtc, &anio, &mes, &dia, &hora, &minuto, &segundo);
    Reloj_SetHoraUtc(anio, mes, dia, hora, minuto, segundo);
}

bool Reloj_EstaSincronizado(void)
{
    return s_sincronizado;
}

uint32_t Reloj_GetUnixTimeUtc(void)
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

    return Reloj_CalendarioAEpoch((uint16_t)(2000U + sDate.Year), sDate.Month, sDate.Date,
                                  sTime.Hours, sTime.Minutes, sTime.Seconds);
}

uint32_t Reloj_GetUnixTimeLocal(void)
{
    return Reloj_GetUnixTimeUtc() + GUATEMALA_UTC_OFFSET_SEGUNDOS;
}
