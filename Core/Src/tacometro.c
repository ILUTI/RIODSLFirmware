/**
 * @file    tacometro.c
 * @brief   Implementación del módulo de medición de RPM vía Input Capture.
 */

#include "tacometro.h"
#include "calibracion_flash.h"

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

    /* NOTA: se asume que CalibFlash_Init() ya corrió antes en main(),
     * así que CalibFlash_GetPulsosPorRevolucion()/GetAlphaFiltro() ya
     * tienen un valor válido disponible (guardado en flash, o el
     * default si nunca se ha calibrado). */
}

void Tacometro_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != s_htim->Instance) {
        return;
    }

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

    uint32_t periodoUs    = HAL_TIM_ReadCapturedValue(htim, s_canal);
    uint32_t anchoPulsoUs = HAL_TIM_ReadCapturedValue(htim, canalIndirecto);

    if (!s_primerFlancoHecho) {
        s_primerFlancoHecho = true;
        return;
    }

    if (periodoUs < TACOMETRO_PERIODO_MINIMO_US) {
        s_contadorRuido++;
        return;
    }

    float duty = (float)anchoPulsoUs / (float)periodoUs;
    if (duty < TACOMETRO_DUTY_MINIMO || duty > TACOMETRO_DUTY_MAXIMO) {
        s_contadorRuido++;
        return;
    }

    s_ultimoPeriodoUs     = periodoUs;
    s_hayPeriodoNuevo     = true;
    s_ultimoTickCapturaMs = HAL_GetTick();
}

void Tacometro_Update(void)
{
    if (s_hayPeriodoNuevo) {
        __disable_irq();
        uint32_t periodoUs = s_ultimoPeriodoUs;
        s_hayPeriodoNuevo = false;
        __enable_irq();

        /* Valores calibrados en flash (downlink SET_RATIO/ALPHA o CALIB=3). */
        float pulsosPorRevolucion = CalibFlash_GetPulsosPorRevolucion();
        float alphaFiltro = CalibFlash_GetAlphaFiltro();

        s_frecuenciaHz   = (float)TACOMETRO_TICKS_POR_SEGUNDO / (float)periodoUs;
        s_rpmInstantanea = (s_frecuenciaHz * 60.0f) / pulsosPorRevolucion;

        s_rpmFiltrada = (alphaFiltro * s_rpmInstantanea) +
                         ((1.0f - alphaFiltro) * s_rpmFiltrada);

        s_motorDetenido = false;
    }

    uint32_t ahora = HAL_GetTick();
    uint32_t transcurrido = ahora - s_ultimoTickCapturaMs;

    if (!s_motorDetenido && transcurrido > TACOMETRO_TIMEOUT_DETENIDO_MS) {
        s_motorDetenido  = true;
        s_rpmInstantanea = 0.0f;
        s_rpmFiltrada    = 0.0f;
        s_frecuenciaHz   = 0.0f;
        s_primerFlancoHecho = false;
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
