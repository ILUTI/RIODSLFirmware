/**
 * @file    presion_pid_remoto.c
 * @brief   PID#3 en triple cascada (presion remota -> ajuste del objetivo local)
 *          -- ver presion_pid_remoto.h.
 */

/* -Os tambien en Debug: mismo motivo que autotune.c (que Debug quepa en 126 KB). */
#pragma GCC optimize ("Os")

#include "presion_pid_remoto.h"
#include "calibracion_flash.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* Ventana del ajuste alrededor del objetivo que la rampa ya alcanzo (ver
 * PresionPidRemoto_CalcularAjustePsi). */
#define PRESION_PID_REMOTO_VENTANA_PSI   1.0f

/* ==================== ESTADO INTERNO ==================== */

static float    s_integral = 0.0f;
static uint32_t s_ultimoCalculoMs = 0;

/* Ganancias temporales (solo RAM), mismo patron que pid.c/presion_pid.c. */
static bool     s_gananciasTemporalesActivas = false;
static float    s_kpTemporal = 0.0f;
static float    s_kiTemporal = 0.0f;

/* Rampa del objetivo local aplicado. */
static float    s_aplicado = 0.0f;
static uint32_t s_ultimaRampaMs = 0;

/* ==================== API PÚBLICA ==================== */

void PresionPidRemoto_Init(void)
{
    s_integral = 0.0f;
    s_ultimoCalculoMs = HAL_GetTick();
}

void PresionPidRemoto_SetGananciasTemporales(float kp, float ki)
{
    s_kpTemporal = kp;
    s_kiTemporal = ki;
    s_gananciasTemporalesActivas = true;
}

void PresionPidRemoto_LimpiarGananciasTemporales(void)
{
    s_gananciasTemporalesActivas = false;
}

float PresionPidRemoto_CalcularAjustePsi(float objetivoRemotoPsi, float presionRemotaPsi,
                                         float basePsi, float techoPsi)
{
    uint32_t ahora = HAL_GetTick();
    float dtS = (ahora - s_ultimoCalculoMs) / 1000.0f;
    s_ultimoCalculoMs = ahora;

    float error = objetivoRemotoPsi - presionRemotaPsi;
    float kp = s_gananciasTemporalesActivas ? s_kpTemporal : CalibFlash_GetPidAspKp();
    float ki = s_gananciasTemporalesActivas ? s_kiTemporal : CalibFlash_GetPidAspKi();

    /* Limites del ajuste: los absolutos (objetivo local entre 0 y el techo)
     * y ademas una VENTANA de +-PRESION_PID_REMOTO_VENTANA_PSI alrededor del
     * objetivo que la rampa ya alcanzo. Sin esa ventana, mientras la rampa
     * de llenado frena un cambio grande el integral sigue acumulando, y al
     * llegar la presion se pasa: simulado 6-8 PSI de sobrepaso tras cambiar
     * la tuberia; con la ventana, <=1.2 PSI y sin oscilar. */
    float ajusteMin = -basePsi;
    float ajusteMax = techoPsi - basePsi;
    float ventMin = s_aplicado - basePsi - PRESION_PID_REMOTO_VENTANA_PSI;
    float ventMax = s_aplicado - basePsi + PRESION_PID_REMOTO_VENTANA_PSI;
    if (ventMin > ajusteMin) ajusteMin = ventMin;
    if (ventMax < ajusteMax) ajusteMax = ventMax;

    if (ki > 0.0f) {
        /* Integrador SATURADO a esos limites (no congelado): se recorta para
         * que kp*error + ki*integral quede dentro. Congelarlo (integracion
         * condicional) trababa el lazo: el primer paso ya superaba la ventana
         * y el integral nunca arrancaba (visto en simulacion). */
        s_integral += error * dtS;
        float integralMin = (ajusteMin - kp * error) / ki;
        float integralMax = (ajusteMax - kp * error) / ki;
        if (s_integral < integralMin) s_integral = integralMin;
        if (s_integral > integralMax) s_integral = integralMax;
    } else {
        s_integral = 0.0f;
    }

    float ajuste = kp * error + ki * s_integral;
    if (ajuste > ajusteMax) ajuste = ajusteMax;
    if (ajuste < ajusteMin) ajuste = ajusteMin;
    return ajuste;
}

void PresionPidRemoto_ReiniciarRampa(float valorPsi)
{
    s_aplicado = valorPsi;
    s_ultimaRampaMs = HAL_GetTick();
}

float PresionPidRemoto_ObjetivoAplicado(float objetivoDeseadoPsi, float tasaPsiS)
{
    uint32_t ahora = HAL_GetTick();
    float dtS = (ahora - s_ultimaRampaMs) / 1000.0f;
    s_ultimaRampaMs = ahora;

    if (tasaPsiS <= 0.0f) {
        /* Sin ritmo configurado (TASA_LLENADO_PSI_S = 0): directo, igual que
         * MODO=1 sin rampa. */
        s_aplicado = objetivoDeseadoPsi;
        return s_aplicado;
    }
    float paso = tasaPsiS * dtS;
    if (objetivoDeseadoPsi > s_aplicado + paso)      s_aplicado += paso;
    else if (objetivoDeseadoPsi < s_aplicado - paso) s_aplicado -= paso;
    else                                             s_aplicado = objetivoDeseadoPsi;
    return s_aplicado;
}
