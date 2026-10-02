/**
 * @file    pid.c
 * @brief   Implementación del lazo de control PID de RPM -> servo.
 */

#include "pid.h"
#include "calibracion_flash.h"
#include "stm32g4xx_hal.h"

/* ==================== ESTADO INTERNO ==================== */

static float s_integral = 0.0f;
static uint32_t s_ultimoCalculoMs = 0;

/* Ganancias temporales (solo RAM) -- usadas por la autosintonia
 * (autotune_rpm.c) para probar candidatas SIN escribir flash: un corte de
 * energia a mitad del ensayo no deja una Kp de prueba pegada, y no se gasta
 * flash en cada intento. Si estan activas, pisan a las de CalibFlash. */
static bool  s_gananciasTemporalesActivas = false;
static float s_kpTemporal = 0.0f;
static float s_kiTemporal = 0.0f;

/* ==================== API PÚBLICA ==================== */

void PID_SetGananciasTemporales(float kp, float ki)
{
    s_kpTemporal = kp;
    s_kiTemporal = ki;
    s_gananciasTemporalesActivas = true;
}

void PID_LimpiarGananciasTemporales(void)
{
    s_gananciasTemporalesActivas = false;
}

void PID_Init(void)
{
    s_integral = 0.0f;
    s_ultimoCalculoMs = HAL_GetTick();
}

uint16_t PID_CalcularSalidaUs(float setpointRpm, float rpmMedida)
{
    uint32_t ahora = HAL_GetTick();
    float dtS = (ahora - s_ultimoCalculoMs) / 1000.0f;
    s_ultimoCalculoMs = ahora;

    float error = setpointRpm - rpmMedida;

    float kp = s_gananciasTemporalesActivas ? s_kpTemporal : CalibFlash_GetPidRpmKp();
    float ki = s_gananciasTemporalesActivas ? s_kiTemporal : CalibFlash_GetPidRpmKi();

    uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
    uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();
    float rango = (float)(maximo - minimo);

    if (ki > 0.0f) {
        /* Anti-windup por integración condicional: solo se acumula si
         * el resultado tentativo no está empujando más allá del
         * límite ya saturado -- evita que la integral siga creciendo
         * sin límite mientras la salida no puede moverse más. */
        float integralTentativa = s_integral + error * dtS;
        float salidaTentativa = kp * error + ki * integralTentativa;
        bool saturaAlto = salidaTentativa > rango;
        bool saturaBajo = salidaTentativa < 0.0f;
        if (!(saturaAlto && error > 0.0f) && !(saturaBajo && error < 0.0f)) {
            s_integral = integralTentativa;
        }
    } else {
        /* KI=0 -- no acumular en silencio mientras esta ganancia está
         * apagada (si no, al activarla más adelante el termino
         * integral aparecería con un salto de golpe). */
        s_integral = 0.0f;
    }

    float salida = kp * error + ki * s_integral;
    if (salida > rango) {
        salida = rango;
    } else if (salida < 0.0f) {
        salida = 0.0f;
    }

    return (uint16_t)(minimo + salida);
}
