/**
 * @file    pid.c
 * @brief   Nucleo P+I compartido (Pi_*) y lazo de RPM -> servo (PID#1).
 */

#include "pid.h"
#include "calibracion_flash.h"
#include "stm32g4xx_hal.h"

/* ==================== NUCLEO P+I COMPARTIDO ==================== */

void Pi_Init(Pi_t *pi)
{
    pi->integral = 0.0f;
    pi->ultimoCalculoMs = HAL_GetTick();
}

void Pi_SetGananciasTemporales(Pi_t *pi, float kp, float ki)
{
    pi->kpTemporal = kp;
    pi->kiTemporal = ki;
    pi->temporalesActivas = true;
}

void Pi_LimpiarGananciasTemporales(Pi_t *pi)
{
    pi->temporalesActivas = false;
}

float Pi_Calcular(Pi_t *pi, float error, float kpGuardada, float kiGuardada,
                  float awMin, float awMax, float salidaMin, float salidaMax)
{
    uint32_t ahora = HAL_GetTick();
    float dtS = (ahora - pi->ultimoCalculoMs) / 1000.0f;
    pi->ultimoCalculoMs = ahora;

    float kp = pi->temporalesActivas ? pi->kpTemporal : kpGuardada;
    float ki = pi->temporalesActivas ? pi->kiTemporal : kiGuardada;

    if (ki > 0.0f) {
        /* Anti-windup por integración condicional: solo se acumula si el
         * resultado tentativo no está empujando más allá de [awMin, awMax]
         * en la misma dirección del error -- evita que la integral siga
         * creciendo mientras la salida no puede (o no debe) moverse más. */
        float integralTentativa = pi->integral + error * dtS;
        float salidaTentativa = kp * error + ki * integralTentativa;
        bool saturaAlto = salidaTentativa > awMax;
        bool saturaBajo = salidaTentativa < awMin;
        if (!(saturaAlto && error > 0.0f) && !(saturaBajo && error < 0.0f)) {
            pi->integral = integralTentativa;
        }
    } else {
        /* KI=0 -- no acumular en silencio mientras esta ganancia está
         * apagada (si no, al activarla más adelante el termino integral
         * aparecería con un salto de golpe). */
        pi->integral = 0.0f;
    }

    float salida = kp * error + ki * pi->integral;
    if (salida > salidaMax) {
        salida = salidaMax;
    } else if (salida < salidaMin) {
        salida = salidaMin;
    }
    return salida;
}

/* ==================== PID#1: RPM -> SERVO ==================== */

static Pi_t s_pid1;

void PID_SetGananciasTemporales(float kp, float ki)
{
    Pi_SetGananciasTemporales(&s_pid1, kp, ki);
}

void PID_LimpiarGananciasTemporales(void)
{
    Pi_LimpiarGananciasTemporales(&s_pid1);
}

void PID_Init(void)
{
    Pi_Init(&s_pid1);
}

uint16_t PID_CalcularSalidaUs(float setpointRpm, float rpmMedida)
{
    uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
    uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();
    float rango = (float)(maximo - minimo);

    /* Mismas cuentas que antes de juntar los PID (2026-10-02): corrección en
     * [0, rango] tanto para el anti-windup como para la salida, y se suma a
     * SERVO_PULSO_MIN. Valores de campo validados: Kp=0.047, Ki=0.11. */
    float salida = Pi_Calcular(&s_pid1, setpointRpm - rpmMedida,
                               CalibFlash_GetPidRpmKp(), CalibFlash_GetPidRpmKi(),
                               0.0f, rango, 0.0f, rango);

    return (uint16_t)(minimo + salida);
}
