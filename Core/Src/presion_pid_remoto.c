/**
 * @file    presion_pid_remoto.c
 * @brief   Implementacion del lazo externo (cascada) presion REMOTA -> setpoint de RPM.
 */

#include "presion_pid_remoto.h"
#include "calibracion_flash.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* ==================== ESTADO INTERNO ==================== */

static float s_integral = 0.0f;
static uint32_t s_ultimoCalculoMs = 0;

/* ==================== API PÚBLICA ==================== */

void PresionPidRemoto_Init(void)
{
    s_integral = 0.0f;
    s_ultimoCalculoMs = HAL_GetTick();
}

float PresionPidRemoto_CalcularSetpointRpm(float setpointPresionPsi, float presionMedidaPsi)
{
    uint32_t ahora = HAL_GetTick();
    float dtS = (ahora - s_ultimoCalculoMs) / 1000.0f;
    s_ultimoCalculoMs = ahora;

    float error = setpointPresionPsi - presionMedidaPsi;

    float kp = CalibFlash_GetPidAspKp();
    float ki = CalibFlash_GetPidAspKi();

    float maximo = CalibFlash_GetRpmMax();

    /* Mismo piso feedforward que presion_pid.c (RPM_MIN) y mismo
     * motivo: sin esto, el termino integral tendria que reconstruir
     * TODO el RPM real necesario desde 0, y con el aspersor reportando
     * cada 20s-25min, eso podria tardar horas en cruzar el umbral de
     * RPM_MIN (controlSolicitado en main.c). */
    float base = CalibFlash_GetRpmMin();

    if (ki > 0.0f) {
        /* Mismo anti-windup por integracion condicional que
         * presion_pid.c: solo se acumula si la salida tentativa no esta
         * ya empujando mas alla del limite saturado. Con dtS potencialmente
         * grande (varios minutos entre reportes), este anti-windup es
         * todavia mas importante aca que en el lazo local -- un solo
         * paso de integracion ya puede mover mucho el termino I. */
        float integralTentativa = s_integral + error * dtS;
        float salidaTentativa = base + kp * error + ki * integralTentativa;
        bool saturaAlto = salidaTentativa > maximo;
        bool saturaBajo = salidaTentativa < 0.0f;
        if (!(saturaAlto && error > 0.0f) && !(saturaBajo && error < 0.0f)) {
            s_integral = integralTentativa;
        }
    } else {
        /* KI=0 -- no acumular en silencio mientras esta ganancia esta
         * apagada (mismo motivo que presion_pid.c: evita un salto al
         * activarla). */
        s_integral = 0.0f;
    }

    float salida = base + kp * error + ki * s_integral;
    if (salida > maximo) {
        salida = maximo;
    } else if (salida < 0.0f) {
        salida = 0.0f;
    }

    return salida;
}
