/**
 * @file    presion_pid.c
 * @brief   Implementacion del lazo externo (cascada) presion -> setpoint de RPM.
 */

#include "presion_pid.h"
#include "calibracion_flash.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* ==================== ESTADO INTERNO ==================== */

static float s_integral = 0.0f;
static uint32_t s_ultimoCalculoMs = 0;

/* ==================== API PÚBLICA ==================== */

void PresionPid_Init(void)
{
    s_integral = 0.0f;
    s_ultimoCalculoMs = HAL_GetTick();
}

float PresionPid_CalcularSetpointRpm(float setpointPresionPsi, float presionMedidaPsi)
{
    uint32_t ahora = HAL_GetTick();
    float dtS = (ahora - s_ultimoCalculoMs) / 1000.0f;
    s_ultimoCalculoMs = ahora;

    float error = setpointPresionPsi - presionMedidaPsi;

    float kp = CalibFlash_GetPresionPidKp();
    float ki = CalibFlash_GetPresionPidKi();

    float maximo = CalibFlash_GetRpmMax();

    /* Piso de arranque (feedforward) = RPM_MIN, mismo patron que pid.c
     * (SERVO_PULSO_MIN + correccion). Encontrado en campo 2026-09-21:
     * sin esto, para sostener una presion objetivo real (que
     * tipicamente requiere un RPM bien por encima de RPM_MIN) el
     * termino integral tiene que reconstruir SOLO todo ese RPM desde
     * cero -- con el motor ya en ralenti (setpoint < RPM_MIN,
     * controlSolicitado=false) eso tarda varios minutos en cruzar el
     * umbral y se veia como si el lazo "no reaccionara". Con RPM_MIN
     * de piso, el termino P+I solo tiene que corregir la diferencia
     * entre RPM_MIN y el RPM real necesario -- igual de valido, mucho
     * mas rapido. */
    float base = CalibFlash_GetRpmMin();

    if (ki > 0.0f) {
        /* Mismo anti-windup por integracion condicional que pid.c: solo
         * se acumula si la salida tentativa no esta ya empujando mas
         * alla del limite saturado. */
        float integralTentativa = s_integral + error * dtS;
        float salidaTentativa = base + kp * error + ki * integralTentativa;
        bool saturaAlto = salidaTentativa > maximo;
        bool saturaBajo = salidaTentativa < 0.0f;
        if (!(saturaAlto && error > 0.0f) && !(saturaBajo && error < 0.0f)) {
            s_integral = integralTentativa;
        }
    } else {
        /* KI=0 -- no acumular en silencio mientras esta ganancia esta
         * apagada (mismo motivo que pid.c: evita un salto al activarla). */
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
