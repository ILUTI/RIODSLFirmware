/**
 * @file    presion_pid.c
 * @brief   Lazo externo (cascada) presion -> setpoint de RPM (PID#2).
 *          Usa el nucleo P+I compartido de pid.c (Pi_Calcular).
 */

#include "presion_pid.h"
#include "pid.h"
#include "calibracion_flash.h"

static Pi_t s_pid2;

void PresionPid_SetGananciasTemporales(float kp, float ki)
{
    Pi_SetGananciasTemporales(&s_pid2, kp, ki);
}

void PresionPid_LimpiarGananciasTemporales(void)
{
    Pi_LimpiarGananciasTemporales(&s_pid2);
}

void PresionPid_Init(void)
{
    Pi_Init(&s_pid2);
}

float PresionPid_CalcularSetpointRpm(float setpointPresionPsi, float presionMedidaPsi)
{
    /* Piso de arranque (feedforward) = RPM_MIN, mismo patron que pid.c
     * (SERVO_PULSO_MIN + correccion). Encontrado en campo 2026-09-21: sin
     * esto, el integral tenia que reconstruir SOLO todo el RPM necesario
     * desde cero y tardaba minutos en cruzar RPM_MIN -- parecia que el lazo
     * "no reaccionaba". Con el piso, P+I solo corrige la diferencia. */
    float base = CalibFlash_GetRpmMin();

    /* Techo = RPM_MAX_CARGA, no RPM_MAX (2026-10-02): este lazo siempre corre
     * con la bomba embragada y main.c recorta su salida a RPM_MAX_CARGA.
     * Antes el anti-windup usaba RPM_MAX: con RPM_MAX_CARGA < RPM_MAX el
     * integral seguia acumulando entre los dos techos mientras el motor ya
     * no podia dar mas, y al alcanzar la presion se pasaba del objetivo. */
    float maximo = CalibFlash_GetRpmMaxCarga();

    /* Anti-windup hacia ABAJO en RPM_MIN (corrección 0), no en 0 RPM
     * (2026-10-02): con la presion por encima del objetivo, el integral se
     * vaciaba hasta pedir 0 RPM -- ~RPM_MIN por debajo del ralenti, que en
     * la practica es lo mismo -- y al volver a bajar la presion tenia que
     * rellenar esos RPM "de mentira" antes de acelerar (reaccion lenta).
     * La SALIDA si puede seguir bajando hasta 0 (corrección -base): con la
     * presion muy alta el termino P aun pide "sin comandar" y deja que
     * controlSolicitado caiga a ralenti natural. ⚠️ Validar en campo. */
    float correccion = Pi_Calcular(&s_pid2, setpointPresionPsi - presionMedidaPsi,
                                   CalibFlash_GetPidPsiKp(), CalibFlash_GetPidPsiKi(),
                                   0.0f, maximo - base, -base, maximo - base);

    return base + correccion;
}
