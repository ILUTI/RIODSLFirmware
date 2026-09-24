/**
 * @file    presion_pid.h
 * @brief   Lazo externo (cascada) de control de presion -> setpoint de RPM.
 *
 * Es el lazo EXTERNO de la cascada descrita en el README (seccion 9/12,
 * MODO=1 / CALIB_MODO_PRESION_LOCAL): compara la presion real medida
 * (hoy PresionV_GetPresionPsi(), el canal temporal de voltaje -- ver
 * presion_voltaje.h) contra PRESION_OBJETIVO, y su salida es
 * directamente el setpoint de RPM que main.c le pasa al lazo INTERNO ya
 * existente (pid.c, PID_CalcularSalidaUs) sin que ese modulo cambie en
 * nada -- la cascada es transparente para el.
 *
 * Solo P+I (sin derivativo) a proposito, mismo criterio que el lazo de
 * RPM (Kd=0 confirmado en campo, ver README seccion 9): el lazo externo
 * de una cascada es tipicamente mas lento que el interno por diseno, y
 * va a trabajar sobre una senal de presion probablemente mas ruidosa
 * que el tacometro -- un termino derivativo ahi amplificaria ese ruido
 * sin beneficio claro. Si hiciera falta, confirmar con
 * tools/pid_tuning/pid_tuning.py identify contra datos reales, no
 * asumir.
 *
 * Ganancias propias (PRESION_PID_KP/KI, independientes de PID_KP/KI del
 * lazo interno) -- ver calibracion_flash.h. Arrancan en 0/0 (sin
 * calibrar) hasta que se caracterice la relacion presion<->RPM real del
 * sistema hidraulico, mismo metodo que se uso para el mecanismo directo
 * (identify-before-implement, README seccion 9).
 */

#ifndef PRESION_PID_H
#define PRESION_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Resetea el estado interno (integral, ancla de tiempo). Llamar en el
 * flanco de entrada a MODO=CALIB_MODO_PRESION_LOCAL (ver main.c), mismo
 * motivo que PID_Init(): evita un dt inflado por tiempo inactivo en el
 * primer calculo tras volver a este modo.
 */
void PresionPid_Init(void);

/**
 * Calcula el setpoint de RPM para este ciclo, ya recortado a
 * [0, RPM_MAX]. El paso de integracion se calcula por tiempo real
 * transcurrido (HAL_GetTick()), no por conteo de llamadas.
 *
 * La salida es RPM_MIN (piso feedforward) + P + I, mismo patron que
 * pid.c (SERVO_PULSO_MIN + correccion) -- agregado 2026-09-21 porque
 * sin piso el termino integral tenia que reconstruir TODO el RPM real
 * necesario desde 0, lo cual tardaba varios minutos en cruzar el
 * umbral de RPM_MIN (ver controlSolicitado en main.c) y parecia que el
 * lazo no reaccionaba. El recorte final SI sigue bajando hasta 0 (no
 * hasta RPM_MIN) a proposito: si la presion ya esta por encima del
 * objetivo (error muy negativo), este lazo debe poder pedir "sin
 * comandar" y dejar que controlSolicitado caiga a ralenti natural.
 *
 * @param setpointPresionPsi  Presion objetivo, ej. CalibFlash_GetPresionObjetivo().
 * @param presionMedidaPsi    Presion real medida, ej. PresionV_GetPresionPsi().
 * @return Setpoint de RPM a pasarle al lazo interno (PID_CalcularSalidaUs).
 */
float PresionPid_CalcularSetpointRpm(float setpointPresionPsi, float presionMedidaPsi);

#ifdef __cplusplus
}
#endif

#endif /* PRESION_PID_H */
