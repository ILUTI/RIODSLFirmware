/**
 * @file    presion_pid_remoto.h
 * @brief   Lazo externo (cascada) de presion REMOTA -> setpoint de RPM,
 *          para MODO=2 (CALIB_MODO_REMOTO).
 *
 * Mismo patron que presion_pid.h (lazo externo de presion LOCAL,
 * MODO=1), pero es un modulo APARTE con su propio estado interno y sus
 * propias ganancias (PRESION_REMOTO_PID_KP/KI, ver calibracion_flash.h)
 * -- no comparte nada con presion_pid.c, porque agregado 2026-09-23,
 * reemplaza la formula lineal abierta que tenia MODO=2 hasta entonces
 * (setpoint = PRESION_OFFSET_RPM + PRESION_GANANCIA_RPM * PRESION).
 *
 * Motivo del reemplazo (decision explicita del usuario, ver
 * conversacion 2026-09-23): la formula lineal exigia calibrar
 * PRESION_OFFSET_RPM/PRESION_GANANCIA_RPM en CADA instalacion (la
 * relacion RPM->presion remota depende de la tuberia/bomba/aspersor
 * fisicos de cada sitio). Un PID no necesita conocer esa relacion de
 * antemano -- reacciona al ERROR real (PRESION_OBJETIVO_REMOTO menos lo
 * que reporta el aspersor) y se autoajusta a cualquier instalacion,
 * igual que ya hace presion_pid.c para MODO=1.
 *
 * DIFERENCIA CLAVE con presion_pid.c: este lazo NO se llama en cada
 * ciclo del main loop -- el aspersor remoto reporta con una cadencia
 * muy irregular (20s a 20-25min segun su propio estado, confirmado en
 * campo 2026-09-23), asi que main.c solo debe llamar a
 * PresionPidRemoto_CalcularSetpointRpm() cuando llega un reporte NUEVO
 * de PRESION (deteccion de flanco via CalibFlash_GetPresionUltimoTickMs(),
 * mismo truco que ya usa el supervisor de MODO=2). Entre reportes,
 * main.c debe sostener el ultimo valor devuelto, no volver a llamar a
 * esta funcion con el mismo dato viejo. Justamente por eso el calculo
 * del paso de integracion usa tiempo REAL transcurrido (HAL_GetTick()),
 * no un dt fijo de ciclo -- el dt entre dos llamadas puede ser de
 * segundos o de minutos, y el termino integral tiene que reflejar eso
 * correctamente.
 */

#ifndef PRESION_PID_REMOTO_H
#define PRESION_PID_REMOTO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Resetea el estado interno (integral, ancla de tiempo). Llamar en el
 * flanco de entrada a MODO=CALIB_MODO_REMOTO (ver main.c), mismo motivo
 * que PresionPid_Init(): evita un dt inflado por tiempo inactivo ni una
 * integral vieja de una sesion anterior.
 */
void PresionPidRemoto_Init(void);

/**
 * Calcula el setpoint de RPM, ya recortado a [0, RPM_MAX]. Llamar SOLO
 * cuando llega un reporte nuevo de PRESION (ver nota de arriba) -- el
 * paso de integracion se calcula por tiempo real transcurrido desde la
 * ULTIMA VEZ que se llamo esta funcion (HAL_GetTick()), no por conteo
 * de ciclos del main loop.
 *
 * Mismo patron que PresionPid_CalcularSetpointRpm(): la salida es
 * RPM_MIN (piso feedforward) + P + I, para que el termino integral no
 * tenga que reconstruir desde 0 todo el RPM real necesario. El recorte
 * final SI baja hasta 0 (no hasta RPM_MIN): si la presion remota ya
 * esta por encima del objetivo, este lazo debe poder pedir "sin
 * comandar" y dejar que controlSolicitado caiga a ralenti natural.
 *
 * @param setpointPresionPsi  Objetivo, CalibFlash_GetPresionObjetivoRemoto().
 * @param presionMedidaPsi    Ultima PRESION reportada por el aspersor,
 *                             CalibFlash_GetPresion().
 * @return Setpoint de RPM a pasarle al lazo interno (PID_CalcularSalidaUs).
 */
float PresionPidRemoto_CalcularSetpointRpm(float setpointPresionPsi, float presionMedidaPsi);

#ifdef __cplusplus
}
#endif

#endif /* PRESION_PID_REMOTO_H */
