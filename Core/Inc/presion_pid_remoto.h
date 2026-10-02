/**
 * @file    presion_pid_remoto.h
 * @brief   PID#3 (MODO=2, aspersor remoto) -- TRIPLE CASCADA desde 2026-09-30:
 *          presion remota -> AJUSTE del objetivo de presion LOCAL.
 *
 * Cadena completa en MODO=2:
 *
 *   PSI aspersor --PID#3--> objetivo local --PID#2--> RPM --PID#1--> servo
 *                            = PRESION_OBJETIVO_LOCAL + ajuste
 *
 * Decision del usuario 2026-09-30 (ver README seccion 4.3): la BASE de
 * MODO=2 es PRESION_OBJETIVO_LOCAL que MODO=1 ya dejo sostenida (por eso
 * MODO=2 solo se arma viniendo de MODO=1). El PID#3 calcula cuantos PSI
 * sumarle (o restarle) a esa base para que el aspersor llegue a
 * PRESION_OBJETIVO_REMOTO. PRESION_OBJETIVO_LOCAL nunca se modifica: el
 * ajuste vive solo en RAM.
 *
 * Por que asi y no presion remota -> RPM directo (diseño anterior,
 * 2026-09-23 a 2026-09-30): la relacion RPM -> presion remota depende de
 * toda la cadena bomba + tuberia + aspersor, y cambia cada vez que mueven
 * el aspersor o la tuberia. La relacion presion local -> presion remota
 * es ~1 PSI por PSI menos lo que pierde el tubo: un cambio de tuberia mueve
 * sobre todo esa PERDIDA (un desfase), que el integral compensa solo, sin
 * recalibrar. La dinamica de la bomba la sigue resolviendo el PID#2 con el
 * sensor local rapido.
 *
 * EVENTO-DRIVEN (igual que antes): el aspersor reporta cada 20 s (fuera de
 * su objetivo) / 90 s (en su objetivo) / 25 min (sin presion), asi que
 * PresionPidRemoto_CalcularAjustePsi() se llama SOLO cuando llega un
 * reporte nuevo, con el tiempo REAL transcurrido para el integral.
 *
 * El objetivo local que realmente se aplica al PID#2 cambia a ritmo
 * limitado (PRESION_OBJETIVO_LOCAL / TIEMPO_LLENADO_S, ver
 * CalibFlash_GetTasaLlenadoPsiS()), en las dos direcciones -- regla de la
 * tuberia del usuario: la presion nunca cambia mas rapido que el ritmo de
 * llenado que valido el operador (ver PresionPidRemoto_ObjetivoAplicado()).
 */

#ifndef PRESION_PID_REMOTO_H
#define PRESION_PID_REMOTO_H

#ifdef __cplusplus
extern "C" {
#endif

/** Resetea integral y ancla de tiempo (entrada a MODO=2, perdida de senal,
 *  inicio de una validacion). */
void PresionPidRemoto_Init(void);

/**
 * Ajuste (PSI) a sumar a basePsi (PRESION_OBJETIVO_LOCAL). Llamar SOLO con un
 * reporte nuevo del aspersor. Salida = Kp*error + Ki*integral, limitada a que
 * el objetivo local quede en [0, techoPsi] Y a no mas de 1 PSI por delante del
 * objetivo que la rampa ya alcanzo (integrador saturado a esos limites -- ver
 * el .c). Ganancias PID_ASP_KP (PSI local por PSI remoto) y PID_ASP_KI (1/s).
 * Con ambas en 0 (sin calibrar) el ajuste es 0: MODO=2 sostiene la base, como MODO=1.
 */
float PresionPidRemoto_CalcularAjustePsi(float objetivoRemotoPsi, float presionRemotaPsi,
                                         float basePsi, float techoPsi);

/** Ganancias TEMPORALES (solo RAM) para la autosintonia (autotune_asp.c, CALIB=12). */
void PresionPidRemoto_SetGananciasTemporales(float kp, float ki);
void PresionPidRemoto_LimpiarGananciasTemporales(void);

/**
 * Rampa del objetivo local APLICADO: cada llamada lo acerca a
 * 'objetivoDeseadoPsi' como mucho tasaPsiS * (tiempo desde la llamada
 * anterior), en cualquier direccion. Llamar en cada vuelta del loop.
 * PresionPidRemoto_ReiniciarRampa() lo ancla a un valor (tipicamente la
 * presion medida en ese instante) -- al entrar a MODO=2, al volver de un
 * retroceso, al empezar la autosintonia.
 */
void  PresionPidRemoto_ReiniciarRampa(float valorPsi);
float PresionPidRemoto_ObjetivoAplicado(float objetivoDeseadoPsi, float tasaPsiS);

#ifdef __cplusplus
}
#endif

#endif /* PRESION_PID_REMOTO_H */
