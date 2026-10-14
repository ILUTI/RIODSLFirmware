/**
 * @file    presion_pid.h
 * @brief   Lazo externo (cascada) de control de presion -> setpoint de RPM.
 *
 * Es el lazo EXTERNO de la cascada (README seccion 4.3): compara la presion
 * real medida (hoy PresionV_GetPresionPsi(), el canal temporal de voltaje
 * -- el final sera presion.c) contra el objetivo vigente
 * (PRESION_OBJETIVO_LOCAL en MODO=1, SET_PRESION en MODO=3, el objetivo que
 * mueve el PID#3 en MODO=2, o el de prueba de un autotune), y su salida es
 * directamente el setpoint de RPM que main.c le pasa al lazo INTERNO
 * (pid.c, PID_CalcularSalidaUs). Usa el nucleo P+I compartido de pid.h.
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
 * Ganancias propias (PID_PSI_KP/KI, independientes de PID_RPM_KP/KI del
 * lazo interno) -- ver calibracion_flash.h. Default = las validadas en campo,
 * Kp=1.72 / Ki=0.39 (desde 2026-10-13; antes 0/0), y CALIB=14 las calcula
 * solo (ver autotune_psi.h).
 */

#ifndef PRESION_PID_H
#define PRESION_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Tope de velocidad de SUBIDA del objetivo de presion local (2026-10-12,
 * usuario): nunca sube mas rapido que esto, aunque TIEMPO_LLENADO_S sea 0 o
 * pida algo mas rapido. Sale de la subida 18 -> 35 PSI con PID#2 1.72/0.39
 * que le gusta al usuario (pico ~0.39 PSI/s; 18 -> 45 llegaba a ~0.62 y se
 * sentia brusco) y queda debajo del limite de 0.4 PSI/s de la lectura
 * filtrada, asi que el PID siempre ve la presion real. Lo usan MODO=1,
 * SET_PRESION (MODO=3), la subida de MODO=2 (main.c) y la subida de CALIB
 * 9/14 (autotune_psi.c, desde 2026-10-13). */
#define PRESION_LOCAL_TASA_MAX_SUBIDA_PSI_S       0.35f

/**
 * Resetea el estado interno (integral, ancla de tiempo). Llamar en el
 * flanco de entrada a la cascada (presionLocalActivoAhora en main.c), mismo
 * motivo que PID_Init(): evita un dt inflado por tiempo inactivo en el
 * primer calculo.
 */
void PresionPid_Init(void);

/**
 * Como PresionPid_Init(), pero precarga el integral para que el primer
 * setpoint salga en rpmActual en vez de RPM_MIN (arranque sin salto). Para
 * MODO=3 al pasar de SET_RPM a SET_PRESION con el motor ya acelerado
 * (bug de campo 2026-10-07: el motor caia a ralenti).
 */
void PresionPid_InitDesdeRpm(float rpmActual);

/**
 * Calcula el setpoint de RPM para este ciclo, ya recortado a
 * [0, RPM_MAX_CARGA] (2026-10-02, antes RPM_MAX). El paso de integracion
 * se calcula por tiempo real transcurrido (HAL_GetTick()).
 *
 * La salida es RPM_MIN (piso feedforward) + P + I, mismo patron que
 * pid.c (SERVO_PULSO_MIN + correccion) -- agregado 2026-09-21 porque
 * sin piso el termino integral tenia que reconstruir TODO el RPM real
 * necesario desde 0 y parecia que el lazo no reaccionaba.
 *
 * Anti-windup: hacia arriba en RPM_MAX_CARGA y hacia abajo en RPM_MIN
 * (2026-10-02: antes el integral se vaciaba hasta pedir 0 RPM y tardaba en
 * reaccionar cuando la presion volvia a bajar). La SALIDA si puede bajar
 * hasta 0 a proposito: con la presion muy por encima del objetivo el
 * termino P pide "sin comandar" y controlSolicitado cae a ralenti natural.
 *
 * @param setpointPresionPsi  Presion objetivo, ej. CalibFlash_GetPresionObjetivoLocal().
 * @param presionMedidaPsi    Presion real medida, ej. PresionV_GetPresionPsi().
 * @return Setpoint de RPM a pasarle al lazo interno (PID_CalcularSalidaUs).
 */
float PresionPid_CalcularSetpointRpm(float setpointPresionPsi, float presionMedidaPsi);

/**
 * Ganancias TEMPORALES (solo RAM, no tocan flash) que pisan a
 * PID_PSI_KP/KI mientras esten activas -- uso exclusivo de la autosintonia
 * (autotune_psi.c, CALIB=14). Limpiar siempre al terminar/abortar.
 */
void PresionPid_SetGananciasTemporales(float kp, float ki);
void PresionPid_LimpiarGananciasTemporales(void);

#ifdef __cplusplus
}
#endif

#endif /* PRESION_PID_H */
