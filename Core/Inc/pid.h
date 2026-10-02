/**
 * @file    pid.h
 * @brief   Nucleo P+I compartido (Pi_*) y lazo de control de RPM -> pulso
 *          del servo del acelerador (PID#1).
 *
 * NUCLEO COMPARTIDO (2026-10-02): PID#1 (este archivo) y PID#2
 * (presion_pid.c) eran casi el mismo codigo; ahora usan Pi_Calcular(), que
 * hace el P+I con anti-windup por integracion condicional y ganancias
 * temporales. Cada lazo le pasa sus propios limites. Para el PID#1 las
 * cuentas son las mismas de antes, en el mismo orden.
 *
 * PID#1 cierra el lazo entre la RPM real (tacometro.c) y el pulso del servo
 * (servo.c): la salida es directamente el ancho de pulso en microsegundos
 * que se le manda a Servo_MoverHacia(), usando SERVO_PULSO_MIN (posición
 * sin aceleración, ver servo.h) como línea base -- la corrección se suma
 * sobre esa base, no la reemplaza.
 *
 * No decide POR SÍ MISMO cuándo debe correr ni de dónde sale el setpoint
 * -- eso lo resuelve main.c según MODO: SET_RPM directo (MODO 3/4), la
 * salida del PID#2 (MODO 1/2), o lo que pida un autotune.
 *
 * Ganancias PID_RPM_KP/KI (calibracion_flash.h). Default de fábrica Kp=1,
 * Ki=0 (solo para ver que el servo se mueve hacia el lado correcto); las de
 * campo validadas son Kp=0.047 / Ki=0.11, y CALIB=13 las calcula solo
 * (SIMC, ver autotune_rpm.h y README sección 9).
 *
 * Solo P+I (sin derivativo) -- PID_KD se eliminó del protocolo
 * 2026-09-23: el ruido de RPM lo hacía inútil en la práctica.
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==================== NUCLEO P+I COMPARTIDO ==================== */

typedef struct {
    float    integral;
    uint32_t ultimoCalculoMs;
    bool     temporalesActivas;  /* ganancias de prueba en RAM (autotunes) */
    float    kpTemporal;
    float    kiTemporal;
} Pi_t;

/** Resetea integral y ancla de tiempo (flanco de entrada al lazo). */
void Pi_Init(Pi_t *pi);

/** Ganancias TEMPORALES (solo RAM) que pisan a las guardadas mientras
 *  esten activas -- para que los autotunes prueben candidatas sin escribir
 *  flash. Limpiar siempre al terminar/abortar. */
void Pi_SetGananciasTemporales(Pi_t *pi, float kp, float ki);
void Pi_LimpiarGananciasTemporales(Pi_t *pi);

/**
 * Corrección P+I de este ciclo: kp*error + ki*integral, recortada a
 * [salidaMin, salidaMax]. El integral solo acumula si la salida tentativa no
 * pasa de [awMin, awMax] en la dirección del error (anti-windup). El dt se
 * mide con HAL_GetTick(). kpGuardada/kiGuardada se usan salvo que haya
 * ganancias temporales activas.
 */
float Pi_Calcular(Pi_t *pi, float error, float kpGuardada, float kiGuardada,
                  float awMin, float awMax, float salidaMin, float salidaMax);

/* ==================== PID#1: RPM -> SERVO ==================== */

/**
 * Resetea el estado interno (integral, ancla de tiempo). Llamar
 * siempre en el flanco de entrada a "PID activo" -- si el lazo
 * llevaba rato sin correr, el primer cálculo después de un Init usa
 * un dt chico en vez de uno inflado por el tiempo inactivo.
 */
void PID_Init(void);

/**
 * Calcula la salida del lazo para este ciclo, en µs directos, ya
 * recortada contra SERVO_PULSO_MIN/MAX (vía CalibFlash). El paso de
 * integración se calcula por tiempo real transcurrido
 * (HAL_GetTick()), no por conteo de llamadas.
 *
 * @param setpointRpm  RPM objetivo (main.c ya la recorta a RPM_MAX o
 *                      RPM_MAX_CARGA antes de pasarla).
 * @param rpmMedida    RPM real, típicamente Tacometro_GetRPMFiltrada().
 * @return Pulso en µs a aplicar (pásalo a Servo_MoverHacia()).
 */
uint16_t PID_CalcularSalidaUs(float setpointRpm, float rpmMedida);

/**
 * Ganancias TEMPORALES (solo RAM, no tocan flash) que pisan a
 * PID_RPM_KP/KI mientras esten activas -- uso exclusivo de la autosintonia
 * (autotune_rpm.c, CALIB 5/6/7/13). Limpiar siempre al terminar/abortar.
 */
void PID_SetGananciasTemporales(float kp, float ki);
void PID_LimpiarGananciasTemporales(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
