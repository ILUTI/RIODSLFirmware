/**
 * @file    autotune_psi.h
 * @brief   Autosintonia del lazo de PRESION LOCAL (PID#2, cascada
 *          presion -> RPM), CALIB_AUTOTUNE_PID2.
 *
 * ⚠️ NO PROBADA CON MOTOBOMBA REAL (2026-09-30: la bomba no estaba
 * disponible). Compila y su nucleo de calculo (autotune.c) se probo contra
 * una planta simulada, pero la secuencia completa hay que validarla en
 * banco antes de confiar en ella.
 *
 * Requisitos: MODO=CALIBRACION, motor operando CON la bomba embragada,
 * PID#1 ya sintonizado (PID_RPM_KP > 0), PRESION_OBJETIVO_LOCAL y
 * TIEMPO_LLENADO_S configurados, PRESION_MAX > objetivo.
 *
 * Secuencia (sin laptop; dura ~TIEMPO_LLENADO_S + ~10-15 min):
 *   1. BASE      : SET_RPM=RPM_MIN, mide la presion base (5 s).
 *   2. SUBIDA    : rampa de SET_RPM en lazo abierto, PAUSADA por PSI real al
 *                  ritmo de llenado (PRESION_OBJETIVO_LOCAL / TIEMPO_LLENADO_S,
 *                  regla de seguridad de la tuberia) hasta
 *                  PRESION_OBJETIVO_LOCAL. Registra el RPM al cruzar 4 tramos
 *                  de presion y con la K del primero calcula la Kp de PRUEBA
 *                  (la que da G_cl = 0.3, como `sugerir-kp-presion`).
 *   3. VOLVER    : regresa a RPM_MIN y espera presion estable.
 *   4. ID        : escalon CERRADO de +5 PSI desde ralenti con la Kp de
 *                  prueba y Ki=0 (180 s), identificado como
 *                  `identify-presion` -- el mismo ensayo con el que salio la
 *                  1.72/0.39 de campo.
 *   5. SIMC      : Kp/Ki candidatas, fila "Medio" (tau_c = 2L).
 *   6. VALIDACION: vuelve a ralenti, espera presion estable y prueba la
 *                  candidata en la cascada REAL con un escalon de +5 PSI desde
 *                  ralenti (o hasta el objetivo si queda mas cerca), con vigia
 *                  de oscilacion en vivo. Un escalon grande hasta el objetivo
 *                  iria sin rampa de llenado, por eso no se hace.
 *   7. Aprobada -> persiste PID_PSI_KP/KI. Si no, reintenta con tau_c x2
 *                  (hasta 3 veces). Si ninguna pasa, deja las ganancias
 *                  guardadas como estaban.
 * Todo con ganancias temporales en RAM; flash se escribe una sola vez, al
 * final. PRESION_OBJETIVO_LOCAL tampoco se modifica (objetivo de prueba en
 * RAM, ver AutotunePsi_ObjetivoPsi).
 */

#ifndef AUTOTUNE_PSI_H
#define AUTOTUNE_PSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "autotune.h"
#include "calibracion_flash.h"

/* Etapas sueltas (2026-10-01, reemplazan a los viejos CALIB 9/10/11
 * manuales). Ninguna escribe flash:
 *   CALIB 9  CURVA   : base + subida pausada por PSI real hasta
 *                      PRESION_OBJETIVO_LOCAL (log PRESION_GANANCIA_CAL,PASO);
 *                      calcula la K por tramos y deja la Kp de PRUEBA en RAM.
 *   CALIB 10 ID      : con la Kp de prueba de CALIB 9 (si no hay, aborta):
 *                      base, escalon CERRADO de +5 PSI con Ki=0 (180 s, log
 *                      PRESION_PID_TEST -> `identify-presion --kp`), SIMC
 *                      Medio, imprime la candidata.
 *   CALIB 11 VALIDAR : base + validacion zona 1 (+5 PSI desde ralenti) de las
 *                      PID_PSI_KP/KI YA guardadas, sin reintentos.
 *   CALIB 14 completo (9 -> 10 -> validacion), el unico que guarda. */

/** true si 'calib' es alguna de las etapas de esta rutina (9/10/11/14). */
static inline bool AutotunePsi_EsCalib(uint8_t calib)
{
    return calib == CALIB_PID2_CURVA || calib == CALIB_PID2_ID
        || calib == CALIB_PID2_VALIDAR || calib == CALIB_AUTOTUNE_PID2;
}

/** Llamar UNA vez al entrar a una etapa (flanco de entrada); 'etapa' = el CALIB. */
void AutotunePsi_Iniciar(uint8_t etapa);
void AutotunePsi_Paso(bool motorOperando, bool modoCalibracion);
void AutotunePsi_Limpiar(void);
Autotune_Resultado_t AutotunePsi_UltimoResultado(void);

/** true mientras corre la cascada real (escalon de identificacion y
 *  validacion): main.c corre presion_pid.c. Fuera de eso el SET_RPM es directo. */
bool  AutotunePsi_CascadaActiva(void);

/** Objetivo de presion de la validacion en curso (RAM; no toca flash). */
float AutotunePsi_ObjetivoPsi(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTOTUNE_PSI_H */
