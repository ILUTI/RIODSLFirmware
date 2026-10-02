/**
 * @file    autotune_rpm.h
 * @brief   Autosintonia del lazo INTERNO RPM -> servo (PID#1), CALIB_AUTOTUNE_PID1.
 *
 * Secuencia completa, sin laptop (requiere MODO=CALIBRACION y el motor
 * operando, bomba DESEMBRAGADA):
 *   1. BASE      : SET_RPM=0 (ralenti), mide la RPM base y0 (3 s).
 *   2. CURVA     : barrido de ganancia en lazo abierto (mismo metodo que
 *                  CALIB=5, el servo lo mueve esta rutina) -> K local
 *                  MAXIMA (peor caso) y la RPM donde ocurre. Es lo que hace
 *                  `pid_tuning.py auto --ganancia-log`: SIMC usa ese K, no
 *                  el de un solo punto.
 *   3. ID        : escalon en LAZO ABIERTO del pulso del servo (sin PID),
 *                  del tamaño que en la curva dio +200 RPM; graba 30 s e
 *                  identifica (K, tau, L). NO con Kp=1/Ki=0 como CALIB=6:
 *                  en la semi-simulacion calibrada contra campo ese lazo P
 *                  oscila (ver autotune_rpm.c).
 *   4. SIMC      : calcula Kp/Ki candidatas con tau_c = max(2L, 3*tau) (el
 *                  3*tau es lo que ya se valido en campo), con
 *                  K = max(K del escalon, K peor caso de la curva).
 *   5. VALIDACION: prueba la candidata en el motor real con vigia de
 *                  oscilacion en vivo, en DOS zonas: el escalon chico
 *                  (idle+200) y un escalon hasta la RPM del peor caso de la
 *                  curva (donde mas riesgo hay de oscilar). Evalua
 *                  sobre-impulso / asentamiento / error final / cruces.
 *   6. Si pasan ambas -> persiste Kp/Ki en flash. Si no -> reintenta con un
 *                  tau_c mas conservador (x2), hasta AUTOTUNE1_INTENTOS_MAX.
 *                  Si ninguna pasa, NO toca las ganancias guardadas.
 * Todo el ensayo usa ganancias temporales en RAM (pid.h): flash solo se
 * escribe UNA vez, al final, con una candidata ya validada en el motor.
 *
 * Etapas sueltas (2026-10-01, reemplazan a los viejos CALIB 5/6/7 manuales).
 * Ninguna escribe flash; todas dejan el log para `pid_tuning.py`:
 *   CALIB 5  CURVA   : pasos 1-2, termina al cerrar la curva (log GANANCIA_CAL).
 *   CALIB 6  ID      : base + pasos 3-4 con la curva que dejo CALIB 5 en RAM
 *                      (no la repite; si no hay, aborta), imprime la Kp/Ki
 *                      candidata y termina (log PID_TEST del escalon abierto).
 *   CALIB 7  VALIDAR : paso 1 + validacion zona 1 de las Kp/Ki YA guardadas,
 *                      sin reintentos (log PID_TEST para `compare`).
 *   CALIB 13 completo, el unico que guarda.
 */

#ifndef AUTOTUNE_RPM_H
#define AUTOTUNE_RPM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "autotune.h"   /* Autotune_Resultado_t */
#include "calibracion_flash.h"

/** true si 'calib' es alguna de las etapas de esta rutina (5/6/7/13). */
static inline bool AutotuneRpm_EsCalib(uint8_t calib)
{
    return calib == CALIB_PID1_CURVA || calib == CALIB_PID1_ID
        || calib == CALIB_PID1_VALIDAR || calib == CALIB_AUTOTUNE_PID1;
}

/** Llamar UNA vez al entrar a una etapa (flanco de entrada); 'etapa' = el CALIB. */
void AutotuneRpm_Iniciar(uint8_t etapa);

/**
 * Llamar en cada pasada del loop mientras AutotuneRpm_EsCalib(CALIB).
 * Al terminar (bien, mal o abortada) deja CALIB=0 por si sola.
 */
void AutotuneRpm_Paso(bool motorOperando, bool modoCalibracion);

/** Llamar UNA vez al SALIR de la etapa por cualquier motivo:
 *  limpia las ganancias temporales y el SET_RPM. Idempotente. */
void AutotuneRpm_Limpiar(void);

Autotune_Resultado_t AutotuneRpm_UltimoResultado(void);

/**
 * true mientras la rutina esta en una fase de lazo abierto (barrido de la
 * curva o escalon de identificacion): en ese caso main.c debe llevar el
 * servo a *pulsoDestinoUs en vez de correr el PID (mismo patron que
 * CALIB=5). En el resto de las fases devuelve false y el servo lo maneja
 * pid.c normalmente.
 */
bool AutotuneRpm_ServoDirecto(uint16_t *pulsoDestinoUs);

#ifdef __cplusplus
}
#endif

#endif /* AUTOTUNE_RPM_H */
