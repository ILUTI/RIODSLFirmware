/**
 * @file    pid.h
 * @brief   Lazo de control PID de RPM -> pulso del servo del acelerador.
 *
 * Cierra el lazo entre la RPM real (tacometro.c) y el pulso del servo
 * (servo.c): la salida es directamente el ancho de pulso en
 * microsegundos que se le manda a Servo_MoverHacia(), usando
 * SERVO_PULSO_MIN (posición sin aceleración, ver servo.h) como línea
 * base -- la corrección del PID se suma sobre esa base, no la
 * reemplaza.
 *
 * No decide POR SÍ MISMO cuándo debe correr ni de dónde sale el
 * setpoint -- eso lo resuelve el llamador (main.c): hoy el setpoint es
 * SET_RPM (downlink directo, uso de prueba); más adelante lo dará la
 * máquina de estados Modo 0/1/2 (ver README sección 4.3) según la
 * presión, sin que este módulo tenga que cambiar.
 *
 * Ganancias (PID_RPM_KP/KI) vienen de calibracion_flash.h -- mientras
 * no se sintonicen en el motor real (ver README sección 9, método de
 * ganancia última / Ziegler-Nichols en lazo cerrado), quedan en sus
 * valores default (Kp=1.0, Ki=0), suficientes para validar que
 * el lazo completo mueve el servo en la dirección correcta, no para
 * un control ya afinado.
 *
 * Solo P+I (sin derivativo) -- PID_KD se eliminó del protocolo
 * 2026-09-23: el ruido de RPM lo hacía inútil en la práctica (ver
 * README sección 9).
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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
 * @param setpointRpm  RPM objetivo (el llamador es responsable de
 *                      recortarla contra RPM_MIN/RPM_MAX antes de
 *                      pasarla -- ver nota en
 *                      CalibFlash_SetSetRpm()).
 * @param rpmMedida    RPM real, típicamente Tacometro_GetRPMFiltrada().
 * @return Pulso en µs a aplicar (pásalo a Servo_MoverHacia()).
 */
uint16_t PID_CalcularSalidaUs(float setpointRpm, float rpmMedida);

/**
 * Ganancias TEMPORALES (solo RAM, no tocan flash) que pisan a
 * PID_RPM_KP/KI mientras esten activas -- uso exclusivo de la autosintonia
 * (autotune_rpm.c, CALIB=13). Limpiar siempre al terminar/abortar.
 */
void PID_SetGananciasTemporales(float kp, float ki);
void PID_LimpiarGananciasTemporales(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
