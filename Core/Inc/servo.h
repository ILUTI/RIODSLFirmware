/**
 * @file    servo.h
 * @brief   Control del servo del acelerador (MG996R/análogo) vía PWM
 *          (TIM3_CH3, PB0).
 *
 * NOTA: se usa PB0 (TIM3_CH3) en vez de PA6 (TIM3_CH1) porque en el
 * Nucleo-32 (NUCLEO-G431KB), el header físico donde vive PA6 comparte
 * pin con PA15 mediante el solder bridge SB3, cuyo estado por defecto
 * de fábrica enruta ese header a PA15, no a PA6 (ver UM2397, Table 9).
 * PB0/TIM3_CH3 evita ese problema sin necesidad de resoldar nada.
 *
 * El timer ya está configurado (MX_TIM3_Init, generado por CubeMX) a
 * 1 tick = 1µs, periodo de 20,000 ticks = 20ms (50Hz) -- el estándar
 * de servos de RC. Este módulo solo escribe el registro de
 * comparación (ancho de pulso), en microsegundos directos, con
 * recorte de seguridad contra los límites mecánicos configurados
 * (SERVO_PULSO_MIN/MAX en calibracion_flash.h).
 *
 * Uso típico en main.c:
 *   Servo_Init(&htim3, TIM_CHANNEL_3);
 *   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
 *   ...
 *   Servo_SetPulsoUs(CalibFlash_GetServoPulsoMinUs());  // posición segura (sin aceleración) al arrancar
 *
 * IMPORTANTE (prueba en banco antes de conectar a la varilla real):
 *   Antes de conectar el servo a la varilla del acelerador, pruébalo
 *   suelto, confirmando visualmente que los valores de
 *   SERVO_PULSO_MIN/MAX corresponden a los topes mecánicos deseados
 *   y que el servo no "tiembla" ni se sale de rango.
 */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Velocidad máxima de movimiento del servo, en µs de pulso por segundo.
 * Protección mecánica: limita qué tan rápido puede saltar el pulso
 * aplicado de un ciclo a otro (evita choques en la varilla del
 * acelerador ante una corrección brusca del PID). Distinto de
 * TASA_MAX_CAMBIO_RPM_S (esa limita el setpoint de RPM, no el pulso
 * físico del servo). 1250 µs/s == el barrido de la prueba de banco
 * original (25µs cada 20ms).
 *
 * PROBADO en 200 µs/s (2026-09-08) para ver si actuaba como filtro
 * extra sobre correcciones bruscas/ruido de medicion -- revertido a
 * 1250 el mismo dia: dejaba el barrido/calibracion manual (=1/=2)
 * demasiado lento para uso practico (un ciclo completo del barrido
 * pasaba de ~2.4s a ~9s con el rango real de ~526us), y no se llegó a
 * confirmar ninguna mejora real sobre la oscilacion lenta observada en
 * el PID (esa prueba se interrumpio por un reset de la calibracion en
 * flash, no relacionado a este valor -- ver README seccion 9). */
#define SERVO_VELOCIDAD_MAX_US_S    1250U

/* Pausa en cada extremo del barrido de calibración (main.c,
 * CALIB=2), en ms. El pulso PWM comandado llega exacto a
 * SERVO_PULSO_MIN/MAX apenas se cumple el límite de velocidad de
 * arriba, pero el servo físico tiene su propio tiempo de asentamiento
 * (no es instantáneo) -- sin esta pausa, el firmware invierte la
 * dirección en la misma vuelta del loop en que el pulso llega al
 * extremo, y el brazo real nunca alcanza a terminar de llegar antes de
 * que se le pida ir al otro lado. No configurable por downlink, a
 * propósito -- es un margen mecánico fijo, igual que
 * SERVO_VELOCIDAD_MAX_US_S. */
#define SERVO_BARRIDO_PAUSA_EXTREMOS_MS    1000U

/* ==================== API PÚBLICA ==================== */

/**
 * Inicializa el módulo. Debe llamarse una vez en main(), después de
 * que el timer ya fue inicializado por MX_TIM3_Init() (generado por
 * CubeMX). No arranca el PWM por sí solo -- llamar
 * HAL_TIM_PWM_Start(htim, canal) por separado, igual que se hace con
 * el Input Capture del tacómetro.
 */
void Servo_Init(TIM_HandleTypeDef *htim, uint32_t canal);

/**
 * Mueve el servo al ancho de pulso indicado, en microsegundos.
 * El valor se recorta (clamp) contra
 * CalibFlash_GetServoPulsoMinUs()/GetServoPulsoMaxUs() antes de
 * aplicarse -- nunca se manda un pulso fuera de esos límites, sin
 * importar qué tan fuera de rango venga el valor solicitado.
 *
 * @param microsegundos  Ancho de pulso deseado, en µs.
 * @return El valor REAL aplicado (después del recorte), útil para
 *         que el llamador sepa si su solicitud se ajustó.
 */
uint16_t Servo_SetPulsoUs(uint16_t microsegundos);

/**
 * Mueve el servo gradualmente hacia el ancho de pulso indicado, sin
 * bloquear -- pensada para llamarse en cada vuelta del loop principal
 * (cadencia irregular; el paso se calcula por tiempo transcurrido
 * real vía HAL_GetTick(), no por conteo de llamadas). Cada llamada
 * avanza como máximo lo que permite SERVO_VELOCIDAD_MAX_US_S desde la
 * llamada anterior; si el destino ya se alcanzó, no hace nada más que
 * mantenerlo.
 *
 * Único camino de movimiento del servo mientras hay un lazo activo:
 * tanto el barrido de calibración (main.c) como la salida de pid.c
 * (en µs directos, ver README sección 2.4/4.4) llaman esta función en
 * vez de Servo_SetPulsoUs() directamente, para heredar el límite de
 * velocidad mecánica.
 *
 * @param destinoUs  Ancho de pulso deseado, en µs (se recorta contra
 *                    SERVO_PULSO_MIN/MAX igual que Servo_SetPulsoUs()).
 * @return El valor REAL aplicado en este ciclo (después del límite de
 *         velocidad y del recorte por límites mecánicos).
 */
uint16_t Servo_MoverHacia(uint16_t destinoUs);

/** Último ancho de pulso realmente aplicado (después de cualquier
 * recorte por límites). Útil para telemetría/depuración. Esta es la
 * convención LOGICA (comparable directo contra SERVO_PULSO_MIN/MAX) --
 * si SERVO_SENTIDO_INVERTIDO esta activo en servo.c, el pulso FISICO
 * que de verdad recibe el registro del timer es distinto, ver
 * Servo_GetPulsoFisicoActualUs(). */
uint16_t Servo_GetPulsoActualUs(void);

/** Último ancho de pulso FISICO que efectivamente se escribio en el
 * registro de comparacion del timer (__HAL_TIM_SET_COMPARE) -- si
 * SERVO_SENTIDO_INVERTIDO esta en 0 es identico a
 * Servo_GetPulsoActualUs(); si esta en 1, es el valor ya espejado.
 * Agregado para depurar en campo el sentido de giro (ver SERVO_CAL en
 * main.c) -- comparar este valor contra lo que se observa girar
 * fisicamente en el servo. */
uint16_t Servo_GetPulsoFisicoActualUs(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
