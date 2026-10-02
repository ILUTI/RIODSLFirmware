/**
 * @file    tacometro.h
 * @brief   Módulo de medición de RPM vía Input Capture (TIM2, canal 1).
 *
 * Mide el período entre flancos de SUBIDA de la señal proveniente del
 * H11AA1 (terminal W del alternador, ya acondicionada) -- TIM2 en modo PWM
 * Input: canal 1 = período (subida), canal 2 = ancho del pulso (bajada), con
 * el contador reseteado en cada subida (Slave Mode Reset) -- y calcula
 * frecuencia de pulsos y RPM con:
 *   - Filtrado de ruido en tres capas: filtro de hardware del timer (Input
 *     Capture Filter), guarda mínima de período y rango de duty cycle por
 *     software (el ancho del pulso se mide en el canal 2, indirecto).
 *   - Arranque: con el motor detenido hacen falta
 *     TACOMETRO_CAPTURAS_PARA_ARRANQUE capturas válidas seguidas para
 *     declararlo operando (ruido suelto no "prende" el motor).
 *   - Detección de motor detenido (timeout sin capturas nuevas).
 *   - Filtro de suavizado (media móvil exponencial) para no entregarle
 *     al lazo de control una lectura con jitter.
 *
 * Uso típico en main.c:
 *   CalibFlash_Init();                          // ver calibracion_flash.h
 *   Tacometro_Init(&htim2, TIM_CHANNEL_1);
 *   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // periodo (con interrupcion)
 *   HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);      // ancho del pulso (indirecto)
 *   ...
 *   while (1) {
 *       Tacometro_Update();
 *       float rpm = Tacometro_GetRPMFiltrada();
 *   }
 *
 * Y en el callback global de HAL (en main.c, USER CODE):
 *   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
 *       Tacometro_CaptureCallback(htim);
 *   }
 *
 * Calibración en tiempo de ejecución:
 *   Pulsos por revolución y ALPHA del filtro viven en flash
 *   (calibracion_flash.c; sus valores de fábrica son
 *   DEFAULT_PULSOS_POR_REVOLUCION / DEFAULT_ALPHA_FILTRO). Los downlinks
 *   SET_RATIO / SET_RATIO_AUTO / ALPHA los guarda el dispatcher de
 *   calibracion_flash.c, y la auto-calibración CALIB=3 usa
 *   CalibFlash_SetAlphaFiltro(). Tacometro_Update() los lee en cada
 *   cálculo, así que valen al instante y también después de un reset.
 */

#ifndef TACOMETRO_H
#define TACOMETRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACIÓN AJUSTABLE ==================== */

/* Guarda mínima de período por software (µs). Descarta capturas que
 * lleguen más rápido que este intervalo (ruido/rebote), como segunda
 * capa además del filtro de hardware del timer (Input Capture Filter).
 * Calculado con margen sobre el techo real medido en banco (~1800-1850
 * pulsos/seg con el clamp instalado, ver reporte secc. 3.7/7.6).
 */
#define TACOMETRO_PERIODO_MINIMO_US       450U

/* Timeout sin capturas nuevas para declarar el motor detenido (ms).
 * A la RPM mínima real de interés (cranking, ~150 RPM), el período entre
 * pulsos es de decenas de ms — 500ms da margen amplio sin retrasar
 * demasiado la detección de "motor parado".
 */
#define TACOMETRO_TIMEOUT_DETENIDO_MS     500U

/* Resolución del timer: 1 tick = 1µs (ajustar si se cambia el Prescaler
 * en el .ioc; debe coincidir exactamente con esa configuración). */
#define TACOMETRO_TICKS_POR_SEGUNDO       1000000UL

/* Rango de duty cycle aceptado como "semiciclo real" del H11AA1 (~50%
 * esperado). Un glitch de disparo simétrico del clamp Zener (dos LEDs
 * antiparalelos conduciendo por igual) produce picos angostos muy por
 * debajo de este rango — ver validación de banco, disparo simétrico
 * ±8.2V duplicando 300Hz -> 600Hz con duty << 30%.
 */
#define TACOMETRO_DUTY_MINIMO   0.30f
#define TACOMETRO_DUTY_MAXIMO   0.70f

/* Capturas válidas SEGUIDAS necesarias para pasar de "detenido" a
 * "operando" (hallazgo B18, 2026-10-02). A ~150 RPM de arranque con 17.5
 * pulsos/vuelta llegan ~44 pulsos/s: 3 son ~70 ms de demora, imperceptible. */
#define TACOMETRO_CAPTURAS_PARA_ARRANQUE  3U

/* ==================== API PÚBLICA ==================== */

/**
 * Inicializa el módulo. Debe llamarse una vez en main(), después de
 * CalibFlash_Init() (para tener disponible la calibración guardada,
 * si existe) y después de que el timer ya fue inicializado por
 * MX_TIM2_Init() (generado por CubeMX), pero antes de arrancar la
 * captura con HAL_TIM_IC_Start_IT().
 */
void Tacometro_Init(TIM_HandleTypeDef *htim, uint32_t canal);

/**
 * Debe llamarse desde el callback global HAL_TIM_IC_CaptureCallback()
 * en main.c, pasando el mismo puntero de timer que recibe ese callback.
 * Es seguro llamarlo aunque el callback dispare por otros timers/canales:
 * la función verifica internamente que sea el timer/canal correcto.
 */
void Tacometro_CaptureCallback(TIM_HandleTypeDef *htim);

/**
 * Debe llamarse periódicamente desde el loop principal (cada iteración,
 * o al menos cada pocos ms). Actualiza la detección de timeout y el
 * filtro de suavizado. No hace nada pesado — es seguro llamarla seguido.
 */
void Tacometro_Update(void);

/** RPM instantánea, sin suavizar (última medición cruda válida). */
float Tacometro_GetRPMInstantanea(void);

/** RPM suavizada con el filtro de media móvil — usar esta para control. */
float Tacometro_GetRPMFiltrada(void);

/** Frecuencia de pulsos en Hz (para diagnóstico/depuración). */
float Tacometro_GetFrecuenciaHz(void);

/** true si el motor se considera detenido (timeout sin pulsos nuevos). */
bool Tacometro_EstaDetenido(void);

/** Contador de capturas descartadas por software (período demasiado corto
 * o duty cycle fuera de rango = ruido filtrado). Útil para diagnóstico en
 * campo. */
uint32_t Tacometro_GetContadorRuidoFiltrado(void);

#ifdef __cplusplus
}
#endif

#endif /* TACOMETRO_H */
