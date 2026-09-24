/**
 * @file    presion.h
 * @brief   Lectura del sensor de presion de la motobomba (4-20mA) via ADC1/PA1.
 *
 * Sensor: PCM300-1M-G-B1-C15-J4 (0-1MPa, salida 4-20mA), ver README
 * seccion 5 para el circuito de acondicionamiento completo:
 *
 *   Lazo 4-20mA -> R_shunt (2x100 ohm en paralelo = 50 ohm) -> GND
 *                       |
 *                 LM358 (no inversora, ganancia 3: R9=2k/R8=1k)
 *                       |
 *                 PA1 (ADC1_IN2 del G431)
 *
 * Rango de salida del op-amp: 0.6V (4mA) a 3.0V (20mA), dentro del
 * rango 0-3.3V del ADC interno de 12 bits -- no hace falta ADC externo.
 *
 * Calibracion mA->PSI confirmada por el fabricante del sensor (tabla de
 * 17 puntos, perfectamente lineal): 4mA=0 PSI, 20mA=145.04 PSI.
 *
 * Uso tipico en main.c:
 *   MX_ADC1_Init();               // generado por CubeMX
 *   ...
 *   Presion_Init(&hadc1);
 *   while (1) {
 *       Presion_Update();
 *       float psi = Presion_GetPresionPsi();
 *   }
 */

#ifndef PRESION_H
#define PRESION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACION DEL CIRCUITO ==================== */

/* Tension de referencia del ADC (VDDA del G431) usada como arranque,
 * antes de la primera calibracion real via VREFINT -- ver
 * PRESION_VREFINT_INTERVALO_MS y Presion_GetVdda() mas abajo. El valor
 * medido real (Presion_GetVdda()) reemplaza esto en cuanto corre
 * Presion_Init(). */
#define PRESION_VREF_ADC_V_DEFAULT    3.3f

/* Intervalo de refresco de la medicion de VDDA via VREFINT (ms) --
 * VDDA varia lento (temperatura/carga de la fuente), no hace falta
 * medirlo en cada vuelta del loop; ver ActualizarVdda() en presion.c. */
#define PRESION_VREFINT_INTERVALO_MS  10000U

/* Resolucion del ADC1 (12 bits, ver ADC1.Init.Resolution en el .ioc). */
#define PRESION_ADC_CUENTAS_MAX       4095.0f

/* Ganancia del LM358 no inversor: 1 + R9/R8 = 1 + 2k/1k. */
#define PRESION_GANANCIA_OPAMP        3.0f

/* Resistencia de shunt equivalente (R10 || R11, 100 ohm cada una). */
#define PRESION_R_SHUNT_OHM           50.0f

/* Corriente del lazo 4-20mA en los extremos de la escala del sensor. */
#define PRESION_CORRIENTE_MIN_MA      4.0f
#define PRESION_CORRIENTE_MAX_MA      20.0f

/* Calibracion del fabricante (0-1MPa): PSI en cada extremo de la escala. */
#define PRESION_PSI_EN_CORRIENTE_MIN  0.0f
#define PRESION_PSI_EN_CORRIENTE_MAX  145.04f

/* Margen de tolerancia fuera de 4-20mA antes de declarar el sensor/lazo
 * en falla (cable cortado da ~0mA, corto/sobre-rango da >20mA) -- ver
 * Presion_SensorValido(). Margen generoso para no disparar falsos
 * positivos por ruido/offset del ADC cerca de los extremos. */
#define PRESION_CORRIENTE_FALLA_BAJA_MA   3.5f
#define PRESION_CORRIENTE_FALLA_ALTA_MA   20.5f

/* Coeficiente del filtro EMA aplicado a la lectura de presion, mismo
 * proposito que TACOMETRO_ALPHA_FILTRO -- suaviza el ruido del ADC sin
 * agregar lag perceptible frente a la dinamica lenta de una bomba. */
#define PRESION_ALPHA_FILTRO          0.2f

/* ==================== API PUBLICA ==================== */

/**
 * Inicializa el modulo: guarda el handle del ADC y corre la
 * calibracion interna del ADC (HAL_ADCEx_Calibration_Start()), que el
 * fabricante recomienda ejecutar una vez antes de la primera
 * conversion para reducir el offset de medicion.
 *
 * Debe llamarse una vez en main(), despues de MX_ADC1_Init().
 */
void Presion_Init(ADC_HandleTypeDef *hadc);

/**
 * Dispara una conversion, la lee (bloqueante, con timeout corto -- el
 * ADC interno del G431 resuelve en microsegundos, no hay riesgo real
 * de colgar el loop principal) y actualiza la lectura filtrada.
 * Llamar periodicamente desde el loop principal.
 */
void Presion_Update(void);

/** Presion filtrada, en PSI. */
float Presion_GetPresionPsi(void);

/** Corriente del lazo 4-20mA calculada, en mA (diagnostico/depuracion). */
float Presion_GetCorrienteMa(void);

/**
 * true si la ultima corriente leida cae dentro de 4-20mA (con margen,
 * ver PRESION_CORRIENTE_FALLA_BAJA_MA/ALTA_MA) -- false indica lazo
 * abierto (cable cortado, sensor desconectado) o corto/sobre-rango.
 * Mientras sea false, Presion_GetPresionPsi() sigue devolviendo el
 * ultimo valor filtrado valido (no se actualiza con lecturas en falla).
 */
bool Presion_SensorValido(void);

/** VDDA real medido via VREFINT (voltios), refrescado cada
 * PRESION_VREFINT_INTERVALO_MS -- lo reutiliza presion_voltaje.c
 * (hadc2) para no duplicar la logica de VREFINT, ya que VDDA es el
 * mismo riel fisico para ambos ADC. */
float Presion_GetVdda(void);

#ifdef __cplusplus
}
#endif

#endif /* PRESION_H */
