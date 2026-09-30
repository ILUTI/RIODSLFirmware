/**
 * @file    presion_voltaje.h
 * @brief   Canal alternativo de presion: sensor de salida en voltaje
 *          (GPT203, variante 0.5-2.5V / 3.3V / 0-150PSI) via ADC2/PA4.
 *
 * TEMPORAL: mientras se resuelve el circuito de acondicionamiento del
 * lazo 4-20mA (LM358/LM324, ver presion.c/h y el README de hardware,
 * seccion 5), este modulo permite tener una lectura de presion real
 * para probar MODO/PID en cascada sin depender de ese circuito.
 * Independiente de presion.c/h -- no comparte estado ni estructura de
 * datos, usa su propio ADC (hadc2) para no interferir con nada de lo
 * que ya usa hadc1/PA1.
 *
 * Conexion del sensor (segun etiqueta real, confirmada 2026-09-14):
 *   Rojo (VDC)    -> 3.3V del STM32 (NO el VIN de 9V -- este sensor
 *                    especificamente pide 3.3V)
 *   Amarillo (Vout) -> PA4 (ADC2_IN17)
 *   Verde (GND)     -> GND comun
 *   Negro (PE)      -> blindaje/tierra de chasis, si la instalacion
 *                      real lo usa; si no, puede quedar sin conectar
 *
 * Sin circuito de acondicionamiento -- el rango 0.5V-2.5V del sensor
 * cabe directo dentro de los 0-3.3V del ADC, sin necesitar divisor ni
 * ganancia.
 *
 * Uso tipico en main.c:
 *   MX_ADC2_Init();                 // generado por CubeMX
 *   ...
 *   PresionV_Init(&hadc2);
 *   while (1) {
 *       PresionV_Update();
 *       float psi = PresionV_GetPresionPsi();
 *   }
 */

#ifndef PRESION_VOLTAJE_H
#define PRESION_VOLTAJE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CALIBRACION DEL SENSOR (etiqueta real) ==================== */

#define PRESIONV_ADC_CUENTAS_MAX      4095.0f

/* VDDA real: no se mide aqui -- se reutiliza Presion_GetVdda() (ver
 * presion.h), que ya lo calibra cada PRESION_VREFINT_INTERVALO_MS via
 * VREFINT en hadc1. VDDA es el mismo riel fisico para ADC1 y ADC2, asi
 * que no hace falta duplicar la logica de VREFINT en hadc2 -- basta con
 * que presion.c siga corriendo en paralelo (ya es el caso en main.c). */

/* Rango de salida del sensor (etiqueta): 0.5V-2.5V <-> 0-150 PSI */
#define PRESIONV_VOLTAJE_MIN          0.5f
#define PRESIONV_VOLTAJE_MAX          2.5f
#define PRESIONV_PSI_EN_VOLTAJE_MIN   0.0f
#define PRESIONV_PSI_EN_VOLTAJE_MAX   150.0f

/* Offset de calibracion en campo -- REVERTIDO a 0.0f (2026-09-18,
 * decision explicita del usuario): el desfase medido el 2026-09-14
 * (0.53V en vez de 0.5V al aire, ~2.41 PSI) resulto ser mas grande que
 * la propia tolerancia de fabrica del sensor (Accuracy 0.5% FS =
 * ±0.75 PSI en el rango de 150 PSI) -- probablemente ruido de la
 * medicion de ese momento (antes de agregar el promedio recortado),
 * no un offset real de la unidad. Se decidio no corregir nada, mismo
 * criterio que usa el compañero en su repo Sensor-de-Presion
 * (VOLTAGE_OFFSET_V = 0.0F en su config.h, con el mismo razonamiento:
 * el margen de 0.5% FS de fabrica no justifica el esfuerzo de
 * calibrar). Si en el futuro se sospecha un offset real (ej. se
 * reemplaza el sensor por uno claramente fuera de spec), volver a
 * medir con el sensor al aire (linea abierta a la atmosfera, motobomba
 * apagada) y ya con el filtro de promedio recortado activo antes de
 * concluir que el offset es real. */
#define PRESIONV_OFFSET_V             0.0f

/* Margen fuera de 0.5-2.5V antes de declarar el sensor en falla (cable
 * suelto, corto, sensor desconectado) -- igual criterio que
 * presion.c/Presion_SensorValido(), adaptado a este rango de voltaje. */
#define PRESIONV_FALLA_BAJA_V         0.2f
#define PRESIONV_FALLA_ALTA_V         3.0f

/* Interruptor de los filtros de ruido posteriores al promedio recortado
 * (limitador de velocidad + EMA, definidos mas abajo). DESACTIVADOS
 * (2026-09-29, a pedido del usuario): se pusieron cuando la lectura no
 * era fiel (GND mal ubicado del sensor, ya corregido), y juntos limitan
 * el cambio maximo a 0.4 PSI/s (ver README seccion 5). Con 0 la salida
 * es directamente el resultado del promedio recortado. Poner en 1 si
 * vuelve a aparecer ruido significativo en PresionV. El promedio
 * recortado de 21 muestras NO se desactiva. */
#define PRESIONV_FILTROS_ACTIVOS      0

/* Mismo criterio de suavizado que presion.c/tacometro.c -- se aplica
 * DESPUES del promedio recortado de abajo, no sobre la muestra cruda.
 *
 * Bajado de 0.2 a 0.08 (2026-09-21): con el PID en cascada ya
 * funcionando en campo se confirmo que la presion real tarda del
 * orden de 60-120s en asentarse tras un escalon de RPM (constante de
 * tiempo hidraulica del sistema motor+bomba+tuberia, ver README
 * seccion 9) -- un EMA mas lento (mas filtrado) sigue siendo
 * instantaneo comparado con esa dinamica real, y a cambio rechaza
 * mucho mejor el ruido de ±1-2 PSI que se veia en PRESION_PID_TEST
 * (alimentaba directo al lazo externo, amplificando su termino P). */
#define PRESIONV_ALPHA_FILTRO         0.08f

/* Promedio recortado (trimmed mean) sobre varias conversiones ADC
 * antes de convertir a PSI, para rechazar picos de ruido (rizado del
 * alternador acoplado al GND compartido, ver README hardware sec 5.6)
 * en vez de dejar que un solo dato corrupto contamine el EMA de
 * arriba -- mismo enfoque que usa el compañero en su repo
 * Sensor-de-Presion (measureVoltagePressure(), con insertion sort +
 * recorte de extremos), adaptado aqui sin delay() bloqueante: las
 * muestras se acumulan una por cada llamada a PresionV_Update() (que ya
 * se llama periodicamente desde el loop principal), no en un burst con
 * delay entre cada una.
 *
 * Subido de 11 a 21 muestras, recorte de 2 a 4 por lado (2026-09-21,
 * mismo motivo que el ALPHA de arriba -- hay mucho margen de tiempo
 * real para filtrar mas fuerte). Sigue siendo impar y el recorte sigue
 * siendo bien conservador (~38% de cada lado, deja 13 de 21) para no
 * sesgar el promedio si el ruido no es simetrico. */
#define PRESIONV_MUESTRAS_POR_PROMEDIO   21U  /* debe ser impar */
#define PRESIONV_RECORTE_EXTREMOS        4U   /* se descartan las 4 mas
                                                * altas y las 4 mas
                                                * bajas de cada tanda */

/* Limitador de velocidad de cambio (slew-rate), agregado 2026-09-22:
 * el promedio recortado rechaza hasta 4 muestras contaminadas de cada
 * lado dentro de UNA tanda, pero si una rafaga de ruido (encontrado en
 * campo: transmisiones/reintentos de join del RAK3172, que ya se sabe
 * que meten ruido electrico -- ver el comentario sobre
 * CALIB=8 saltando RAK3172_Update()/GPS_Update() en
 * main.c) contamina MAS de 4 de las 21 muestras de una tanda, el
 * promedio recortado igual se corre. En banco esto solo se vio con
 * CALIB=8 (que ya evita el RAK3172), pero en operacion
 * real (MODO=1, CALIB=0) el RAK3172 SI transmite uplinks
 * normalmente mientras el PID de presion esta activo -- este piso evita
 * que un salto de ese tipo (ej. +1.4 PSI de un instante a otro, muy por
 * encima de lo que la dinamica hidraulica real permite, tau~60-120s)
 * se filtre hacia PresionV_GetPresionPsi() y contamine el termino P del
 * lazo externo. 5.0 PSI/s es generoso: la dinamica real observada en
 * campo es de centesimas de PSI cada 100-200ms, muy por debajo de este
 * piso, asi que un cambio real nunca lo toca. */
#define PRESIONV_TASA_MAX_CAMBIO_PSI_S   5.0f

/* ==================== API PUBLICA ==================== */

/** Inicializa el modulo: guarda el handle del ADC2 y corre la
 * calibracion interna del ADC. Llamar una vez en main(), despues de
 * MX_ADC2_Init(). */
void PresionV_Init(ADC_HandleTypeDef *hadc);

/** Dispara una conversion, la lee, y actualiza la lectura filtrada.
 * Llamar periodicamente desde el loop principal. */
void PresionV_Update(void);

/** Presion filtrada, en PSI. */
float PresionV_GetPresionPsi(void);

/** Voltaje crudo leido en PA4 (diagnostico/depuracion). */
float PresionV_GetVoltaje(void);

/** true si el ultimo voltaje leido cae dentro de 0.5-2.5V (con margen,
 * ver PRESIONV_FALLA_BAJA_V/ALTA_V). Mientras sea false,
 * PresionV_GetPresionPsi() conserva el ultimo valor filtrado valido. */
bool PresionV_SensorValido(void);

#ifdef __cplusplus
}
#endif

#endif /* PRESION_VOLTAJE_H */
