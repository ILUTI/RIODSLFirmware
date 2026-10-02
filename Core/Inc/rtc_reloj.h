/**
 * @file    rtc_reloj.h
 * @brief   Reloj interno (RTC) del STM32G431 -- usa LSI (no hay cristal
 *          LSE poblado en este Nucleo-32), SIN respaldo de batería/VBAT
 *          dedicado. Consecuencia práctica: la hora se PIERDE en cada
 *          corte real de energía (no en un NVIC_SystemReset() mientras
 *          VDD no se interrumpa) -- hay que re-sincronizar contra la red
 *          LoRaWAN (DeviceTimeReq) después de cada arranque. Ver
 *          Reloj_EstaSincronizado().
 *
 * El RTC guarda la hora en UTC (la misma que entrega DeviceTimeReq/
 * AT+LTIME) -- el offset de -6h de Guatemala se aplica SOLO al leer la
 * hora para el payload de salida (Reloj_GetUnixTimeLocal()), nunca se
 * le resta al RTC en sí, para no mezclar "hora que guarda el RTC" con
 * "hora que se manda por LoRaWAN".
 *
 * ⚠️ LSI es impreciso (típicamente ±5-10% de fábrica, y deriva con
 * temperatura) -- para un reporte cada 30s esto es aceptable (unos
 * pocos segundos de error entre resincronizaciones), pero NO usar este
 * reloj para nada que requiera precisión de sub-segundo o largos
 * períodos sin resincronizar.
 */
#ifndef RTC_RELOJ_H
#define RTC_RELOJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"  /* para RTC_HandleTypeDef */

#define GUATEMALA_UTC_OFFSET_SEGUNDOS   (-6 * 3600)

/** Debe llamarse una sola vez en main(), después de MX_RTC_Init(). */
void Reloj_Init(RTC_HandleTypeDef *hrtc);

/** Setea el RTC directo desde campos de calendario UTC (hora/fecha ya
 * separados). Ej. real de AT+LTIME=? en RUI3: "AT+LTIME=15h08m55s on 08/17/2026"
 * (formato MM/DD/YYYY, confirmado en campo 2026-08-17) -- parsear eso
 * en el llamador y pasar los campos sueltos aca. Marca el reloj como
 * sincronizado -- ver Reloj_EstaSincronizado(). */
void Reloj_SetHoraUtc(uint16_t anio, uint8_t mes, uint8_t dia,
                       uint8_t hora, uint8_t minuto, uint8_t segundo);

/** true si el RTC ya fue seteado al menos una vez desde el arranque
 * actual (por DeviceTimeReq). false significa que la hora que devuelve
 * el RTC es solo el valor por defecto de MX_RTC_Init() -- NO confiable,
 * no se debe mandar en ningún uplink todavía. */
bool Reloj_EstaSincronizado(void);

/** Epoch UTC actual según el RTC (sin aplicar ningún offset). */
uint32_t Reloj_GetUnixTimeUtc(void);

/** Epoch YA con el offset de Guatemala aplicado (-6h) -- este es el
 * valor que se debe mandar en 'fecha_hora'/'inicio_operacion' del
 * payload LoRaWAN, para que el Lambda no tenga que restar nada de
 * nuevo del otro lado (ver decoder.py, _formatear_fecha()). */
uint32_t Reloj_GetUnixTimeLocal(void);

#ifdef __cplusplus
}
#endif

#endif /* RTC_RELOJ_H */
