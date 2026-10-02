/**
 * @file    gps.h
 * @brief   Módulo de posición GPS/GNSS del módulo SIM7600X, vía
 *          auto-reporte periódico "+CGPSINFO:" sobre USART2 (PB3=TX,
 *          PB4=RX). No transmite NMEA crudo por su cuenta en esta UART
 *          (confirmado en campo) -- ver gps.c.
 *
 * Diseño de recepción (modo Circular + IDLE):
 *   A diferencia de rak3172.c (modo Normal, comando/respuesta), acá se
 *   usa DMA en modo Circular -- el buffer se rearma solo, y el evento
 *   de línea inactiva (IDLE) dispara el procesamiento de las sentencias
 *   que el módulo manda por su cuenta de forma continua (no hay
 *   "comando" que esperar como con el RAK3172).
 *
 * Armado del auto-reporte: responsabilidad del llamador (main.c),
 * mandando AT+CGPS=1 y AT+CGPSINFO=10 por GPS_EnviarComandoAT() después
 * de GPS_Init() -- ver ejemplo en main.c.
 *
 * Uso típico en main.c:
 *   GPS_Init(&huart2);
 *   ...
 *   while (1) {
 *       GPS_Update();
 *       if (GPS_TieneFix()) {
 *           latitud = GPS_GetLatitud();
 *           longitud = GPS_GetLongitud();
 *       }
 *   }
 *
 * Y en el callback global de HAL (en main.c, USER CODE):
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *       ...
 *       else if (huart->Instance == USART2) {
 *           GPS_RxEventCallback(huart, Size);
 *       }
 *   }
 */

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACIÓN AJUSTABLE ==================== */

/* Tamaño del buffer circular de recepción por DMA. A 115200 baud, una
 * ráfaga típica de sentencias NMEA (RMC+GGA+GSA+3xGSV+VTG+GLL) de un
 * receptor GNSS puede rondar 400-500 bytes por ciclo de 1 Hz -- 512
 * deja margen sin desperdiciar demasiada RAM. Si se ven sentencias
 * truncadas/corruptas en GPS_Update(), subir este valor primero. */
#define GPS_RX_BUFFER_SIZE            512U

/* Longitud máxima de una sentencia NMEA reconstruida (el estándar NMEA
 * 0183 limita a 82 caracteres incluyendo '$' y checksum; 96 deja
 * margen). */
#define GPS_LINEA_MAX_LEN             96U

/* Tamaño máximo de un comando AT a transmitir hacia el módulo. */
#define GPS_TX_BUFFER_SIZE            64U

/* Cantidad de reportes "+CGPSINFO:" vacíos (sin fix) SEGUIDOS antes de
 * volver a mandar "AT+CGPS=1" -- a un reporte cada 10s (ver
 * AT+CGPSINFO=10 en main.c), 10 seguidos equivalen a ~100s sin fix. */
#define GPS_REPORTES_VACIOS_ANTES_DE_REENVIAR   10U

/* ==================== API PÚBLICA ==================== */

/**
 * Inicializa el módulo y arranca la recepción continua por DMA en modo
 * Circular con detección de línea inactiva (IDLE). Llamar una vez en
 * main(), después de que MX_USART2_UART_Init() ya corrió.
 *
 * No envía ningún comando AT por sí sola -- armar el auto-reporte
 * (AT+CGPS=1 + AT+CGPSINFO=10) es responsabilidad del llamador, vía
 * GPS_EnviarComandoAT() (ver ejemplo en main.c).
 *
 * @param huart   Puntero al handle de USART2 (&huart2 en main.c).
 */
void GPS_Init(UART_HandleTypeDef *huart);

/**
 * Debe llamarse periódicamente desde el loop principal. Parsea las
 * líneas "+CGPSINFO:" completas encoladas por el callback (fix,
 * latitud, longitud). No bloquea.
 */
void GPS_Update(void);

/**
 * Debe llamarse desde el callback global HAL_UARTEx_RxEventCallback()
 * en main.c, cuando el evento provenga de USART2. Es seguro llamarlo
 * siempre; internamente valida que la instancia coincida con la
 * pasada a GPS_Init().
 */
void GPS_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * Debe llamarse desde el callback global HAL_UART_ErrorCallback() en
 * main.c, cuando el error provenga de USART2. Mismo motivo que
 * RAK3172_ErrorCallback(): sin rearmar, un solo error de UART deja la
 * recepcion de GPS muerta para el resto de la sesion -- el modulo
 * puede seguir mandando "+CGPSINFO:" perfectamente y el host nunca lo
 * volveria a ver.
 */
void GPS_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * Envía un comando AT crudo al módulo (se agrega "\r\n" internamente),
 * de forma bloqueante -- igual que RAK3172_EnviarComandoAT(), pero acá
 * no hay que esperar una respuesta "OK"/"ERROR" explícita: una vez
 * armado el auto-reporte, el flujo de "+CGPSINFO:" es un URC
 * periódico, no un protocolo de comando/respuesta.
 *
 * @param comando   Comando AT sin terminador, ej. "AT+CGPS=1".
 * @return true si se pudo transmitir, false si el comando es muy largo.
 */
bool GPS_EnviarComandoAT(const char *comando);

/**
 * true si el último "+CGPSINFO:" procesado trae fix válido (campo de
 * latitud no vacío). Se pone en false de nuevo si llega un
 * "+CGPSINFO: ,,,,,,,,," (sin fix) -- pero la última posición conocida
 * (lat/lon) se conserva, no se resetea a 0.
 */
bool GPS_TieneFix(void);

/** Última latitud válida conocida (grados decimales, + = Norte). 0.0f si nunca hubo fix. */
float GPS_GetLatitud(void);

/** Última longitud válida conocida (grados decimales, + = Este). 0.0f si nunca hubo fix. */
float GPS_GetLongitud(void);


/**
 * Fecha/hora UTC (NO local) del último "+CGPSINFO:" con fix válido,
 * tal como la reporta el propio receptor GNSS del SIM7600X (campos
 * "fecha ddmmyy"/"hora hhmmss.s" del formato documentado en
 * GPS_ProcesarCGPSInfo() -- la parte fraccionaria de los segundos se
 * descarta). Requiere GPS_TieneFix() == true para ser confiable: si no
 * hay fix vivo, devuelve false y no toca los punteros de salida.
 *
 * @return true si los punteros de salida se llenaron con una fecha/hora
 *         válida del fix actual.
 */
bool GPS_GetFechaHoraUtc(uint16_t *anio, uint8_t *mes, uint8_t *dia,
                          uint8_t *hora, uint8_t *minuto, uint8_t *segundo);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
