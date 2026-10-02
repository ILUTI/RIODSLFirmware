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

/* Tamaño del buffer circular de recepción por DMA. El modulo NO manda
 * NMEA (ver gps.c): solo un "+CGPSINFO:" de ~70-80 caracteres cada 10 s,
 * mas unos pocos "OK"/banners al arrancar. El DMA avisa a la mitad y al
 * final del buffer (y en cada pausa de la linea), asi que 128 alcanza con
 * margen (2026-10-02, antes 512 pensando en rafagas NMEA). Si se ven
 * lineas truncadas en el log "GPS RX crudo", subir este valor primero. */
#define GPS_RX_BUFFER_SIZE            128U

/* Longitud máxima de una línea "+CGPSINFO:" reconstruida (~70-80
 * caracteres con fix; 96 deja margen). */
#define GPS_LINEA_MAX_LEN             96U

/* Tamaño máximo de un comando AT a transmitir hacia el módulo. */
#define GPS_TX_BUFFER_SIZE            64U

/* Cantidad de reportes "+CGPSINFO:" vacíos (sin fix) SEGUIDOS antes de
 * volver a mandar "AT+CGPS=1" -- a un reporte cada 10s (ver
 * AT+CGPSINFO=10 en main.c), 10 seguidos equivalen a ~100s sin fix. */
#define GPS_REPORTES_VACIOS_ANTES_DE_REENVIAR   10U

/* Si no llega NINGUN "+CGPSINFO:" (ni vacio) en este tiempo, el fix se da
 * por vencido (GPS_TieneFix() = false): 3 reportes perdidos a uno cada 10 s. */
#define GPS_REPORTE_VIGENCIA_MS      30000U

/* Re-armado (2026-10-02): si no llega NINGUN reporte en este tiempo (ej. el
 * SIM7600X se reinicio solo y olvido AT+CGPS=1 / AT+CGPSINFO=10), se le
 * vuelven a mandar las dos ordenes del arranque, separadas por
 * GPS_REARMAR_PAUSA_MS (sin pausa el modulo deja de contestar, ver main.c).
 * Se repite como mucho una vez por GPS_REARMAR_SIN_REPORTES_MS mientras siga
 * mudo. No bloquea: lo maneja GPS_Update(). */
#define GPS_REARMAR_SIN_REPORTES_MS  60000U
#define GPS_REARMAR_PAUSA_MS          1500U

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
 * latitud no vacío) Y llegó hace menos de GPS_REPORTE_VIGENCIA_MS. Se pone
 * en false si llega un "+CGPSINFO: ,,,,,,,,," (sin fix) o si el módulo deja
 * de reportar -- la última posición conocida (lat/lon) se conserva en los
 * getters, pero main.c manda 0/0 cuando esto es false.
 */
bool GPS_TieneFix(void);

/** Milisegundos desde el último "+CGPSINFO:" recibido (con fix, si
 *  GPS_TieneFix() es true). main.c lo suma a la hora del GPS al poner el
 *  RTC, para no arrastrar el atraso del reporte (hasta 10 s). */
uint32_t GPS_GetEdadReporteMs(void);

/** Última latitud válida conocida (grados decimales, + = Norte). 0.0f si nunca hubo fix. */
float GPS_GetLatitud(void);

/** Última longitud válida conocida (grados decimales, + = Este). 0.0f si nunca hubo fix. */
float GPS_GetLongitud(void);


/**
 * Fecha/hora UTC (NO local) del último "+CGPSINFO:" con fix válido,
 * tal como la reporta el propio receptor GNSS del SIM7600X (campos
 * "fecha ddmmyy"/"hora hhmmss.s" del formato documentado en
 * GPS_ProcesarCGPSInfo() -- la parte fraccionaria de los segundos se
 * descarta). Es la hora del MOMENTO del reporte: sumarle
 * GPS_GetEdadReporteMs() para la hora actual. Si no hay fix vivo
 * (GPS_TieneFix() == false), devuelve false y no toca los punteros.
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
