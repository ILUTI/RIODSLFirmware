/**
 * @file    comando_serial.h
 * @brief   Consola por el puerto de debug (LPUART1): parámetros escritos a
 *          mano (mismo efecto que un downlink) y pass-through de comandos AT
 *          al RAK3172. PERMANENTE (decisión 2026-10-02): se usa en banco y en
 *          campo con la laptop conectada, también con la red LoRa activa.
 *
 * Reutiliza el mismo punto de entrada que un downlink real
 * (CalibFlash_ProcesarParametroConEstado(), el mismo que llama
 * rak3172.c en RAK3172_ProcesarEventoDownlink()) -- el comportamiento
 * (validación de rango, bloqueo por categoría con el motor operando,
 * persistencia en flash) es idéntico al de un downlink real, solo
 * cambia el transporte. A diferencia de un downlink real, esto NO
 * manda el Application ACK por LoRaWAN (RAK3172_EnviarAck) -- el
 * resultado se imprime directo al mismo puerto serial, como si fuera
 * el ACK.
 *
 * Formato de línea (escrita a mano en el monitor serie, termina al
 * mandar Enter):
 *   NOMBRE_PARAMETRO [VALOR]
 * ej.:
 *   CALIB 1
 *   SET_RPM 900
 *   REPORTAR_PARAMETROS       (los comandos no llevan VALOR)
 *   AT+DEVEUI=?               (pass-through al RAK3172, ver comando_serial.c)
 *
 * Nombres válidos: los mismos (exactos, en mayúsculas) de la tabla de
 * parámetros del README y de PARAM_TABLE de la Lambda de downlink -- mismo
 * valor "humano" que se manda por MQTT (ej. RPM directo, no el raw
 * escalado).
 *
 * La línea se ARMA en la interrupción (letra por letra, con eco) pero se
 * PROCESA en ComandoSerial_Update(), en el loop normal (2026-10-02,
 * hallazgo B16): procesarla dentro de la ISR podía escribir flash y hacer
 * printf bloqueante en medio de cualquier otra cosa.
 */

#ifndef COMANDO_SERIAL_H
#define COMANDO_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/** Inicializa el módulo sobre el UART de debug ya inicializado
 * (MX_LPUART1_UART_Init(), generado por CubeMX) -- no reconfigura el
 * periférico: guarda el handle y arranca la recepción por interrupción. */
void ComandoSerial_Init(UART_HandleTypeDef *huart);

/** Llamar en cada vuelta del loop principal. Procesa la línea que dejó
 * lista la ISR (parámetro o comando AT) y maneja el pass-through AT. No
 * hace polling de UART (ver ComandoSerial_RxCpltCallback). */
void ComandoSerial_Update(void);

/**
 * Debe llamarse desde el callback global HAL_UART_RxCpltCallback() en
 * main.c, cuando el evento provenga del UART de debug (LPUART1). Cada
 * byte recibido por interrupción arma la línea carácter por carácter
 * con eco local, y al recibir '\r'/'\n' la deja lista para
 * ComandoSerial_Update() (no la procesa aquí) -- luego siempre rearma la
 * recepción del siguiente byte con HAL_UART_Receive_IT().
 *
 * Recepción por interrupción en vez de polling en ComandoSerial_Update()
 * (agregado 2026-09-24, recuperando un fix perdido en el incidente de
 * pérdida de archivos): el polling dependía de que el loop principal
 * llamara a ComandoSerial_Update() seguido -- pero __io_putchar() manda
 * cada printf() de telemetría por ESTE MISMO UART con
 * HAL_UART_Transmit() bloqueante, byte por byte. Mientras se imprime
 * una línea larga de log, el loop no vuelve a sondear el UART, y como
 * LPUART1 tiene el overrun deshabilitado (ver .ioc,
 * UART_ADVFEATURE_OVERRUN_DISABLE), cualquier byte que llegue mientras
 * el anterior sigue sin leerse se pierde en silencio (se sobrescribe en
 * RDR, sin ORE, sin aviso) -- síntoma de campo: letras faltantes al
 * escribir un comando rápido mientras el firmware está logueando
 * (ej. "TIMEOUT_SIN_COMANDO_S" llega como "TIMEOUT_SIN_COMANDOS"). La
 * recepción por interrupción no depende de la cadencia del loop
 * principal -- el HAL lee el byte y limpia RXNE apenas llega,
 * sin importar qué tan ocupado esté imprimiendo.
 */
void ComandoSerial_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * Debe llamarse desde el callback global HAL_UART_ErrorCallback() en
 * main.c, cuando el error provenga del UART de debug (LPUART1). Un
 * error de framing/ruido (el overrun ya está deshabilitado por .ioc,
 * ver ComandoSerial_RxCpltCallback) aborta el HAL_UART_Receive_IT()
 * pendiente -- sin rearmarlo acá, el comando manual queda sordo para
 * siempre tras el primer error, mismo criterio que
 * RAK3172_ErrorCallback()/GPS_ErrorCallback().
 */
void ComandoSerial_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* COMANDO_SERIAL_H */
