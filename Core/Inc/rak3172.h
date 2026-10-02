/**
 * @file    rak3172.h
 * @brief   Módulo de comunicación con el módulo RAK3172 (LoRaWAN) vía
 *          comandos AT, sobre LPUART1 (PA2=TX, PA3=RX).
 *
 * Responsabilidades:
 *   - Enviar comandos AT y esperar su respuesta ("OK", "ERROR", o el
 *     valor consultado) de forma no bloqueante, integrada al loop
 *     principal (Tacometro_Update()/Modbus_Update()).
 *   - Recibir y parsear eventos asíncronos "+EVT:" que el módulo manda
 *     por su cuenta (ej. downlink recibido, hora de red disponible),
 *     sin que el host los haya solicitado.
 *   - Exponer funciones para mandar el uplink LIVE del nodo motor y
 *     para sincronizar la hora contra la red (DeviceTimeReq).
 *
 * Uso típico en main.c:
 *   RAK3172_Init(&hlpuart1);
 *   ...
 *   while (1) {
 *       RAK3172_Update();
 *       if (debe_mandar_uplink) {
 *           RAK3172_EnviarUplinkLive(...);
 *       }
 *   }
 *
 * Y en el callback global de HAL (en main.c, USER CODE):
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *       ...
 *       else if (huart->Instance == LPUART1) {
 *           RAK3172_RxEventCallback(huart, Size);
 *       }
 *   }
 */

#ifndef RAK3172_H
#define RAK3172_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACIÓN AJUSTABLE ==================== */

/* Tamaño del buffer circular de recepción por DMA (debe coincidir con
 * el tamaño de bufferRAKRx que se pasa a HAL_UARTEx_ReceiveToIdle_DMA
 * en main.c). */
#define RAK3172_RX_BUFFER_SIZE        128U

/* Tamaño máximo de un comando AT a transmitir (incluye "AT+...\r\n").
 * ⚠️ El uplink LIVE de 28 bytes en hex ocupa 56 caracteres + "AT+SEND=" +
 * FPort + ":" + "\r\n" (~68-72 bytes) -- 96U alcanza con margen, pero si
 * se agregan más campos al payload en el futuro, revisar este tamaño. */
#define RAK3172_TX_BUFFER_SIZE        96U

/* Timeout por defecto esperando respuesta "OK"/"ERROR" a un comando AT
 * enviado (ms). Comandos de join a la red pueden tardar más -- se
 * puede pasar un timeout distinto por comando si se requiere. */
#define RAK3172_TIMEOUT_DEFAULT_MS    2000U

/* FPort usado para el uplink LIVE del motor (RPM + presión + estado +
 * timestamps + posición, ver RAK3172_EnviarUplinkLive()). Mantenlo
 * consistente con lo que espera decoder.py del lado AWS. */
#define RAK3172_FPORT_UPLINK_RPM      1U

/* FPort usado para downlinks de configuración de parámetros.
 * El payload en este FPort sigue el esquema [ID de 1 byte][valor],
 * donde ID es uno de los CALIB_ID_* definidos en calibracion_flash.h.
 * Toda la lógica de qué ID corresponde a qué parámetro vive en
 * calibracion_flash.c -- este módulo solo separa los bytes. */
#define RAK3172_FPORT_PARAMETRO       2U

/* FPort usado para el Application ACK que confirma el resultado de
 * procesar un downlink de parámetro (protocolo Quick-Set acordado).
 * Payload de 4 bytes: [PARAMETER_ID][STATUS][VALUE_H][VALUE_L]. */
#define RAK3172_FPORT_ACK             3U

/* ==================== TIPOS ==================== */

typedef enum {
    RAK3172_OK = 0,
    RAK3172_ERROR,
    RAK3172_TIMEOUT,
    RAK3172_BUSY   /* ya hay un comando en curso esperando respuesta */
} RAK3172_Resultado_t;

/* ==================== API PÚBLICA ==================== */

/**
 * true si se recibió el evento "+EVT:JOINED" (el módulo se unió
 * exitosamente a la red LoRaWAN). Se pone en false de nuevo al llamar
 * RAK3172_Join().
 */
bool RAK3172_EstaUnido(void);

/**
 * true si el propio módulo pidió no transmitir por ahora (línea
 * "Restricted_Wait_<ms>_ms", ver rak3172.c) -- backoff de join que
 * exige la especificación LoRaWAN tras fallos prolongados de join. El
 * llamador debe evitar pedir join o mandar uplinks mientras esto sea
 * true (reintentar de todas formas no logra nada y puede extender la
 * restricción).
 */
bool RAK3172_EstaRestringido(void);

/**
 * Inicia el proceso de join a la red LoRaWAN, usando el comando
 * recomendado por RAK (join=1, auto=0, intervalo=10s, 8 intentos).
 * No bloquea -- el resultado se refleja después en RAK3172_EstaUnido().
 */
bool RAK3172_Join(void);

/**
 * Inicializa el módulo. Llamar una vez en main(), después de que
 * MX_LPUART1_UART_Init() ya corrió y de haber arrancado la recepción
 * con HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, bufferRAKRx, ...).
 *
 * @param huart   Puntero al handle de LPUART1 (&hlpuart1 en main.c).
 */
void RAK3172_Init(UART_HandleTypeDef *huart);

/**
 * Debe llamarse periódicamente desde el loop principal. Atiende
 * timeouts de comandos AT pendientes de respuesta. No bloquea.
 */
void RAK3172_Update(void);

/**
 * Debe llamarse desde el callback global HAL_UARTEx_RxEventCallback()
 * en main.c, cuando el evento provenga de LPUART1. Es seguro llamarlo
 * solo cuando huart->Instance == LPUART1 (ver ejemplo de uso arriba).
 */
void RAK3172_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * Debe llamarse desde el callback global HAL_UART_ErrorCallback() en
 * main.c, cuando el error provenga de la UART del RAK3172. Sin esto,
 * un solo error de UART (overrun/framing/ruido -- ej. si el USB-a-PC
 * mete ruido al plano de tierra, ver hallazgos de hardware) deja al
 * modulo sordo para siempre: HAL aborta la recepcion por DMA
 * internamente al entrar en error y RAK3172_RxEventCallback() nunca
 * vuelve a dispararse, aunque el RAK3172 siga contestando bien por su
 * lado -- todo comando futuro solo expira por TIMEOUT sin que el host
 * jamas vea la respuesta real. Limpia los flags de error y rearma la
 * recepcion.
 */
void RAK3172_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * Envía un comando AT crudo (sin "\r\n", se agrega internamente) y
 * regresa inmediatamente -- no bloquea esperando la respuesta.
 * La respuesta se procesa de forma asíncrona; usar
 * RAK3172_HayRespuestaPendiente()/RAK3172_GetUltimoResultado() si se
 * necesita conocer el resultado desde el loop principal.
 *
 * @param comando   Comando AT sin terminador, ej. "AT+VER=?".
 * @return true si se pudo encolar/transmitir, false si ya hay un
 *         comando en curso (RAK3172_BUSY) o el comando es muy largo.
 */
bool RAK3172_EnviarComandoAT(const char *comando);

/**
 * true si el último comando enviado con RAK3172_EnviarComandoAT() ya
 * recibió respuesta (OK/ERROR/timeout) y no hay uno en curso.
 */
bool RAK3172_ComandoListo(void);

/** Resultado del último comando AT completado. */
RAK3172_Resultado_t RAK3172_GetUltimoResultado(void);

/**
 * Copia la última línea de respuesta cruda (por ejemplo el valor
 * devuelto por "AT+VER=?") a un buffer del llamador.
 *
 * @param destino     Buffer donde copiar.
 * @param tamDestino  Tamaño del buffer destino.
 * @return true si había una respuesta disponible para copiar.
 */
bool RAK3172_GetUltimaRespuesta(char *destino, uint32_t tamDestino);


/**
 * Arma y envía el uplink LIVE extendido (28 bytes) del nodo motor:
 * motorIdNumeric + RPM + presión + estado + fecha/hora + inicio de
 * operación + segundos transcurridos + lat/lon + codigoAlerta. Ver
 * decoder.py del lado AWS (_decodificar_live()) para el layout exacto
 * de offsets -- cualquier cambio acá debe reflejarse allá.
 *
 * @param motorIdNumeric           Numérico del nodo (ej. 1 -> "DSL-0001").
 * @param rpm                      RPM actual.
 * @param presion                  Presión de salida de la motobomba, en
 *                                 PSI (ver presion.c,
 *                                 Presion_GetPresionPsi()).
 * @param estado                   1=ACTIVO, 2=APAGADO, 3=ENCENDIDO (ver
 *                                 ESTADO_* en main.c).
 * @param fechaHoraLocal           Epoch unix, YA con el offset de
 *                                 Guatemala aplicado (Reloj_GetUnixTimeLocal()).
 * @param inicioOperacionLocal     Epoch unix local de cuándo empezó el
 *                                 estado actual.
 * @param segundosTranscurridos    Segundos desde inicioOperacionLocal.
 * @param latitud                  Latitud fija del sitio (sin GPS aún).
 * @param longitud                 Longitud fija del sitio (sin GPS aún).
 * @param codigoAlerta             Snapshot de la alerta vigente en el
 *                                 momento del envío (0=sin alerta, ver
 *                                 ALERTA_* en main.c). En uplinks
 *                                 disparados por un evento de alerta,
 *                                 usar CalibFlash_ForzarReporte() para
 *                                 que el timestamp del uplink coincida
 *                                 con el momento del evento.
 * @return true si se pudo encolar el comando AT+SEND correspondiente.
 */
bool RAK3172_EnviarUplinkLive(uint16_t motorIdNumeric, float rpm, float presion,
                               uint8_t estado, uint32_t fechaHoraLocal,
                               uint32_t inicioOperacionLocal,
                               uint32_t segundosTranscurridos,
                               float latitud, float longitud,
                               uint8_t codigoAlerta);

/**
 * Arma y envía el Application ACK del protocolo Quick-Set (4 bytes:
 * [PARAMETER_ID][STATUS][VALUE_H][VALUE_L]) en el FPort
 * RAK3172_FPORT_ACK, confirmando el resultado de procesar un
 * downlink de parámetro.
 *
 * Si el canal AT está ocupado en el momento de la llamada (por
 * ejemplo, coincide con el uplink periódico de RPM), el ACK se
 * encola internamente y RAK3172_Update() lo reintenta automáticamente
 * en cuanto el canal se libere, hasta un máximo de
 * RAK3172_ACK_REINTENTO_TIMEOUT_MS -- no hace falta que el llamador
 * reintente por su cuenta.
 *
 * @param id            ID del parámetro procesado.
 * @param status        Uno de los CalibFlash_ProtocoloStatus_t.
 * @param valorRaw      Valor de 2 bytes (big-endian) que quedó
 *                       vigente, tal como lo entrega
 *                       CalibFlash_ProcesarParametroConEstado().
 * @return true si se pudo mandar de inmediato. false si quedó
 *         encolado para reintento (no es necesariamente un error).
 */
bool RAK3172_EnviarAck(uint8_t id, uint8_t status, uint16_t valorRaw);

/**
 * Manda un uplink con 'len' bytes crudos por 'fport' (AT+SEND, no
 * bloqueante, mismo canal AT que el resto). Lo usan el uplink LIVE y el
 * reporte REPORTAR_PARAMETROS (FPort 3, grupos de 4 bytes como el ACK).
 * Máximo RAK3172_UPLINK_MAX_BYTES (= tamaño del LIVE, que ya se sabe que
 * pasa con el data rate actual).
 */
#define RAK3172_UPLINK_MAX_BYTES  28U
bool RAK3172_EnviarUplink(uint8_t fport, const uint8_t *datos, uint8_t len);

/** true mientras un Application ACK espera canal para salir (ver
 *  RAK3172_EnviarAck). Lo usa RESET_REMOTO para no reiniciar antes de
 *  confirmar. */
bool RAK3172_HayAckPendiente(void);

/**
 * Pide la hora UTC a la red LoRaWAN (DeviceTimeReq de la especificación
 * LoRaWAN, vía "AT+TIMEREQ=1" en RUI3). El valor llega "montado" en el
 * próximo uplink exitoso -- no es instantáneo. Se puede llamar más de
 * una vez por arranque (p. ej. para un resync periódico que corrija la
 * deriva del RTC) -- cada llamada limpia el flag de
 * RAK3172_HoraDeRedDisponible() del ciclo anterior. Requiere que
 * RAK3172_EstaUnido() sea true.
 *
 * @return true si se pudo encolar el comando AT+TIMEREQ=1.
 */
bool RAK3172_SolicitarHoraRed(void);

/**
 * true una vez que la red confirmó el envío de la hora (evento
 * "+EVT:TIMEREQ" visto tras un uplink) para el ciclo de solicitud MÁS
 * RECIENTE (ver RAK3172_SolicitarHoraRed()) -- en ese punto ya se puede
 * llamar RAK3172_ConsultarHoraRed() para leer el valor.
 */
bool RAK3172_HoraDeRedDisponible(void);

/**
 * Consulta el valor de hora ya recibido de la red ("AT+LTIME=?"). La
 * respuesta llega por el mecanismo genérico RAK3172_GetUltimaRespuesta()
 * -- leer ese buffer después de que este comando termine
 * (RAK3172_ComandoListo() vuelva a true) y parsear el epoch (ver nota
 * en rak3172.c sobre el formato de respuesta aún sin confirmar en campo).
 *
 * @return true si se pudo encolar el comando AT+LTIME=?.
 */
bool RAK3172_ConsultarHoraRed(void);

#ifdef __cplusplus
}
#endif

#endif /* RAK3172_H */
