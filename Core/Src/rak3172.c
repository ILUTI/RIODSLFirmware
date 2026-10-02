/**
 * @file    rak3172.c
 * @brief   Implementación del módulo de comunicación con el RAK3172.
 *
 * Diseño de recepción (modo Normal, no circular):
 *   El módulo posee su propio buffer de recepción y arranca la
 *   captura por DMA (HAL_UARTEx_ReceiveToIdle_DMA) dentro de
 *   RAK3172_Init(). Cada evento de recepción (línea inactiva o buffer
 *   lleno) entrega en 'Size' la cantidad de bytes nuevos recibidos, y
 *   la recepción se rearma manualmente al final del callback -- a
 *   propósito, en vez de modo Circular, para tener un punto de
 *   control donde limpiar flags de error antes de rearmar, y evitar
 *   que un error de línea puntual (ruido, glitch) se vuelva un ciclo
 *   que se retroalimenta solo.
 *
 *   Las líneas completas (terminadas en '\n') se acumulan en una cola
 *   circular pequeña para que RAK3172_Update() las procese fuera de
 *   contexto de interrupción.
 */

#include "rak3172.h"
#include "calibracion_flash.h"
#include "tacometro.h"
#include "numero_texto.h"
#include <string.h>
#include <stdio.h>

/* ==================== ESTADO INTERNO ==================== */

static UART_HandleTypeDef *s_huart = NULL;

/* Buffer de recepción, llenado por DMA en modo Normal (no circular). */
static uint8_t  s_rxDmaBuffer[RAK3172_RX_BUFFER_SIZE];

/* Ensamblador de línea (acumula bytes hasta '\n'). */
static char     s_lineaEnConstruccion[RAK3172_RX_BUFFER_SIZE];
static uint16_t s_lineaLen = 0;

/* Cola circular de líneas completas pendientes de procesar. */
#define RAK3172_MAX_LINEAS_PENDIENTES  4U
static char     s_colaLineas[RAK3172_MAX_LINEAS_PENDIENTES][RAK3172_RX_BUFFER_SIZE];
static volatile uint8_t s_colaHead = 0; /* siguiente a leer */
static volatile uint8_t s_colaTail = 0; /* siguiente a escribir */
static volatile uint8_t s_colaCount = 0;

/* Estado del comando AT en curso. */
static volatile bool     s_comandoEnCurso   = false;
static volatile uint32_t s_comandoTickInicio = 0;
static RAK3172_Resultado_t s_ultimoResultado = RAK3172_OK;
static char     s_ultimaRespuesta[RAK3172_RX_BUFFER_SIZE];
static bool     s_hayRespuestaNueva = false;
static volatile bool s_estaUnido = false;

/* Tick hasta el cual el modulo pidio no intentar transmitir (join ni
 * uplink) -- ver "Restricted_Wait_<ms>_ms" en RAK3172_ProcesarLinea().
 * 0 = sin restriccion vigente. */
static volatile uint32_t s_restringidoHastaTickMs = 0;

/* true una vez que la red confirmó el envío de la hora (DeviceTimeReq)
 * -- ver "+EVT:TIMEREQ_OK" en RAK3172_ProcesarLinea() y
 * RAK3172_SolicitarHoraRed()/RAK3172_ConsultarHoraRed(). */
static volatile bool s_horaDeRedDisponible = false;

/* Reintento de Application ACK: si al momento de querer mandarlo el
 * canal AT está ocupado (otro comando en curso), se guarda aquí y
 * RAK3172_Update() lo reintenta en cuanto el canal se libere, hasta
 * un timeout máximo -- para no perder la confirmación en silencio
 * solo porque coincidió con, por ejemplo, el uplink periódico de RPM. */
#define RAK3172_ACK_REINTENTO_TIMEOUT_MS  5000U

static volatile bool     s_ackPendiente = false;
static uint8_t           s_ack[4];          /* [ID][STATUS][VALUE_H][VALUE_L] */
static uint32_t          s_ackTickInicio = 0;

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static void RAK3172_EncolarLinea(const char *linea, uint16_t len);
static void RAK3172_ProcesarLinea(const char *linea);
static void RAK3172_ProcesarEventoDownlink(const char *linea);
static bool RAK3172_HexAAlBytes(const char *hex, uint8_t longitudHex, uint8_t *bytesSalida, uint8_t maxBytes, uint8_t *cantidadBytes);

/* ==================== API PÚBLICA ==================== */

void RAK3172_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_lineaLen = 0;
    memset(s_lineaEnConstruccion, 0, sizeof(s_lineaEnConstruccion));

    s_colaHead = 0;
    s_colaTail = 0;
    s_colaCount = 0;

    s_comandoEnCurso = false;
    s_ultimoResultado = RAK3172_OK;
    s_hayRespuestaNueva = false;
    memset(s_ultimaRespuesta, 0, sizeof(s_ultimaRespuesta));
    s_horaDeRedDisponible = false;

    /* Antes de arrancar la recepción, limpiar cualquier flag de error
     * (Overrun/Framing/Noise) y vaciar el registro de datos. Esto es
     * importante porque el RAK3172 NO se resetea junto con el G431 --
     * si había actividad en la línea justo en el instante del reset,
     * puede quedar un error de recepción "colgado" que, combinado con
     * Overrun/DMA-on-RX-Error habilitados, entra en un ciclo de
     * reintento continuo antes de que la aplicación tenga oportunidad
     * de intervenir. */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    volatile uint32_t dummy = s_huart->Instance->RDR; /* vaciar el registro de datos */
    (void)dummy;

    /* Arranca la recepción continua por DMA con detección de línea
     * inactiva (IDLE). El callback HAL_UARTEx_RxEventCallback de main.c
     * reenvía el evento a RAK3172_RxEventCallback() cuando viene de la
     * UART del RAK3172 (USART1, ver ejemplo en rak3172.h). */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, RAK3172_RX_BUFFER_SIZE);
    /* __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT); -- causaba HardFault:
     * s_huart->hdmarx llegaba NULL en este punto. No es crítico (solo evita
     * el evento de "medio buffer" del DMA), así que se quita por ahora. */
}

void RAK3172_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != s_huart->Instance) {
        return;
    }

    /* En modo Normal (no circular), 'Size' SÍ es la cantidad de bytes
     * nuevos recibidos en esta transacción (a diferencia de modo
     * circular, donde era una posición absoluta). Se procesan desde
     * el inicio del buffer cada vez. */
    for (uint16_t i = 0; i < Size; i++) {
        char c = (char)s_rxDmaBuffer[i];

        if (c == '\r') {
            continue; /* ignorar CR, solo nos importa el LF como fin de línea */
        }

        if (c == '\n') {
            if (s_lineaLen > 0) {
                RAK3172_EncolarLinea(s_lineaEnConstruccion, s_lineaLen);
                s_lineaLen = 0;
            }
            continue;
        }

        if (s_lineaLen < (RAK3172_RX_BUFFER_SIZE - 1)) {
            s_lineaEnConstruccion[s_lineaLen++] = c;
        }
    }

    /* Limpiar posibles flags de error acumulados antes de rearmar,
     * para no heredar un estado de error de la transacción anterior. */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);

    /* Modo Normal: hay que rearmar la recepción explícitamente cada
     * vez, a diferencia del modo Circular que se rearma solo. */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, RAK3172_RX_BUFFER_SIZE);
}

void RAK3172_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance) {
        return;
    }

    /* (El printf de diagnostico con HAL ErrorCode se quito 2026-10-02: ya
     * funciona bien. Si vuelven los TIMEOUT repetidos, reponerlo aqui.) */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    volatile uint32_t dummy = s_huart->Instance->RDR;
    (void)dummy;

    /* Descartar la linea a medio construir -- pudo quedar corrupta o
     * incompleta por el mismo error que disparo este callback. */
    s_lineaLen = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, RAK3172_RX_BUFFER_SIZE);
}

void RAK3172_Update(void)
{
    /* --- 1. Procesar líneas completas encoladas por el callback --- */
    while (s_colaCount > 0) {
        __disable_irq();
        char linea[RAK3172_RX_BUFFER_SIZE];
        strncpy(linea, s_colaLineas[s_colaHead], RAK3172_RX_BUFFER_SIZE);
        s_colaHead = (s_colaHead + 1) % RAK3172_MAX_LINEAS_PENDIENTES;
        s_colaCount--;
        __enable_irq();

        RAK3172_ProcesarLinea(linea);
    }

    /* --- 2. Timeout de comando AT en curso --- */
    if (s_comandoEnCurso) {
        uint32_t transcurrido = HAL_GetTick() - s_comandoTickInicio;
        if (transcurrido > RAK3172_TIMEOUT_DEFAULT_MS) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_TIMEOUT;
        }
    }

    /* --- 3. Reintento de Application ACK pendiente --- */
    if (s_ackPendiente && RAK3172_ComandoListo()) {
        if (RAK3172_EnviarUplink(RAK3172_FPORT_ACK, s_ack, sizeof(s_ack))) {
            s_ackPendiente = false;
        } else if ((HAL_GetTick() - s_ackTickInicio) > RAK3172_ACK_REINTENTO_TIMEOUT_MS) {
            /* Se agotó el tiempo de reintento -- se descarta el ACK
             * para no quedar atascado reintentando indefinidamente.
             * El parámetro en sí ya se aplicó correctamente; solo se
             * pierde esta confirmación puntual. */
            printf("RAK3172: Application ACK (ID=%u) descartado tras reintentar %lums sin éxito\r\n",
                   s_ack[0], (unsigned long)RAK3172_ACK_REINTENTO_TIMEOUT_MS);
            s_ackPendiente = false;
        }
    }
}

bool RAK3172_EnviarComandoAT(const char *comando)
{
    if (s_comandoEnCurso) {
        return false; /* RAK3172_BUSY -- espera a que termine el anterior */
    }

    char buffer[RAK3172_TX_BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "%s\r\n", comando);
    if (len <= 0 || (uint32_t)len >= sizeof(buffer)) {
        return false; /* comando demasiado largo */
    }

    s_comandoEnCurso = true;
    s_comandoTickInicio = HAL_GetTick();

    /* Transmisión bloqueante: los comandos AT son cortos (unas pocas
     * decenas de bytes), igual que el patrón ya usado en __io_putchar
     * para el printf de depuración. */
    HAL_UART_Transmit(s_huart, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);

    return true;
}

bool RAK3172_ComandoListo(void)
{
    return !s_comandoEnCurso;
}

RAK3172_Resultado_t RAK3172_GetUltimoResultado(void)
{
    return s_ultimoResultado;
}

bool RAK3172_GetUltimaRespuesta(char *destino, uint32_t tamDestino)
{
    if (!s_hayRespuestaNueva) {
        return false;
    }
    strncpy(destino, s_ultimaRespuesta, tamDestino - 1);
    destino[tamDestino - 1] = '\0';
    s_hayRespuestaNueva = false;
    return true;
}

bool RAK3172_EstaUnido(void)
{
    return s_estaUnido;
}

bool RAK3172_EstaRestringido(void)
{
    return (int32_t)(s_restringidoHastaTickMs - HAL_GetTick()) > 0;
}

bool RAK3172_Join(void)
{
    s_estaUnido = false;
    /* AT+JOIN=<unirse>:<AutoJoin>:<intervalo s>:<intentos> = 1:0:10:8:
     * unirse ya, SIN AutoJoin, reintento cada 10 s, hasta 8 intentos
     * (~80 s). Definitivo desde 2026-10-02 (antes era una prueba temporal
     * mientras se buscaba un AT_ERROR, que resulto ser el bug de sub-banda
     * de RUI3, resuelto con AT+MASK=0002).
     * AutoJoin queda apagado a proposito: los reintentos los maneja el
     * host (main.c, cada 2 min tras AT_NO_NETWORK_JOINED), que ANTES de
     * cada uno reafirma AT+MASK=0002 y respeta el Restricted_Wait. Con
     * AutoJoin=1 el modulo podria unirse solo antes de esa mascara y sumar
     * intentos que alargan el backoff. */
    return RAK3172_EnviarComandoAT("AT+JOIN=1:0:10:8");
}

bool RAK3172_HayAckPendiente(void)
{
    return s_ackPendiente;
}

bool RAK3172_EnviarAck(uint8_t id, uint8_t status, uint16_t valorRaw)
{
    /* Protocolo Quick-Set (Application ACK), 4 bytes:
     * [PARAMETER_ID][STATUS][VALUE_H][VALUE_L] */
    uint8_t ack[4] = { id, status, (uint8_t)(valorRaw >> 8), (uint8_t)(valorRaw & 0xFFU) };

    if (RAK3172_ComandoListo() && RAK3172_EnviarUplink(RAK3172_FPORT_ACK, ack, sizeof(ack))) {
        return true; /* se pudo mandar de inmediato */
    }

    /* Canal ocupado (u otro fallo puntual) -- se guarda para
     * reintentar automáticamente desde RAK3172_Update(). Si ya había
     * un ACK pendiente sin enviar, se sobreescribe con este (caso
     * raro de dos downlinks casi simultáneos); se prioriza no
     * bloquear el sistema sobre no perder ningún ACK en ese
     * escenario extremo. */
    memcpy(s_ack, ack, sizeof(s_ack));
    s_ackPendiente = true;
    s_ackTickInicio = HAL_GetTick();

    return false;
}

bool RAK3172_EnviarUplinkLive(uint16_t motorIdNumeric, float rpm, float presion,
                               uint8_t estado, uint32_t fechaHoraLocal,
                               uint32_t inicioOperacionLocal,
                               uint32_t segundosTranscurridos,
                               float latitud, float longitud,
                               uint8_t codigoAlerta, uint8_t bateriaPct,
                               uint32_t horometroDecimas)
{
    if (rpm < 0.0f) { rpm = 0.0f; }
    if (rpm > 6553.5f) { rpm = 6553.5f; }
    if (presion < 0.0f) { presion = 0.0f; }
    if (presion > 6553.5f) { presion = 6553.5f; }

    uint16_t rpmRaw = (uint16_t)(rpm * 10.0f + 0.5f);
    uint16_t presionRaw = (uint16_t)(presion * 10.0f + 0.5f);
    int32_t latRaw = (int32_t)(latitud * 10000000.0f);
    int32_t lonRaw = (int32_t)(longitud * 10000000.0f);

    if (horometroDecimas > 0xFFFFFFUL) { horometroDecimas = 0xFFFFFFUL; } /* 3 bytes */

    /* Layout de 32 bytes -- DEBE coincidir byte a byte con
     * decoder.py::_decodificar_live() del lado AWS. Cualquier cambio
     * acá tiene que reflejarse allá, y viceversa:
     *   0-1   motorIdNumeric   uint16 BE
     *   2-3   rpm              uint16 BE, x10
     *   4-5   presion          uint16 BE, x10
     *   6     estado           uint8 (1=ACTIVO,2=APAGADO,3=ENCENDIDO)
     *   7-10  fechaHoraLocal   uint32 BE (epoch, ya con -6h aplicado)
     *   11-14 inicioOperacionLocal  uint32 BE
     *   15-18 segundosTranscurridos uint32 BE
     *   19-22 latitud          int32 BE, x10,000,000
     *   23-26 longitud         int32 BE, x10,000,000
     *   27    codigoAlerta     uint8 (0=sin alerta, ver ALERTA_* en main.c)
     *   28    bateriaPct       uint8, 0-100 % de la bateria del nodo; 0xFF = sin
     *                          medicion (agregado 2026-10-02, falta el hardware)
     *   29-31 horometro        uint24 BE, en decimas de hora (horometro.c,
     *                          agregado 2026-10-02)
     */
    uint8_t payload[32];
    payload[0]  = (uint8_t)((motorIdNumeric >> 8) & 0xFFU);
    payload[1]  = (uint8_t)(motorIdNumeric & 0xFFU);
    payload[2]  = (uint8_t)((rpmRaw >> 8) & 0xFFU);
    payload[3]  = (uint8_t)(rpmRaw & 0xFFU);
    payload[4]  = (uint8_t)((presionRaw >> 8) & 0xFFU);
    payload[5]  = (uint8_t)(presionRaw & 0xFFU);
    payload[6]  = estado;
    payload[7]  = (uint8_t)((fechaHoraLocal >> 24) & 0xFFU);
    payload[8]  = (uint8_t)((fechaHoraLocal >> 16) & 0xFFU);
    payload[9]  = (uint8_t)((fechaHoraLocal >> 8) & 0xFFU);
    payload[10] = (uint8_t)(fechaHoraLocal & 0xFFU);
    payload[11] = (uint8_t)((inicioOperacionLocal >> 24) & 0xFFU);
    payload[12] = (uint8_t)((inicioOperacionLocal >> 16) & 0xFFU);
    payload[13] = (uint8_t)((inicioOperacionLocal >> 8) & 0xFFU);
    payload[14] = (uint8_t)(inicioOperacionLocal & 0xFFU);
    payload[15] = (uint8_t)((segundosTranscurridos >> 24) & 0xFFU);
    payload[16] = (uint8_t)((segundosTranscurridos >> 16) & 0xFFU);
    payload[17] = (uint8_t)((segundosTranscurridos >> 8) & 0xFFU);
    payload[18] = (uint8_t)(segundosTranscurridos & 0xFFU);
    payload[19] = (uint8_t)(((uint32_t)latRaw >> 24) & 0xFFU);
    payload[20] = (uint8_t)(((uint32_t)latRaw >> 16) & 0xFFU);
    payload[21] = (uint8_t)(((uint32_t)latRaw >> 8) & 0xFFU);
    payload[22] = (uint8_t)((uint32_t)latRaw & 0xFFU);
    payload[23] = (uint8_t)(((uint32_t)lonRaw >> 24) & 0xFFU);
    payload[24] = (uint8_t)(((uint32_t)lonRaw >> 16) & 0xFFU);
    payload[25] = (uint8_t)(((uint32_t)lonRaw >> 8) & 0xFFU);
    payload[26] = (uint8_t)((uint32_t)lonRaw & 0xFFU);
    payload[27] = codigoAlerta;
    payload[28] = bateriaPct;
    payload[29] = (uint8_t)((horometroDecimas >> 16) & 0xFFU);
    payload[30] = (uint8_t)((horometroDecimas >> 8) & 0xFFU);
    payload[31] = (uint8_t)(horometroDecimas & 0xFFU);

    return RAK3172_EnviarUplink(RAK3172_FPORT_UPLINK_RPM, payload, sizeof(payload));
}

bool RAK3172_EnviarUplink(uint8_t fport, const uint8_t *datos, uint8_t len)
{
    if (len > RAK3172_UPLINK_MAX_BYTES) {
        return false;
    }
    char comando[RAK3172_TX_BUFFER_SIZE];
    int n = snprintf(comando, sizeof(comando), "AT+SEND=%u:", (unsigned int)fport);
    for (uint8_t i = 0; i < len; i++) {
        n += snprintf(&comando[n], sizeof(comando) - (uint32_t)n, "%02X", datos[i]);
    }
    return RAK3172_EnviarComandoAT(comando);
}

bool RAK3172_SolicitarHoraRed(void)
{
    /* Arranca un ciclo nuevo (sync inicial o resync periodico) --
     * limpiar el flag del ciclo anterior para que RAK3172_HoraDeRedDisponible()
     * no reporte "disponible" con el evento viejo antes de que llegue
     * el +EVT:TIMEREQ_OK de ESTA solicitud. */
    s_horaDeRedDisponible = false;
    return RAK3172_EnviarComandoAT("AT+TIMEREQ=1");
}

bool RAK3172_HoraDeRedDisponible(void)
{
    return s_horaDeRedDisponible;
}

bool RAK3172_ConsultarHoraRed(void)
{
    /* Respuesta confirmada en campo (2026-08-17):
     * "AT+LTIME=15h08m55s on 08/17/2026" -- la lee main.c. */
    return RAK3172_EnviarComandoAT("AT+LTIME=?");
}

/* ==================== FUNCIONES PRIVADAS ==================== */

static void RAK3172_EncolarLinea(const char *linea, uint16_t len)
{
    if (s_colaCount >= RAK3172_MAX_LINEAS_PENDIENTES) {
        return; /* cola llena -- se descarta esta línea NUEVA (las ya encoladas se conservan) */
    }

    uint16_t copiar = (len < (RAK3172_RX_BUFFER_SIZE - 1)) ? len : (RAK3172_RX_BUFFER_SIZE - 1);
    memcpy(s_colaLineas[s_colaTail], linea, copiar);
    s_colaLineas[s_colaTail][copiar] = '\0';

    s_colaTail = (s_colaTail + 1) % RAK3172_MAX_LINEAS_PENDIENTES;
    s_colaCount++;
}

static void RAK3172_ProcesarLinea(const char *linea)
{
    /* (El printf "RAK3172 RX crudo" de cada linea se quito 2026-10-02: era
     * diagnostico de unos TIMEOUT repetidos y ya funciona bien. Si vuelven,
     * reponerlo aqui.) */

    if (strcmp(linea, "+EVT:JOINED") == 0) {
        s_estaUnido = true;
        return;
    }

    /* El uplink que se acaba de mandar trajo la hora de la red
     * (DeviceTimeReq) -- ya se puede consultar con AT+LTIME=? (ver
     * RAK3172_ConsultarHoraRed()). Nombre real confirmado en campo
     * (2026-08-14): "+EVT:TIMEREQ_OK"; "+EVT:TIMEREQ" sin sufijo se acepta
     * por si alguna version de RUI3 lo usa. */
    if (strcmp(linea, "+EVT:TIMEREQ_OK") == 0 || strcmp(linea, "+EVT:TIMEREQ") == 0) {
        s_horaDeRedDisponible = true;
        printf("RAK3172: hora de red disponible (%s) -- listo para AT+LTIME=?\r\n", linea);
        return;
    }

    if (strncmp(linea, "+EVT:", 5) == 0) {
        RAK3172_ProcesarEventoDownlink(linea);
        return;
    }

    if (strncmp(linea, "Restricted_Wait_", 16) == 0) {
        /* El propio RUI3 esta rechazando transmitir (join o send) para
         * cumplir el backoff de join que exige el estandar LoRaWAN --
         * NO es un error nuestro puntual, es el modulo aplicando la
         * regla "si llevas mucho tiempo sin poder unirte, reduce cuanto
         * intentas". Visto en campo con valores de varias HORAS
         * (ej. Restricted_Wait_116064740_ms ~= 32h) -- probablemente
         * consecuencia de que el reintento de join cada 2 min
         * (INTERVALO_REINTENTO_JOIN_MS, main.c) estuvo corriendo sin
         * parar durante un fallo prolongado (ej. el modulo
         * reiniciandose solo, ver hallazgos de hardware), acumulando
         * mas intentos de los que el backoff de la especificacion
         * permite. Sin este chequeo, el firmware seguia reintentando
         * join/uplink cada 2 min de todas formas -- cada intento
         * durante la restriccion es, en el mejor caso, inutil, y en el
         * peor vuelve a resetear/extender el propio backoff. */
        const char *cursor = linea + 16; /* tras "Restricted_Wait_" */
        uint32_t esperaMs = 0;
        if (NumeroTexto_LeerUint(&cursor, 10U, &esperaMs)) {
            s_restringidoHastaTickMs = HAL_GetTick() + (uint32_t)esperaMs;
            printf("RAK3172: modulo restringido por backoff de join/duty-cycle -- %lu ms (~%.1fh). "
                   "Pausando reintentos de join/uplink hasta que pase.\r\n",
                   (unsigned long)esperaMs, (double)esperaMs / 3600000.0);
        }
        if (s_comandoEnCurso) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_ERROR;
        }
        return;
    }

    if (strcmp(linea, "AT_NO_NETWORK_JOINED") == 0) {
        /* El modulo perdio su estado de join -- visto en campo justo
         * despues de que reaparece su banner de arranque
         * ("RAKwireless RAK3172-E...") en medio de la sesion, es decir
         * el modulo se reinicio solo (no el STM32) y olvido el join.
         * s_estaUnido seguia en true porque nada lo desmentia: sin
         * este chequeo, el reintento periodico de join en main.c
         * (que solo se dispara si !RAK3172_EstaUnido()) nunca se
         * activaba, y cada intento de uplink se quedaba pegado los
         * 2s completos del timeout en vez de fallar al instante. */
        s_estaUnido = false;
        if (s_comandoEnCurso) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_ERROR;
        }
        printf("RAK3172: el modulo ya no esta unido a la red (AT_NO_NETWORK_JOINED) -- se reintentara el join\r\n");
        return;
    }

    if (s_comandoEnCurso) {
        if (strcmp(linea, "OK") == 0) {
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_OK;
        } else if (strncmp(linea, "ERROR", 5) == 0 || strncmp(linea, "AT_ERROR", 8) == 0
                   || strstr(linea, "_ERROR") != NULL) {
            /* RUI3 puede responder "ERROR" a secas, o variantes con
             * prefijo/sufijo como "AT_ERROR", "AT_PARAM_ERROR", etc.
             * -- sin este chequeo mas amplio, esas respuestas caian en
             * la rama de "linea de datos" de abajo y el comando se
             * quedaba colgado hasta expirar por timeout, en vez de
             * detectarse al instante como error. */
            s_comandoEnCurso = false;
            s_ultimoResultado = RAK3172_ERROR;
        } else if (linea[0] != '\0') {
            /* Línea de datos previa al OK, ej. respuesta de AT+VER=? o
             * de AT+LTIME=? */
            strncpy(s_ultimaRespuesta, linea, sizeof(s_ultimaRespuesta) - 1);
            s_ultimaRespuesta[sizeof(s_ultimaRespuesta) - 1] = '\0';
            s_hayRespuestaNueva = true;
        }
    }
    /* Líneas fuera de un comando en curso y que no son "+EVT:" se
     * ignoran (ej. banners de arranque del módulo). */
}

static void RAK3172_ProcesarEventoDownlink(const char *linea)
{
    /* Formato REAL confirmado en campo (firmware RUI_4.2.4_RAK3172-E):
     *
     *   +EVT:RX_1:-28:8:UNICAST:2:00000710
     *        │    │   │    │    │    │
     *        │    │   │    │    │    └─ payload en hex: [ID][valor...]
     *        │    │   │    │    └────── FPort (RAK3172_FPORT_PARAMETRO)
     *        │    │   │    └─────────── tipo (UNICAST)
     *        │    │   └──────────────── SNR
     *        │    └──────────────────── RSSI
     *        └───────────────────────── ventana de recepción (RX_1/RX_2)
     *
     * El payload ya NO se interpreta como "todo es un solo número" --
     * el PRIMER byte es el ID del parámetro (ver calibracion_flash.h,
     * CALIB_ID_*), y los bytes restantes son el valor de ese parámetro
     * en el formato/escala que le corresponda. Toda esa lógica vive
     * en CalibFlash_ProcesarParametroConEstado(), este módulo solo separa los
     * bytes y se los pasa.
     */
    /* Lector a mano en vez de sscanf("+EVT:%15[^:]:%d:%d:%15[^:]:%u:%63s")
     * (2026-10-01, ahorra el lector de la libreria de C). Mismas reglas:
     * 6 campos separados por ':', ventana y tipo no vacios (<= 15
     * caracteres), RSSI/SNR enteros con signo, FPort entero, y payload sin
     * espacios (<= 63). Solo se usan el FPort y el payload. */
    const char *p = linea + 5; /* tras "+EVT:" */
    const char *payloadHex = NULL;
    uint8_t largoPayload = 0U;
    uint32_t fport = 0U;
    for (uint8_t campo = 0U; campo < 6U; campo++) {
        const char *inicio = p;
        if (campo == 1U || campo == 2U) {          /* RSSI, SNR */
            uint32_t ignorado;
            if (*p == '-' || *p == '+') p++;
            if (!NumeroTexto_LeerUint(&p, 6U, &ignorado)) return;
        } else if (campo == 4U) {                  /* FPort */
            if (!NumeroTexto_LeerUint(&p, 3U, &fport)) return;
        } else if (campo == 5U) {                  /* payload hex */
            while (*p != '\0' && *p != ' ' && *p != '\r' && *p != '\n') p++;
            if (p == inicio || p - inicio > 63) return;
            payloadHex = inicio;
            largoPayload = (uint8_t)(p - inicio);
            break;
        } else {                                   /* ventana, tipo */
            while (*p != '\0' && *p != ':') p++;
            if (p == inicio || p - inicio > 15) return;
        }
        if (*p != ':') return; /* no es una línea de downlink con payload, se ignora */
        p++;
    }

    if (fport != RAK3172_FPORT_PARAMETRO) {
        return; /* downlink en un FPort que no es el de parámetros -- se ignora */
    }

    uint8_t bytesPayload[16];
    uint8_t cantidadBytes = 0;

    if (!RAK3172_HexAAlBytes(payloadHex, largoPayload, bytesPayload, sizeof(bytesPayload), &cantidadBytes)) {
        return; /* payload no era hexadecimal válido */
    }

    if (cantidadBytes < 1) {
        return; /* no hay ni siquiera el byte de ID */
    }

    uint8_t idParametro = bytesPayload[0];
    const uint8_t *datosValor = &bytesPayload[1];
    uint8_t longitudValor = (uint8_t)(cantidadBytes - 1U);

    /* El motor "opera" si el tacómetro NO lo reporta detenido -- se
     * consulta aquí (en vez de que calibracion_flash.c dependa
     * directamente de tacometro.h) para mantener a calibracion_flash
     * desacoplado del módulo de medición de RPM. */
    bool motorOperando = !Tacometro_EstaDetenido();

    uint16_t valorAplicadoRaw = 0U;
    CalibFlash_ProtocoloStatus_t status = CalibFlash_ProcesarParametroConEstado(
        idParametro, datosValor, longitudValor, motorOperando,
        Tacometro_GetFrecuenciaHz(), &valorAplicadoRaw);

    printf("Downlink ID=%u (%u bytes de valor) -> STATUS=%d, valor vigente=0x%04X\r\n",
           idParametro, longitudValor, (int)status, valorAplicadoRaw);

    /* Application ACK del protocolo Quick-Set: confirma al servidor
     * qué quedó realmente vigente, sin importar si se aplicó o se
     * rechazó el valor solicitado. */
    RAK3172_EnviarAck(idParametro, (uint8_t)status, valorAplicadoRaw);
}

static bool RAK3172_HexAAlBytes(const char *hex, uint8_t longitudHex, uint8_t *bytesSalida, uint8_t maxBytes, uint8_t *cantidadBytes)
{
    /* Se espera un número par de caracteres hex (2 por byte). Si el
     * payload real trae longitud impar (poco común, pero posible si
     * el servidor no rellena con cero a la izquierda), se rechaza en
     * vez de intentar adivinar el alineamiento. */
    if (longitudHex == 0 || (longitudHex % 2) != 0) {
        return false;
    }

    uint8_t cantidad = (uint8_t)(longitudHex / 2);
    if (cantidad > maxBytes) {
        return false; /* payload más grande de lo que se esperaba */
    }

    for (uint8_t i = 0; i < cantidad; i++) {
        int alto = NumeroTexto_HexDigito(hex[i * 2]);
        int bajo = NumeroTexto_HexDigito(hex[i * 2 + 1]);
        if (alto < 0 || bajo < 0) {
            return false; /* carácter no hexadecimal encontrado */
        }
        bytesSalida[i] = (uint8_t)((alto << 4) | bajo);
    }

    *cantidadBytes = cantidad;
    return true;
}
