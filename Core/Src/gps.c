/**
 * @file    gps.c
 * @brief   Implementación del módulo de lectura de posición GPS/GNSS
 *          (SIM7600X) vía auto-reporte periódico de AT+CGPSINFO.
 *
 * ⚠️ Cambio de diseño (confirmado en campo, 2026-08-18): este módulo
 * NO transmite sentencias NMEA por su cuenta en esta UART -- probado
 * a mano vía USB-TTL directo, sin ninguna sentencia '$...' espontánea.
 * Lo que SÍ soporta (documentado en el AT Command Manual de SIMCom) es
 * "AT+CGPSINFO=<1-255>": configurado una sola vez, el módulo empieza a
 * mandar una línea "+CGPSINFO: ..." por su cuenta cada N segundos, sin
 * que el host tenga que volver a pedirla -- un URC, igual que los
 * "+EVT:" del RAK3172, no una respuesta a comando en curso.
 *
 * Diseño de recepción (modo Circular): el buffer de DMA se rearma
 * solo -- este módulo lleva su propio índice de "última posición
 * leída" (s_ultimaPosLeida) y en cada callback consume desde ahí hasta
 * la posición absoluta que reporta HAL ('Size'), con aritmética
 * módulo el tamaño del buffer para manejar el "dar la vuelta". Las
 * líneas completas (terminadas en '\n') se encolan para que
 * GPS_Update() las parsee fuera de contexto de interrupción -- mismo
 * patrón que rak3172.c.
 */

#include "gps.h"
#include "numero_texto.h"
#include <string.h>
#include <stdio.h>

/* ==================== ESTADO INTERNO ==================== */

static UART_HandleTypeDef *s_huart = NULL;

/* Buffer de recepción, llenado continuamente por DMA en modo Circular. */
static uint8_t  s_rxDmaBuffer[GPS_RX_BUFFER_SIZE];
static uint16_t s_ultimaPosLeida = 0;

/* Ensamblador de línea (acumula bytes hasta '\n'). */
static char     s_lineaEnConstruccion[GPS_LINEA_MAX_LEN];
static uint16_t s_lineaLen = 0;

/* Cola circular de lineas completas pendientes de procesar. El modulo
 * manda una linea cada 10 s: 4 lugares sobran (2026-10-02, antes 8). */
#define GPS_MAX_LINEAS_PENDIENTES  4U
static char     s_colaLineas[GPS_MAX_LINEAS_PENDIENTES][GPS_LINEA_MAX_LEN];
static volatile uint8_t s_colaHead = 0; /* siguiente a leer */
static volatile uint8_t s_colaTail = 0; /* siguiente a escribir */
static volatile uint8_t s_colaCount = 0;

/* Última posición conocida. */
static volatile bool  s_tieneFix = false;
/* HAL_GetTick() del ultimo "+CGPSINFO:" recibido (con o sin fix) -- si deja
 * de llegar por mas de GPS_REPORTE_VIGENCIA_MS, el fix se da por vencido
 * aunque el ultimo reporte lo trajera (ver GPS_ReporteVigente()). */
static uint32_t s_ultimoReporteMs = 0;
static bool     s_huboReporte = false;

/* Reinicio del GNSS (ver GPS_REARMAR_* en gps.h). */
static uint32_t s_inicioMs = 0;        /* referencia mientras nunca hubo reporte */
static uint32_t s_rearmeMs = 0;        /* ultimo paso del reinicio */
static uint8_t  s_reinicioPaso = 0U;   /* 0 = nada, 1 = mandado AT+CGPS=0, 2 = mandado AT+CGPS=1 */
static float           s_latitud = 0.0f;
static float           s_longitud = 0.0f;

/* Fecha/hora UTC del último fix (campos "fecha ddmmyy"/"hora hhmmss.s"
 * de "+CGPSINFO:", ver GPS_ProcesarCGPSInfo()). s_fechaHoraValida se
 * pone en false cuando se pierde el fix -- igual que s_tieneFix, para
 * que GPS_GetFechaHoraUtc() nunca entregue una hora de un fix viejo. */
static bool     s_fechaHoraValida = false;
static uint16_t s_fhAnio = 0;
static uint8_t  s_fhMes = 0;
static uint8_t  s_fhDia = 0;
static uint8_t  s_fhHora = 0;
static uint8_t  s_fhMinuto = 0;
static uint8_t  s_fhSegundo = 0;

/* Cuenta reportes "+CGPSINFO:" vacíos (sin fix) SEGUIDOS -- se reinicia
 * a 0 en cuanto llega un fix real. Al llegar a
 * GPS_REPORTES_VACIOS_ANTES_DE_REINICIAR, se reinicia el GNSS (ver
 * GPS_ProcesarCGPSInfo()). */
static uint16_t s_reportesVaciosSeguidos = 0;

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static void GPS_EncolarLinea(const char *linea, uint16_t len);
static void GPS_ProcesarSentencia(const char *linea);
static void GPS_ProcesarCGPSInfo(char *linea);
static uint8_t GPS_DividirCampos(char *sentencia, char *campos[], uint8_t maxCampos);
static float GPS_NmeaACoordenadaDecimal(const char *campo, uint8_t digitosGrados);
static void GPS_IniciarReinicioGnss(void);

/* true si llego un "+CGPSINFO:" hace menos de GPS_REPORTE_VIGENCIA_MS. Sin
 * esto, si el modulo dejaba de hablar (colgado, cable, reinicio propio), el
 * ultimo fix quedaba valido para siempre y main.c re-seteaba el RTC con esa
 * hora vieja cada 30 s (hallazgo B15, corregido 2026-10-02). */
static bool GPS_ReporteVigente(void)
{
    return s_huboReporte && (HAL_GetTick() - s_ultimoReporteMs) <= GPS_REPORTE_VIGENCIA_MS;
}

/* Hasta 'n' digitos decimales desde 's' (reemplaza atoi() sobre un buffer
 * de n caracteres -- mismo resultado: se detiene en el primer no-digito y
 * da 0 si no hay ninguno). */
static uint32_t GPS_LeerDigitos(const char *s, uint8_t n)
{
    uint32_t valor;
    (void)NumeroTexto_LeerUint(&s, n, &valor);
    return valor;
}

/* ==================== API PÚBLICA ==================== */

void GPS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_ultimaPosLeida = 0;
    s_lineaLen = 0;
    memset(s_lineaEnConstruccion, 0, sizeof(s_lineaEnConstruccion));

    s_colaHead = 0;
    s_colaTail = 0;
    s_colaCount = 0;

    s_tieneFix = false;
    s_latitud = 0.0f;
    s_longitud = 0.0f;
    s_reportesVaciosSeguidos = 0;
    s_fechaHoraValida = false;
    s_huboReporte = false;
    s_inicioMs = HAL_GetTick();
    s_rearmeMs = s_inicioMs;
    s_reinicioPaso = 0U;

    /* Igual que en RAK3172_Init(): limpiar flags de error y vaciar el
     * registro de datos antes de arrancar, por si quedó algo colgado
     * de un reset previo del host mientras el módulo seguía activo. */
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    volatile uint32_t dummy = s_huart->Instance->RDR;
    (void)dummy;

    /* Arranca la recepción continua por DMA en modo Circular (definido
     * en el .ioc) con detección de línea inactiva (IDLE). El callback
     * HAL_UARTEx_RxEventCallback debe reenviar el evento a
     * GPS_RxEventCallback() cuando huart->Instance == USART2 (ver
     * ejemplo en gps.h). A diferencia de RAK3172_Init(), acá NO hace
     * falta rearmar nada en el callback -- el modo Circular se
     * rearma solo.
     *
     * Nota: NO se deshabilita el evento de medio buffer (DMA_IT_HT)
     * a propósito -- rak3172.c documenta que intentar hacerlo justo
     * después de esta llamada causó un HardFault (hdmarx llegaba
     * NULL en ese punto). No es necesario deshabilitarlo de todas
     * formas: GPS_RxEventCallback() procesa correctamente cualquier
     * evento (HT, TC o IDLE) sin importar cuál lo disparó. */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, GPS_RX_BUFFER_SIZE);
}

void GPS_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance) {
        return;
    }

    /* En modo Circular, 'Size' es la posición ABSOLUTA de escritura
     * del DMA dentro del buffer (0..GPS_RX_BUFFER_SIZE-1), no una
     * cantidad de bytes nuevos -- se consume byte a byte desde la
     * última posición leída hasta acá, con módulo para el "dar la
     * vuelta" del buffer circular.
     *
     * ⚠️ BUG CORREGIDO (2026-08-18, encontrado con el depurador -- se
     * quedaba colgado dentro de este while): cuando el DMA completa una
     * vuelta EXACTA del buffer, HAL puede reportar Size==GPS_RX_BUFFER_SIZE
     * en vez de 0. s_ultimaPosLeida jamas puede valer eso porque el modulo
     * lo mantiene siempre en 0..GPS_RX_BUFFER_SIZE-1 -- sin este ajuste, la
     * condicion del while nunca se cumple y el ciclo gira para siempre
     * consumiendo basura, congelando el loop principal completo (todo
     * lo que dependa de HAL_UART_Transmit/printf sobre hlpuart1 se
     * queda sin correr, incluida la Frecuencia del tacometro). */
    uint16_t posActual = Size;
    if (posActual >= GPS_RX_BUFFER_SIZE) {
        posActual = 0;
    }

    while (s_ultimaPosLeida != posActual) {
        char c = (char)s_rxDmaBuffer[s_ultimaPosLeida];
        s_ultimaPosLeida = (uint16_t)((s_ultimaPosLeida + 1U) % GPS_RX_BUFFER_SIZE);

        if (c == '\r') {
            continue; /* ignorar CR, solo nos importa el LF como fin de sentencia */
        }

        if (c == '\n') {
            if (s_lineaLen > 0) {
                GPS_EncolarLinea(s_lineaEnConstruccion, s_lineaLen);
                s_lineaLen = 0;
            }
            continue;
        }

        if (s_lineaLen < (GPS_LINEA_MAX_LEN - 1U)) {
            s_lineaEnConstruccion[s_lineaLen++] = c;
        } else {
            /* Sentencia más larga que el buffer de línea (corrupta o
             * el parser se desincronizó) -- se descarta y se espera
             * al próximo '\n' en vez de partirla a la mitad. */
            s_lineaLen = 0;
        }
    }

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
}

void GPS_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance) {
        return;
    }

    /* (El printf de diagnostico con HAL ErrorCode se quito 2026-10-02, igual
     * que en RAK3172_ErrorCallback(). Si hace falta, reponerlo aqui.) */

    /* Descartar la sentencia a medio construir -- pudo quedar corrupta
     * o incompleta por el mismo error que disparo este callback (un digito
     * de lat/lon cambiado por ruido quedaria guardado como "ultima
     * posicion conocida"). El resto de esa linea llega sin el prefijo
     * "+CGPSINFO:" y se ignora. */
    s_lineaLen = 0;

    /* Errores leves (ruido/trama/paridad): HAL avisa pero NO detiene el DMA,
     * que sigue escribiendo donde iba (RxState sigue ocupado). Antes se
     * volvia s_ultimaPosLeida a 0 y se intentaba rearrancar igual -- el
     * rearranque fallaba en silencio (HAL_BUSY) y el lector quedaba
     * desfasado del DMA, releyendo datos viejos (2026-10-05). Solo con
     * errores graves (desborde, error de DMA) HAL detiene la recepcion y
     * deja RxState en READY: ahi si hay que rearrancar. Las banderas de
     * los errores leves ya las limpio HAL, y leer RDR con el DMA andando
     * le robaria un byte -- por eso va despues de este chequeo. */
    if (s_huart->RxState != HAL_UART_STATE_READY) {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    volatile uint32_t dummy = s_huart->Instance->RDR;
    (void)dummy;

    /* Rearrancar el DMA en modo Circular reinicia su puntero interno
     * de escritura a 0 -- s_ultimaPosLeida debe volver a 0 junto con
     * el, o quedaria leyendo desde una posicion que ya no corresponde
     * al contenido real del buffer. */
    s_ultimaPosLeida = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rxDmaBuffer, GPS_RX_BUFFER_SIZE);
}

void GPS_Update(void)
{
    while (s_colaCount > 0) {
        __disable_irq();
        char linea[GPS_LINEA_MAX_LEN];
        strncpy(linea, s_colaLineas[s_colaHead], GPS_LINEA_MAX_LEN);
        s_colaHead = (s_colaHead + 1) % GPS_MAX_LINEAS_PENDIENTES;
        s_colaCount--;
        __enable_irq();

        GPS_ProcesarSentencia(linea);
    }

    /* Reinicio del GNSS en 3 pasos sin bloquear el loop (AT+CGPS=0 ya se
     * mando en GPS_IniciarReinicioGnss()): AT+CGPS=1, despues AT+CGPSINFO=10.
     * Si no hay reinicio en curso, se dispara si el modulo se quedo mudo: el
     * de "reportes vacios" no cubre el caso en que NO llega ningun reporte
     * (el SIM7600X se reinicio solo y olvido AT+CGPSINFO=10), porque ese
     * contador solo sube con reportes vacios. */
    uint32_t ahora = HAL_GetTick();
    if (s_reinicioPaso != 0U) {
        if (ahora - s_rearmeMs >= GPS_REARMAR_PAUSA_MS) {
            if (s_reinicioPaso == 1U) {
                GPS_EnviarComandoAT("AT+CGPS=1");
                s_reinicioPaso = 2U;
            } else {
                GPS_EnviarComandoAT("AT+CGPSINFO=10");
                s_reinicioPaso = 0U;
            }
            s_rearmeMs = ahora;
        }
    } else {
        uint32_t ref = s_huboReporte ? s_ultimoReporteMs : s_inicioMs;
        if (ahora - ref >= GPS_REARMAR_SIN_REPORTES_MS
                && ahora - s_rearmeMs >= GPS_REARMAR_SIN_REPORTES_MS) {
            printf("GPS: sin reportes hace %lus -- reiniciando GNSS\r\n",
                   (unsigned long)((ahora - ref) / 1000UL));
            GPS_IniciarReinicioGnss();
        }
    }
}

/* Paso 1 del reinicio del GNSS: apagarlo. AT+CGPS=1 solo, con el GNSS ya
 * encendido, contesta ERROR y no hace nada (2026-10-05: en banco el GPS
 * estuvo yendo y viniendo tras ~2 dias y solo se arreglo al reiniciar el
 * nodo). Apagar y encender conserva los satelites en la memoria del modulo
 * (arranque en caliente, fix en segundos). Los pasos 2 y 3 en GPS_Update(). */
static void GPS_IniciarReinicioGnss(void)
{
    GPS_EnviarComandoAT("AT+CGPS=0");
    s_reinicioPaso = 1U;
    s_rearmeMs = HAL_GetTick();
}

bool GPS_EnviarComandoAT(const char *comando)
{
    char buffer[GPS_TX_BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "%s\r\n", comando);
    if (len <= 0 || (uint32_t)len >= sizeof(buffer)) {
        return false; /* comando demasiado largo */
    }

    HAL_UART_Transmit(s_huart, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    return true;
}

bool GPS_TieneFix(void)
{
    return s_tieneFix && GPS_ReporteVigente();
}

uint32_t GPS_GetEdadReporteMs(void)
{
    return HAL_GetTick() - s_ultimoReporteMs;
}

float GPS_GetLatitud(void)
{
    return s_latitud;
}

float GPS_GetLongitud(void)
{
    return s_longitud;
}

bool GPS_GetFechaHoraUtc(uint16_t *anio, uint8_t *mes, uint8_t *dia,
                          uint8_t *hora, uint8_t *minuto, uint8_t *segundo)
{
    if (!GPS_TieneFix() || !s_fechaHoraValida) {
        return false;
    }

    *anio = s_fhAnio;
    *mes = s_fhMes;
    *dia = s_fhDia;
    *hora = s_fhHora;
    *minuto = s_fhMinuto;
    *segundo = s_fhSegundo;
    return true;
}

/* ==================== FUNCIONES PRIVADAS ==================== */

static void GPS_EncolarLinea(const char *linea, uint16_t len)
{
    if (s_colaCount >= GPS_MAX_LINEAS_PENDIENTES) {
        return; /* cola llena -- se descarta esta línea NUEVA (las ya encoladas se conservan) */
    }

    if (len == 0) {
        return;
    }

    /* No se filtra por prefijo acá -- el volumen de líneas es bajo (un
     * "OK" ocasional a los comandos de arranque, más un "+CGPSINFO:"
     * cada N segundos), así que no vale la pena descartar nada antes
     * de la cola. GPS_ProcesarSentencia() decide qué le importa. */

    uint16_t copiar = (len < (GPS_LINEA_MAX_LEN - 1U)) ? len : (GPS_LINEA_MAX_LEN - 1U);
    memcpy(s_colaLineas[s_colaTail], linea, copiar);
    s_colaLineas[s_colaTail][copiar] = '\0';

    s_colaTail = (s_colaTail + 1) % GPS_MAX_LINEAS_PENDIENTES;
    s_colaCount++;
}

static void GPS_ProcesarSentencia(const char *linea)
{
    /* Se deja A PROPOSITO (decision 2026-10-02): en el monitor serie es la
     * forma de ver si el modulo tiene fix al satelite ("+CGPSINFO:" con
     * datos) o no (",,,,,,,,") y que siguen llegando cada 10 s. */
    printf("GPS RX crudo: '%s'\r\n", linea);

    if (strncmp(linea, "+CGPSINFO:", 10) == 0) {
        /* Saltar el prefijo "+CGPSINFO:" y cualquier espacio despues
         * (el modulo manda "+CGPSINFO: 1416.43...", con un espacio) --
         * si no se salta, ese espacio se cuela como primer caracter del
         * campo de latitud y desalinea los "digitosGrados" que espera
         * GPS_NmeaACoordenadaDecimal(). */
        const char *datos = linea + 10;
        while (*datos == ' ') {
            datos++;
        }
        char copia[GPS_LINEA_MAX_LEN];
        strncpy(copia, datos, sizeof(copia) - 1);
        copia[sizeof(copia) - 1] = '\0';
        GPS_ProcesarCGPSInfo(copia);
    }
    /* Otras líneas ("RDY"/"OK" de arranque, banners, etc.) se ignoran. */
}

static void GPS_ProcesarCGPSInfo(char *linea)
{
    /* Formato de respuesta/URC de SIMCom (SIM7500_SIM7600 Series AT
     * Command Manual), NO es NMEA -- sin checksum:
     *   +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<fecha ddmmyy>,<hora hhmmss.s>,<alt>,<vel>,<rumbo>
     * Sin fix, todos los campos vienen vacíos:
     *   +CGPSINFO: ,,,,,,,,
     */
    char *campos[10] = { 0 };
    uint8_t n = GPS_DividirCampos(linea, campos, 10);
    if (n < 5) {
        return; /* línea incompleta/corrupta */
    }
    /* Llego un reporte (con o sin fix): el modulo sigue hablando. */
    s_ultimoReporteMs = HAL_GetTick();
    s_huboReporte = true;

    /* GPS_ProcesarSentencia() ya salto el prefijo "+CGPSINFO: ", asi que
     * campos[0] es directamente el campo de latitud. */
    if (campos[0][0] == '\0') {
        s_tieneFix = false; /* sin fix -- se conserva la ultima lat/lon conocida */
        s_fechaHoraValida = false;

        s_reportesVaciosSeguidos++;
        if (s_reportesVaciosSeguidos >= GPS_REPORTES_VACIOS_ANTES_DE_REINICIAR
                && s_reinicioPaso == 0U) {
            s_reportesVaciosSeguidos = 0;
            /* Por si el GNSS quedo en un estado raro tras muchas horas
             * encendido (ver GPS_IniciarReinicioGnss()). Si solo le esta
             * costando conseguir fix (mal clima, sin vista al cielo), el
             * reinicio en caliente no le hace perder lo ya adquirido. */
            printf("GPS: %u reportes seguidos sin fix -- reiniciando GNSS\r\n",
                   (unsigned int)GPS_REPORTES_VACIOS_ANTES_DE_REINICIAR);
            GPS_IniciarReinicioGnss();
        }
        return;
    }

    float lat = GPS_NmeaACoordenadaDecimal(campos[0], 2);
    if (campos[1][0] == 'S') {
        lat = -lat;
    }

    float lon = GPS_NmeaACoordenadaDecimal(campos[2], 3);
    if (campos[3][0] == 'W') {
        lon = -lon;
    }

    s_latitud = lat;
    s_longitud = lon;
    s_tieneFix = true;
    s_reportesVaciosSeguidos = 0;

    /* campos[4]="fecha ddmmyy", campos[5]="hora hhmmss.s" (UTC, del
     * propio receptor GNSS) -- se descarta la parte fraccionaria de
     * los segundos. n>=6 garantiza que ambos campos existen (pueden
     * venir vacíos en teoría, aunque no debería pasar con fix válido,
     * de ahí el chequeo de longitud). */
    if (n >= 6 && strlen(campos[4]) >= 6 && strlen(campos[5]) >= 6) {
        s_fhDia     = (uint8_t)GPS_LeerDigitos(campos[4], 2);
        s_fhMes     = (uint8_t)GPS_LeerDigitos(campos[4] + 2, 2);
        s_fhAnio    = (uint16_t)(2000U + GPS_LeerDigitos(campos[4] + 4, 2));

        s_fhHora    = (uint8_t)GPS_LeerDigitos(campos[5], 2);
        s_fhMinuto  = (uint8_t)GPS_LeerDigitos(campos[5] + 2, 2);
        s_fhSegundo = (uint8_t)GPS_LeerDigitos(campos[5] + 4, 2);

        s_fechaHoraValida = true;
    } else {
        s_fechaHoraValida = false;
    }
}

static uint8_t GPS_DividirCampos(char *sentencia, char *campos[], uint8_t maxCampos)
{
    /* Tokeniza a mano porque "+CGPSINFO:" trae campos vacios entre comas
     * consecutivas (todos vacios sin fix) -- separar a mano no desalinea
     * los campos siguientes. Se corta tambien en '*' por si alguna vez
     * llega con checksum estilo NMEA. */
    uint8_t n = 0;
    char *cursor = sentencia;
    campos[n++] = cursor;

    while (*cursor != '\0' && n < maxCampos) {
        if (*cursor == ',' || *cursor == '*') {
            *cursor = '\0';
            cursor++;
            campos[n++] = cursor;
        } else {
            cursor++;
        }
    }

    return n;
}

static float GPS_NmeaACoordenadaDecimal(const char *campo, uint8_t digitosGrados)
{
    if (campo == NULL || campo[0] == '\0') {
        return 0.0f;
    }

    /* Formato (el mismo de NMEA): los primeros 'digitosGrados' caracteres
     * son los grados (2 para latitud, 3 para longitud), el resto son
     * minutos decimales (mm.mmmm). */
    uint32_t grados = GPS_LeerDigitos(campo, digitosGrados);
    /* NumeroTexto_LeerDecimal() en vez de atof(): ahorra ~6-7 KB de flash
     * (ver numero_texto.h). atof() ya se truncaba a float aca, asi que la
     * precision final es la misma (~1e-5 minutos, centimetros). */
    float minutos = NumeroTexto_LeerDecimal(campo + digitosGrados, NULL);

    return (float)grados + (minutos / 60.0f);
}
