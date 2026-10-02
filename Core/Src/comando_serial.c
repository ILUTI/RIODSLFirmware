/**
 * @file    comando_serial.c
 * @brief   Implementación del mando manual por serial (ver comando_serial.h).
 */

#include "comando_serial.h"
#include "calibracion_flash.h"
#include "tacometro.h"
#include "presion_voltaje.h"
#include "rak3172.h"
#include "numero_texto.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define COMANDO_SERIAL_LINEA_MAX   48U

typedef enum {
    TIPO_U16_RAW,     /* entero directo, sin escala (us, segundos, flags/enum de 1 byte) */
    TIPO_X10,
    TIPO_X100,
    TIPO_X1000,
    TIPO_S16_X100,    /* con signo -- ganancias PID */
    TIPO_S16_X1000,
    TIPO_COMANDO      /* no lleva VALOR -- siempre manda el byte de confirmación */
} ComandoSerial_Tipo_t;

typedef struct {
    const char *nombre;
    uint8_t id;
    ComandoSerial_Tipo_t tipo;
} ComandoSerial_Descriptor_t;

/* Espejo EXACTO de la tabla de parámetros del TID (README sección 3) y
 * de PARAMETER_TABLE en send_downlink.py -- si el firmware agrega o
 * cambia un parámetro en calibracion_flash.c, actualizar aquí también. */
static const ComandoSerial_Descriptor_t TABLA[] = {
    {"SET_RATIO",                       CALIB_ID_SET_RATIO,                       TIPO_X100},
    {"SET_RATIO_AUTO",                  CALIB_ID_SET_RATIO_AUTO,                  TIPO_X10},
    {"ALPHA",                           CALIB_ID_ALPHA,                           TIPO_X1000},
    {"SET_RPM",                         CALIB_ID_SET_RPM,                         TIPO_X10},
    {"RPM_MAX",                         CALIB_ID_RPM_MAX,                         TIPO_X10},
    {"RPM_MIN",                         CALIB_ID_RPM_MIN,                         TIPO_X10},
    {"PID_RPM_KP",                      CALIB_ID_PID_RPM_KP,                      TIPO_S16_X1000},
    {"PID_RPM_KI",                      CALIB_ID_PID_RPM_KI,                      TIPO_S16_X1000},
    {"SERVO_PULSO_MIN",                 CALIB_ID_SERVO_PULSO_MIN,                 TIPO_U16_RAW},
    {"SERVO_PULSO_MAX",                 CALIB_ID_SERVO_PULSO_MAX,                 TIPO_U16_RAW},
    {"CALIB",                           CALIB_ID_CALIB,                           TIPO_U16_RAW},
    {"INTERVALO_ENVIO_OPERATIVO_S",     CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S,     TIPO_U16_RAW},
    {"INTERVALO_ENVIO_STANDBY_S",       CALIB_ID_INTERVALO_ENVIO_STANDBY_S,       TIPO_U16_RAW},
    {"MODO",                            CALIB_ID_MODO,                            TIPO_U16_RAW},
    {"PRESION_REMOTO",                  CALIB_ID_PRESION_REMOTO,                  TIPO_X10},
    {"REPORTAR_PARAMETROS",             CALIB_ID_REPORTAR_PARAMETROS,             TIPO_COMANDO},
    {"FORZAR_REPORTE",                  CALIB_ID_FORZAR_REPORTE,                  TIPO_COMANDO},
    {"RESET_REMOTO",                    CALIB_ID_RESET_REMOTO,                    TIPO_COMANDO},
    {"PRESION_OBJETIVO_LOCAL",          CALIB_ID_PRESION_OBJETIVO_LOCAL,          TIPO_X10},
    {"PID_ASP_KP",                      CALIB_ID_PID_ASP_KP,                      TIPO_S16_X1000},
    {"PID_ASP_KI",                      CALIB_ID_PID_ASP_KI,                      TIPO_S16_X1000},
    {"PID_PSI_KP",                      CALIB_ID_PID_PSI_KP,                      TIPO_S16_X1000},
    {"PID_PSI_KI",                      CALIB_ID_PID_PSI_KI,                      TIPO_S16_X1000},
    {"PRESION_OBJETIVO_REMOTO",         CALIB_ID_PRESION_OBJETIVO_REMOTO,         TIPO_X10},
    {"SET_PRESION",                     CALIB_ID_SET_PRESION,                     TIPO_X10},
    {"RPM_MAX_CARGA",                   CALIB_ID_RPM_MAX_CARGA,                   TIPO_X10},
    {"TIEMPO_LLENADO_S",                CALIB_ID_TIEMPO_LLENADO_S,                TIPO_U16_RAW},
    {"PRESION_MAX",                     CALIB_ID_PRESION_MAX,                     TIPO_X10},
};
#define TABLA_CANTIDAD (sizeof(TABLA) / sizeof(TABLA[0]))

static UART_HandleTypeDef *s_huart;
static char     s_linea[COMANDO_SERIAL_LINEA_MAX];
static uint8_t  s_indice;
static uint8_t  s_rxByte;

static const ComandoSerial_Descriptor_t *BuscarDescriptor(const char *nombre)
{
    for (size_t i = 0; i < TABLA_CANTIDAD; i++) {
        if (strcmp(TABLA[i].nombre, nombre) == 0) {
            return &TABLA[i];
        }
    }
    return NULL;
}

/* Misma convención de escala/redondeo que CodificarUint16()/
 * CodificarInt16ComoRaw() en calibracion_flash.c (no expuestas fuera
 * de ese archivo), para que el valor que llega a
 * CalibFlash_ProcesarParametroConEstado() sea bit a bit igual al que
 * mandaría un downlink real con el mismo valor humano. */
static void CodificarValor(ComandoSerial_Tipo_t tipo, float valor, uint8_t datos[2])
{
    uint16_t raw;

    if (tipo == TIPO_COMANDO) {
        raw = CALIB_BYTE_CONFIRMACION; /* VALUE_H=0 implícito, VALUE_L=0xA5 */
    } else {
        float escala = 1.0f;
        float minimo = 0.0f, maximo = 65535.0f;
        switch (tipo) {
            case TIPO_X10:        escala = 10.0f;   break;
            case TIPO_X100:       escala = 100.0f;  break;
            case TIPO_X1000:      escala = 1000.0f; break;
            case TIPO_S16_X100:   escala = 100.0f;  minimo = -32768.0f; maximo = 32767.0f; break;
            case TIPO_S16_X1000:  escala = 1000.0f; minimo = -32768.0f; maximo = 32767.0f; break;
            case TIPO_U16_RAW:
            default:              escala = 1.0f;    break;
        }
        float escalado = valor * escala;
        if (escalado < minimo) escalado = minimo;
        if (escalado > maximo) escalado = maximo;
        raw = (uint16_t)(int32_t)(escalado >= 0.0f ? (escalado + 0.5f) : (escalado - 0.5f));
    }

    datos[0] = (uint8_t)(raw >> 8);
    datos[1] = (uint8_t)(raw & 0xFFU);
}

/* Inversa de CodificarValor(), para mostrar el "valor vigente" del ACK
 * en unidades humanas (igual que valorAplicado en el ACK real). */
static float DecodificarValor(ComandoSerial_Tipo_t tipo, uint16_t raw)
{
    switch (tipo) {
        case TIPO_X10:       return raw / 10.0f;
        case TIPO_X100:      return raw / 100.0f;
        case TIPO_X1000:     return raw / 1000.0f;
        case TIPO_S16_X100:  return (float)(int16_t)raw / 100.0f;
        case TIPO_S16_X1000: return (float)(int16_t)raw / 1000.0f;
        case TIPO_U16_RAW:
        case TIPO_COMANDO:
        default:             return (float)raw;
    }
}

static const char *EstadoTexto(CalibFlash_ProtocoloStatus_t status)
{
    switch (status) {
        case CALIB_STATUS_OK:                     return "OK";
        case CALIB_STATUS_OUT_OF_RANGE:           return "OUT_OF_RANGE";
        case CALIB_STATUS_UNKNOWN_PARAMETER_ID:   return "UNKNOWN_PARAMETER_ID";
        case CALIB_STATUS_STORAGE_ERROR:          return "STORAGE_ERROR";
        case CALIB_STATUS_APPLY_ERROR:            return "APPLY_ERROR";
        case CALIB_STATUS_REJECTED_ENGINE_RUNNING:return "REJECTED_ENGINE_RUNNING";
        case CALIB_STATUS_REJECTED_ENGINE_STOPPED:return "REJECTED_ENGINE_STOPPED";
        case CALIB_STATUS_REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO:
            return "REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO";
        default:                                  return "?";
    }
}

/* Pass-through permanente hacia el RAK3172: consultas "AT+XXX=?" (leer
 * APPKEY/DEVEUI/VER/etc.) mas una lista corta de escrituras de
 * provisionamiento (ver AT_ESCRITURAS_PERMITIDAS y ATZ). Cualquier otro
 * comando AT se rechaza. Comparte el canal AT con los uplinks (ocupa
 * hasta PASSTHROUGH_TIMEOUT_MS). */
#define PASSTHROUGH_TIMEOUT_MS  3000U
static char              s_atPendiente[COMANDO_SERIAL_LINEA_MAX];
static volatile bool     s_atPorEnviar;
static bool              s_atEsperando;
static uint32_t          s_atTickInicio;

/* Escrituras permitidas (prefijo hasta el '='); ATZ (reset del RAK) va
 * aparte por no llevar '='. Todo lo demas que no sea consulta "=?" se
 * rechaza -- en particular AT+FACTORY, AT+NWM, AT+BOOT, AT+JOIN, etc. */
static const char *const AT_ESCRITURAS_PERMITIDAS[] = {
    "AT+NJM=", "AT+BAND=", "AT+MASK=", "AT+CLASS=",
};

/* Escrituras de PROVISIONAMIENTO de identidad LoRaWAN (DevEUI/JoinEUI/
 * AppKey), agregado 2026-09-30. Solo se aceptan con MODO=CALIBRACION (4)
 * Y el motor detenido -- mismo espiritu que el candado de RPM_MAX/MIN. Se
 * guardan en la memoria NO volatil del propio RAK3172, NO en la flash del
 * STM32 (este modulo solo reenvia el comando), asi que sobreviven a
 * cualquier cambio del magic de calibracion (CALx) y a RESTAURAR_DEFAULTS. */
static const char *const AT_ESCRITURAS_SOLO_CALIBRACION[] = {
    "AT+DEVEUI=", "AT+APPEUI=", "AT+APPKEY=",
};

static bool EsEscrituraSoloCalibracion(const char *linea)
{
    for (size_t i = 0; i < sizeof(AT_ESCRITURAS_SOLO_CALIBRACION) / sizeof(AT_ESCRITURAS_SOLO_CALIBRACION[0]); i++) {
        size_t p = strlen(AT_ESCRITURAS_SOLO_CALIBRACION[i]);
        if (strlen(linea) > p && strncmp(linea, AT_ESCRITURAS_SOLO_CALIBRACION[i], p) == 0) {
            return true;
        }
    }
    return false;
}

static bool EsComandoATPermitido(const char *linea)
{
    size_t n = strlen(linea);
    if (n > 5U && strncmp(linea, "AT+", 3U) == 0 &&
        linea[n - 2U] == '=' && linea[n - 1U] == '?') {
        return true; /* consulta */
    }
    if (strcmp(linea, "ATZ") == 0) {
        return true;
    }
    for (size_t i = 0; i < sizeof(AT_ESCRITURAS_PERMITIDAS) / sizeof(AT_ESCRITURAS_PERMITIDAS[0]); i++) {
        size_t p = strlen(AT_ESCRITURAS_PERMITIDAS[i]);
        if (n > p && strncmp(linea, AT_ESCRITURAS_PERMITIDAS[i], p) == 0) {
            return true;
        }
    }
    /* DEVEUI/APPEUI/APPKEY: permitido solo si se cumplen las condiciones
     * (MODO=4 + motor detenido); si no, ProcesarLinea() da el motivo. */
    if (EsEscrituraSoloCalibracion(linea)) {
        return CalibFlash_GetModo() == CALIB_MODO_CALIBRACION && Tacometro_EstaDetenido();
    }
    return false;
}

static void ProcesarLinea(char *linea)
{
    if (strncmp(linea, "AT", 2U) == 0) {
        if (!EsComandoATPermitido(linea)) {
            if (EsEscrituraSoloCalibracion(linea)) {
                printf("[AT] DEVEUI/APPEUI/APPKEY solo con MODO=4 (CALIBRACION) y el motor detenido\r\n");
                return;
            }
            printf("[AT] no permitido -- solo consultas 'AT+XXX=?', AT+NJM/BAND/MASK/CLASS=<v>, AT+DEVEUI/APPEUI/APPKEY=<v> (MODO=4, motor detenido) y ATZ\r\n");
        } else if (s_atPorEnviar || s_atEsperando) {
            printf("[AT] ocupado, espera la respuesta anterior\r\n");
        } else {
            strncpy(s_atPendiente, linea, sizeof(s_atPendiente) - 1U);
            s_atPendiente[sizeof(s_atPendiente) - 1U] = '\0';
            s_atPorEnviar = true; /* se transmite desde ComandoSerial_Update(), fuera de la ISR */
        }
        return;
    }

    /* strtok en vez de sscanf("%s %f") -- RESTAURAR_DEFAULTS/FORZAR_REPORTE/
     * RESET_REMOTO no llevan VALOR, y sscanf con %f de menos fallaría
     * la conversión completa en vez de solo dejar valor en 0. */
    char *nombre = strtok(linea, " \t");
    if (nombre == NULL) {
        return; /* línea vacía (solo Enter) */
    }
    char *valorTexto = strtok(NULL, " \t");
    /* NumeroTexto_LeerDecimal() en vez de strtof(): ahorra ~6-7 KB de flash
     * (ver numero_texto.h). Mismo resultado para "[signo]digitos[.digitos]";
     * no acepta notacion cientifica (1e3), que nunca se usa aca. */
    float valor = (valorTexto != NULL) ? NumeroTexto_LeerDecimal(valorTexto, NULL) : 0.0f;

    const ComandoSerial_Descriptor_t *desc = BuscarDescriptor(nombre);
    if (desc == NULL) {
        printf("[CMD] \"%s\" no reconocido -- nombres validos: ver tabla de parametros, README seccion 3\r\n", nombre);
        return;
    }

    uint8_t datosValor[2];
    CodificarValor(desc->tipo, valor, datosValor);

    /* Mismo criterio que rak3172.c: el motor "opera" si el tacometro no
     * lo reporta detenido -- CalibFlash_ProcesarParametroConEstado()
     * aplica exactamente las mismas reglas de categoria/motor-operando
     * que un downlink real por LoRa. */
    bool motorOperando = !Tacometro_EstaDetenido();
    uint16_t valorAplicadoRaw = 0U;
    CalibFlash_ProtocoloStatus_t status = CalibFlash_ProcesarParametroConEstado(
        desc->id, datosValor, 2U, motorOperando, Tacometro_GetFrecuenciaHz(),
        &valorAplicadoRaw);

    /* SET_RATIO_AUTO es la única entrada donde el VALOR recibido no
     * coincide con el valor vigente devuelto: recibe RPM (x10) pero
     * devuelve el ratio resultante (x100, misma escala que SET_RATIO). */
    ComandoSerial_Tipo_t tipoValorVigente =
        (desc->id == CALIB_ID_SET_RATIO_AUTO) ? TIPO_X100 : desc->tipo;

    printf("[CMD] %s(ID=%u) <- %.3f -> STATUS=%s, valor vigente=%.3f (raw=0x%04X)\r\n",
           desc->nombre, desc->id, valor, EstadoTexto(status),
           DecodificarValor(tipoValorVigente, valorAplicadoRaw), valorAplicadoRaw);
}

void ComandoSerial_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_indice = 0U;
    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1U);
}

void ComandoSerial_Update(void)
{
    /* Recepcion de la consola: ver ComandoSerial_RxCpltCallback().
     * Aqui solo el pass-through temporal de consultas AT al RAK3172. */
    if (s_atPorEnviar && RAK3172_ComandoListo()) {
        if (RAK3172_EnviarComandoAT(s_atPendiente)) {
            s_atEsperando = true;
            s_atTickInicio = HAL_GetTick();
        } else {
            printf("[AT] no se pudo enviar\r\n");
        }
        s_atPorEnviar = false;
    }

    if (s_atEsperando) {
        char respuesta[64];
        if (RAK3172_GetUltimaRespuesta(respuesta, sizeof(respuesta))) {
            printf("[AT] %s\r\n", respuesta);
        }
        if (RAK3172_ComandoListo()) {
            printf("[AT] resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n",
                   RAK3172_GetUltimoResultado());
            s_atEsperando = false;
        } else if ((HAL_GetTick() - s_atTickInicio) > PASSTHROUGH_TIMEOUT_MS) {
            s_atEsperando = false;
        }
    }
}

void ComandoSerial_RxCpltCallback(UART_HandleTypeDef *huart)
{
    char c = (char)s_rxByte;

    if (c == '\r' || c == '\n') {
        printf("\r\n");
        if (s_indice > 0U) {
            s_linea[s_indice] = '\0';
            ProcesarLinea(s_linea);
            s_indice = 0U;
        }
    } else if (c == '\b' || c == 0x7F) {
        if (s_indice > 0U) {
            s_indice--;
            printf("\b \b");
        }
    } else if (s_indice < (COMANDO_SERIAL_LINEA_MAX - 1U)) {
        s_linea[s_indice++] = c;
        /* Eco local con timeout 0 (no bloqueante de verdad) -- esto
         * corre DENTRO de la ISR de LPUART1, que comparte prioridad con
         * TIM2 (captura de RPM, ver .ioc: todo a 0:0 con
         * PriorityGroup=NVIC_PRIORITYGROUP_4, sin preferencia entre
         * ellas) -- un timeout largo acá demoraria la captura de un
         * pulso de RPM si coincide con esta ISR. Si el UART no está
         * listo de inmediato (caso raro, solo con TX ya ocupado por el
         * printf() de telemetría), se descarta ese carácter de eco --
         * es cosmético (el operador ya tipeó el caracter, solo no lo ve
         * reflejado en su terminal), nunca afecta qué se guarda en
         * s_linea ni qué comando se procesa. */
        HAL_UART_Transmit(s_huart, (uint8_t *)&c, 1U, 0U);
    }

    /* Siempre rearmar, incluso si la línea se descartó por estar llena
     * -- sin esto, un solo byte recibido deja el UART sordo para
     * siempre (mismo motivo que RAK3172_ErrorCallback/GPS_ErrorCallback
     * rearman su DMA circular tras un error). */
    HAL_UART_Receive_IT(huart, &s_rxByte, 1U);
}

void ComandoSerial_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance) {
        return;
    }
    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1U);
}
