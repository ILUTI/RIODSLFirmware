/**
 * @file    comando_serial.c
 * @brief   Implementación del mando manual por serial (ver comando_serial.h).
 */

#include "comando_serial.h"
#include "calibracion_flash.h"
#include "tacometro.h"
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
    {"PID_KP",                          CALIB_ID_PID_KP,                          TIPO_S16_X100},
    {"PID_KI",                          CALIB_ID_PID_KI,                          TIPO_S16_X1000},
    {"TASA_LLENADO_PSI_S",              CALIB_ID_TASA_LLENADO_PSI_S,              TIPO_X100},
    {"SERVO_PULSO_MIN",                 CALIB_ID_SERVO_PULSO_MIN,                 TIPO_U16_RAW},
    {"SERVO_PULSO_MAX",                 CALIB_ID_SERVO_PULSO_MAX,                 TIPO_U16_RAW},
    {"TIMEOUT_SIN_COMANDO_S",           CALIB_ID_TIMEOUT_SIN_COMANDO_S,           TIPO_U16_RAW},
    {"TASA_MAX_CAMBIO_RPM_S",           CALIB_ID_TASA_MAX_CAMBIO_RPM_S,           TIPO_X10},
    {"CONTROL_HABILITADO",              CALIB_ID_CONTROL_HABILITADO,              TIPO_U16_RAW},
    {"INTERVALO_ENVIO_OPERATIVO_S",     CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S,     TIPO_U16_RAW},
    {"INTERVALO_ENVIO_STANDBY_S",       CALIB_ID_INTERVALO_ENVIO_STANDBY_S,       TIPO_U16_RAW},
    {"MODO",                            CALIB_ID_MODO,                            TIPO_U16_RAW},
    {"PRESION",                         CALIB_ID_PRESION,                         TIPO_X10},
    {"NODE_ID",                         CALIB_ID_NODE_ID,                         TIPO_U16_RAW},
    {"RESTAURAR_DEFAULTS",              CALIB_ID_RESTAURAR_DEFAULTS,              TIPO_COMANDO},
    {"FORZAR_REPORTE",                  CALIB_ID_FORZAR_REPORTE,                  TIPO_COMANDO},
    {"HISTERESIS_MODO_S",               CALIB_ID_HISTERESIS_MODO_S,               TIPO_U16_RAW},
    {"RESET_REMOTO",                    CALIB_ID_RESET_REMOTO,                    TIPO_COMANDO},
    {"PRESION_OBJETIVO",                CALIB_ID_PRESION_OBJETIVO,                TIPO_X10},
    {"TASA_MAX_CAMBIO_RPM_LLENADO_S",   CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S,   TIPO_X10},
    {"PRESION_REMOTO_PID_KP",           CALIB_ID_PRESION_REMOTO_PID_KP,           TIPO_S16_X100},
    {"PRESION_REMOTO_PID_KI",           CALIB_ID_PRESION_REMOTO_PID_KI,           TIPO_S16_X1000},
    {"PRESION_PID_KP",                  CALIB_ID_PRESION_PID_KP,                  TIPO_S16_X100},
    {"PRESION_PID_KI",                  CALIB_ID_PRESION_PID_KI,                  TIPO_S16_X1000},
    {"PRESION_OBJETIVO_REMOTO",         CALIB_ID_PRESION_OBJETIVO_REMOTO,         TIPO_X10},
};
#define TABLA_CANTIDAD (sizeof(TABLA) / sizeof(TABLA[0]))

static UART_HandleTypeDef *s_huart;
static char     s_linea[COMANDO_SERIAL_LINEA_MAX];
static uint8_t  s_indice;

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

static void ProcesarLinea(char *linea)
{
    /* strtok en vez de sscanf("%s %f") -- RESTAURAR_DEFAULTS/FORZAR_REPORTE/
     * RESET_REMOTO no llevan VALOR, y sscanf con %f de menos fallaría
     * la conversión completa en vez de solo dejar valor en 0. */
    char *nombre = strtok(linea, " \t");
    if (nombre == NULL) {
        return; /* línea vacía (solo Enter) */
    }
    char *valorTexto = strtok(NULL, " \t");
    float valor = (valorTexto != NULL) ? strtof(valorTexto, NULL) : 0.0f;

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
        desc->id, datosValor, 2U, motorOperando, Tacometro_GetFrecuenciaHz(), &valorAplicadoRaw);

    /* SET_RATIO_AUTO es la única entrada de la tabla donde el tipo de
     * VALOR recibido (RPM de referencia, x10) no coincide con el tipo
     * del valor vigente devuelto (el ratio resultante, x100 -- misma
     * escala que SET_RATIO) -- decodificar el "vigente" siempre como
     * x100 para este ID en particular, no con desc->tipo. */
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
}

void ComandoSerial_Update(void)
{
    /* Sondeo directo de la bandera RXNE (sin bloquear, sin IT/DMA) --
     * un operador tipeando a mano nunca llega a saturar esto; leer el
     * registro de datos limpia la bandera por hardware. */
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        char c = (char)(s_huart->Instance->RDR & 0xFFU);

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
            HAL_UART_Transmit(s_huart, (uint8_t *)&c, 1U, 10U); /* eco local */
        }
    }
}
