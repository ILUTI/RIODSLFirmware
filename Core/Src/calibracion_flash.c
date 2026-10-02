/**
 * @file    calibracion_flash.c
 * @brief   Almacenamiento de calibración en flash y dispatcher
 *          centralizado de parámetros por downlink.
 *
 * Usa la ÚLTIMA página de flash del STM32G431KB (página 63 de 2 KB,
 * dirección 0x0801F800). El STM32G4 programa en "double word" (64 bits)
 * alineados, de ahí que la estructura se rellene a múltiplo de 8 bytes.
 *
 * Formato del valor en el downlink (2 bytes big-endian después del ID):
 * cada parámetro tiene su escala en la tabla k_params[] (x10, x100,
 * x1000, int16 con signo para las ganancias PID, o directo en µs/s).
 * Los parámetros de 1 byte lógico (CALIB, MODO, comandos) usan solo el
 * segundo byte.
 */

#include "calibracion_flash.h"
#include "main.h"
#include "modo_fsm.h"
#include "tacometro.h"
#include <stddef.h>
#include <string.h>

/* ==================== CONFIGURACIÓN DE UBICACIÓN EN FLASH ==================== */

#define CALIB_FLASH_PAGE_NUMBER   63U
#define CALIB_FLASH_ADDRESS       0x0801F800UL
#define CALIB_FLASH_MAGIC         0x43414C4FUL  /* "CALO" (2026-09-29, al agregar presionMax).
                                                 * Subirlo cada vez que cambie el layout de
                                                 * CalibFlash_Datos_t -- ver README sección 2.3. */

/* ==================== VALORES POR DEFECTO ==================== */

#define DEFAULT_PULSOS_POR_REVOLUCION     17.5f    /* confirmado en campo con tacometro digital, ya incluye la relacion de poleas alternador:motor */
#define DEFAULT_ALPHA_FILTRO              0.35f    /* EMA de la RPM: mas alto = responde mas rapido pero mas ruidoso */
#define DEFAULT_RPM_MAX                   2500.0f
#define DEFAULT_RPM_MAX_CARGA             2500.0f  /* = RPM_MAX hasta que se configure el limite hidraulico real de la instalacion */
#define DEFAULT_PRESION_MAX               100.0f   /* PSI -- conservador hasta configurar el limite real de la tuberia */
#define DEFAULT_RPM_MIN                   700.0f
#define DEFAULT_PID_RPM_KP                1.0f
#define DEFAULT_PID_RPM_KI                0.0f
#define DEFAULT_SERVO_PULSO_MIN_US        1000U
#define DEFAULT_SERVO_PULSO_MAX_US        2000U
#define DEFAULT_INTERVALO_OPERATIVO_S     30U      /* uplink con el motor encendido */
#define DEFAULT_INTERVALO_STANDBY_S       30U      /* uplink con el motor apagado */
#define DEFAULT_TIEMPO_LLENADO_S        1800U      /* 30 min, conservador */
#define DEFAULT_PRESION_OBJETIVO_LOCAL    40.0f
#define DEFAULT_PRESION_OBJETIVO_REMOTO    0.0f    /* sin configurar */
#define DEFAULT_PID_ASP_KP                 0.0f    /* sin calibrar (MODO=2) */
#define DEFAULT_PID_ASP_KI                 0.0f
#define DEFAULT_PID_PSI_KP                 0.0f    /* sin calibrar (MODO=1) */
#define DEFAULT_PID_PSI_KI                 0.0f

/* ==================== VALORES FIJOS ==================== */
/* Eran parámetros por downlink (TIMEOUT_SIN_COMANDO_S, HISTERESIS_MODO_S y
 * TASA_MAX_CAMBIO_RPM_LLENADO_S); el 2026-10-01 se dejaron fijos porque no
 * se ajustan por instalación. */

#define FIJO_TIMEOUT_SIN_COMANDO_S          360U   /* 4x el reporte del aspersor en MODO=2 activo (90 s, campo 2026-09-09) */
#define FIJO_HISTERESIS_MODO_S               30U   /* enlace malo antes de degradar MODO */
#define FIJO_TASA_MAX_CAMBIO_RPM_LLENADO_S  10.0f  /* rampa de bajada: 2000->700 RPM en ~2 min */

/* ==================== ESTRUCTURA GUARDADA EN FLASH ==================== */
/* Debe quedar en múltiplo de 8 bytes (double word). Los campos
 * "reservado" son campos eliminados que se dejaron como relleno para no
 * tener que subir el magic (subirlo borra la calibración de campo); se
 * quitan en el próximo cambio de layout que ya obligue a subirlo. */

typedef struct {
    uint32_t magic;

    float    pulsosPorRevolucion;      /* SET_RATIO */
    float    alphaFiltro;              /* ALPHA */
    float    rpmMax;                   /* RPM_MAX -- techo MECANICO del motor, fijo por modelo */
    float    rpmMaxCarga;              /* RPM_MAX_CARGA -- techo con carga (hidraulico), SIEMPRE <= rpmMax */
    float    presionMax;               /* PRESION_MAX -- guarda dura de presion local */
    float    rpmMin;                   /* RPM_MIN */
    float    pidRpmKp;                 /* PID_RPM_KP */
    float    pidRpmKi;                 /* PID_RPM_KI */
    uint32_t reservado4;               /* antes tasaMaxCambioRpmS (ID 12, eliminado 2026-10-01) */
    float    presionObjetivoLocal;     /* PRESION_OBJETIVO_LOCAL */
    uint32_t reservado5;               /* antes tasaMaxCambioRpmLlenadoS (ID 24, ahora fijo 2026-10-01) */
    uint32_t reservado1;               /* antes ultimaLatitudConocida (eliminada 2026-10-01) */
    uint32_t reservado2;               /* antes ultimaLongitudConocida (eliminada 2026-10-01) */
    float    pidAspKp;                 /* PID_ASP_KP -- lazo de presion remota (MODO=2) */
    float    pidAspKi;                 /* PID_ASP_KI */
    float    pidPsiKp;                 /* PID_PSI_KP -- lazo de presion local (MODO=1) */
    float    pidPsiKi;                 /* PID_PSI_KI */
    float    presionObjetivoRemoto;    /* PRESION_OBJETIVO_REMOTO */
    uint32_t reservado8;               /* antes tasaLlenadoPsiS (2026-10-01: se calcula al leerla
                                        * desde TIEMPO_LLENADO_S, ver CalibFlash_GetTasaLlenadoPsiS) */

    uint32_t reservado6;               /* antes timeoutSinComandoS (ID 11, ahora fijo 2026-10-01) */
    uint32_t reservado0;               /* antes ultimaHoraUtcConocida (eliminada 2026-10-01) */

    uint16_t servoPulsoMinUs;          /* SERVO_PULSO_MIN */
    uint16_t servoPulsoMaxUs;          /* SERVO_PULSO_MAX */
    uint16_t intervaloOperativoS;      /* INTERVALO_ENVIO_OPERATIVO_S */
    uint16_t intervaloStandbyS;        /* INTERVALO_ENVIO_STANDBY_S */
    uint16_t reservado7;               /* antes histeresisModoS (ID 21, ahora fijo 2026-10-01) */
    uint16_t tiempoLlenadoS;           /* TIEMPO_LLENADO_S: segundos para llenar de 0 PSI a
                                         * PRESION_OBJETIVO_LOCAL. 0 = sin rampa. Lo usan la rampa
                                         * de MODO=1/2 y CALIB=9/14 */

    uint8_t  calib;                    /* CALIB -- vive en RAM, Init lo fuerza a 0 */
    uint8_t  modo;                     /* MODO -- vive en RAM, Init lo fuerza a RALENTI */
    uint8_t  reservado3;               /* antes nodeId (NODE_ID, eliminado 2026-10-01) */
    uint8_t  relleno[1];               /* completa a multiplo de 8 bytes */
} CalibFlash_Datos_t;

/* Verificación en tiempo de compilación de que el tamaño es múltiplo de 8. */
typedef char CalibFlash_VerificarTamano[
    (sizeof(CalibFlash_Datos_t) % 8U == 0U) ? 1 : -1
];

/* ==================== ESTADO EN RAM ==================== */

static CalibFlash_Datos_t s_datos;

/* Parámetros NO persistentes (llegan seguido, solo RAM) */
static float s_setRpm = 0.0f;
static float s_setPresion = 0.0f;
static float s_presionRemoto = 0.0f;

/* HAL_GetTick() del último downlink de PRESION_REMOTO válido. Arranca en el
 * tick del boot para que el watchdog de TIMEOUT_SIN_COMANDO_S cuente desde
 * el arranque si nunca llega ninguna. */
static uint32_t s_presionRemotoUltimoTickMs = 0U;
/* Downlinks MODO aceptados (solo del operador, no los cambios automáticos
 * de main.c) -- ver CalibFlash_GetModoDownlinkCount(). */
static uint32_t s_modoDownlinkCount = 0U;
static bool s_rechazoModoPendiente = false;
static bool s_arranqueConDefaults = false;

/* Banderas de comandos */
static volatile bool s_reporteForzadoPendiente = false;
static volatile bool s_reporteConfigPendiente = false;
static volatile bool s_resetPendiente = false;
static uint32_t s_resetPendienteTickMs = 0U;

/* Objetivo pendiente del servo manual (CALIB=1): se marca al aplicar un
 * SERVO_PULSO_MIN/MAX y main.c lo consume. Por bandera (no comparando
 * valores) para que repetir el valor ya guardado igual mueva el servo. */
static volatile bool s_objetivoManualServoPendiente = false;
static volatile uint16_t s_objetivoManualServoUs = 0U;

/* true si la última escritura a flash falló por hardware (borrado o
 * programación) -- distingue STORAGE_ERROR de OUT_OF_RANGE en el ACK. */
static volatile bool s_ultimaEscrituraFallo = false;

/* ==================== TABLA DE PARÁMETROS ==================== */
/* Una fila por ID. Los parámetros "normales" se procesan todos con el
 * mismo código (candado -> decodificar -> validar rango -> guardar ->
 * ACK). Los que tienen lógica propia (comandos, MODO, CALIB, los de RAM y
 * los "auto") están en la tabla solo para su categoría y su valor de ACK,
 * y se manejan aparte en el switch del dispatcher. */

/* Cómo se guarda el campo en s_datos y cómo viaja en los 2 bytes. */
typedef enum {
    T_NINGUNO = 0,  /* sin campo propio (comandos, valores de RAM) -- ACK 0 */
    T_F_U16,        /* float, viaja como uint16 x escala */
    T_F_S16,        /* float, viaja como int16 con signo x escala (ganancias PID) */
    T_U16,          /* uint16 directo */
    T_U8            /* uint8 directo (CALIB, MODO) */
} TipoParam_t;

/* Condición extra (además de la categoría) para aceptar el cambio. */
typedef enum {
    C_NINGUNO = 0,
    C_MODO_CAL,          /* exige MODO=4 (CALIBRACION) */
    C_MODO_CAL_CALIB0,   /* exige MODO=4 y ningún CALIB activo (ganancias PID) */
    C_SESION_SERVO       /* exige CALIB=1/2; en sesión no escribe flash hasta salir */
} Candado_t;

typedef struct {
    uint8_t id;
    uint8_t categoria;   /* CalibFlash_Categoria_t */
    uint8_t tipo;        /* TipoParam_t */
    uint8_t candado;     /* Candado_t */
    uint8_t campo;       /* offsetof() dentro de CalibFlash_Datos_t */
    uint8_t reporte;     /* 1 = viaja en el reporte REPORTAR_PARAMETROS */
    float   escala;
} Param_t;

#define CAMPO(c)  ((uint8_t)offsetof(CalibFlash_Datos_t, c))
#define CAT_CAL   CALIB_CATEGORIA_CALIBRACION
#define CAT_CONF  CALIB_CATEGORIA_CONFIGURACION
#define CAT_PROC  CALIB_CATEGORIA_PROCESO
#define CAT_CMD   CALIB_CATEGORIA_COMANDO

static const Param_t k_params[] = {
    /* id                                     categoria tipo       candado            campo                        reporte escala */
    { CALIB_ID_SET_RATIO,                     CAT_CAL,  T_F_U16,   C_MODO_CAL,        CAMPO(pulsosPorRevolucion),  1,  100.0f },
    { CALIB_ID_ALPHA,                         CAT_CAL,  T_F_U16,   C_NINGUNO,         CAMPO(alphaFiltro),          1, 1000.0f },
    { CALIB_ID_RPM_MAX,                       CAT_CONF, T_F_U16,   C_MODO_CAL,        CAMPO(rpmMax),               1,   10.0f },
    { CALIB_ID_RPM_MIN,                       CAT_CONF, T_F_U16,   C_MODO_CAL,        CAMPO(rpmMin),               1,   10.0f },
    { CALIB_ID_RPM_MAX_CARGA,                 CAT_CONF, T_F_U16,   C_MODO_CAL,        CAMPO(rpmMaxCarga),          1,   10.0f },
    { CALIB_ID_PRESION_MAX,                   CAT_CONF, T_F_U16,   C_MODO_CAL,        CAMPO(presionMax),           1,   10.0f },
    { CALIB_ID_PID_RPM_KP,                    CAT_CAL,  T_F_S16,   C_MODO_CAL_CALIB0, CAMPO(pidRpmKp),             1, 1000.0f },
    { CALIB_ID_PID_RPM_KI,                    CAT_CAL,  T_F_S16,   C_MODO_CAL_CALIB0, CAMPO(pidRpmKi),             1, 1000.0f },
    { CALIB_ID_PID_ASP_KP,                    CAT_CAL,  T_F_S16,   C_MODO_CAL_CALIB0, CAMPO(pidAspKp),             1, 1000.0f },
    { CALIB_ID_PID_ASP_KI,                    CAT_CAL,  T_F_S16,   C_MODO_CAL_CALIB0, CAMPO(pidAspKi),             1, 1000.0f },
    { CALIB_ID_PID_PSI_KP,                    CAT_CAL,  T_F_S16,   C_MODO_CAL_CALIB0, CAMPO(pidPsiKp),             1, 1000.0f },
    { CALIB_ID_PID_PSI_KI,                    CAT_CAL,  T_F_S16,   C_MODO_CAL_CALIB0, CAMPO(pidPsiKi),             1, 1000.0f },
    { CALIB_ID_SERVO_PULSO_MIN,               CAT_CONF, T_U16,     C_SESION_SERVO,    CAMPO(servoPulsoMinUs),      1,    1.0f },
    { CALIB_ID_SERVO_PULSO_MAX,               CAT_CONF, T_U16,     C_SESION_SERVO,    CAMPO(servoPulsoMaxUs),      1,    1.0f },
    { CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S,   CAT_CONF, T_U16,     C_NINGUNO,         CAMPO(intervaloOperativoS),  1,    1.0f },
    { CALIB_ID_INTERVALO_ENVIO_STANDBY_S,     CAT_CONF, T_U16,     C_NINGUNO,         CAMPO(intervaloStandbyS),    1,    1.0f },
    { CALIB_ID_PRESION_OBJETIVO_LOCAL,        CAT_PROC, T_F_U16,   C_NINGUNO,         CAMPO(presionObjetivoLocal), 1,   10.0f },
    { CALIB_ID_PRESION_OBJETIVO_REMOTO,       CAT_PROC, T_F_U16,   C_NINGUNO,         CAMPO(presionObjetivoRemoto),1,   10.0f },
    { CALIB_ID_TIEMPO_LLENADO_S,              CAT_PROC, T_U16,     C_NINGUNO,         CAMPO(tiempoLlenadoS),       1,    1.0f },

    /* Con lógica propia (switch del dispatcher). El campo/escala es lo que
     * se devuelve en el ACK y en el reporte. */
    { CALIB_ID_CALIB,                         CAT_CAL,  T_U8,      C_NINGUNO,         CAMPO(calib),                1,    1.0f },
    { CALIB_ID_MODO,                          CAT_CAL,  T_U8,      C_NINGUNO,         CAMPO(modo),                 1,    1.0f },
    { CALIB_ID_SET_RATIO_AUTO,                CAT_CAL,  T_F_U16,   C_MODO_CAL,        CAMPO(pulsosPorRevolucion),  0,  100.0f },
    { CALIB_ID_SET_RPM,                       CAT_PROC, T_NINGUNO, C_NINGUNO,         0U,                          0,    1.0f },
    { CALIB_ID_SET_PRESION,                   CAT_PROC, T_NINGUNO, C_NINGUNO,         0U,                          0,    1.0f },
    { CALIB_ID_PRESION_REMOTO,                CAT_PROC, T_NINGUNO, C_NINGUNO,         0U,                          0,    1.0f },
    { CALIB_ID_REPORTAR_PARAMETROS,           CAT_CMD,  T_NINGUNO, C_NINGUNO,         0U,                          0,    1.0f },
    { CALIB_ID_FORZAR_REPORTE,                CAT_CMD,  T_NINGUNO, C_NINGUNO,         0U,                          0,    1.0f },
    { CALIB_ID_RESET_REMOTO,                  CAT_CMD,  T_NINGUNO, C_NINGUNO,         0U,                          0,    1.0f },
};

#define N_PARAMS  (sizeof(k_params) / sizeof(k_params[0]))

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static bool CalibFlash_EscribirEnFlash(void);
static void CargarDefaults(void);
static const Param_t *BuscarParam(uint8_t id);
static bool RangoValido(uint8_t id, float v);
static bool CandadoAbierto(uint8_t candado);
static bool Aplicar(const Param_t *p, float v);
static bool AplicarId(uint8_t id, float v);
static uint16_t ValorRaw(const Param_t *p);
static uint16_t ValorRawId(uint8_t id);
static float Decodificar(const Param_t *p, const uint8_t *datos);
static bool EnSesionServo(void);
static uint16_t LeerUint16BigEndian(const uint8_t *datos);
static uint16_t CodificarUint16(float valorReal, float escala);
static uint16_t CodificarInt16ComoRaw(float valorReal, float escala);

/* ==================== API DE INICIALIZACIÓN ==================== */

void CalibFlash_Init(void)
{
    const CalibFlash_Datos_t *datosEnFlash = (const CalibFlash_Datos_t *)CALIB_FLASH_ADDRESS;

    if (datosEnFlash->magic == CALIB_FLASH_MAGIC) {
        memcpy(&s_datos, datosEnFlash, sizeof(CalibFlash_Datos_t));
        /* Seguridad: nunca se arranca en una calibración (si se fue la luz
         * a mitad de una, el servo vuelve a modo seguro) ni en un MODO de
         * control (decisión 2026-09-22: tras un reset con la tubería sin
         * llenar, el PID de presión podría perseguir una lectura irreal y
         * reventarla). Siempre CALIB=0 y MODO=RALENTI; el operador confirma
         * a mano antes de volver a control automático. No se reescribe la
         * flash aquí. */
        s_datos.calib = 0U;
        s_datos.modo = (uint8_t)CALIB_MODO_RALENTI;
    } else {
        s_arranqueConDefaults = true;
        CargarDefaults(); /* no se escribe a flash hasta el primer Set */
    }

    s_setRpm = 0.0f;
    s_setPresion = 0.0f;
    s_presionRemoto = 0.0f;
    s_presionRemotoUltimoTickMs = HAL_GetTick();
    s_reporteForzadoPendiente = false;
    s_reporteConfigPendiente = true; /* al arrancar se reportan los parámetros guardados */
    s_resetPendiente = false;
}

static void CargarDefaults(void)
{
    memset(&s_datos, 0, sizeof(s_datos));
    s_datos.magic                    = CALIB_FLASH_MAGIC;
    s_datos.pulsosPorRevolucion      = DEFAULT_PULSOS_POR_REVOLUCION;
    s_datos.alphaFiltro              = DEFAULT_ALPHA_FILTRO;
    s_datos.rpmMax                   = DEFAULT_RPM_MAX;
    s_datos.rpmMaxCarga              = DEFAULT_RPM_MAX_CARGA;
    s_datos.presionMax               = DEFAULT_PRESION_MAX;
    s_datos.rpmMin                   = DEFAULT_RPM_MIN;
    s_datos.pidRpmKp                 = DEFAULT_PID_RPM_KP;
    s_datos.pidRpmKi                 = DEFAULT_PID_RPM_KI;
    s_datos.presionObjetivoLocal     = DEFAULT_PRESION_OBJETIVO_LOCAL;
    s_datos.presionObjetivoRemoto    = DEFAULT_PRESION_OBJETIVO_REMOTO;
    s_datos.pidAspKp                = DEFAULT_PID_ASP_KP;
    s_datos.pidAspKi                 = DEFAULT_PID_ASP_KI;
    s_datos.pidPsiKp                 = DEFAULT_PID_PSI_KP;
    s_datos.pidPsiKi                 = DEFAULT_PID_PSI_KI;
    s_datos.servoPulsoMinUs          = DEFAULT_SERVO_PULSO_MIN_US;
    s_datos.servoPulsoMaxUs          = DEFAULT_SERVO_PULSO_MAX_US;
    s_datos.intervaloOperativoS      = DEFAULT_INTERVALO_OPERATIVO_S;
    s_datos.intervaloStandbyS        = DEFAULT_INTERVALO_STANDBY_S;
    s_datos.tiempoLlenadoS          = DEFAULT_TIEMPO_LLENADO_S;
    s_datos.calib                    = 0U;
    s_datos.modo                     = (uint8_t)CALIB_MODO_RALENTI;
}

/* ==================== DISPATCHER CENTRALIZADO ==================== */

/* Categoría de un ID. Un ID que no está en la tabla cae en CONFIGURACION
 * (lo más conservador: con el motor operando se rechaza). */
static CalibFlash_Categoria_t CalibFlash_CategoriaDe(uint8_t id)
{
    const Param_t *p = BuscarParam(id);
    return (p != NULL) ? (CalibFlash_Categoria_t)p->categoria : CALIB_CATEGORIA_CONFIGURACION;
}

/* Resultado estándar de un Set: OK, o STORAGE_ERROR / OUT_OF_RANGE según
 * si falló la flash o el rango. */
static CalibFlash_ProtocoloStatus_t StatusDe(bool ok)
{
    if (ok) return CALIB_STATUS_OK;
    return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
}

CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, float frecuenciaHzActual, uint16_t *valorAplicadoRaw)
{
    *valorAplicadoRaw = 0U;
    s_ultimaEscrituraFallo = false;

    /* 1) CONFIGURACION con el motor operando: rechazo antes que nada
     *    (incluso antes de saber si el ID existe). */
    if (motorOperando && CalibFlash_CategoriaDe(id) == CALIB_CATEGORIA_CONFIGURACION) {
        *valorAplicadoRaw = ValorRawId(id);
        return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
    }

    /* 2) El protocolo manda siempre 2 bytes de valor. Menos = malformado
     *    (no hay código propio para "longitud inválida"). */
    if (longitudDatos < 2) {
        return CALIB_STATUS_UNKNOWN_PARAMETER_ID;
    }

    const Param_t *p = BuscarParam(id);
    if (p == NULL) {
        return CALIB_STATUS_UNKNOWN_PARAMETER_ID;
    }

    /* 3) Candado propio del parámetro (modo correcto). Rechazado = el ACK
     *    lleva el valor que sigue vigente. */
    if (!CandadoAbierto(p->candado)) {
        *valorAplicadoRaw = ValorRaw(p);
        return CALIB_STATUS_APPLY_ERROR;
    }

    switch (id) {

        /* El operador manda el RPM que marca un tacómetro externo y el
         * firmware despeja el ratio con su propia frecuencia (misma fórmula
         * que Tacometro_Update()). Sin motor girando no hay qué calcular. */
        case CALIB_ID_SET_RATIO_AUTO: {
            float rpmReferencia = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = (rpmReferencia > 0.0f && frecuenciaHzActual > 0.0f)
                      && AplicarId(CALIB_ID_SET_RATIO, (frecuenciaHzActual * 60.0f) / rpmReferencia);
            *valorAplicadoRaw = ValorRaw(p);
            return StatusDe(ok);
        }

        /* SET_RPM y SET_PRESION: solo RAM, solo en MODO=3 (en otro MODO no
         * tendrían efecto, se rechaza para que el ACK lo avise). Son
         * mutuamente excluyentes: gana el último que llega. */
        case CALIB_ID_SET_RPM: {
            if (CalibFlash_GetModo() != CALIB_MODO_MANUAL_BANCO) {
                *valorAplicadoRaw = CodificarUint16(s_setRpm, 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            s_setRpm = (float)LeerUint16BigEndian(datos) / 10.0f;
            s_setPresion = 0.0f;
            *valorAplicadoRaw = CodificarUint16(s_setRpm, 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_SET_PRESION: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            if (CalibFlash_GetModo() != CALIB_MODO_MANUAL_BANCO) {
                *valorAplicadoRaw = CodificarUint16(s_setPresion, 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            if (nuevoValor >= s_datos.presionMax) { /* SIEMPRE < PRESION_MAX */
                *valorAplicadoRaw = CodificarUint16(s_setPresion, 10.0f);
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            s_setPresion = nuevoValor;
            s_setRpm = 0.0f;
            *valorAplicadoRaw = CodificarUint16(s_setPresion, 10.0f);
            return CALIB_STATUS_OK;
        }

        /* Lectura del aspersor remoto (MODO=2). Marca la hora para el
         * watchdog de TIMEOUT_SIN_COMANDO_S -- solo un downlink real cuenta
         * como "fresco", no una llamada interna al setter. */
        case CALIB_ID_PRESION_REMOTO: {
            s_presionRemoto = (float)LeerUint16BigEndian(datos) / 10.0f;
            s_presionRemotoUltimoTickMs = HAL_GetTick();
            *valorAplicadoRaw = CodificarUint16(s_presionRemoto, 10.0f);
            return CALIB_STATUS_OK;
        }

        /* CALIB: reglas de la hoja CALIB del Excel.
         *  - 1/2 mueven el servo sin realimentación: exigen motor detenido.
         *  - Cualquier CALIB != 0 exige MODO=4, salvo 12 (autotune PID#3),
         *    que también arranca desde MODO=1.
         *  - 3..14 solo se piden viniendo de CALIB=0.
         *  - Entrar a cualquier CALIB != 0 limpia SET_RPM. CALIB=0 siempre
         *    se acepta. */
        case CALIB_ID_CALIB: {
            uint8_t nuevo = datos[1];
            uint8_t actual = s_datos.calib;

            if ((nuevo == 1U || nuevo == 2U) && motorOperando) {
                *valorAplicadoRaw = actual;
                return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
            }
            bool excepcionPid3DesdeModo1 = (nuevo == CALIB_AUTOTUNE_PID3)
                    && (CalibFlash_GetModo() == CALIB_MODO_PRESION_LOCAL);
            if (nuevo != 0U && CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                && !excepcionPid3DesdeModo1) {
                *valorAplicadoRaw = actual;
                return CALIB_STATUS_APPLY_ERROR;
            }
            if (nuevo >= 3U && nuevo <= CALIB_VALOR_MAXIMO && actual != 0U) {
                *valorAplicadoRaw = actual;
                return CALIB_STATUS_APPLY_ERROR;
            }
            bool ok = CalibFlash_SetCalib(nuevo);
            if (ok && nuevo != 0U) {
                s_setRpm = 0.0f;
            }
            *valorAplicadoRaw = s_datos.calib;
            return StatusDe(ok);
        }

        /* MODO: reglas de entrada de la hoja "Máquina de estados" (Tabla A,
         * modo_fsm.c). Todo rechazo se avisa con la alerta "modo rechazado". */
        case CALIB_ID_MODO: {
            uint8_t solicitado = datos[1];
            CalibFlash_Modo_t anterior = CalibFlash_GetModo();
            CalibFlash_ProtocoloStatus_t statusEntrada = Modo_ValidarEntrada(
                    anterior, solicitado, motorOperando, s_datos.presionObjetivoRemoto);
            if (statusEntrada != CALIB_STATUS_OK) {
                s_rechazoModoPendiente = true;
                *valorAplicadoRaw = (uint16_t)anterior;
                return statusEntrada;
            }
            bool ok = CalibFlash_SetModo((CalibFlash_Modo_t)solicitado);
            if (ok) s_modoDownlinkCount++;
            /* Solo en la TRANSICIÓN (un MODO repetido no corta la sesión). */
            if (ok && solicitado == (uint8_t)CALIB_MODO_MANUAL_BANCO
                && anterior != CALIB_MODO_MANUAL_BANCO) {
                /* Nota 1 de la hoja: SET_PRESION limpio; viniendo de MODO 1/2
                 * conserva el RPM actual (no baja a ralentí con la tubería
                 * con agua), desde otro MODO SET_RPM = 0. */
                s_setPresion = 0.0f;
                s_setRpm = Modo_SetRpmAlEntrarManual(anterior, Tacometro_GetRPMFiltrada(),
                        s_datos.rpmMin, s_datos.rpmMaxCarga);
            }
            if (ok && solicitado == (uint8_t)CALIB_MODO_CALIBRACION
                && anterior != CALIB_MODO_CALIBRACION) {
                /* Nota 2 de la hoja: MODO 4 parte limpio. */
                s_setRpm = 0.0f;
                s_setPresion = 0.0f;
                CalibFlash_SetCalib(0U);
            }
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetModo();
            return StatusDe(ok);
        }

        /* Comandos: exigen el byte de confirmación 0xA5. */
        case CALIB_ID_REPORTAR_PARAMETROS:
        case CALIB_ID_FORZAR_REPORTE:
        case CALIB_ID_RESET_REMOTO: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            if (id == CALIB_ID_REPORTAR_PARAMETROS) {
                s_reporteConfigPendiente = true;
            } else if (id == CALIB_ID_FORZAR_REPORTE) {
                s_reporteForzadoPendiente = true;
            } else {
                if (!CalibFlash_ResetEsSeguro(motorOperando)) {
                    return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
                }
                s_resetPendiente = true;
                s_resetPendienteTickMs = HAL_GetTick();
            }
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return CALIB_STATUS_OK;
        }

        /* Todos los demás: el camino genérico de la tabla. */
        default: {
            bool ok = Aplicar(p, Decodificar(p, datos));
            *valorAplicadoRaw = ValorRaw(p);
            /* Servo manual (CALIB=1): mover el servo al valor recién aplicado. */
            if (ok && p->candado == C_SESION_SERVO && s_datos.calib == 1U) {
                s_objetivoManualServoUs = *valorAplicadoRaw;
                s_objetivoManualServoPendiente = true;
            }
            return StatusDe(ok);
        }
    }
}

/* ==================== REGLAS DE LA TABLA ==================== */

static const Param_t *BuscarParam(uint8_t id)
{
    for (uint32_t i = 0; i < N_PARAMS; i++) {
        if (k_params[i].id == id) return &k_params[i];
    }
    return NULL;
}

static bool CandadoAbierto(uint8_t candado)
{
    switch (candado) {
        case C_MODO_CAL:
            return s_datos.modo == (uint8_t)CALIB_MODO_CALIBRACION;
        case C_MODO_CAL_CALIB0:
            return s_datos.modo == (uint8_t)CALIB_MODO_CALIBRACION && s_datos.calib == 0U;
        case C_SESION_SERVO:
            return EnSesionServo();
        default:
            return true;
    }
}

/* Rango válido de cada parámetro de la tabla. Varios dependen de otro
 * (ej. RPM_MAX > RPM_MIN, PRESION_OBJETIVO_LOCAL < PRESION_MAX). Las
 * ganancias PID aceptan cualquier valor con signo. */
static bool RangoValido(uint8_t id, float v)
{
    switch (id) {
        case CALIB_ID_SET_RATIO:              return v >= 0.1f && v <= 200.0f;
        case CALIB_ID_ALPHA:                  return v > 0.0f && v <= 1.0f;
        case CALIB_ID_RPM_MAX:                return v > s_datos.rpmMin && v <= 6000.0f;
        case CALIB_ID_RPM_MIN:                return v >= 0.0f && v < s_datos.rpmMax;
        case CALIB_ID_RPM_MAX_CARGA:          return v > s_datos.rpmMin && v <= s_datos.rpmMax;
        case CALIB_ID_PRESION_MAX:            return v > 0.0f && v <= 500.0f
                                                     && v > s_datos.presionObjetivoLocal
                                                     && v > s_setPresion;
        case CALIB_ID_SERVO_PULSO_MIN:        return v >= 500.0f && v < (float)s_datos.servoPulsoMaxUs;
        case CALIB_ID_SERVO_PULSO_MAX:        return v > (float)s_datos.servoPulsoMinUs && v <= 2500.0f;
        case CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S:
        case CALIB_ID_INTERVALO_ENVIO_STANDBY_S: return v >= 5.0f && v <= 3600.0f;
        case CALIB_ID_PRESION_OBJETIVO_LOCAL: return v > 0.0f && v <= 500.0f && v < s_datos.presionMax;
        case CALIB_ID_PRESION_OBJETIVO_REMOTO: return v >= 0.0f && v <= 500.0f; /* 0 = sin configurar */
        /* 0 = sin rampa; si no, al menos 1 min. El mínimo también rechaza un
         * valor viejo de TASA_LLENADO_PSI_S (PSI/s x100, que usaba este mismo
         * ID 8) mandado por una Lambda sin actualizar. */
        case CALIB_ID_TIEMPO_LLENADO_S:       return v == 0.0f || v >= 60.0f;
        default:                              return true;                      /* ganancias PID */
    }
}

/* Valida, guarda en RAM y persiste. En sesión de servo (CALIB=1/2)
 * SERVO_PULSO_MIN/MAX quedan solo en RAM: se guardan una vez al salir
 * (ver CalibFlash_SetCalib). */
static bool Aplicar(const Param_t *p, float v)
{
    if (!RangoValido(p->id, v)) return false;

    uint8_t *campo = (uint8_t *)&s_datos + p->campo;
    switch (p->tipo) {
        case T_U16: *(uint16_t *)campo = (uint16_t)v; break;
        default:    *(float *)campo = v;             break;
    }

    if (p->candado == C_SESION_SERVO && EnSesionServo()) return true;
    return CalibFlash_EscribirEnFlash();
}

static bool AplicarId(uint8_t id, float v)
{
    return Aplicar(BuscarParam(id), v);
}

static float Decodificar(const Param_t *p, const uint8_t *datos)
{
    uint16_t crudo = LeerUint16BigEndian(datos);
    if (p->tipo == T_F_S16) return (float)(int16_t)crudo / p->escala;
    if (p->tipo == T_F_U16) return (float)crudo / p->escala;
    return (float)crudo;
}

/* Valor vigente del parámetro, ya codificado en los 2 bytes del ACK. */
static uint16_t ValorRaw(const Param_t *p)
{
    const uint8_t *campo = (const uint8_t *)&s_datos + p->campo;
    switch (p->tipo) {
        case T_F_U16: return CodificarUint16(*(const float *)campo, p->escala);
        case T_F_S16: return CodificarInt16ComoRaw(*(const float *)campo, p->escala);
        case T_U16:   return *(const uint16_t *)campo;
        case T_U8:    return *campo;
        default:      return 0U;
    }
}

static uint16_t ValorRawId(uint8_t id)
{
    const Param_t *p = BuscarParam(id);
    return (p != NULL) ? ValorRaw(p) : 0U;
}

/* ==================== GETTERS ==================== */

float    CalibFlash_GetPulsosPorRevolucion(void)      { return s_datos.pulsosPorRevolucion; }
float    CalibFlash_GetAlphaFiltro(void)              { return s_datos.alphaFiltro; }
float    CalibFlash_GetRpmMax(void)                   { return s_datos.rpmMax; }
float    CalibFlash_GetRpmMaxCarga(void)              { return s_datos.rpmMaxCarga; }
float    CalibFlash_GetPresionMax(void)               { return s_datos.presionMax; }
float    CalibFlash_GetRpmMin(void)                   { return s_datos.rpmMin; }
float    CalibFlash_GetPidRpmKp(void)                 { return s_datos.pidRpmKp; }
float    CalibFlash_GetPidRpmKi(void)                 { return s_datos.pidRpmKi; }
uint16_t CalibFlash_GetServoPulsoMinUs(void)          { return s_datos.servoPulsoMinUs; }
uint16_t CalibFlash_GetServoPulsoMaxUs(void)          { return s_datos.servoPulsoMaxUs; }
uint32_t CalibFlash_GetTimeoutSinComandoS(void)       { return FIJO_TIMEOUT_SIN_COMANDO_S; }
uint8_t  CalibFlash_GetCalib(void)                    { return s_datos.calib; }
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void) { return s_datos.intervaloOperativoS; }
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void)   { return s_datos.intervaloStandbyS; }
CalibFlash_Modo_t CalibFlash_GetModo(void)            { return (CalibFlash_Modo_t)s_datos.modo; }
uint16_t CalibFlash_GetHisteresisModoS(void)          { return FIJO_HISTERESIS_MODO_S; }
uint16_t CalibFlash_GetTiempoLlenadoS(void)           { return s_datos.tiempoLlenadoS; }
float    CalibFlash_GetPresionObjetivoLocal(void)     { return s_datos.presionObjetivoLocal; }
float    CalibFlash_GetPresionObjetivoRemoto(void)    { return s_datos.presionObjetivoRemoto; }
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void) { return FIJO_TASA_MAX_CAMBIO_RPM_LLENADO_S; }
float    CalibFlash_GetPidAspKp(void)                 { return s_datos.pidAspKp; }
float    CalibFlash_GetPidAspKi(void)                 { return s_datos.pidAspKi; }
float    CalibFlash_GetPidPsiKp(void)                 { return s_datos.pidPsiKp; }
float    CalibFlash_GetPidPsiKi(void)                 { return s_datos.pidPsiKi; }
uint32_t CalibFlash_GetPresionRemotoUltimoTickMs(void) { return s_presionRemotoUltimoTickMs; }
uint32_t CalibFlash_GetModoDownlinkCount(void)         { return s_modoDownlinkCount; }
bool     CalibFlash_ArranqueConDefaults(void)          { return s_arranqueConDefaults; }

/* Velocidad de la rampa de llenado (PSI/s): llenar de 0 PSI a
 * PRESION_OBJETIVO_LOCAL en TIEMPO_LLENADO_S segundos. Se calcula al
 * leerla, así que si después cambia el objetivo la velocidad se ajusta
 * sola. 0 = sin rampa. */
float CalibFlash_GetTasaLlenadoPsiS(void)
{
    if (s_datos.tiempoLlenadoS == 0U) return 0.0f;
    return s_datos.presionObjetivoLocal / (float)s_datos.tiempoLlenadoS;
}

/* ==================== REPORTE DE PARÁMETROS ==================== */

bool CalibFlash_HayReportePendiente(void)    { return s_reporteConfigPendiente; }
void CalibFlash_SolicitarReporte(void)       { s_reporteConfigPendiente = true; }
void CalibFlash_TerminarReporte(void)        { s_reporteConfigPendiente = false; }

uint8_t CalibFlash_ArmarReporte(uint8_t *indice, uint8_t *buf, uint8_t maxGrupos)
{
    uint8_t n = 0U;
    while (*indice < N_PARAMS && n < maxGrupos) {
        const Param_t *p = &k_params[(*indice)++];
        if (!p->reporte) continue;
        uint16_t raw = ValorRaw(p);
        buf[n * 4U + 0U] = p->id;
        buf[n * 4U + 1U] = (uint8_t)CALIB_STATUS_REPORTE;
        buf[n * 4U + 2U] = (uint8_t)(raw >> 8);
        buf[n * 4U + 3U] = (uint8_t)(raw & 0xFFU);
        n++;
    }
    return (uint8_t)(n * 4U);
}

bool CalibFlash_ConsumirRechazoModo(void)
{
    bool hubo = s_rechazoModoPendiente;
    s_rechazoModoPendiente = false;
    return hubo;
}

/* ==================== SETTERS USADOS FUERA DEL DISPATCHER ==================== */
/* Mismas reglas de rango y persistencia que el downlink, sin el candado de
 * MODO (los llaman las auto-calibraciones de main.c, que ya corren dentro
 * de su propio CALIB). */

bool CalibFlash_SetAlphaFiltro(float v)          { return AplicarId(CALIB_ID_ALPHA, v); }
bool CalibFlash_SetRpmMin(float v)               { return AplicarId(CALIB_ID_RPM_MIN, v); }
bool CalibFlash_SetServoPulsoMinUs(uint16_t v)   { return AplicarId(CALIB_ID_SERVO_PULSO_MIN, (float)v); }

static bool EnSesionServo(void)
{
    return s_datos.calib == 1U || s_datos.calib == 2U;
}

bool CalibFlash_SetCalib(uint8_t modo)
{
    if (modo > CALIB_VALOR_MAXIMO) return false;
    bool salePorServo = EnSesionServo() && modo != 1U && modo != 2U;
    s_datos.calib = modo;
    if (salePorServo) {
        /* Sale de la sesión de servo (por downlink o por el apagado de
         * seguridad de main.c): guarda los SERVO_PULSO_MIN/MAX ajustados.
         * Si no cambió nada, la comparación de EscribirEnFlash no borra. */
        (void)CalibFlash_EscribirEnFlash();
    }
    /* Solo RAM: Init ignora el CALIB guardado, así que escribirlo solo
     * gastaría ciclos de borrado. Viaja a flash "de rebote" en el próximo
     * Set de otro parámetro. */
    return true;
}

bool CalibFlash_SetModo(CalibFlash_Modo_t v)
{
    if ((uint8_t)v > (uint8_t)CALIB_MODO_CALIBRACION) return false;
    s_datos.modo = (uint8_t)v; /* solo RAM, mismo motivo que CALIB */
    return true;
}

/* Kp y Ki juntos en UNA escritura -- para los autotunes, que siempre
 * guardan el par. */
bool CalibFlash_SetPidRpmGanancias(float kp, float ki)
{
    s_datos.pidRpmKp = kp;
    s_datos.pidRpmKi = ki;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidPsiGanancias(float kp, float ki)
{
    s_datos.pidPsiKp = kp;
    s_datos.pidPsiKi = ki;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidAspGanancias(float kp, float ki)
{
    s_datos.pidAspKp = kp;
    s_datos.pidAspKi = ki;
    return CalibFlash_EscribirEnFlash();
}

/* ==================== NO PERSISTENTES ==================== */

float CalibFlash_GetSetRpm(void)          { return s_setRpm; }
void  CalibFlash_SetSetRpm(float v)       { s_setRpm = v; }
float CalibFlash_GetSetPresion(void)      { return s_setPresion; }
void  CalibFlash_SetSetPresion(float v)   { s_setPresion = v; }
float CalibFlash_GetPresionRemoto(void)   { return s_presionRemoto; }
void  CalibFlash_SetPresionRemoto(float v) { s_presionRemoto = v; }

/* ==================== COMANDOS ==================== */

bool     CalibFlash_HayReporteForzado(void)       { return s_reporteForzadoPendiente; }
void     CalibFlash_ForzarReporte(void)           { s_reporteForzadoPendiente = true; }
void     CalibFlash_LimpiarReporteForzado(void)   { s_reporteForzadoPendiente = false; }
bool     CalibFlash_HayResetPendiente(void)       { return s_resetPendiente; }
uint32_t CalibFlash_GetResetPendienteTickMs(void) { return s_resetPendienteTickMs; }
void     CalibFlash_LimpiarResetPendiente(void)   { s_resetPendiente = false; }

bool CalibFlash_ResetEsSeguro(bool motorOperando)
{
    /* Durante el arranque el servo queda sin PWM ~2.5-8.5 s y después MODO
     * arranca en RALENTI: con el motor bombeando se cortaría el riego de
     * golpe, sin vigía de PRESION_MAX mientras tanto. Por eso solo motor
     * detenido, o en ralentí sin nada comandado. */
    if (!motorOperando) return true;
    return s_datos.modo == (uint8_t)CALIB_MODO_RALENTI && s_datos.calib == 0U && s_setRpm <= 0.0f;
}

bool     CalibFlash_HayObjetivoManualServo(void)    { return s_objetivoManualServoPendiente; }
uint16_t CalibFlash_GetObjetivoManualServoUs(void)  { return s_objetivoManualServoUs; }
void     CalibFlash_LimpiarObjetivoManualServo(void) { s_objetivoManualServoPendiente = false; }

/* ==================== FUNCIONES PRIVADAS ==================== */

static uint16_t LeerUint16BigEndian(const uint8_t *datos)
{
    return (uint16_t)(((uint16_t)datos[0] << 8) | (uint16_t)datos[1]);
}

/* Valor real -> uint16 con escala, redondeado y saturado (sin desbordar). */
static uint16_t CodificarUint16(float valorReal, float escala)
{
    float escalado = valorReal * escala;
    if (escalado < 0.0f) {
        escalado = 0.0f;
    }
    if (escalado > 65535.0f) {
        escalado = 65535.0f;
    }
    return (uint16_t)(escalado + 0.5f);
}

/* Valor real con signo -> patrón de 16 bits (int16 reinterpretado como
 * uint16 para el transporte), para las ganancias PID. Saturado a int16. */
static uint16_t CodificarInt16ComoRaw(float valorReal, float escala)
{
    float escalado = valorReal * escala;
    if (escalado < -32768.0f) {
        escalado = -32768.0f;
    }
    if (escalado > 32767.0f) {
        escalado = 32767.0f;
    }
    int16_t valorEntero = (int16_t)(escalado >= 0.0f ? (escalado + 0.5f) : (escalado - 0.5f));
    return (uint16_t)valorEntero;
}

static bool CalibFlash_EscribirEnFlash(void)
{
    HAL_StatusTypeDef estado;
    FLASH_EraseInitTypeDef borrado = {0};
    uint32_t paginaConError = 0;

    s_datos.magic = CALIB_FLASH_MAGIC;
    s_ultimaEscrituraFallo = false;

    /* Si la página ya tiene exactamente lo mismo, no se borra: protege a
     * todos los Set, incluidos downlinks repetidos con el mismo valor. */
    if (memcmp((const void *)CALIB_FLASH_ADDRESS, &s_datos, sizeof(CalibFlash_Datos_t)) == 0) {
        return true;
    }

    HAL_FLASH_Unlock();

    borrado.TypeErase = FLASH_TYPEERASE_PAGES;
    borrado.Banks = FLASH_BANK_1;
    borrado.Page = CALIB_FLASH_PAGE_NUMBER;
    borrado.NbPages = 1;

    estado = HAL_FLASHEx_Erase(&borrado, &paginaConError);
    if (estado != HAL_OK) {
        HAL_FLASH_Lock();
        s_ultimaEscrituraFallo = true;
        return false;
    }

    uint64_t *origen = (uint64_t *)&s_datos;
    uint32_t direccion = CALIB_FLASH_ADDRESS;
    uint32_t cantidadDoubleWords = sizeof(CalibFlash_Datos_t) / sizeof(uint64_t);

    for (uint32_t i = 0; i < cantidadDoubleWords; i++) {
        estado = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, direccion, origen[i]);
        if (estado != HAL_OK) {
            HAL_FLASH_Lock();
            s_ultimaEscrituraFallo = true;
            return false;
        }
        direccion += sizeof(uint64_t);
    }

    HAL_FLASH_Lock();
    return true;
}
