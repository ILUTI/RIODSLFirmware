/**
 * @file    calibracion_flash.h
 * @brief   Guarda en la última página de flash del G431 los parámetros de
 *          configuración remota del nodo motor, y centraliza el punto de
 *          entrada para aplicar cualquier parámetro recibido por downlink
 *          (LoRaWAN) o por consola serial.
 *
 * Tipos de parámetro:
 *   - PERSISTENTES: se guardan en flash, sobreviven resets (SET_RATIO,
 *     RPM_MAX, PID_RPM_KP, ...).
 *   - NO PERSISTENTES (solo RAM): SET_RPM, SET_PRESION y PRESION_REMOTO --
 *     llegan seguido y no vale la pena desgastar flash con ellos. MODO y
 *     CALIB también viven en RAM (el arranque los fuerza a 0).
 *   - COMANDOS: RESTAURAR_DEFAULTS, FORZAR_REPORTE, RESET_REMOTO -- acciones,
 *     no valores. Exigen el byte de confirmación CALIB_BYTE_CONFIRMACION.
 *
 * La tabla completa (ID, escala, categoría, rango) está en el README,
 * sección "Tabla completa de parámetros", y en k_params[] de
 * calibracion_flash.c.
 */

#ifndef CALIBRACION_FLASH_H
#define CALIBRACION_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==================== IDs DE PARÁMETROS ==================== */
/* Deben coincidir con PARAMETER_TABLE de la Lambda RIODSLSendDownlink y con
 * NOMBRES_PARAMETRO de decoder.py (AWS). Renumerados 2026-10-02 por
 * categoría, 1-29 sin huecos (la tabla vieja -> nueva está en el README,
 * "Tabla completa de parámetros"). Firmware y las dos Lambdas deben
 * actualizarse JUNTOS: un nodo con firmware viejo interpretaría los IDs
 * nuevos como otro parámetro. */

/* ---- Modos ---- */
#define CALIB_ID_MODO                              1U
#define CALIB_ID_CALIB                             2U

/* ---- Proceso (se aceptan con el motor operando) ---- */
#define CALIB_ID_SET_RPM                           3U
/* 4: equivalente de SET_RPM pero de presión, solo RAM, solo MODO=3, arma la
 * misma cascada de MODO=1 sin rampa de llenado. Mutuamente excluyente con
 * SET_RPM (gana el último). Siempre < PRESION_MAX. */
#define CALIB_ID_SET_PRESION                       4U
#define CALIB_ID_PRESION_OBJETIVO_LOCAL            5U
/* 6: presión que SE ESPERA que reporte el aspersor en MODO=2 -- es el
 * setpoint del PID remoto (no confundir con PRESION_REMOTO, ID 7, que es la
 * lectura real). 0 = sin configurar: MODO=2 se rechaza. */
#define CALIB_ID_PRESION_OBJETIVO_REMOTO           6U
#define CALIB_ID_PRESION_REMOTO                    7U
/* 8: segundos para llenar la tubería de 0 PSI a PRESION_OBJETIVO_LOCAL. La
 * velocidad de la rampa de MODO=1/2 (PSI/s) se calcula al usarla:
 * objetivo / segundos. 0 = sin rampa. También lo usan CALIB=9/14. */
#define CALIB_ID_TIEMPO_LLENADO_S                  8U

/* ---- Configuración (solo con el motor detenido) ---- */
#define CALIB_ID_RPM_MIN                           9U
#define CALIB_ID_RPM_MAX                          10U
/* 11: techo de RPM CON CARGA (hidráulico), propio de cada instalación,
 * siempre <= RPM_MAX (techo mecánico del motor). */
#define CALIB_ID_RPM_MAX_CARGA                    11U
/* 12: guarda dura de presión local. Si la presión medida la supera, el
 * setpoint cae a ralentí (histéresis 90 %) y sale la alerta 5. Exenta en
 * MODO=0. */
#define CALIB_ID_PRESION_MAX                      12U
#define CALIB_ID_SERVO_PULSO_MIN                  13U
#define CALIB_ID_SERVO_PULSO_MAX                  14U
#define CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S      15U  /* uplink LIVE con el motor encendido */
#define CALIB_ID_INTERVALO_ENVIO_STANDBY_S        16U  /* uplink LIVE con el motor apagado */
/* 17: horas del horometro (horas enteras, 0-65535), para FIJARLO al instalar
 * el TID en un motor que ya trae horas o corregirlo; despues sigue sumando
 * solo (ver horometro.h). Solo con el motor detenido. El ACK y el reporte
 * de parametros devuelven el valor ACTUAL en horas enteras (topado en
 * 65535); el valor completo, con decimas, va en el uplink LIVE. */
#define CALIB_ID_HOROMETRO_H                      17U

/* ---- Calibración ---- */
#define CALIB_ID_SET_RATIO                        18U
#define CALIB_ID_SET_RATIO_AUTO                   19U  /* comando con valor: RPM externa -> calcula SET_RATIO */
#define CALIB_ID_ALPHA                            20U
#define CALIB_ID_PID_RPM_KP                       21U  /* PID#1, lazo interno RPM */
#define CALIB_ID_PID_RPM_KI                       22U
#define CALIB_ID_PID_PSI_KP                       23U  /* PID#2, presión local MODO=1 */
#define CALIB_ID_PID_PSI_KI                       24U
#define CALIB_ID_PID_ASP_KP                       25U  /* PID#3, aspersor remoto MODO=2 */
#define CALIB_ID_PID_ASP_KI                       26U

/* ---- Comandos (byte de confirmación 0xA5) ---- */
/* 27: manda por FPort 3 el valor vigente de cada parámetro guardado
 * (grupos [ID][STATUS=REPORTE][VAL_H][VAL_L]). También se ejecuta solo al
 * arrancar. */
#define CALIB_ID_REPORTAR_PARAMETROS              27U
#define CALIB_ID_FORZAR_REPORTE                   28U  /* uplink LIVE inmediato */
#define CALIB_ID_RESET_REMOTO                     29U

/* Byte de confirmación de los comandos -- debe coincidir con la Lambda. */
#define CALIB_BYTE_CONFIRMACION   0xA5U

/* Categorías -- deciden si se permite cambiar con el motor operando:
 *   CALIBRACION    -- sí; cada uno trae su candado propio (MODO=4, etc.).
 *   CONFIGURACION  -- no: solo con el motor detenido (límites, servo,
 *                      tiempos). Un ID desconocido cae aquí.
 *   PROCESO        -- sí, siempre (setpoints y objetivos de operación).
 *   COMANDO        -- evaluados aparte (byte de confirmación). */
typedef enum {
    CALIB_CATEGORIA_CALIBRACION = 0,
    CALIB_CATEGORIA_CONFIGURACION = 1,
    CALIB_CATEGORIA_PROCESO = 2,
    CALIB_CATEGORIA_COMANDO = 3
} CalibFlash_Categoria_t;

/* Valores de MODO -- de dónde sale el setpoint de RPM (ver main.c y la
 * hoja "Máquina de estados" del Excel). */
typedef enum {
    CALIB_MODO_RALENTI       = 0,  /* sin control activo */
    CALIB_MODO_PRESION_LOCAL = 1,  /* cascada contra PRESION_OBJETIVO_LOCAL con el sensor propio del TID */
    CALIB_MODO_REMOTO        = 2,  /* cascada contra PRESION_OBJETIVO_REMOTO con PRESION_REMOTO del aspersor */
    CALIB_MODO_MANUAL_BANCO  = 3,  /* SET_RPM o SET_PRESION directo -- uso operativo puntual de campo */
    CALIB_MODO_CALIBRACION   = 4   /* sesión de banco/ingeniería: único MODO que acepta CALIB != 0
                                     * (salvo CALIB=12 desde MODO=1), y los límites/ganancias */
} CalibFlash_Modo_t;

/* STATUS del ACK de aplicación (FPort 3). */
typedef enum {
    CALIB_STATUS_OK                     = 0,
    CALIB_STATUS_OUT_OF_RANGE           = 1,
    CALIB_STATUS_UNKNOWN_PARAMETER_ID   = 2,  /* ID no existe o longitud < 2 */
    CALIB_STATUS_STORAGE_ERROR          = 3,  /* falló la flash (hardware) */
    CALIB_STATUS_APPLY_ERROR            = 4,  /* valor válido pero el MODO/CALIB actual no lo permite */
    CALIB_STATUS_REJECTED_ENGINE_RUNNING  = 5, /* CONFIGURACION (o CALIB 1/2, RESET_REMOTO) con el motor operando */
    CALIB_STATUS_REJECTED_ENGINE_STOPPED  = 6, /* MODO=1/2 con el motor detenido: el PID de presión no
                                                 * debe quedar "armado" antes de confirmar la tubería llena */
    CALIB_STATUS_REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO = 7, /* MODO=2 con PRESION_OBJETIVO_REMOTO = 0 */
    CALIB_STATUS_REPORTE                = 8   /* no es respuesta a un downlink: valor guardado,
                                                 * enviado por REPORTAR_PARAMETROS */
} CalibFlash_ProtocoloStatus_t;

/* ==================== VALORES DE CALIB ==================== */
/* Numeración vigente (hoja CALIB del Excel, README sección 4.4):
 *   0  = operación normal (único valor que deja editar ganancias PID)
 *   1  = servo manual (SERVO_PULSO_MIN/MAX lo mueven), motor detenido
 *   2  = barrido MIN<->MAX del servo, motor detenido
 *   3  = auto-ALPHA            4 = auto SERVO_PULSO_MIN (zona muerta)
 *   5/6/7   = etapas sueltas del autotune PID#1 (curva / ID+SIMC / validar)
 *   8  = auto-ralentí (RPM_MIN)
 *   9/10/11 = etapas sueltas del autotune PID#2 (curva / ID+SIMC / validar)
 *   12 = autotune PID#3 (MODO=2), único que también arranca desde MODO=1
 *   13 = autotune PID#1 completo   14 = autotune PID#2 completo
 * 3..14 solo se piden viniendo de CALIB=0 y vuelven solos a 0 al terminar. */
#define CALIB_PID1_CURVA       5U
#define CALIB_PID1_ID          6U
#define CALIB_PID1_VALIDAR     7U
#define CALIB_PID2_CURVA       9U
#define CALIB_PID2_ID         10U
#define CALIB_PID2_VALIDAR    11U
#define CALIB_AUTOTUNE_PID3   12U
#define CALIB_AUTOTUNE_PID1   13U
#define CALIB_AUTOTUNE_PID2   14U
#define CALIB_VALOR_MAXIMO    CALIB_AUTOTUNE_PID2

/* ==================== API ==================== */

/** Carga la calibración guardada, o los defaults si la flash no tiene el
 * magic válido (primera vez, magic cambiado o página corrupta). Siempre
 * deja CALIB=0 y MODO=RALENTI. */
void CalibFlash_Init(void);

/**
 * Procesa un parámetro ya separado en ID + bytes: valida, aplica y
 * persiste si corresponde. Único punto de entrada de rak3172.c y
 * comando_serial.c. Devuelve el STATUS del ACK y en 'valorAplicadoRaw' el
 * valor que quedó REALMENTE vigente (el anterior si se rechazó), ya
 * codificado en los 2 bytes del protocolo.
 *
 * @param motorOperando      !Tacometro_EstaDetenido() del llamador.
 * @param frecuenciaHzActual Tacometro_GetFrecuenciaHz() -- solo SET_RATIO_AUTO.
 */
CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id,
                                                                     const uint8_t *datos,
                                                                     uint8_t longitudDatos,
                                                                     bool motorOperando,
                                                                     float frecuenciaHzActual,
                                                                     uint16_t *valorAplicadoRaw);

/* ==================== GETTERS ==================== */

float    CalibFlash_GetPulsosPorRevolucion(void);
float    CalibFlash_GetAlphaFiltro(void);
float    CalibFlash_GetRpmMax(void);
float    CalibFlash_GetRpmMaxCarga(void);
float    CalibFlash_GetPresionMax(void);
uint16_t CalibFlash_GetTiempoLlenadoS(void);   /* TIEMPO_LLENADO_S, 0 = sin rampa */
float    CalibFlash_GetRpmMin(void);
float    CalibFlash_GetPidRpmKp(void);
float    CalibFlash_GetPidRpmKi(void);
uint16_t CalibFlash_GetServoPulsoMinUs(void);
uint16_t CalibFlash_GetServoPulsoMaxUs(void);
uint32_t CalibFlash_GetTimeoutSinComandoS(void);     /* fijo, 360 s */
uint8_t  CalibFlash_GetCalib(void);
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void);
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void);
CalibFlash_Modo_t CalibFlash_GetModo(void);
uint16_t CalibFlash_GetHisteresisModoS(void);        /* fijo, 30 s */
float    CalibFlash_GetPresionObjetivoLocal(void);
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void); /* fijo, 10 RPM/s */
float    CalibFlash_GetPresionObjetivoRemoto(void); /* setpoint de MODO=2; 0 = sin configurar */
float    CalibFlash_GetTasaLlenadoPsiS(void);       /* objetivo local / TIEMPO_LLENADO_S; 0 = sin rampa */
float    CalibFlash_GetPidAspKp(void);              /* lazo remoto, MODO=2 */
float    CalibFlash_GetPidAspKi(void);
float    CalibFlash_GetPidPsiKp(void);              /* lazo local, MODO=1 */
float    CalibFlash_GetPidPsiKi(void);

/** HAL_GetTick() del último PRESION_REMOTO válido (o del arranque) -- para el
 * watchdog de TIMEOUT_SIN_COMANDO_S en MODO=2. */
uint32_t CalibFlash_GetPresionRemotoUltimoTickMs(void);

/** Downlinks MODO aceptados desde el arranque (solo del operador) -- detecta
 * que el operador intervino tras una degradación automática de MODO. */
uint32_t CalibFlash_GetModoDownlinkCount(void);

/** true UNA vez por cada MODO rechazado (para la alerta "modo rechazado"). */
bool CalibFlash_ConsumirRechazoModo(void);

/** true si el último arranque cargó defaults (flash sin magic válido). */
bool CalibFlash_ArranqueConDefaults(void);

/* ==================== SETTERS ==================== */
/* Validan rango y persisten igual que un downlink, sin el candado de MODO.
 * Los usan las auto-calibraciones y autotunes de main.c. */

bool CalibFlash_SetAlphaFiltro(float nuevoValor);           /* CALIB=3 */
bool CalibFlash_SetServoPulsoMinUs(uint16_t nuevoValor);    /* CALIB=4 */
bool CalibFlash_SetRpmMin(float nuevoValor);                /* CALIB=8 */
bool CalibFlash_SetCalib(uint8_t modo);                     /* 0..CALIB_VALOR_MAXIMO, solo RAM */
bool CalibFlash_SetModo(CalibFlash_Modo_t nuevoModo);       /* solo RAM */
/** Kp+Ki en una sola escritura de flash (autotunes). */
bool CalibFlash_SetPidRpmGanancias(float kp, float ki);
bool CalibFlash_SetPidPsiGanancias(float kp, float ki);
bool CalibFlash_SetPidAspGanancias(float kp, float ki);

/* ==================== NO PERSISTENTES (solo RAM) ==================== */

/* SET_PRESION (ID 4) y PRESION_REMOTO (ID 7) los escribe SOLO el
 * dispatcher, al llegar su downlink (o el comando serial): por eso no tienen
 * setter publico. SET_RPM si, porque main.c y los autotunes lo mueven. */
float CalibFlash_GetSetRpm(void);
void  CalibFlash_SetSetRpm(float nuevoValor);     /* sin rango propio, el control lo recorta */
float CalibFlash_GetSetPresion(void);
float CalibFlash_GetPresionRemoto(void);
/** SET_RPM = 0 y SET_PRESION = 0. main.c lo llama cada vez que el motor se
 *  detiene (en cualquier MODO): al rearrancar no se vuelve solo a un valor
 *  viejo -- en MODO 4, que no pasa a MODO 0 al detenerse, el motor
 *  aceleraba solo al SET_RPM anterior (hallazgo B8, 2026-10-02). */
void  CalibFlash_LimpiarComandosManuales(void);

/* ==================== COMANDOS ==================== */

/** FORZAR_REPORTE pendiente: main.c manda el uplink y lo limpia.
 * CalibFlash_ForzarReporte() arma la misma bandera desde el propio firmware
 * (ej. al terminar una auto-calibración). */
bool CalibFlash_HayReporteForzado(void);
void CalibFlash_LimpiarReporteForzado(void);
void CalibFlash_ForzarReporte(void);

/** REPORTAR_PARAMETROS pendiente (también queda pedido al arrancar).
 * main.c llama CalibFlash_ArmarReporte() hasta que devuelve 0, mandando
 * cada parte por FPort 3, y después CalibFlash_TerminarReporte().
 * ArmarReporte llena 'buf' con hasta 'maxGrupos' grupos de 4 bytes
 * [ID][CALIB_STATUS_REPORTE][VAL_H][VAL_L], avanza '*indice' (arrancar en
 * 0) y devuelve la cantidad de bytes (0 = ya no queda nada). */
bool    CalibFlash_HayReportePendiente(void);
void    CalibFlash_TerminarReporte(void);
uint8_t CalibFlash_ArmarReporte(uint8_t *indice, uint8_t *buf, uint8_t maxGrupos);

/** RESET_REMOTO aceptado: main.c espera que salga el ACK, vuelve a
 * verificar CalibFlash_ResetEsSeguro() y ejecuta NVIC_SystemReset(). */
bool CalibFlash_HayResetPendiente(void);
uint32_t CalibFlash_GetResetPendienteTickMs(void);
void CalibFlash_LimpiarResetPendiente(void);

/** RESET_REMOTO solo con motor detenido, o en ralentí sin nada comandado
 * (MODO=RALENTI, CALIB=0, SET_RPM=0). */
bool CalibFlash_ResetEsSeguro(bool motorOperando);

/** Servo manual (CALIB=1): objetivo pendiente marcado al aplicar
 * SERVO_PULSO_MIN/MAX. main.c mueve el servo ahí y lo limpia. */
bool     CalibFlash_HayObjetivoManualServo(void);
uint16_t CalibFlash_GetObjetivoManualServoUs(void);
void     CalibFlash_LimpiarObjetivoManualServo(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRACION_FLASH_H */
