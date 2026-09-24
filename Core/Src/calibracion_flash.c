/**
 * @file    calibracion_flash.c
 * @brief   Implementación del almacenamiento de calibración en flash
 *          y del dispatcher centralizado de parámetros por downlink.
 *
 * Usa la ÚLTIMA página de flash del STM32G431KB (128 Kbytes, páginas
 * de 2 Kbytes -- 64 páginas, página 63 es la última, dirección
 * 0x0801F800). El STM32G4 requiere programar en "double word" (64
 * bits) alineados -- de ahí que la estructura se rellene a múltiplo
 * de 8 bytes.
 *
 * Codificación asumida para cada parámetro recibido por downlink
 * (payload después del byte de ID, big-endian):
 *   - SET_RATIO, ALPHA, PID_KP/KI/KD, TASA_MAX_CAMBIO_*,
 *     PRESION_OBJETIVO: 4 bytes, int32 con signo, escala según tabla.
 *   - SET_RPM, RPM_MAX, RPM_MIN, PRESION: 2 bytes, uint16, x10.
 *   - SERVO_PULSO_MIN/MAX: 2 bytes, uint16, en µs directo (sin escala).
 *   - TIMEOUT_SIN_COMANDO_S: 4 bytes, uint32, en segundos directo.
 *   - INTERVALO_*, HISTERESIS_MODO_S: 2 bytes, uint16, en segundos.
 *   - CONTROL_HABILITADO, MODO, NODE_ID, y los 3 comandos: 1 byte.
 */

#include "calibracion_flash.h"
#include "main.h"
#include <string.h>

/* ==================== CONFIGURACIÓN DE UBICACIÓN EN FLASH ==================== */

#define CALIB_FLASH_PAGE_NUMBER   63U
#define CALIB_FLASH_ADDRESS       0x0801F800UL
#define CALIB_FLASH_MAGIC         0x43414C4CUL  /* "CALL" -- subido desde
                                                   * "CALK" porque se quito
                                                   * pidKd de
                                                   * CalibFlash_Datos_t
                                                   * (2026-09-23, PID_KD
                                                   * eliminado -- Kd=0
                                                   * confirmado en campo,
                                                   * sin uso, ver README
                                                   * seccion 9). */

/* ==================== VALORES POR DEFECTO ==================== */
/* Los mismos que ya tenías validados en campo para SET_RATIO/ALPHA;
 * el resto son placeholders razonables -- AJUSTAR antes de operar
 * con el motor/servo real. */

#define DEFAULT_PULSOS_POR_REVOLUCION     17.5f
#define DEFAULT_ALPHA_FILTRO              0.35f
#define DEFAULT_RPM_MAX                   2500.0f
#define DEFAULT_RPM_MIN                   700.0f
#define DEFAULT_PID_KP                    1.0f
#define DEFAULT_PID_KI                    0.0f
#define DEFAULT_SERVO_PULSO_MIN_US        1000U
#define DEFAULT_SERVO_PULSO_MAX_US        2000U
#define DEFAULT_TIMEOUT_SIN_COMANDO_S     360U    /* 4x el reporte real del aspersor durante MODO=2
                                                     * activo (90s), confirmado en campo 2026-09-09 --
                                                     * el "15-20 min" documentado antes era del reporte
                                                     * en standby del aspersor, NO del estado activo
                                                     * que corresponde a MODO=2. Solo importa en
                                                     * MODO_REMOTO (ver watchdog en main.c) -- MODO=0/1
                                                     * no usan este parametro para nada. */
#define DEFAULT_TASA_MAX_CAMBIO_RPM_S     50.0f
#define DEFAULT_CONTROL_HABILITADO        0U
#define DEFAULT_INTERVALO_OPERATIVO_S     30U
#define DEFAULT_INTERVALO_STANDBY_S       300U
#define DEFAULT_MODO                      CALIB_MODO_RALENTI
#define DEFAULT_NODE_ID                   0U
#define DEFAULT_HISTERESIS_MODO_S         30U
#define DEFAULT_PRESION_OBJETIVO          40.0f
#define DEFAULT_PRESION_OBJETIVO_REMOTO    0.0f  /* sin configurar -- ver CALIB_ID_PRESION_OBJETIVO_REMOTO */
#define DEFAULT_TASA_LLENADO_PSI_S         0.0f  /* sin rampa -- ver CALIB_ID_TASA_LLENADO_PSI_S */
#define DEFAULT_TASA_MAX_CAMBIO_LLENADO_S 10.0f  /* mas lenta que la normal, a proposito */
#define DEFAULT_PRESION_REMOTO_PID_KP      0.0f  /* sin calibrar -- lazo de presion remota (MODO=2), ver README */
#define DEFAULT_PRESION_REMOTO_PID_KI      0.0f  /* idem */
#define DEFAULT_PRESION_PID_KP             0.0f  /* sin calibrar -- lazo de presion local (MODO=1), ver README */
#define DEFAULT_PRESION_PID_KI             0.0f  /* idem */

/* ==================== RANGOS VÁLIDOS ==================== */

#define RANGO_PULSOS_MIN         0.1f
#define RANGO_PULSOS_MAX         200.0f
#define RANGO_ALPHA_MAX          1.0f
#define RANGO_RPM_MAX_TECHO      6000.0f
#define RANGO_TASA_CAMBIO_MAX    2000.0f
#define RANGO_PRESION_MAX        500.0f
#define RANGO_TASA_LLENADO_PSI_S_MAX    10.0f    /* generoso -- en la practica se usan valores bien chicos (fracciones de PSI/s) */
/* PRESION_REMOTO_PID_KP/KI no tienen rango propio -- mismo criterio que
 * PRESION_PID_KP/KI (lazo local): son ganancias de PID, se aceptan con
 * cualquier signo, la sintonizacion en campo decide que es razonable. */

/* ==================== ESTRUCTURA GUARDADA EN FLASH ==================== */
/* Debe quedar en múltiplo de 8 bytes (double word) para el
 * STM32G4. Se agrega 'relleno' al final si hace falta. */

typedef struct {
    uint32_t magic;

    float    pulsosPorRevolucion;      /* SET_RATIO */
    float    alphaFiltro;              /* ALPHA */
    float    rpmMax;                   /* RPM_MAX */
    float    rpmMin;                   /* RPM_MIN */
    float    pidKp;                    /* PID_KP */
    float    pidKi;                    /* PID_KI */
    float    tasaMaxCambioRpmS;        /* TASA_MAX_CAMBIO_RPM_S */
    float    presionObjetivo;          /* PRESION_OBJETIVO */
    float    tasaMaxCambioRpmLlenadoS; /* TASA_MAX_CAMBIO_RPM_LLENADO_S */
    float    ultimaLatitudConocida;    /* GPS, 0.0f = nunca hubo fix */
    float    ultimaLongitudConocida;   /* GPS, 0.0f = nunca hubo fix */
    float    presionRemotoPidKp;       /* PRESION_REMOTO_PID_KP -- lazo externo de presion remota (MODO=2) */
    float    presionRemotoPidKi;       /* PRESION_REMOTO_PID_KI -- idem */
    float    presionPidKp;             /* PRESION_PID_KP -- lazo externo de presion local (MODO=1) */
    float    presionPidKi;             /* PRESION_PID_KI -- idem */
    float    presionObjetivoRemoto;    /* PRESION_OBJETIVO_REMOTO -- solo para el supervisor de MODO=2, ver header */
    float    tasaLlenadoPsiS;          /* TASA_LLENADO_PSI_S -- rampa de llenado de MODO=1, ver header */

    uint32_t timeoutSinComandoS;       /* TIMEOUT_SIN_COMANDO_S */
    uint32_t ultimaHoraUtcConocida;    /* epoch UTC, 0 = nunca sincronizado */

    uint16_t servoPulsoMinUs;          /* SERVO_PULSO_MIN */
    uint16_t servoPulsoMaxUs;          /* SERVO_PULSO_MAX */
    uint16_t intervaloOperativoS;      /* INTERVALO_ENVIO_OPERATIVO_S */
    uint16_t intervaloStandbyS;        /* INTERVALO_ENVIO_STANDBY_S */
    uint16_t histeresisModoS;          /* HISTERESIS_MODO_S */

    uint8_t  controlHabilitado;        /* CONTROL_HABILITADO (0/1/2) -- se
                                         * persiste en flash como cualquier otro
                                         * parametro CONFIGURACION, pero
                                         * CalibFlash_Init() lo fuerza a 0 en RAM
                                         * en cada arranque (ver mas abajo), nunca
                                         * arranca directo en modo calibracion */
    uint8_t  modo;                     /* MODO (0/1/2) */
    uint8_t  nodeId;                   /* NODE_ID */
    uint8_t  relleno[11];                /* completa a múltiplo de 8 bytes -- ajustar si
                                         * CalibFlash_VerificarTamano rompe la compilación.
                                         * Subido de 7 a 11 el 2026-09-23 al quitar pidKd
                                         * (un float, -4 bytes) de la struct. */
} CalibFlash_Datos_t;

/* Verificación en tiempo de compilación de que el tamaño es múltiplo
 * de 8 -- si esto rompe la compilación, ajustar 'relleno'. */
typedef char CalibFlash_VerificarTamano[
    (sizeof(CalibFlash_Datos_t) % 8U == 0U) ? 1 : -1
];

/* ==================== ESTADO EN RAM ==================== */

static CalibFlash_Datos_t s_datos;

/* Parámetros NO persistentes (se reciben seguido, solo RAM) */
static float s_setRpm = 0.0f;
static float s_presion = 0.0f;

/* HAL_GetTick() del último downlink de PRESION valido -- ver
 * CalibFlash_GetPresionUltimoTickMs(). Arranca en el tick del boot (no
 * en 0) para que el watchdog de TIMEOUT_SIN_COMANDO_S en MODO_REMOTO
 * cuente desde el arranque si nunca llega ninguna PRESION, en vez de
 * pensar que ya expiro desde "tiempo epoch 0". */
static uint32_t s_presionUltimoTickMs = 0U;

/* Banderas de comandos */
static volatile bool s_reporteForzadoPendiente = false;
static volatile bool s_resetPendiente = false;

/* Objetivo pendiente para el modo manual de calibracion del servo
 * (CONTROL_HABILITADO=2, ver main.c) -- se marca aqui cuando un downlink
 * de SERVO_PULSO_MIN/MAX se aplica con exito estando en ese modo, y
 * main.c lo consume y limpia en la siguiente vuelta del loop. Por
 * bandera (no por comparar valores) para que un downlink que repite el
 * valor ya guardado (ej. los defaults de fabrica) igual mueva el servo. */
static volatile bool s_objetivoManualServoPendiente = false;
static volatile uint16_t s_objetivoManualServoUs = 0U;

/* Se pone en true si la última llamada a CalibFlash_EscribirEnFlash()
 * falló por un error real de hardware (borrado/programación), en vez
 * de un rechazo de validación de rango. El dispatcher la consulta
 * justo después de un Set fallido para distinguir STORAGE_ERROR de
 * OUT_OF_RANGE en el ACK. */
static volatile bool s_ultimaEscrituraFallo = false;

/* ==================== PROTOTIPOS PRIVADOS ==================== */

static bool CalibFlash_EscribirEnFlash(void);
static uint16_t CalibFlash_ValorActualRaw(uint8_t id);
static uint16_t LeerUint16BigEndian(const uint8_t *datos);
static int16_t LeerInt16BigEndian(const uint8_t *datos);
static uint16_t CodificarUint16(float valorReal, float escala);
static uint16_t CodificarInt16ComoRaw(float valorReal, float escala);

/* ==================== API DE INICIALIZACIÓN ==================== */

void CalibFlash_Init(void)
{
    const CalibFlash_Datos_t *datosEnFlash = (const CalibFlash_Datos_t *)CALIB_FLASH_ADDRESS;

    if (datosEnFlash->magic == CALIB_FLASH_MAGIC) {
        memcpy(&s_datos, datosEnFlash, sizeof(CalibFlash_Datos_t));
        /* ultimaHoraUtcConocida/ultimaLatitudConocida/ultimaLongitudConocida
         * vienen tal cual estaban en flash -- NO se tocan aca, son
         * justamente los valores que queremos preservar entre arranques. */
        s_datos.controlHabilitado = 0U; /* CONTROL_HABILITADO NUNCA arranca en
                                           * modo calibracion (1 o 2), sin importar
                                           * lo que haya quedado guardado en flash
                                           * -- medida de seguridad: si se fue la
                                           * energia mientras se calibraba, al
                                           * volver el servo debe arrancar en modo
                                           * seguro (0), no reanudar el barrido ni
                                           * el modo manual solo. No se reescribe a
                                           * flash aqui -- queda en 0 en RAM hasta
                                           * el proximo Set explicito, igual que el
                                           * resto de esta funcion. */

        s_datos.modo = (uint8_t)CALIB_MODO_RALENTI; /* MODO tampoco arranca
                                           * nunca en el valor que haya quedado
                                           * guardado -- decision explicita del
                                           * usuario 2026-09-22, tras confirmar que
                                           * el equipo puede resetearse solo (ver
                                           * hallazgo del RAK3172/riel de 3.3V).
                                           * Si el reset ocurre con la tuberia
                                           * todavia sin llenar (aire adentro) y
                                           * el equipo volviera directo a MODO=1/2
                                           * persistido, el PID de presion podria
                                           * pedir RPM alta persiguiendo una
                                           * lectura de presion irreal y reventar
                                           * la tuberia -- arrancar siempre en
                                           * RALENTI exige que un operador
                                           * confirme a mano que todo esta en
                                           * condiciones antes de pasar a control
                                           * automatico. Mismo criterio que
                                           * CONTROL_HABILITADO arriba: no se
                                           * reescribe a flash aqui, el valor
                                           * guardado se preserva por si se
                                           * necesita inspeccionar despues, pero
                                           * en RAM (lo que de verdad usa el
                                           * control) siempre parte de RALENTI. */
    } else {
        memset(&s_datos, 0, sizeof(s_datos));
        s_datos.magic = CALIB_FLASH_MAGIC;
        s_datos.pulsosPorRevolucion      = DEFAULT_PULSOS_POR_REVOLUCION;
        s_datos.alphaFiltro              = DEFAULT_ALPHA_FILTRO;
        s_datos.rpmMax                   = DEFAULT_RPM_MAX;
        s_datos.rpmMin                   = DEFAULT_RPM_MIN;
        s_datos.pidKp                    = DEFAULT_PID_KP;
        s_datos.pidKi                    = DEFAULT_PID_KI;
        s_datos.tasaMaxCambioRpmS        = DEFAULT_TASA_MAX_CAMBIO_RPM_S;
        s_datos.presionObjetivo          = DEFAULT_PRESION_OBJETIVO;
        s_datos.presionObjetivoRemoto    = DEFAULT_PRESION_OBJETIVO_REMOTO;
        s_datos.tasaLlenadoPsiS          = DEFAULT_TASA_LLENADO_PSI_S;
        s_datos.tasaMaxCambioRpmLlenadoS = DEFAULT_TASA_MAX_CAMBIO_LLENADO_S;
        s_datos.timeoutSinComandoS       = DEFAULT_TIMEOUT_SIN_COMANDO_S;
        s_datos.ultimaHoraUtcConocida    = 0U;  /* 0 = nunca sincronizado con la red --
                                                   * SOLO se resetea aca (primera vez /
                                                   * flash invalida), no en cada arranque. */
        s_datos.ultimaLatitudConocida    = 0.0f; /* 0.0f = nunca hubo fix GPS -- mismo
                                                     * criterio que ultimaHoraUtcConocida. */
        s_datos.ultimaLongitudConocida   = 0.0f;
        s_datos.presionRemotoPidKp       = DEFAULT_PRESION_REMOTO_PID_KP;
        s_datos.presionRemotoPidKi       = DEFAULT_PRESION_REMOTO_PID_KI;
        s_datos.presionPidKp             = DEFAULT_PRESION_PID_KP;
        s_datos.presionPidKi             = DEFAULT_PRESION_PID_KI;
        s_datos.servoPulsoMinUs          = DEFAULT_SERVO_PULSO_MIN_US;
        s_datos.servoPulsoMaxUs          = DEFAULT_SERVO_PULSO_MAX_US;
        s_datos.intervaloOperativoS      = DEFAULT_INTERVALO_OPERATIVO_S;
        s_datos.intervaloStandbyS        = DEFAULT_INTERVALO_STANDBY_S;
        s_datos.histeresisModoS          = DEFAULT_HISTERESIS_MODO_S;
        s_datos.controlHabilitado        = DEFAULT_CONTROL_HABILITADO;
        s_datos.modo                     = (uint8_t)DEFAULT_MODO;
        s_datos.nodeId                   = DEFAULT_NODE_ID;
        /* No se escribe a flash aquí -- solo al primer Set explícito. */
    }

    s_setRpm = 0.0f;
    s_presion = 0.0f;
    s_presionUltimoTickMs = HAL_GetTick();
    s_reporteForzadoPendiente = false;
    s_resetPendiente = false;
}

/* ==================== DISPATCHER CENTRALIZADO ==================== */

/* Categoría de cada parámetro -- ver CalibFlash_Categoria_t en el header
 * para el criterio completo. IDs no listados aquí se tratan como
 * CONFIGURACION por defecto (más conservador: si se agrega un parámetro
 * nuevo y se olvida clasificarlo, queda bloqueado durante operación en
 * vez de quedar accidentalmente siempre permitido). */
static CalibFlash_Categoria_t CalibFlash_CategoriaDe(uint8_t id)
{
    switch (id) {
        case CALIB_ID_SET_RATIO:
        case CALIB_ID_SET_RATIO_AUTO:
        case CALIB_ID_ALPHA:
        /* PID_KP/KI como CALIBRACION (no CONFIGURACION por default) --
         * misma razon que SET_RATIO/ALPHA: la sintonizacion en lazo
         * cerrado (README seccion 9, metodo Ziegler-Nichols) exige subir
         * Kp de a poco CON EL MOTOR OPERANDO, observando la respuesta
         * real de RPM en cada paso -- si cayeran en CONFIGURACION
         * (bloqueadas con el motor operando), el procedimiento de esa
         * seccion seria irrealizable. El candado real que evita tocarlas
         * fuera de una sesion deliberada de sintonizacion NO es esta
         * categoria -- es el gate explicito por CONTROL_HABILITADO==10
         * dentro de cada case de abajo (mismo patron que
         * SERVO_PULSO_MIN/MAX con CONTROL_HABILITADO==1/2). PID_KD
         * eliminado 2026-09-23 -- ver nota de CALIB_ID_TASA_LLENADO_PSI_S
         * en calibracion_flash.h (ID 8 reasignado). */
        case CALIB_ID_PID_KP:
        case CALIB_ID_PID_KI:
        /* PRESION_REMOTO_PID_KP/KI (lazo EXTERNO de presion remota,
         * MODO=2, ver presion_pid_remoto.h) -- misma razon que
         * PID_KP/KI/KD arriba: se calibran en campo con el motor
         * operando, el candado real es el gate por CONTROL_HABILITADO==10
         * dentro del case de abajo. */
        case CALIB_ID_PRESION_REMOTO_PID_KP:
        case CALIB_ID_PRESION_REMOTO_PID_KI:
        /* PRESION_PID_KP/KI (lazo EXTERNO de presion local, MODO=1) --
         * misma razon que PID_KP/KI/KD arriba: se calibran en campo con
         * el motor operando, el candado real es el gate por
         * CONTROL_HABILITADO==10 dentro del case de abajo. */
        case CALIB_ID_PRESION_PID_KP:
        case CALIB_ID_PRESION_PID_KI:
            return CALIB_CATEGORIA_CALIBRACION;

        /* CONTROL_HABILITADO tambien sale del bloqueo generico de
         * CONFIGURACION, por la misma razon que PID_KP/KI/KD arriba --
         * los modos 7 (sintonizacion PID) y 3 (calibracion de ralenti)
         * NECESITAN poder activarse con el motor ya operando (no tiene
         * sentido "parar, activar, arrancar" para algo que solo sirve
         * con el motor corriendo). El candado real para 1/2 (que SI
         * deben exigir motor detenido, mueven el servo sin
         * realimentacion de RPM) esta adentro del case de abajo, no en
         * esta categoria. */
        case CALIB_ID_CONTROL_HABILITADO:
        /* MODO como CALIBRACION -- INHABILITADO el bloqueo con motor
         * operando que traia por default (2026-09-08). Antes de que
         * MODO controlara de donde sale el setpoint (ver bloque "MODO:
         * fuente del setpoint de RPM" en main.c), esto no importaba;
         * ahora que si -- si cae en CONFIGURACION, cambiar de RALENTI a
         * LOCAL/REMOTO exige parar el motor primero, lo cual es
         * innecesariamente incomodo (encontrado en campo justo
         * despues de migrar de un nodo que nunca habia tocado MODO,
         * que arranco en RALENTI e ignoraba SET_RPM hasta mandar
         * "MODO 1" -- con el motor detenido no se podia). Mismo
         * criterio que CONTROL_HABILITADO arriba: cambiar la FUENTE del
         * setpoint no es mas riesgoso que cambiar el setpoint mismo
         * (SET_RPM/PRESION, ya PROCESO, siempre en caliente). */
        case CALIB_ID_MODO:
            return CALIB_CATEGORIA_CALIBRACION;

        /* SET_RPM sigue como PROCESO (siempre permitido con el motor
         * operando) -- el candado real (exige MODO=3/MANUAL_BANCO, sin
         * el cual no tiene ningun efecto) esta adentro del case de
         * abajo, mismo patron que PID_KP/KI/KD con CONTROL_HABILITADO=10.
         *
         * PRESION_OBJETIVO es el equivalente de SET_RPM para el lazo
         * de presion (MODO=1) -- tiene que poder ajustarse en caliente
         * con el motor operando, si no el PID en cascada no serviria
         * de nada en campo. Encontrado 2026-09-21: caia en
         * CONFIGURACION por el default de abajo y quedaba bloqueado
         * justo cuando mas se necesitaba cambiarlo (motor corriendo). */
        case CALIB_ID_SET_RPM:
        case CALIB_ID_PRESION:
        case CALIB_ID_PRESION_OBJETIVO:
        /* PRESION_OBJETIVO_REMOTO -- mismo criterio que PRESION_OBJETIVO
         * arriba, es su equivalente para MODO=2 (solo alimenta el
         * supervisor, ver main.c/README seccion 8). */
        case CALIB_ID_PRESION_OBJETIVO_REMOTO:
        /* TASA_LLENADO_PSI_S -- misma razon, se puede querer ajustar el
         * ritmo de la rampa de llenado con el motor ya operando (ej. a
         * mitad de un llenado, sin tener que detener nada). */
        case CALIB_ID_TASA_LLENADO_PSI_S:
            return CALIB_CATEGORIA_PROCESO;

        case CALIB_ID_FORZAR_REPORTE:
        case CALIB_ID_RESTAURAR_DEFAULTS:
        case CALIB_ID_RESET_REMOTO:
            return CALIB_CATEGORIA_COMANDO;

        /* Todo lo demas (RPM_MAX/MIN, SERVO_PULSO_MIN/MAX,
         * TIMEOUT_SIN_COMANDO_S, TASA_MAX_CAMBIO_*, CONTROL_HABILITADO,
         * INTERVALO_*, NODE_ID, HISTERESIS_MODO_S) cae en CONFIGURACION
         * por el default. */
        default:
            return CALIB_CATEGORIA_CONFIGURACION;
    }
}


CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, float frecuenciaHzActual, uint16_t *valorAplicadoRaw)
{
    *valorAplicadoRaw = 0U;
    s_ultimaEscrituraFallo = false;

    /* Bloqueo de parámetros de CONFIGURACION mientras el motor opera --
     * se evalúa ANTES que cualquier otra validación (rango, flash),
     * y antes incluso de saber si el ID es válido o no, porque un ID
     * desconocido de todas formas cae en CATEGORIA_CONFIGURACION por
     * el default conservador de CalibFlash_CategoriaDe(). */
    if (motorOperando && CalibFlash_CategoriaDe(id) == CALIB_CATEGORIA_CONFIGURACION) {
        *valorAplicadoRaw = CalibFlash_ValorActualRaw(id);
        return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
    }

    /* El protocolo Quick-Set acordado manda siempre 2 bytes de valor
     * (VALUE_H+VALUE_L), incluso para parámetros de 1 byte lógico
     * (se usa solo VALUE_L, VALUE_H debe ir en 0). Si llega menos de
     * 2 bytes, es un downlink malformado -- se reporta como ID
     * desconocido, ya que no hay un código de protocolo específico
     * para "longitud inválida". */
    if (longitudDatos < 2) {
        return CALIB_STATUS_UNKNOWN_PARAMETER_ID;
    }

    switch (id) {

        case CALIB_ID_SET_RATIO: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetPulsosPorRevolucion(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        /* SET_RATIO_AUTO: en vez de mandar el ratio ya calculado a mano
         * (SET_RATIO de arriba), el operador manda el RPM que lee en
         * ese instante en un tacómetro de referencia externo, y el
         * firmware calcula el ratio el mismo a partir de su propia
         * frecuencia actual -- misma formula que Tacometro_Update()
         * pero despejada (ver tacometro.c:109). Requiere el motor
         * operando con lectura valida (si no, no hay nada que calcular). */
        case CALIB_ID_SET_RATIO_AUTO: {
            float rpmReferencia = (float)LeerUint16BigEndian(datos) / 10.0f;
            if (rpmReferencia <= 0.0f || frecuenciaHzActual <= 0.0f) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            float ratioCalculado = (frecuenciaHzActual * 60.0f) / rpmReferencia;
            bool ok = CalibFlash_SetPulsosPorRevolucion(ratioCalculado);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_ALPHA: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetAlphaFiltro(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetAlphaFiltro(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SET_RPM: {
            /* No persistente, sin validación de rango propia -- el
             * módulo de control debe recortarla contra RPM_MIN/MAX.
             * Solo tiene efecto real en MODO=3 (MANUAL_BANCO, ver
             * sección 4.3) -- en cualquier otro MODO el setpoint de RPM
             * sale de otro lado (ralentí, el lazo de presión local, o
             * la fórmula del aspersor remoto) y SET_RPM simplemente no
             * se lee en main.c. Rechazar acá (en vez de aceptarlo en
             * silencio sin que haga nada) para que el ACK avise de una
             * vez -- mismo patrón que PID_KP/KI/KD exigiendo
             * CONTROL_HABILITADO=10, agregado 2026-09-17 tras una
             * confusión real en campo con la renumeración de MODO. */
            if (CalibFlash_GetModo() != CALIB_MODO_MANUAL_BANCO) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetRpm(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            CalibFlash_SetSetRpm(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetRpm(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_RPM_MAX: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMax(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMax(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RPM_MIN: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMin(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_KP: {
            /* Escala x100. Solo se puede tocar en CONTROL_HABILITADO=10
             * (modo sintonizacion PID, ver README seccion 4.4/9) -- fuera
             * de ese modo (incluida la operacion normal, =0) se rechaza
             * con APPLY_ERROR, aunque el motor este detenido. Mismo
             * patron que SERVO_PULSO_MIN/MAX con CONTROL_HABILITADO=1/2:
             * por medida de seguridad, cambiar las ganancias del PID
             * requiere entrar deliberadamente a sintonizar, no es algo
             * que deba poder pasar por accidente en operacion normal. */
            if (CalibFlash_GetControlHabilitado() != 10U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKp(), 100.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetPidKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKp(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_KI: {
            /* Ver nota de CALIB_ID_PID_KP. */
            if (CalibFlash_GetControlHabilitado() != 10U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKi(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidKi(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SERVO_PULSO_MIN: {
            /* Solo se puede recalibrar el rango del servo en modo
             * calibracion (CONTROL_HABILITADO=1 o =2, NO 10 -- ese es
             * sintonizacion de PID, no recalibracion mecanica del servo)
             * -- ademas del bloqueo general de CONFIGURACION con el motor
             * operando (arriba), esto evita que un downlink suelto
             * cambie los topes mecanicos del acelerador fuera de una
             * sesion de banco deliberada. */
            uint8_t modoActual = CalibFlash_GetControlHabilitado();
            if (modoActual != 1U && modoActual != 2U) {
                *valorAplicadoRaw = CalibFlash_GetServoPulsoMinUs();
                return CALIB_STATUS_APPLY_ERROR;
            }
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetServoPulsoMinUs(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetServoPulsoMinUs();
            /* Modo manual (=1): marcar el valor recien aplicado como
             * objetivo pendiente para que main.c mueva el servo ahi --
             * por bandera, NO por comparar contra el valor anterior,
             * porque el valor "nuevo" puede coincidir con el que ya
             * estaba (ej. los defaults de fabrica) y un downlink real
             * igual debe mover el servo. */
            if (ok && CalibFlash_GetControlHabilitado() == 1U) {
                s_objetivoManualServoUs = CalibFlash_GetServoPulsoMinUs();
                s_objetivoManualServoPendiente = true;
            }
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SERVO_PULSO_MAX: {
            /* Ver nota de CALIB_ID_SERVO_PULSO_MIN. */
            uint8_t modoActualMax = CalibFlash_GetControlHabilitado();
            if (modoActualMax != 1U && modoActualMax != 2U) {
                *valorAplicadoRaw = CalibFlash_GetServoPulsoMaxUs();
                return CALIB_STATUS_APPLY_ERROR;
            }
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetServoPulsoMaxUs(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetServoPulsoMaxUs();
            if (ok && CalibFlash_GetControlHabilitado() == 1U) {
                s_objetivoManualServoUs = CalibFlash_GetServoPulsoMaxUs();
                s_objetivoManualServoPendiente = true;
            }
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TIMEOUT_SIN_COMANDO_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetTimeoutSinComandoS((uint32_t)nuevoValor);
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetTimeoutSinComandoS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TASA_MAX_CAMBIO_RPM_S: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetTasaMaxCambioRpmS(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaMaxCambioRpmS(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_CONTROL_HABILITADO: {
            /* ⚠️ RENUMERADO 2026-09-14 (ver README sección 12, discusión
             * sobre por qué el orden numérico no coincidía con el orden
             * de uso recomendado): 0=desactivado (operacion normal /
             * ralenti, segun SET_RPM), 1=manual, 2=barrido automático
             * MIN<->MAX, 3=calibración de ALPHA, 4=calibración de
             * ralentí, 5=calibración de SERVO_PULSO_MIN (umbral de
             * aceleración), 6=mapeo de curva de ganancia, 7=sintonización
             * PID (ver CalibFlash_GetControlHabilitado() y README 4.4).
             * Numerados en el mismo orden en que se usan en una puesta en
             * marcha (ver README sección 12): manual/barrido primero,
             * auto-cals en 3-6, PID al final en 7.
             *
             * Tres grupos por seguridad: 1/2 mueven el servo directo,
             * sin ninguna realimentación de RPM, y NO saben si el motor
             * está operando o no -- exigen el motor detenido para
             * entrar (chequeo explícito acá, ya que esta categoría ya
             * no bloquea todo por defecto). 0/7/4 no cambian cómo se
             * maneja el servo respecto a la operación normal (ver
             * main.c) y de hecho 7/4 solo tienen sentido con el motor ya
             * operando, así que no exigen detenerlo para entrar. 5
             * también mueve el servo directo, pero SÍ mira la RPM en
             * cada paso (a diferencia de 1/2) -- por eso puede permitir
             * entrar sin el motor operando (la rutina en main.c
             * simplemente espera a que arranque antes de mover nada). */
            uint8_t nuevoModo = datos[1];
            uint8_t modoActual = CalibFlash_GetControlHabilitado();

            if ((nuevoModo == 1U || nuevoModo == 2U) && motorOperando) {
                *valorAplicadoRaw = (uint16_t)modoActual;
                return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
            }

            /* Modos 3 a 9 solo se pueden pedir viniendo de modo 0 -- no
             * directo desde 1/2/10 (ni entre sí), para no arrancar una
             * ventana de calibración a mitad de otra sesión. Ninguno de
             * estos exige el motor operando para ACEPTAR el downlink (a
             * diferencia de 1/2) -- si se activa sin el motor corriendo,
             * la rutina en main.c simplemente espera a que arranque antes
             * de medir/mover el servo (o, para 6/8/9, antes de mandar el
             * escalón). Renumerado 2026-09-23, segunda pasada: se agregó
             * el modo 6 (auto-escalón interno, nuevo) a esta lista. */
            if ((nuevoModo == 3U || nuevoModo == 4U || nuevoModo == 5U || nuevoModo == 6U
                 || nuevoModo == 7U || nuevoModo == 8U || nuevoModo == 9U) && modoActual != 0U) {
                *valorAplicadoRaw = (uint16_t)modoActual;
                return CALIB_STATUS_OUT_OF_RANGE;
            }

            bool ok = CalibFlash_SetControlHabilitado(nuevoModo);
            /* Medida de seguridad: al ENTRAR a cualquier modo de
             * calibración (1-9) se limpia SET_RPM a su "sin comandar"
             * (0.0f) -- así el operador siempre tiene que volver a pedir
             * un setpoint explícito para esa sesión, en vez de que un
             * SET_RPM que haya quedado de una operación anterior active
             * el PID solo al entrar a modo 9 (en el resto no tiene efecto
             * directo, pero se limpia igual por consistencia; modo 8 se
             * pisa solo con su propio escalón de todas formas). No aplica
             * al volver a 0 -- ahí no hay nada que "limpiar hacia atrás". */
            if (ok && nuevoModo != 0U) {
                CalibFlash_SetSetRpm(0.0f);
            }
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetControlHabilitado();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetIntervaloEnvioOperativoS(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetIntervaloEnvioOperativoS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_INTERVALO_ENVIO_STANDBY_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetIntervaloEnvioStandbyS(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetIntervaloEnvioStandbyS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_MODO: {
            uint8_t modoSolicitado = datos[1];
            /* No se puede "armar" un modo de control automatico (presion
             * local/remota) con el motor detenido -- decision explicita
             * del usuario 2026-09-22, mismo motivo que el arranque
             * siempre en RALENTI (ver CalibFlash_Init()): si MODO=1/2
             * queda guardado desde antes de arrancar, el PID de presion
             * entraria a controlar desde el primer instante que el motor
             * arranque, sin que el operador haya podido confirmar a mano
             * que la tuberia ya esta llena. MODO=0 (RALENTI, siempre
             * seguro) y MODO=3 (MANUAL_BANCO, requiere SET_RPM explicito
             * del operador, no reacciona solo) quedan exentos de este
             * candado. */
            if (!motorOperando &&
                (modoSolicitado == (uint8_t)CALIB_MODO_PRESION_LOCAL ||
                 modoSolicitado == (uint8_t)CALIB_MODO_REMOTO)) {
                *valorAplicadoRaw = (uint16_t)CalibFlash_GetModo();
                return CALIB_STATUS_REJECTED_ENGINE_STOPPED;
            }
            /* No se puede "armar" MODO=2 (Remoto) sin PRESION_OBJETIVO_REMOTO
             * configurado -- decision explicita del usuario 2026-09-23. No
             * es un riesgo de seguridad (el PID persigue 0 PSI, el motor cae
             * solo cerca de ralenti) sino para evitar que quede "andando"
             * sin hacer nada util, en silencio -- ver nota de
             * CALIB_STATUS_REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO en el
             * header. */
            if (modoSolicitado == (uint8_t)CALIB_MODO_REMOTO &&
                CalibFlash_GetPresionObjetivoRemoto() <= 0.0f) {
                *valorAplicadoRaw = (uint16_t)CalibFlash_GetModo();
                return CALIB_STATUS_REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO;
            }
            bool ok = CalibFlash_SetModo((CalibFlash_Modo_t)modoSolicitado);
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetModo();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            CalibFlash_SetPresion(nuevoValor);
            /* Marca esta PRESION como "fresca" -- consumido por el
             * watchdog de TIMEOUT_SIN_COMANDO_S en MODO_REMOTO (ver
             * main.c). Se actualiza aca (no en CalibFlash_SetPresion())
             * para que solo cuente como "info fresca del aspersor" un
             * downlink/comando real de PRESION, no cualquier llamada
             * interna al setter. */
            s_presionUltimoTickMs = HAL_GetTick();
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresion(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_PRESION_REMOTO_PID_KP: {
            /* Escala x100, con signo. Ver nota de CALIB_ID_PID_KP --
             * mismo patron exacto: solo se acepta en CONTROL_HABILITADO=10
             * (sesion deliberada de sintonizacion), fuera de ese modo se
             * rechaza con APPLY_ERROR aunque el motor este detenido. */
            if (CalibFlash_GetControlHabilitado() != 10U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionRemotoPidKp(), 100.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetPresionRemotoPidKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionRemotoPidKp(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION_REMOTO_PID_KI: {
            /* Escala x1000, con signo -- mismo formato que
             * CALIB_ID_PRESION_PID_KI (lazo local), da mas resolucion
             * para valores de Ki tipicamente chicos. Ver nota de
             * CALIB_ID_PRESION_REMOTO_PID_KP -- mismo gate. */
            if (CalibFlash_GetControlHabilitado() != 10U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionRemotoPidKi(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPresionRemotoPidKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionRemotoPidKi(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION_PID_KP: {
            /* Escala x100, con signo. Ver nota de CALIB_ID_PID_KP --
             * mismo patron exacto: solo se acepta en CONTROL_HABILITADO=10. */
            if (CalibFlash_GetControlHabilitado() != 10U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionPidKp(), 100.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetPresionPidKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionPidKp(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION_PID_KI: {
            /* Escala x1000, con signo. Ver nota de CALIB_ID_PID_KI --
             * mismo patron exacto: solo se acepta en CONTROL_HABILITADO=10. */
            if (CalibFlash_GetControlHabilitado() != 10U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionPidKi(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPresionPidKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPresionPidKi(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_NODE_ID: {
            bool ok = CalibFlash_SetNodeId(datos[1]);
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetNodeId();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RESTAURAR_DEFAULTS: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                *valorAplicadoRaw = 0U;
                return CALIB_STATUS_OUT_OF_RANGE; /* confirmación inválida */
            }
            memset(&s_datos, 0, sizeof(s_datos));
            CalibFlash_Init(); /* magic queda inválido -> Init reconstruye defaults en RAM */
            bool ok = CalibFlash_EscribirEnFlash();
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return ok ? CALIB_STATUS_OK : CALIB_STATUS_STORAGE_ERROR;
        }

        case CALIB_ID_FORZAR_REPORTE: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                *valorAplicadoRaw = 0U;
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            s_reporteForzadoPendiente = true;
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_HISTERESIS_MODO_S: {
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetHisteresisModoS(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetHisteresisModoS();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RESET_REMOTO: {
            if (datos[1] != CALIB_BYTE_CONFIRMACION) {
                *valorAplicadoRaw = 0U;
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            s_resetPendiente = true;
            *valorAplicadoRaw = CALIB_BYTE_CONFIRMACION;
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_PRESION_OBJETIVO: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetPresionObjetivo(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionObjetivo(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION_OBJETIVO_REMOTO: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetPresionObjetivoRemoto(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionObjetivoRemoto(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TASA_LLENADO_PSI_S: {
            /* x100 -- valores tipicos son fracciones de PSI/s (ej. 0.02),
             * x10 se quedaria corto de resolucion. */
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 100.0f;
            bool ok = CalibFlash_SetTasaLlenadoPsiS(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaLlenadoPsiS(), 100.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetTasaMaxCambioRpmLlenadoS(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaMaxCambioRpmLlenadoS(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        default:
            *valorAplicadoRaw = 0U;
            return CALIB_STATUS_UNKNOWN_PARAMETER_ID;
    }
}

bool CalibFlash_ProcesarParametro(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, float frecuenciaHzActual)
{
    uint16_t valorAplicado;
    CalibFlash_ProtocoloStatus_t status = CalibFlash_ProcesarParametroConEstado(id, datos, longitudDatos, motorOperando, frecuenciaHzActual, &valorAplicado);
    return (status == CALIB_STATUS_OK);
}

/* ==================== GETTERS ==================== */

float    CalibFlash_GetPulsosPorRevolucion(void)      { return s_datos.pulsosPorRevolucion; }
float    CalibFlash_GetAlphaFiltro(void)              { return s_datos.alphaFiltro; }
float    CalibFlash_GetRpmMax(void)                   { return s_datos.rpmMax; }
float    CalibFlash_GetRpmMin(void)                   { return s_datos.rpmMin; }
float    CalibFlash_GetPidKp(void)                    { return s_datos.pidKp; }
float    CalibFlash_GetPidKi(void)                    { return s_datos.pidKi; }
uint16_t CalibFlash_GetServoPulsoMinUs(void)          { return s_datos.servoPulsoMinUs; }
uint16_t CalibFlash_GetServoPulsoMaxUs(void)          { return s_datos.servoPulsoMaxUs; }
uint32_t CalibFlash_GetTimeoutSinComandoS(void)       { return s_datos.timeoutSinComandoS; }
float    CalibFlash_GetTasaMaxCambioRpmS(void)        { return s_datos.tasaMaxCambioRpmS; }
uint8_t  CalibFlash_GetControlHabilitado(void)         { return s_datos.controlHabilitado; }
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void) { return s_datos.intervaloOperativoS; }
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void)   { return s_datos.intervaloStandbyS; }
CalibFlash_Modo_t CalibFlash_GetModo(void)            { return (CalibFlash_Modo_t)s_datos.modo; }
uint8_t  CalibFlash_GetNodeId(void)                   { return s_datos.nodeId; }
uint16_t CalibFlash_GetHisteresisModoS(void)          { return s_datos.histeresisModoS; }
float    CalibFlash_GetPresionObjetivo(void)          { return s_datos.presionObjetivo; }
float    CalibFlash_GetPresionObjetivoRemoto(void)    { return s_datos.presionObjetivoRemoto; }
float    CalibFlash_GetTasaLlenadoPsiS(void)          { return s_datos.tasaLlenadoPsiS; }
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void) { return s_datos.tasaMaxCambioRpmLlenadoS; }
float    CalibFlash_GetPresionRemotoPidKp(void)       { return s_datos.presionRemotoPidKp; }
float    CalibFlash_GetPresionRemotoPidKi(void)       { return s_datos.presionRemotoPidKi; }
float    CalibFlash_GetPresionPidKp(void)             { return s_datos.presionPidKp; }
float    CalibFlash_GetPresionPidKi(void)             { return s_datos.presionPidKi; }
uint32_t CalibFlash_GetPresionUltimoTickMs(void)      { return s_presionUltimoTickMs; }

uint32_t CalibFlash_GetUltimaHoraUtcConocida(void)
{
    return s_datos.ultimaHoraUtcConocida;
}

bool CalibFlash_SetUltimaHoraUtcConocida(uint32_t epochUtc)
{
    s_datos.ultimaHoraUtcConocida = epochUtc;
    return CalibFlash_EscribirEnFlash();
}

float CalibFlash_GetUltimaLatitudConocida(void)
{
    return s_datos.ultimaLatitudConocida;
}

float CalibFlash_GetUltimaLongitudConocida(void)
{
    return s_datos.ultimaLongitudConocida;
}

bool CalibFlash_SetUltimaPosicionConocida(float latitud, float longitud)
{
    /* Un solo borrado/escritura para los dos campos juntos -- se
     * actualizan siempre a la vez (ver GPS_TieneFix() en main.c), no
     * tiene sentido separarlos en dos llamadas y gastar flash doble. */
    s_datos.ultimaLatitudConocida = latitud;
    s_datos.ultimaLongitudConocida = longitud;
    return CalibFlash_EscribirEnFlash();
}

/* ==================== SETTERS (validan rango + persisten) ==================== */

bool CalibFlash_SetPulsosPorRevolucion(float v)
{
    if (v < RANGO_PULSOS_MIN || v > RANGO_PULSOS_MAX) return false;
    s_datos.pulsosPorRevolucion = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetAlphaFiltro(float v)
{
    if (v <= 0.0f || v > RANGO_ALPHA_MAX) return false;
    s_datos.alphaFiltro = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetRpmMax(float v)
{
    if (v <= s_datos.rpmMin || v > RANGO_RPM_MAX_TECHO) return false;
    s_datos.rpmMax = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetRpmMin(float v)
{
    if (v < 0.0f || v >= s_datos.rpmMax) return false;
    s_datos.rpmMin = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidKp(float v)
{
    s_datos.pidKp = v; /* ganancias PID: se permite cualquier valor con signo,
                         * la sintonización decide qué es razonable */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidKi(float v)
{
    s_datos.pidKi = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetServoPulsoMinUs(uint16_t v)
{
    if (v < 500U || v >= s_datos.servoPulsoMaxUs) return false;
    s_datos.servoPulsoMinUs = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetServoPulsoMaxUs(uint16_t v)
{
    if (v <= s_datos.servoPulsoMinUs || v > 2500U) return false;
    s_datos.servoPulsoMaxUs = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTimeoutSinComandoS(uint32_t v)
{
    if (v < 60U || v > 3600U) return false; /* 1 min a 60 min */
    s_datos.timeoutSinComandoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTasaMaxCambioRpmS(float v)
{
    if (v <= 0.0f || v > RANGO_TASA_CAMBIO_MAX) return false;
    s_datos.tasaMaxCambioRpmS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetControlHabilitado(uint8_t modo)
{
    if (modo > 11U) return false; /* ⚠️ RENUMERADO 2026-09-23, SEGUNDA
                                   * PASADA (ver README sección 12), Y
                                   * EXTENDIDO 2026-09-24 (agregado 11):
                                   * 0=desactivado, 1=manual, 2=barrido,
                                   * 3=calibracion de ALPHA, 4=calibracion
                                   * de SERVO_PULSO_MIN (zona muerta,
                                   * antes 5), 5=mapeo de curva de
                                   * ganancia (antes 6), 6=auto-escalon
                                   * interno NUEVO (PID#1), 7=calibracion
                                   * de ralenti (antes 4), 8=auto-escalon
                                   * presion local (antes 7), 9=auto-escalon
                                   * remoto (antes 8), 10=sintonizacion
                                   * PID (antes 9), 11=barrido de ganancia
                                   * de presion NUEVO (PID#2) */
    s_datos.controlHabilitado = modo;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetIntervaloEnvioOperativoS(uint16_t v)
{
    if (v < 5U || v > 3600U) return false;
    s_datos.intervaloOperativoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetIntervaloEnvioStandbyS(uint16_t v)
{
    if (v < 5U || v > 3600U) return false;
    s_datos.intervaloStandbyS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetModo(CalibFlash_Modo_t v)
{
    if (v != CALIB_MODO_RALENTI && v != CALIB_MODO_PRESION_LOCAL
        && v != CALIB_MODO_REMOTO && v != CALIB_MODO_MANUAL_BANCO) {
        return false;
    }
    s_datos.modo = (uint8_t)v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetNodeId(uint8_t v)
{
    s_datos.nodeId = v; /* 0-255, cualquier valor es válido */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetHisteresisModoS(uint16_t v)
{
    if (v > 600U) return false;
    s_datos.histeresisModoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionObjetivo(float v)
{
    if (v <= 0.0f || v > RANGO_PRESION_MAX) return false;
    s_datos.presionObjetivo = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionObjetivoRemoto(float v)
{
    /* A diferencia de PRESION_OBJETIVO (local), 0.0f SI es valido aca --
     * es el valor "sin configurar" que hace que el supervisor de MODO=2
     * caiga al criterio simple (ver main.c). */
    if (v < 0.0f || v > RANGO_PRESION_MAX) return false;
    s_datos.presionObjetivoRemoto = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTasaLlenadoPsiS(float v)
{
    /* 0.0f SI es valido -- es el valor "sin rampa" (ver header). */
    if (v < 0.0f || v > RANGO_TASA_LLENADO_PSI_S_MAX) return false;
    s_datos.tasaLlenadoPsiS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetTasaMaxCambioRpmLlenadoS(float v)
{
    if (v <= 0.0f || v > RANGO_TASA_CAMBIO_MAX) return false;
    /* Advertencia de diseño: se acepta aunque sea mayor a
     * tasaMaxCambioRpmS -- si eso pasa, el llamador (módulo de
     * control) debería tratarlo como configuración inconsistente y
     * usar el menor de los dos por seguridad. */
    s_datos.tasaMaxCambioRpmLlenadoS = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionRemotoPidKp(float v)
{
    s_datos.presionRemotoPidKp = v; /* ganancias PID: se permite cualquier valor con
                                       * signo, la sintonizacion decide que es razonable
                                       * (mismo criterio que CalibFlash_SetPresionPidKp()) */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionRemotoPidKi(float v)
{
    s_datos.presionRemotoPidKi = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionPidKp(float v)
{
    s_datos.presionPidKp = v; /* ganancias PID: se permite cualquier valor con
                                * signo, la sintonizacion decide que es razonable
                                * (mismo criterio que CalibFlash_SetPidKp()) */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionPidKi(float v)
{
    s_datos.presionPidKi = v;
    return CalibFlash_EscribirEnFlash();
}

/* ==================== NO PERSISTENTES ==================== */

float CalibFlash_GetSetRpm(void)  { return s_setRpm; }
void  CalibFlash_SetSetRpm(float v) { s_setRpm = v; }

float CalibFlash_GetPresion(void) { return s_presion; }
void  CalibFlash_SetPresion(float v) { s_presion = v; }

/* ==================== COMANDOS ==================== */

bool CalibFlash_HayReporteForzado(void)
{
    return s_reporteForzadoPendiente;
}

void CalibFlash_ForzarReporte(void)
{
    s_reporteForzadoPendiente = true;
}

void CalibFlash_LimpiarReporteForzado(void)
{
    s_reporteForzadoPendiente = false;
}

bool CalibFlash_HayResetPendiente(void)
{
    return s_resetPendiente;
}

bool CalibFlash_HayObjetivoManualServo(void)
{
    return s_objetivoManualServoPendiente;
}

uint16_t CalibFlash_GetObjetivoManualServoUs(void)
{
    return s_objetivoManualServoUs;
}

void CalibFlash_LimpiarObjetivoManualServo(void)
{
    s_objetivoManualServoPendiente = false;
}

/* ==================== FUNCIONES PRIVADAS ==================== */

static uint16_t LeerUint16BigEndian(const uint8_t *datos)
{
    return (uint16_t)(((uint16_t)datos[0] << 8) | (uint16_t)datos[1]);
}

static int16_t LeerInt16BigEndian(const uint8_t *datos)
{
    uint16_t valor = (uint16_t)(((uint16_t)datos[0] << 8) | (uint16_t)datos[1]);
    return (int16_t)valor;
}

/* Codifica un valor real a uint16, aplicando la escala inversa y
 * redondeando. Se usa para parámetros sin signo (RPM, presión,
 * segundos, etc.). Se satura al rango de uint16 por seguridad, en
 * vez de desbordarse silenciosamente. */

/* Consulta el valor vigente de un parámetro (sin intentar cambiarlo),
 * ya codificado en 2 bytes -- usado para reportar el valor correcto en
 * el ACK cuando un cambio se rechaza (ej. por CALIB_STATUS_REJECTED_
 * ENGINE_RUNNING), en vez de reportar 0. */
static uint16_t CalibFlash_ValorActualRaw(uint8_t id)
{
    switch (id) {
        case CALIB_ID_SET_RATIO:
        case CALIB_ID_SET_RATIO_AUTO:
            return CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
        case CALIB_ID_ALPHA:
            return CodificarUint16(CalibFlash_GetAlphaFiltro(), 1000.0f);
        case CALIB_ID_RPM_MAX:
            return CodificarUint16(CalibFlash_GetRpmMax(), 10.0f);
        case CALIB_ID_RPM_MIN:
            return CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
        case CALIB_ID_PID_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPidKp(), 100.0f);
        case CALIB_ID_PID_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPidKi(), 1000.0f);
        case CALIB_ID_SERVO_PULSO_MIN:
            return CalibFlash_GetServoPulsoMinUs();
        case CALIB_ID_SERVO_PULSO_MAX:
            return CalibFlash_GetServoPulsoMaxUs();
        case CALIB_ID_TIMEOUT_SIN_COMANDO_S:
            return (uint16_t)CalibFlash_GetTimeoutSinComandoS();
        case CALIB_ID_TASA_MAX_CAMBIO_RPM_S:
            return CodificarUint16(CalibFlash_GetTasaMaxCambioRpmS(), 10.0f);
        case CALIB_ID_CONTROL_HABILITADO:
            return (uint16_t)CalibFlash_GetControlHabilitado();
        case CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S:
            return CalibFlash_GetIntervaloEnvioOperativoS();
        case CALIB_ID_INTERVALO_ENVIO_STANDBY_S:
            return CalibFlash_GetIntervaloEnvioStandbyS();
        case CALIB_ID_MODO:
            return (uint16_t)CalibFlash_GetModo();
        case CALIB_ID_NODE_ID:
            return (uint16_t)CalibFlash_GetNodeId();
        case CALIB_ID_HISTERESIS_MODO_S:
            return CalibFlash_GetHisteresisModoS();
        case CALIB_ID_PRESION_OBJETIVO:
            return CodificarUint16(CalibFlash_GetPresionObjetivo(), 10.0f);
        case CALIB_ID_PRESION_OBJETIVO_REMOTO:
            return CodificarUint16(CalibFlash_GetPresionObjetivoRemoto(), 10.0f);
        case CALIB_ID_TASA_LLENADO_PSI_S:
            return CodificarUint16(CalibFlash_GetTasaLlenadoPsiS(), 100.0f);
        case CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S:
            return CodificarUint16(CalibFlash_GetTasaMaxCambioRpmLlenadoS(), 10.0f);
        case CALIB_ID_PRESION_REMOTO_PID_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPresionRemotoPidKp(), 100.0f);
        case CALIB_ID_PRESION_REMOTO_PID_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPresionRemotoPidKi(), 1000.0f);
        case CALIB_ID_PRESION_PID_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPresionPidKp(), 100.0f);
        case CALIB_ID_PRESION_PID_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPresionPidKi(), 1000.0f);
        default:
            return 0U;
    }
}

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

/* Codifica un valor real con signo a un patrón de bits de 16 bits
 * (int16 reinterpretado como uint16 para el transporte), usado para
 * las ganancias del PID. Se satura al rango de int16. */
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
    return (uint16_t)valorEntero; /* reinterpretación de bits, no conversión numérica */
}

static bool CalibFlash_EscribirEnFlash(void)
{
    HAL_StatusTypeDef estado;
    FLASH_EraseInitTypeDef borrado = {0};
    uint32_t paginaConError = 0;

    s_datos.magic = CALIB_FLASH_MAGIC;
    s_ultimaEscrituraFallo = false;

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
