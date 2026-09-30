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
 *   - SET_RATIO, ALPHA, PID_RPM_KP/KI, TASA_MAX_CAMBIO_*,
 *     PRESION_OBJETIVO_LOCAL: 4 bytes, int32 con signo, escala según tabla.
 *   - SET_RPM, RPM_MAX, RPM_MIN, PRESION_REMOTO: 2 bytes, uint16, x10.
 *   - SERVO_PULSO_MIN/MAX: 2 bytes, uint16, en µs directo (sin escala).
 *   - TIMEOUT_SIN_COMANDO_S: 4 bytes, uint32, en segundos directo.
 *   - INTERVALO_*, HISTERESIS_MODO_S: 2 bytes, uint16, en segundos.
 *   - CALIB, MODO, NODE_ID, y los 3 comandos: 1 byte.
 */

#include "calibracion_flash.h"
#include "main.h"
#include <string.h>

/* ==================== CONFIGURACIÓN DE UBICACIÓN EN FLASH ==================== */

#define CALIB_FLASH_PAGE_NUMBER   63U
#define CALIB_FLASH_ADDRESS       0x0801F800UL
#define CALIB_FLASH_MAGIC         0x43414C4FUL  /* "CALO" -- subido desde "CALN"
                                                   * 2026-09-29 al agregar presionMax
                                                   * (PRESION_MAX). Antes: "CALN" -- subido desde
                                                   * "CALM" (mismo dia,
                                                   * 2026-09-28) porque se
                                                   * agrego tiempoLlenadoS a
                                                   * CalibFlash_Datos_t --
                                                   * guarda el valor crudo en
                                                   * segundos de
                                                   * TIEMPO_LLENADO_S,
                                                   * reusado como techo de
                                                   * CALIB=9. */

/* ==================== VALORES POR DEFECTO ==================== */
/* Los mismos que ya tenías validados en campo para SET_RATIO/ALPHA;
 * el resto son placeholders razonables -- AJUSTAR antes de operar
 * con el motor/servo real. */

#define DEFAULT_PULSOS_POR_REVOLUCION     17.5f
#define DEFAULT_ALPHA_FILTRO              0.35f
#define DEFAULT_RPM_MAX                   2500.0f
#define DEFAULT_RPM_MAX_CARGA             2500.0f  /* = DEFAULT_RPM_MAX -- sin restriccion hidraulica adicional hasta que el operador configure el limite real de esa instalacion */
#define DEFAULT_PRESION_MAX               100.0f   /* PSI -- conservador hasta que el operador configure el limite real de la tuberia, ver CALIB_ID_PRESION_MAX */
#define DEFAULT_RPM_MIN                   700.0f
#define DEFAULT_PID_RPM_KP                1.0f
#define DEFAULT_PID_RPM_KI                0.0f
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
#define DEFAULT_CALIB                     0U
#define DEFAULT_INTERVALO_OPERATIVO_S     30U
#define DEFAULT_INTERVALO_STANDBY_S       300U
#define DEFAULT_MODO                      CALIB_MODO_RALENTI
#define DEFAULT_NODE_ID                   0U
#define DEFAULT_HISTERESIS_MODO_S         30U
#define DEFAULT_TIEMPO_LLENADO_S        1800U    /* 30 minutos -- default conservador hasta que el operador configure el real de esta instalacion, ver CALIB_ID_TIEMPO_LLENADO_S */
#define DEFAULT_PRESION_OBJETIVO_LOCAL    40.0f
#define DEFAULT_PRESION_OBJETIVO_REMOTO    0.0f  /* sin configurar -- ver CALIB_ID_PRESION_OBJETIVO_REMOTO */
#define DEFAULT_TASA_LLENADO_PSI_S         0.0f  /* sin rampa -- ver CALIB_ID_TASA_LLENADO_PSI_S */
#define DEFAULT_TASA_MAX_CAMBIO_LLENADO_S 10.0f  /* mas lenta que la normal, a proposito */
#define DEFAULT_PID_ASP_KP                 0.0f  /* sin calibrar -- lazo de presion remota (MODO=2), ver README */
#define DEFAULT_PID_ASP_KI                 0.0f  /* idem */
#define DEFAULT_PID_PSI_KP                 0.0f  /* sin calibrar -- lazo de presion local (MODO=1), ver README */
#define DEFAULT_PID_PSI_KI                 0.0f  /* idem */

/* ==================== RANGOS VÁLIDOS ==================== */

#define RANGO_PULSOS_MIN         0.1f
#define RANGO_PULSOS_MAX         200.0f
#define RANGO_ALPHA_MAX          1.0f
#define RANGO_RPM_MAX_TECHO      6000.0f
#define RANGO_TASA_CAMBIO_MAX    2000.0f
#define RANGO_PRESION_MAX        500.0f
#define RANGO_TASA_LLENADO_PSI_S_MAX    10.0f    /* generoso -- en la practica se usan valores bien chicos (fracciones de PSI/s) */
/* PID_ASP_KP/KI no tienen rango propio -- mismo criterio que
 * PID_PSI_KP/KI (lazo local): son ganancias de PID, se aceptan con
 * cualquier signo, la sintonizacion en campo decide que es razonable. */

/* ==================== ESTRUCTURA GUARDADA EN FLASH ==================== */
/* Debe quedar en múltiplo de 8 bytes (double word) para el
 * STM32G4. Se agrega 'relleno' al final si hace falta. */

typedef struct {
    uint32_t magic;

    float    pulsosPorRevolucion;      /* SET_RATIO */
    float    alphaFiltro;              /* ALPHA */
    float    rpmMax;                   /* RPM_MAX -- limite MECANICO del motor/motobomba, fijo por modelo, no depende de la instalacion */
    float    rpmMaxCarga;              /* RPM_MAX_CARGA -- limite CON CARGA (hidraulico), especifico de cada instalacion, SIEMPRE <= rpmMax (agregado 2026-09-28) */
    float    presionMax;               /* PRESION_MAX -- guarda dura de presion local, ver CALIB_ID_PRESION_MAX (agregado 2026-09-29) */
    float    rpmMin;                   /* RPM_MIN */
    float    pidRpmKp;                 /* PID_RPM_KP */
    float    pidRpmKi;                 /* PID_RPM_KI */
    float    tasaMaxCambioRpmS;        /* TASA_MAX_CAMBIO_RPM_S */
    float    presionObjetivoLocal;     /* PRESION_OBJETIVO_LOCAL */
    float    tasaMaxCambioRpmLlenadoS; /* TASA_MAX_CAMBIO_RPM_LLENADO_S */
    float    ultimaLatitudConocida;    /* GPS, 0.0f = nunca hubo fix */
    float    ultimaLongitudConocida;   /* GPS, 0.0f = nunca hubo fix */
    float    pidAspKp;                 /* PID_ASP_KP -- lazo externo de presion remota (MODO=2) */
    float    pidAspKi;                 /* PID_ASP_KI -- idem */
    float    pidPsiKp;                 /* PID_PSI_KP -- lazo externo de presion local (MODO=1) */
    float    pidPsiKi;                 /* PID_PSI_KI -- idem */
    float    presionObjetivoRemoto;    /* PRESION_OBJETIVO_REMOTO -- solo para el supervisor de MODO=2, ver header */
    float    tasaLlenadoPsiS;          /* TASA_LLENADO_PSI_S -- rampa de llenado de MODO=1, ver header */

    uint32_t timeoutSinComandoS;       /* TIMEOUT_SIN_COMANDO_S */
    uint32_t ultimaHoraUtcConocida;    /* epoch UTC, 0 = nunca sincronizado */

    uint16_t servoPulsoMinUs;          /* SERVO_PULSO_MIN */
    uint16_t servoPulsoMaxUs;          /* SERVO_PULSO_MAX */
    uint16_t intervaloOperativoS;      /* INTERVALO_ENVIO_OPERATIVO_S */
    uint16_t intervaloStandbyS;        /* INTERVALO_ENVIO_STANDBY_S */
    uint16_t histeresisModoS;          /* HISTERESIS_MODO_S */
    uint16_t tiempoLlenadoS;           /* TIEMPO_LLENADO_S -- valor crudo en segundos del
                                         * ULTIMO downlink (agregado 2026-09-28), ademas
                                         * del calculo que ya hace y guarda en
                                         * tasaLlenadoPsiS. Reusado directo como techo de
                                         * tiempo de CALIB=9 (ver main.c) --
                                         * mismo numero que el operador ya dio para el
                                         * llenado real de MODO=1, decision explicita del
                                         * usuario. 0 = nunca configurado. */

    uint8_t  calib;                    /* CALIB (0/1/2) -- se
                                         * persiste en flash como cualquier otro
                                         * parametro CONFIGURACION, pero
                                         * CalibFlash_Init() lo fuerza a 0 en RAM
                                         * en cada arranque (ver mas abajo), nunca
                                         * arranca directo en modo calibracion */
    uint8_t  modo;                     /* MODO (0/1/2) */
    uint8_t  nodeId;                   /* NODE_ID */
    uint8_t  relleno[1];               /* completa a múltiplo de 8 bytes -- ajustar si
                                         * CalibFlash_VerificarTamano rompe la compilación.
                                         * Bajado de 7 a 5 el 2026-09-28 al agregar
                                         * tiempoLlenadoS (un uint16, +2 bytes) a la struct.
                                         * Bajado de 5 a 1 el 2026-09-29 al agregar
                                         * presionMax (un float, +4 bytes). */
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
static float s_presionRemoto = 0.0f;

/* HAL_GetTick() del último downlink de PRESION_REMOTO valido -- ver
 * CalibFlash_GetPresionRemotoUltimoTickMs(). Arranca en el tick del boot (no
 * en 0) para que el watchdog de TIMEOUT_SIN_COMANDO_S en MODO_REMOTO
 * cuente desde el arranque si nunca llega ninguna PRESION_REMOTO, en vez de
 * pensar que ya expiro desde "tiempo epoch 0". */
static uint32_t s_presionRemotoUltimoTickMs = 0U;

/* Banderas de comandos */
static volatile bool s_reporteForzadoPendiente = false;
static volatile bool s_resetPendiente = false;

/* Objetivo pendiente para el modo manual de calibracion del servo
 * (CALIB=2, ver main.c) -- se marca aqui cuando un downlink
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
        s_datos.calib = 0U; /* CALIB NUNCA arranca en
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
                                           * CALIB arriba: no se
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
        s_datos.rpmMaxCarga              = DEFAULT_RPM_MAX_CARGA;
        s_datos.presionMax               = DEFAULT_PRESION_MAX;
        s_datos.rpmMin                   = DEFAULT_RPM_MIN;
        s_datos.pidRpmKp                 = DEFAULT_PID_RPM_KP;
        s_datos.pidRpmKi                 = DEFAULT_PID_RPM_KI;
        s_datos.tasaMaxCambioRpmS        = DEFAULT_TASA_MAX_CAMBIO_RPM_S;
        s_datos.presionObjetivoLocal     = DEFAULT_PRESION_OBJETIVO_LOCAL;
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
        s_datos.pidAspKp                 = DEFAULT_PID_ASP_KP;
        s_datos.pidAspKi                 = DEFAULT_PID_ASP_KI;
        s_datos.pidPsiKp                 = DEFAULT_PID_PSI_KP;
        s_datos.pidPsiKi                 = DEFAULT_PID_PSI_KI;
        s_datos.servoPulsoMinUs          = DEFAULT_SERVO_PULSO_MIN_US;
        s_datos.servoPulsoMaxUs          = DEFAULT_SERVO_PULSO_MAX_US;
        s_datos.intervaloOperativoS      = DEFAULT_INTERVALO_OPERATIVO_S;
        s_datos.intervaloStandbyS        = DEFAULT_INTERVALO_STANDBY_S;
        s_datos.histeresisModoS          = DEFAULT_HISTERESIS_MODO_S;
        s_datos.tiempoLlenadoS           = DEFAULT_TIEMPO_LLENADO_S;
        s_datos.calib                    = DEFAULT_CALIB;
        s_datos.modo                     = (uint8_t)DEFAULT_MODO;
        s_datos.nodeId                   = DEFAULT_NODE_ID;
        /* No se escribe a flash aquí -- solo al primer Set explícito. */
    }

    s_setRpm = 0.0f;
    s_presionRemoto = 0.0f;
    s_presionRemotoUltimoTickMs = HAL_GetTick();
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
        /* PID_RPM_KP/KI como CALIBRACION (no CONFIGURACION por default) --
         * misma razon que SET_RATIO/ALPHA: la sintonizacion en lazo
         * cerrado (README seccion 9, metodo Ziegler-Nichols) exige subir
         * Kp de a poco CON EL MOTOR OPERANDO, observando la respuesta
         * real de RPM en cada paso -- si cayeran en CONFIGURACION
         * (bloqueadas con el motor operando), el procedimiento de esa
         * seccion seria irrealizable. El candado real que evita tocarlas
         * fuera de una sesion deliberada de sintonizacion NO es esta
         * categoria -- es el gate explicito por MODO==CALIBRACION &&
         * CALIB==0 dentro de cada case de abajo (distinto patron que
         * SERVO_PULSO_MIN/MAX, que usan CALIB==1/2 directo). PID_KD
         * eliminado 2026-09-23 -- ver nota de CALIB_ID_TASA_LLENADO_PSI_S
         * en calibracion_flash.h (ID 8 reasignado). */
        case CALIB_ID_PID_RPM_KP:
        case CALIB_ID_PID_RPM_KI:
        /* PID_ASP_KP/KI (lazo EXTERNO de presion remota,
         * MODO=2, ver presion_pid_remoto.h) -- misma razon que
         * PID_RPM_KP/KI arriba: se calibran en campo con el motor
         * operando, el candado real es el gate por MODO==CALIBRACION &&
         * CALIB==0 dentro del case de abajo. */
        case CALIB_ID_PID_ASP_KP:
        case CALIB_ID_PID_ASP_KI:
        /* PID_PSI_KP/KI (lazo EXTERNO de presion local, MODO=1) --
         * misma razon que PID_RPM_KP/KI arriba: se calibran en campo con
         * el motor operando, el candado real es el gate por
         * MODO==CALIBRACION && CALIB==0 dentro del case de abajo. */
        case CALIB_ID_PID_PSI_KP:
        case CALIB_ID_PID_PSI_KI:
            return CALIB_CATEGORIA_CALIBRACION;

        /* CALIB tambien sale del bloqueo generico de CONFIGURACION, por
         * la misma razon que PID_RPM_KP/KI arriba -- la mayoria de sus
         * valores (3 a 12: auto-calibraciones y pruebas automatizadas)
         * NECESITAN poder activarse con el motor ya operando (no tiene
         * sentido "parar, activar, arrancar" para algo que solo sirve
         * con el motor corriendo). El candado real para 1/2 (que SI
         * deben exigir motor detenido, mueven el servo sin
         * realimentacion de RPM) esta adentro del case de abajo, no en
         * esta categoria. */
        case CALIB_ID_CALIB:
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
         * criterio que CALIB arriba: cambiar la FUENTE del
         * setpoint no es mas riesgoso que cambiar el setpoint mismo
         * (SET_RPM/PRESION_REMOTO, ya PROCESO, siempre en caliente). */
        case CALIB_ID_MODO:
            return CALIB_CATEGORIA_CALIBRACION;

        /* SET_RPM sigue como PROCESO (siempre permitido con el motor
         * operando) -- el candado real (exige MODO=3/MANUAL_BANCO, sin
         * el cual no tiene ningun efecto) esta adentro del case de
         * abajo, mismo patron que PID_RPM_KP/KI con su propio candado
         * (MODO==CALIBRACION && CALIB==0).
         *
         * PRESION_OBJETIVO_LOCAL es el equivalente de SET_RPM para el lazo
         * de presion (MODO=1) -- tiene que poder ajustarse en caliente
         * con el motor operando, si no el PID en cascada no serviria
         * de nada en campo. Encontrado 2026-09-21: caia en
         * CONFIGURACION por el default de abajo y quedaba bloqueado
         * justo cuando mas se necesitaba cambiarlo (motor corriendo). */
        case CALIB_ID_SET_RPM:
        case CALIB_ID_PRESION_REMOTO:
        case CALIB_ID_PRESION_OBJETIVO_LOCAL:
        /* SET_PRESION -- mismo criterio que SET_RPM arriba, agregado
         * 2026-09-25 (ver calibracion_flash.h). */
        case CALIB_ID_SET_PRESION:
        /* PRESION_OBJETIVO_REMOTO -- mismo criterio que PRESION_OBJETIVO_LOCAL
         * arriba, es su equivalente para MODO=2 (solo alimenta el
         * supervisor, ver main.c/README seccion 8). */
        case CALIB_ID_PRESION_OBJETIVO_REMOTO:
        /* TASA_LLENADO_PSI_S -- misma razon, se puede querer ajustar el
         * ritmo de la rampa de llenado con el motor ya operando (ej. a
         * mitad de un llenado, sin tener que detener nada). */
        case CALIB_ID_TASA_LLENADO_PSI_S:
        /* TIEMPO_LLENADO_S -- conveniencia sobre TASA_LLENADO_PSI_S
         * (agregado 2026-09-28), misma categoria que ese. */
        case CALIB_ID_TIEMPO_LLENADO_S:
            return CALIB_CATEGORIA_PROCESO;

        case CALIB_ID_FORZAR_REPORTE:
        case CALIB_ID_RESTAURAR_DEFAULTS:
        case CALIB_ID_RESET_REMOTO:
            return CALIB_CATEGORIA_COMANDO;

        /* Todo lo demas (RPM_MAX/MIN, SERVO_PULSO_MIN/MAX,
         * TIMEOUT_SIN_COMANDO_S, TASA_MAX_CAMBIO_*, CALIB,
         * INTERVALO_*, NODE_ID, HISTERESIS_MODO_S) cae en CONFIGURACION
         * por el default. */
        default:
            return CALIB_CATEGORIA_CONFIGURACION;
    }
}


CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, float frecuenciaHzActual, float presionActual, uint16_t *valorAplicadoRaw)
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
            /* Exige MODO=CALIBRACION -- agregado 2026-09-25, decision
             * explicita del usuario: a diferencia de las ganancias PID
             * (que SI necesitan poder tocarse con el lazo real corriendo,
             * para sintonizar en vivo -- ver README seccion 9), no hay
             * ningun motivo legitimo para recalibrar la relacion pulsos/
             * revolucion con produccion real activa (MODO=1/2) -- un solo
             * downlink por error ahi corrompe la lectura de RPM que
             * alimenta al PID que esta controlando de verdad, sin ningun
             * paso intermedio de por medio. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
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
            /* Exige MODO=CALIBRACION -- ver nota de CALIB_ID_SET_RATIO
             * arriba, mismo motivo exacto. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPulsosPorRevolucion(), 100.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
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
             * vez -- mismo patrón que PID_RPM_KP/KI exigiendo
             * MODO=CALIBRACION && CALIB==0, agregado 2026-09-17 tras una
             * confusión real en campo con la renumeración de MODO. */
            if (CalibFlash_GetModo() != CALIB_MODO_MANUAL_BANCO) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetRpm(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            CalibFlash_SetSetRpm(nuevoValor);
            /* Mutuamente excluyente con SET_PRESION (agregado 2026-09-25)
             * -- gana el ultimo que llega, asi que mandar SET_RPM limpia
             * cualquier SET_PRESION que hubiera quedado activo. */
            CalibFlash_SetSetPresion(0.0f);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetRpm(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_SET_PRESION: {
            /* Mismo patron que SET_RPM arriba (mismo comentario aplica:
             * no persistente, sin validacion de rango propia, exclusivo
             * de MODO=3) -- agregado 2026-09-25, ver calibracion_flash.h. */
            if (CalibFlash_GetModo() != CALIB_MODO_MANUAL_BANCO) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetPresion(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            /* SIEMPRE < PRESION_MAX (agregado 2026-09-29) -- pedir una
             * presion igual o mayor al techo no tiene sentido, la guarda
             * la cortaria de inmediato. */
            if (nuevoValor >= CalibFlash_GetPresionMax()) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetPresion(), 10.0f);
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            CalibFlash_SetSetPresion(nuevoValor);
            /* Mutuamente excluyente con SET_RPM -- gana el ultimo que
             * llega, asi que mandar SET_PRESION limpia cualquier SET_RPM
             * que hubiera quedado activo. */
            CalibFlash_SetSetRpm(0.0f);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetSetPresion(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_RPM_MAX: {
            /* Exige MODO=CALIBRACION -- agregado 2026-09-25, mismo
             * criterio que SET_RATIO/SET_RATIO_AUTO arriba, y con más
             * razón: son los limites de seguridad duros de todo el
             * sistema (ver saturacion en presion_pid.c/presion_pid_remoto.c).
             * La categoria CONFIGURACION ya exige motor detenido para
             * tocarlos (chequeo generico, ver CalibFlash_CategoriaDe()/
             * el dispatcher de arriba) -- esto suma una segunda condicion
             * independiente: ademas de estar detenido, tiene que ser una
             * sesion de calibracion deliberada, no cualquier parada
             * momentanea (recarga de diesel, mantenimiento). */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMax(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMax(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMax(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RPM_MIN: {
            /* Exige MODO=CALIBRACION -- ver nota de CALIB_ID_RPM_MAX
             * arriba, mismo motivo exacto. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMin(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_RPM_MAX_CARGA: {
            /* Exige MODO=CALIBRACION -- ver nota de CALIB_ID_RPM_MAX
             * arriba, mismo motivo exacto (limite de seguridad duro,
             * agregado 2026-09-28). */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMaxCarga(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetRpmMaxCarga(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetRpmMaxCarga(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION_MAX: {
            /* Exige MODO=CALIBRACION -- ver nota de CALIB_ID_RPM_MAX
             * arriba, mismo motivo exacto (limite de seguridad duro). */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionMax(), 10.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetPresionMax(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionMax(), 10.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_RPM_KP: {
            /* Escala x1000 (subida de x100 2026-09-24 -- con x100 el Kp
             * real validado en campo para el lazo interno, 0.047, se
             * redondeaba a 0.05 al pasar por el protocolo: 0.047*100=4.7
             * -> 5 -> 0.05, un error de ~6% en una ganancia chica. x1000
             * da 3 decimales, igual que PID_RPM_KI, sin perder rango util --
             * con int16 con signo, x1000 sigue dando +-32.767, de sobra
             * frente a los Kp reales que se manejan (0.047 a ~3.73).
             * ⚠️ CALIB=10 [numero viejo, antes de la renumeracion
             * 2026-09-29] (sintonizacion PID) ELIMINADO 2026-09-29 --
             * decision explicita del usuario: las rutinas CALIB existen
             * justamente para que el operador no tenga que saltar de una
             * a otra, y exigir entrar a =10 solo para poder tocar
             * ganancias iba contra esa idea. El candado real (que ningun
             * CALIB automatico este corriendo Y que se este en
             * MODO=CALIBRACION) se logra igual de bien exigiendo
             * MODO=CALIBRACION Y CALIB=0 -- apenas un CALIB
             * automatico termina y vuelve solo a 0, ya se puede setear
             * la ganancia directo, sin ningun salto de mas. Sigue
             * bloqueado en operacion normal (MODO!=4) y mientras
             * cualquier CALIB este activo (protege que no se cuele un
             * cambio de ganancia a mitad de una prueba automatica que
             * depende de que no cambien). Mismo patron que
             * SERVO_PULSO_MIN/MAX con CALIB=1/2: cambiar
             * las ganancias del PID requiere estar deliberadamente en
             * sesion de calibracion, no es algo que deba poder pasar
             * por accidente en operacion normal. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                || CalibFlash_GetCalib() != 0U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidRpmKp(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidRpmKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidRpmKp(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_RPM_KI: {
            /* Ver nota de CALIB_ID_PID_RPM_KP. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                || CalibFlash_GetCalib() != 0U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidRpmKi(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidRpmKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidRpmKi(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SERVO_PULSO_MIN: {
            /* Solo se puede recalibrar el rango del servo en modo
             * calibracion (CALIB=1 o =2 -- distinto de sintonizacion de
             * PID, que no recalibra nada mecanico del servo) -- ademas
             * del bloqueo general de CONFIGURACION con el motor
             * operando (arriba), esto evita que un downlink suelto
             * cambie los topes mecanicos del acelerador fuera de una
             * sesion de banco deliberada. */
            uint8_t modoActual = CalibFlash_GetCalib();
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
            if (ok && CalibFlash_GetCalib() == 1U) {
                s_objetivoManualServoUs = CalibFlash_GetServoPulsoMinUs();
                s_objetivoManualServoPendiente = true;
            }
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_SERVO_PULSO_MAX: {
            /* Ver nota de CALIB_ID_SERVO_PULSO_MIN. */
            uint8_t modoActualMax = CalibFlash_GetCalib();
            if (modoActualMax != 1U && modoActualMax != 2U) {
                *valorAplicadoRaw = CalibFlash_GetServoPulsoMaxUs();
                return CALIB_STATUS_APPLY_ERROR;
            }
            uint16_t nuevoValor = LeerUint16BigEndian(datos);
            bool ok = CalibFlash_SetServoPulsoMaxUs(nuevoValor);
            *valorAplicadoRaw = CalibFlash_GetServoPulsoMaxUs();
            if (ok && CalibFlash_GetCalib() == 1U) {
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

        case CALIB_ID_CALIB: {
            /* ⚠️ RENUMERADO 2026-09-29 (tercera vez, ver README sección
             * 4.4/12): 0=desactivado (operacion normal/ralenti), 1=manual,
             * 2=barrido automático MIN<->MAX, 3=auto-cal ALPHA, 4=auto-cal
             * zona muerta (SERVO_PULSO_MIN), 5=mapeo curva de ganancia
             * (PID#1, lazo abierto), 6=auto-escalón interno (PID#1),
             * 7=validación interno con ganancias reales (PID#1, antes 12),
             * 8=auto-cal ralentí (RPM_MIN, antes 7), 9=barrido de ganancia
             * de presión local (PID#2, lazo abierto, antes 11),
             * 10=auto-escalón presión local (PID#2, antes 8), 11=validación
             * presión local con ganancias reales (PID#2, antes 13),
             * 12=auto-escalón remoto (PID#3, antes 9).
             *
             * Tres grupos por seguridad: 1/2 mueven el servo directo,
             * sin ninguna realimentación de RPM -- exigen el motor
             * detenido para entrar (chequeo explícito acá). 0/6/7/8/9/10/11/12
             * no cambian cómo se maneja el servo respecto a la operación
             * normal (ver main.c) y de hecho 6/7/8/9/10/11/12 solo tienen
             * sentido con el motor ya operando, así que no exigen
             * detenerlo para entrar. 4/5 también mueven el servo directo,
             * pero SÍ miran la RPM en cada paso (a diferencia de 1/2) --
             * por eso pueden permitir entrar sin el motor operando (la
             * rutina en main.c simplemente espera a que arranque antes de
             * mover nada). */
            uint8_t nuevoModo = datos[1];
            uint8_t modoActual = CalibFlash_GetCalib();

            if ((nuevoModo == 1U || nuevoModo == 2U) && motorOperando) {
                *valorAplicadoRaw = (uint16_t)modoActual;
                return CALIB_STATUS_REJECTED_ENGINE_RUNNING;
            }

            /* Cualquier CALIB != 0 exige MODO=CALIBRACION (4)
             * ya puesto -- SIN EXCEPCIONES, ni siquiera CALIB=10. Agregado
             * 2026-09-24, decisión explícita del usuario, VERSIÓN FINAL
             * tras varios intentos previos en la misma sesión que se
             * revirtieron por romper flujos reales (ver historial en la
             * memoria del proyecto si hace falta el detalle). La solución
             * de fondo no era parchear el candado -- era que MODO=3
             * (MANUAL_BANCO) hacía DOBLE función (uso operativo real de
             * campo, ej. trasladar manguera/aspersor, Y modo de banco para
             * calibrar) sin forma de distinguir una cosa de la otra.
             * CALIB_MODO_CALIBRACION (4, ver calibracion_flash.h) separa
             * esa ambigüedad: es un MODO EXCLUSIVO de sesión de banco/
             * ingeniería, nunca usado en operación real -- MANUAL_BANCO
             * sigue existiendo tal cual para el caso de campo.
             *
             * CALIB=10 (auto-escalón de presión LOCAL) originalmente
             * parecía necesitar una excepción para aceptar MODO=1
             * (necesita el lazo de presión real corriendo para poder
             * meterle un escalón) -- en vez de eso, el lazo en cascada de
             * main.c se ensanchó para tratar (MODO=CALIBRACION +
             * CALIB=10) igual que MODO=1 real (ver
             * `presionLocalActivoAhora` en main.c) -- así CALIB=10
             * también corre EXCLUSIVAMENTE bajo MODO=4, sin necesitar
             * ninguna excepción acá. Regla final, una sola línea, sin
             * casos especiales:
             *
             * Se apoya en el forzado automático de MODO=0 al detenerse el
             * motor (ver main.c) -- detener el motor (o recién haber
             * arrancado el TID) deja el camino libre para pasar a
             * MODO=CALIBRACION y arrancar cualquier sesión.
             * CALIB=0 (salir) siempre se acepta, sin importar
             * MODO. */
            if (nuevoModo != 0U && CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
                *valorAplicadoRaw = (uint16_t)modoActual;
                return CALIB_STATUS_APPLY_ERROR; /* valor valido, pero MODO actual no lo permite -- no es un problema de rango */
            }

            /* Modos 3 a 12 (rango contiguo desde la renumeración
             * 2026-09-29) solo se pueden pedir viniendo de modo 0 -- no
             * directo desde 1/2 (ni entre sí), para no arrancar una
             * ventana de calibración a mitad de otra sesión. Ninguno de
             * estos exige el motor operando para ACEPTAR el downlink (a
             * diferencia de 1/2) -- si se activa sin el motor corriendo,
             * la rutina en main.c simplemente espera a que arranque antes
             * de medir/mover el servo (o, para 6/7/9/10/11/12, antes de
             * mandar el escalón/barrido). */
            if (nuevoModo >= 3U && nuevoModo <= 12U && modoActual != 0U) {
                *valorAplicadoRaw = (uint16_t)modoActual;
                return CALIB_STATUS_APPLY_ERROR; /* valor valido, pero ya hay otro CALIB activo -- no es un problema de rango */
            }

            bool ok = CalibFlash_SetCalib(nuevoModo);
            /* Medida de seguridad: al ENTRAR a cualquier modo de
             * calibración (1-12) se limpia SET_RPM a su "sin comandar"
             * (0.0f) -- así el operador siempre tiene que volver a pedir
             * un setpoint explícito para esa sesión, en vez de que un
             * SET_RPM que haya quedado de una operación anterior active
             * el PID solo al entrar a modo 12 (en el resto no tiene
             * efecto directo, pero se limpia igual por consistencia;
             * modo 10 se pisa solo con su propio escalón de todas
             * formas). No aplica al volver a 0 -- ahí no hay nada que
             * "limpiar hacia atrás". */
            if (ok && nuevoModo != 0U) {
                CalibFlash_SetSetRpm(0.0f);
            }
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetCalib();
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
            CalibFlash_Modo_t modoAnterior = CalibFlash_GetModo();
            bool ok = CalibFlash_SetModo((CalibFlash_Modo_t)modoSolicitado);
            /* SET_PRESION fresco al ENTRAR a MODO=3 -- agregado
             * 2026-09-25, pedido explicito del usuario: evita que quede
             * activando la cascada de presion con un valor de una sesion
             * anterior sin que el operador lo haya vuelto a pedir para
             * esta. Solo en la TRANSICION (no en cada downlink MODO=3
             * redundante mientras ya se esta en MODO=3, para no cortar
             * una sesion de SET_PRESION activa sin motivo). */
            if (ok && modoSolicitado == (uint8_t)CALIB_MODO_MANUAL_BANCO
                && modoAnterior != CALIB_MODO_MANUAL_BANCO) {
                CalibFlash_SetSetPresion(0.0f);
            }
            *valorAplicadoRaw = (uint16_t)CalibFlash_GetModo();
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PRESION_REMOTO: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            CalibFlash_SetPresionRemoto(nuevoValor);
            /* Marca esta PRESION_REMOTO como "fresca" -- consumido por el
             * watchdog de TIMEOUT_SIN_COMANDO_S en MODO_REMOTO (ver
             * main.c). Se actualiza aca (no en CalibFlash_SetPresionRemoto())
             * para que solo cuente como "info fresca del aspersor" un
             * downlink/comando real de PRESION_REMOTO, no cualquier llamada
             * interna al setter. */
            s_presionRemotoUltimoTickMs = HAL_GetTick();
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionRemoto(), 10.0f);
            return CALIB_STATUS_OK;
        }

        case CALIB_ID_PID_ASP_KP: {
            /* Escala x1000 (subida de x100 2026-09-24, mismo motivo que
             * CALIB_ID_PID_RPM_KP: consistencia con las demas ganancias P y
             * margen para valores de 3 decimales, aunque los Kp de
             * presion validados hasta ahora casualmente entraban en 2
             * decimales). Con signo. ⚠️ CALIB=10 [numero viejo] ELIMINADO
             * 2026-09-29 -- ver nota completa junto a CALIB_ID_PID_RPM_KP.
             * Solo se acepta con MODO=CALIBRACION y ningun CALIB activo
             * (CALIB=0). */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                || CalibFlash_GetCalib() != 0U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidAspKp(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidAspKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidAspKp(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_ASP_KI: {
            /* Escala x1000, con signo -- mismo formato que
             * CALIB_ID_PID_PSI_KI (lazo local), da mas resolucion
             * para valores de Ki tipicamente chicos. Ver nota de
             * CALIB_ID_PID_ASP_KP -- mismo gate. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                || CalibFlash_GetCalib() != 0U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidAspKi(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidAspKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidAspKi(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_PSI_KP: {
            /* Escala x1000 (subida de x100 2026-09-24, ver nota de
             * CALIB_ID_PID_ASP_KP arriba). Con signo. Solo se
             * acepta con MODO=CALIBRACION y ningun CALIB activo (ver nota
             * completa junto a CALIB_ID_PID_RPM_KP). */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                || CalibFlash_GetCalib() != 0U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidPsiKp(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidPsiKp(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidPsiKp(), 1000.0f);
            if (ok) return CALIB_STATUS_OK;
            return s_ultimaEscrituraFallo ? CALIB_STATUS_STORAGE_ERROR : CALIB_STATUS_OUT_OF_RANGE;
        }

        case CALIB_ID_PID_PSI_KI: {
            /* Escala x1000, con signo. Ver nota de CALIB_ID_PID_RPM_KI --
             * mismo patron exacto: solo se acepta con MODO=CALIBRACION
             * y ningun CALIB activo. */
            if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
                || CalibFlash_GetCalib() != 0U) {
                *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidPsiKi(), 1000.0f);
                return CALIB_STATUS_APPLY_ERROR;
            }
            float nuevoValor = (float)LeerInt16BigEndian(datos) / 1000.0f;
            bool ok = CalibFlash_SetPidPsiKi(nuevoValor);
            *valorAplicadoRaw = CodificarInt16ComoRaw(CalibFlash_GetPidPsiKi(), 1000.0f);
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

        case CALIB_ID_PRESION_OBJETIVO_LOCAL: {
            float nuevoValor = (float)LeerUint16BigEndian(datos) / 10.0f;
            bool ok = CalibFlash_SetPresionObjetivoLocal(nuevoValor);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetPresionObjetivoLocal(), 10.0f);
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

        case CALIB_ID_TIEMPO_LLENADO_S: {
            /* Conveniencia sobre TASA_LLENADO_PSI_S (agregado
             * 2026-09-28, mismo patron que SET_RATIO_AUTO) -- el
             * operador manda cuantos segundos quiere que dure el
             * llenado, el firmware calcula la tasa (PSI/s) a partir de
             * la presion actual en vivo (presionActual, pasada desde el
             * llamador) y PRESION_OBJETIVO_LOCAL ya configurado. Sin
             * PRESION_OBJETIVO_LOCAL > presionActual no hay nada que
             * calcular (rechaza OUT_OF_RANGE). No exige MODO=CALIBRACION
             * -- misma categoria PROCESO que TASA_LLENADO_PSI_S. */
            uint16_t tiempoSegundos = LeerUint16BigEndian(datos);
            float deltaPsi = CalibFlash_GetPresionObjetivoLocal() - presionActual;
            if (tiempoSegundos == 0U || deltaPsi <= 0.0f) {
                *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaLlenadoPsiS(), 100.0f);
                return CALIB_STATUS_OUT_OF_RANGE;
            }
            float tasaCalculada = deltaPsi / (float)tiempoSegundos;
            /* Guarda tambien el crudo en segundos (agregado 2026-09-28) --
             * CALIB=9 lo reusa directo como su propio techo
             * de tiempo (ver main.c), decision explicita del usuario: el
             * mismo numero que el operador ya dio para el llenado real de
             * MODO=1 aplica igual al barrido de ganancia. Se asigna ANTES
             * de SetTasaLlenadoPsiS() para que quede incluido en la misma
             * escritura a flash (CalibFlash_EscribirEnFlash() persiste
             * TODO s_datos de una vez, no campo por campo). */
            s_datos.tiempoLlenadoS = tiempoSegundos;
            bool okTiempo = CalibFlash_SetTasaLlenadoPsiS(tasaCalculada);
            *valorAplicadoRaw = CodificarUint16(CalibFlash_GetTasaLlenadoPsiS(), 100.0f);
            if (okTiempo) return CALIB_STATUS_OK;
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

bool CalibFlash_ProcesarParametro(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, float frecuenciaHzActual, float presionActual)
{
    uint16_t valorAplicado;
    CalibFlash_ProtocoloStatus_t status = CalibFlash_ProcesarParametroConEstado(id, datos, longitudDatos, motorOperando, frecuenciaHzActual, presionActual, &valorAplicado);
    return (status == CALIB_STATUS_OK);
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
uint32_t CalibFlash_GetTimeoutSinComandoS(void)       { return s_datos.timeoutSinComandoS; }
float    CalibFlash_GetTasaMaxCambioRpmS(void)        { return s_datos.tasaMaxCambioRpmS; }
uint8_t  CalibFlash_GetCalib(void)                    { return s_datos.calib; }
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void) { return s_datos.intervaloOperativoS; }
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void)   { return s_datos.intervaloStandbyS; }
CalibFlash_Modo_t CalibFlash_GetModo(void)            { return (CalibFlash_Modo_t)s_datos.modo; }
uint8_t  CalibFlash_GetNodeId(void)                   { return s_datos.nodeId; }
uint16_t CalibFlash_GetHisteresisModoS(void)          { return s_datos.histeresisModoS; }
uint16_t CalibFlash_GetTiempoLlenadoS(void)           { return s_datos.tiempoLlenadoS; }
float    CalibFlash_GetPresionObjetivoLocal(void)     { return s_datos.presionObjetivoLocal; }
float    CalibFlash_GetPresionObjetivoRemoto(void)    { return s_datos.presionObjetivoRemoto; }
float    CalibFlash_GetTasaLlenadoPsiS(void)          { return s_datos.tasaLlenadoPsiS; }
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void) { return s_datos.tasaMaxCambioRpmLlenadoS; }
float    CalibFlash_GetPidAspKp(void)                 { return s_datos.pidAspKp; }
float    CalibFlash_GetPidAspKi(void)                 { return s_datos.pidAspKi; }
float    CalibFlash_GetPidPsiKp(void)                 { return s_datos.pidPsiKp; }
float    CalibFlash_GetPidPsiKi(void)                 { return s_datos.pidPsiKi; }
uint32_t CalibFlash_GetPresionRemotoUltimoTickMs(void) { return s_presionRemotoUltimoTickMs; }

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

bool CalibFlash_SetRpmMaxCarga(float v)
{
    /* SIEMPRE <= rpmMax -- agregado 2026-09-28, decision explicita del
     * usuario: RPM_MAX_CARGA es una restriccion ADICIONAL sobre el techo
     * mecanico del motor (RPM_MAX), especifica de la instalacion (limite
     * hidraulico, ej. no reventar tuberia) -- no tendria sentido que
     * fuera mayor. Mismo piso que RPM_MAX (> rpmMin). */
    if (v <= s_datos.rpmMin || v > s_datos.rpmMax) return false;
    s_datos.rpmMaxCarga = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionMax(float v)
{
    /* No puede quedar por debajo de PRESION_OBJETIVO_LOCAL ni de
     * SET_PRESION vigente (mismo invariante, por el otro lado). */
    if (v <= 0.0f || v > RANGO_PRESION_MAX
            || v <= s_datos.presionObjetivoLocal
            || v <= CalibFlash_GetSetPresion()) return false;
    s_datos.presionMax = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetRpmMin(float v)
{
    if (v < 0.0f || v >= s_datos.rpmMax) return false;
    s_datos.rpmMin = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidRpmKp(float v)
{
    s_datos.pidRpmKp = v; /* ganancias PID: se permite cualquier valor con signo,
                         * la sintonización decide qué es razonable */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidRpmKi(float v)
{
    s_datos.pidRpmKi = v;
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

bool CalibFlash_SetCalib(uint8_t modo)
{
    if (modo > 12U) return false; /* ⚠️ RENUMERADO 2026-09-29 (tercera
                                   * vez, ver README sección 4.4/12),
                                   * decisión explícita del usuario para
                                   * que el número siga el orden real de
                                   * comisionamiento en campo. Rango
                                   * ahora CONTIGUO 0-12 (el hueco del
                                   * viejo CH=10, sintonización manual,
                                   * eliminado el mismo día -- exigir un
                                   * CALIB separado solo para poder
                                   * tocar ganancias iba contra el
                                   * espiritu de estas rutinas, que
                                   * existen para que el operador NO
                                   * tenga que saltar de una a otra -- el
                                   * candado real ahora es
                                   * MODO=CALIBRACION + CALIB=0, ver
                                   * CALIB_ID_PID_RPM_KP/KI y demas ganancias
                                   * en CalibFlash_ProcesarParametroConEstado.
                                   * Ya no queda excluido explícitamente,
                                   * simplemente no existe ningún número
                                   * que mapee a él):
                                   * 0=desactivado, 1=manual, 2=barrido,
                                   * 3=calibracion de ALPHA, 4=calibracion
                                   * de SERVO_PULSO_MIN (zona muerta),
                                   * 5=mapeo de curva de ganancia (PID#1,
                                   * lazo abierto), 6=auto-escalon
                                   * interno (PID#1), 7=validacion de
                                   * ganancias reales del lazo interno
                                   * (PID#1, con watchdog de oscilacion
                                   * sostenida -- antes 12), 8=calibracion
                                   * de ralenti (RPM_MIN -- antes 7),
                                   * 9=barrido de ganancia de presion
                                   * local (PID#2, lazo abierto -- antes
                                   * 11), 10=auto-escalon presion local
                                   * (PID#2 -- antes 8), 11=validacion de
                                   * ganancias reales del lazo de presion
                                   * local (PID#2, mismo criterio que 7 --
                                   * antes 13), 12=auto-escalon remoto
                                   * (PID#3 -- antes 9) */
    s_datos.calib = modo;
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
    /* CALIB_MODO_CALIBRACION (4) agregado 2026-09-24 -- ver nota completa
     * junto al enum en calibracion_flash.h. */
    if (v != CALIB_MODO_RALENTI && v != CALIB_MODO_PRESION_LOCAL
        && v != CALIB_MODO_REMOTO && v != CALIB_MODO_MANUAL_BANCO
        && v != CALIB_MODO_CALIBRACION) {
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

bool CalibFlash_SetPresionObjetivoLocal(float v)
{
    /* SIEMPRE < PRESION_MAX (agregado 2026-09-29). */
    if (v <= 0.0f || v > RANGO_PRESION_MAX || v >= s_datos.presionMax) return false;
    s_datos.presionObjetivoLocal = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPresionObjetivoRemoto(float v)
{
    /* A diferencia de PRESION_OBJETIVO_LOCAL, 0.0f SI es valido aca --
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

bool CalibFlash_SetPidAspKp(float v)
{
    s_datos.pidAspKp = v; /* ganancias PID: se permite cualquier valor con
                             * signo, la sintonizacion decide que es razonable
                             * (mismo criterio que CalibFlash_SetPidPsiKp()) */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidAspKi(float v)
{
    s_datos.pidAspKi = v;
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidPsiKp(float v)
{
    s_datos.pidPsiKp = v; /* ganancias PID: se permite cualquier valor con
                             * signo, la sintonizacion decide que es razonable
                             * (mismo criterio que CalibFlash_SetPidRpmKp()) */
    return CalibFlash_EscribirEnFlash();
}

bool CalibFlash_SetPidPsiKi(float v)
{
    s_datos.pidPsiKi = v;
    return CalibFlash_EscribirEnFlash();
}

/* ==================== NO PERSISTENTES ==================== */

float CalibFlash_GetSetRpm(void)  { return s_setRpm; }
void  CalibFlash_SetSetRpm(float v) { s_setRpm = v; }

static float s_setPresion = 0.0f;
float CalibFlash_GetSetPresion(void)  { return s_setPresion; }
void  CalibFlash_SetSetPresion(float v) { s_setPresion = v; }

float CalibFlash_GetPresionRemoto(void) { return s_presionRemoto; }
void  CalibFlash_SetPresionRemoto(float v) { s_presionRemoto = v; }

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
        case CALIB_ID_RPM_MAX_CARGA:
            return CodificarUint16(CalibFlash_GetRpmMaxCarga(), 10.0f);
        case CALIB_ID_PRESION_MAX:
            return CodificarUint16(CalibFlash_GetPresionMax(), 10.0f);
        case CALIB_ID_RPM_MIN:
            return CodificarUint16(CalibFlash_GetRpmMin(), 10.0f);
        case CALIB_ID_PID_RPM_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPidRpmKp(), 1000.0f);
        case CALIB_ID_PID_RPM_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPidRpmKi(), 1000.0f);
        case CALIB_ID_SERVO_PULSO_MIN:
            return CalibFlash_GetServoPulsoMinUs();
        case CALIB_ID_SERVO_PULSO_MAX:
            return CalibFlash_GetServoPulsoMaxUs();
        case CALIB_ID_TIMEOUT_SIN_COMANDO_S:
            return (uint16_t)CalibFlash_GetTimeoutSinComandoS();
        case CALIB_ID_TASA_MAX_CAMBIO_RPM_S:
            return CodificarUint16(CalibFlash_GetTasaMaxCambioRpmS(), 10.0f);
        case CALIB_ID_CALIB:
            return (uint16_t)CalibFlash_GetCalib();
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
        case CALIB_ID_PRESION_OBJETIVO_LOCAL:
            return CodificarUint16(CalibFlash_GetPresionObjetivoLocal(), 10.0f);
        case CALIB_ID_PRESION_OBJETIVO_REMOTO:
            return CodificarUint16(CalibFlash_GetPresionObjetivoRemoto(), 10.0f);
        case CALIB_ID_TASA_LLENADO_PSI_S:
        case CALIB_ID_TIEMPO_LLENADO_S:
            return CodificarUint16(CalibFlash_GetTasaLlenadoPsiS(), 100.0f);
        case CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S:
            return CodificarUint16(CalibFlash_GetTasaMaxCambioRpmLlenadoS(), 10.0f);
        case CALIB_ID_PID_ASP_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPidAspKp(), 1000.0f);
        case CALIB_ID_PID_ASP_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPidAspKi(), 1000.0f);
        case CALIB_ID_PID_PSI_KP:
            return CodificarInt16ComoRaw(CalibFlash_GetPidPsiKp(), 1000.0f);
        case CALIB_ID_PID_PSI_KI:
            return CodificarInt16ComoRaw(CalibFlash_GetPidPsiKi(), 1000.0f);
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
