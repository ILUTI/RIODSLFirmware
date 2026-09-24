/**
 * @file    calibracion_flash.h
 * @brief   Almacena en la última página de flash del G431 los
 *          parámetros de configuración remota del nodo motor
 *          (ver "Tabla de Parametros de Configuracion Remota"),
 *          y centraliza el punto de entrada para aplicar cualquier
 *          parámetro recibido por downlink (LoRaWAN, sin importar si
 *          viene de Chirpstack o de una Lambda en AWS IoT Core).
 *
 * Categorías de parámetros:
 *   - PERSISTENTES: se guardan en flash, sobreviven resets. Ej.
 *     SET_RATIO, RPM_MAX, PID_KP, etc.
 *   - NO PERSISTENTES (solo RAM): SET_RPM y PRESION -- llegan con
 *     frecuencia y no vale la pena desgastar flash con ellos.
 *   - COMANDOS: RESTAURAR_DEFAULTS, FORZAR_REPORTE, RESET_REMOTO --
 *     no son valores que se guardan, son acciones que se ejecutan al
 *     recibir el downlink. Requieren un byte de confirmación
 *     específico (CALIB_BYTE_CONFIRMACION) para reducir el riesgo de
 *     ejecución accidental por un downlink corrupto.
 *
 * Uso típico en main.c:
 *   CalibFlash_Init();                          // antes de todo lo demás
 *   Tacometro_Init(&htim2, TIM_CHANNEL_1);
 *   ...
 *   while (1) {
 *       if (CalibFlash_HayReporteForzado()) {
 *           ... mandar uplink inmediato ...
 *           CalibFlash_LimpiarReporteForzado();
 *       }
 *       if (CalibFlash_HayResetPendiente()) {
 *           HAL_Delay(100); // dar tiempo a que salga cualquier log pendiente
 *           NVIC_SystemReset();
 *       }
 *   }
 *
 * Y en rak3172.c, al procesar un downlink ya separado en ID + bytes:
 *   CalibFlash_ProcesarParametro(idRecibido, payloadBytes, longitudPayload);
 *
 * ⚠️ RECONSTRUIDO 2026-09-24: este archivo se perdió (revertido a una
 * versión vieja de git, probablemente por un "Discard Changes" desde
 * el IDE que afecto a todo Core/Inc header) mientras el resto del
 * proyecto (los .c, README.md) seguía con todo el trabajo del
 * 2026-09-23 intacto. Reconstruido cruzando cada ID/función contra
 * Core/Src/calibracion_flash.c (que sí quedó intacto) y contra el
 * historial completo de esta conversación -- no es el texto original
 * exacto (comentarios/redacción pueden variar), pero es funcionalmente
 * equivalente y fue verificado compilando contra el .c real.
 */

#ifndef CALIBRACION_FLASH_H
#define CALIBRACION_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==================== IDs DE PARÁMETROS (ver tabla Excel) ==================== */

#define CALIB_ID_SET_RATIO                       1U
#define CALIB_ID_ALPHA                            2U
#define CALIB_ID_SET_RPM                          3U
#define CALIB_ID_RPM_MAX                          4U
#define CALIB_ID_RPM_MIN                          5U
#define CALIB_ID_PID_KP                           6U
#define CALIB_ID_PID_KI                           7U
/* ID 8 era PID_KD (termino derivativo del lazo interno RPM->servo) --
 * ELIMINADO 2026-09-23, decision explicita del usuario: confirmado en
 * campo que Kd=0 funciona bien (README seccion 9), sin indicio de que
 * haga falta un termino derivativo ahi -- se quito el codigo (pid.c ya
 * no calcula derivada/error anterior) en vez de dejarlo calibrado en
 * 0 sin uso. Se reasigna a TASA_LLENADO_PSI_S (antes ID 31, agregada
 * horas antes en esa misma sesion, todavia sin desplegar en campo)
 * para no dejar un hueco bajo en la numeracion.
 *
 * TASA_LLENADO_PSI_S: tasa de subida (PSI/segundo) de la RAMPA DE
 * LLENADO de MODO=1. En vez de saltar directo a PRESION_OBJETIVO al
 * entrar a MODO=1, el setpoint que recibe presion_pid.c sube
 * gradualmente desde la presion medida AL MOMENTO DE ENTRAR (no desde
 * un valor fijo asumido -- si la tuberia ya estaba parcialmente llena
 * de una sesion anterior, tarda menos sola, sin logica especial) hasta
 * PRESION_OBJETIVO, a esta tasa. Pensada para no forzar la
 * tuberia/purgar aire de golpe. No hace falta una bandera de "fase"
 * separada: el setpoint rampa es min(PRESION_OBJETIVO, presion_inicio +
 * tasa*segundos) -- en cuanto la rampa alcanza el objetivo final, se
 * queda pegada ahi sola (el min() la recorta), asi que "en rampa" vs
 * "estable" es implicito, no un estado explicito. Default 0.0f = "sin
 * rampa, ir directo al objetivo" (comportamiento identico al que ya
 * existia antes de esto -- retrocompatible sin necesidad de configurar
 * nada en unidades que no usen esta funcion). */
#define CALIB_ID_TASA_LLENADO_PSI_S                8U
#define CALIB_ID_SERVO_PULSO_MIN                  9U
#define CALIB_ID_SERVO_PULSO_MAX                  10U
#define CALIB_ID_TIMEOUT_SIN_COMANDO_S             11U
#define CALIB_ID_TASA_MAX_CAMBIO_RPM_S             12U
#define CALIB_ID_CONTROL_HABILITADO                13U
#define CALIB_ID_INTERVALO_ENVIO_OPERATIVO_S       14U
#define CALIB_ID_INTERVALO_ENVIO_STANDBY_S         15U
#define CALIB_ID_MODO                              16U
#define CALIB_ID_PRESION                           17U
#define CALIB_ID_NODE_ID                           18U
#define CALIB_ID_RESTAURAR_DEFAULTS                19U
#define CALIB_ID_FORZAR_REPORTE                    20U
#define CALIB_ID_HISTERESIS_MODO_S                 21U
#define CALIB_ID_RESET_REMOTO                      22U
#define CALIB_ID_PRESION_OBJETIVO                  23U
#define CALIB_ID_TASA_MAX_CAMBIO_RPM_LLENADO_S     24U
#define CALIB_ID_SET_RATIO_AUTO                   25U
/* IDs 26-29 quedaron libres al quitar la correccion geometrica del
 * mecanismo biela-manivela (MECANISMO_MANIVELA_CM/VARILLA_CM/
 * OFFSET_GRADOS/CORRECCION_ACTIVA, 2026-09-11, branch servo-directo --
 * ver README seccion 12). PRESION_GANANCIA_RPM/OFFSET_RPM (antes 30/31)
 * se corrieron a 26/27 para ocupar ese hueco, para la formula lineal
 * abierta que tenia entonces MODO_REMOTO. 28-29 se usaron 2026-09-14
 * para las ganancias del lazo EXTERNO de presion local (cascada, MODO=1).
 *
 * ⚠️ REASIGNADOS 2026-09-23 (decision explicita del usuario): MODO_REMOTO
 * (MODO=2) se rediseño de formula lineal abierta a un 3er PID en
 * cascada, evento-driven (ver presion_pid_remoto.h) -- para no exigir
 * calibrar PRESION_GANANCIA_RPM/OFFSET_RPM en cada instalacion (la
 * relacion RPM->presion remota depende de la tuberia/bomba/aspersor
 * fisicos de cada sitio; un PID se autoajusta sin conocerla de
 * antemano). La formula lineal que los consumia ya no existe, asi que
 * 26/27 se reusan para las ganancias de ese PID nuevo (mismo patron que
 * 28/29 para el lazo local). Como este parametro solo corrio hasta
 * ahora en la unidad de banco (sin flota desplegada todavia), no hace
 * falta coordinar migracion con nodos en campo -- pero si esto cambia,
 * verificar que ningun nodo en campo siga mandando 26/27 con el
 * significado viejo (GANANCIA_RPM/OFFSET_RPM) antes de reflashear. */
#define CALIB_ID_PRESION_REMOTO_PID_KP              26U
#define CALIB_ID_PRESION_REMOTO_PID_KI              27U
#define CALIB_ID_PRESION_PID_KP                    28U
#define CALIB_ID_PRESION_PID_KI                    29U

/* ID 30 quedo libre por la misma razon que 26-29 arriba (antes tambien
 * fue "presion", significado distinto, ver nota de arriba) -- reasignado
 * 2026-09-23 a PRESION_OBJETIVO_REMOTO: el valor de presion que SE
 * ESPERA que reporte el aspersor remoto en MODO=2, configurado en ESTE
 * TID (no confundir con PRESION, ID 17, que es la lectura real que
 * manda el aspersor por downlink).
 *
 * ⚠️ CAMBIO DE ROL 2026-09-23 (mismo dia que el PID en cascada de
 * MODO_REMOTO, ver nota de CALIB_ID_PRESION_REMOTO_PID_KP arriba):
 * hasta ese cambio, este valor SOLO alimentaba el supervisor de "MODO=2
 * no alcanza el objetivo". Ahora TAMBIEN es el setpoint real que recibe
 * PresionPidRemoto_CalcularSetpointRpm() -- dejo de ser un dato
 * "opcional/informativo" para el supervisor y paso a ser una entrada
 * que gobierna directamente el control. Default sigue en 0.0f, pero con
 * este cambio de rol eso ya no es un "sin configurar" inofensivo: con
 * objetivo=0, el PID persigue 0 PSI (empuja hacia ralenti la mayor
 * parte del tiempo) en vez de simplemente relajar el criterio del
 * supervisor como antes. Configurar este valor antes de operar en
 * MODO=2 pasa a ser mas importante que antes. */
#define CALIB_ID_PRESION_OBJETIVO_REMOTO            30U

/* ID 31 quedo libre -- TASA_LLENADO_PSI_S se movio a ID 8 (arriba,
 * ocupando el hueco que dejo PID_KD al eliminarse) para no dejar un
 * hueco bajo en la numeracion. Disponible para un futuro parametro. */

/* Byte de confirmación requerido para ejecutar los comandos críticos
 * (RESTAURAR_DEFAULTS, RESET_REMOTO). Cambiar aquí si se requiere
 * otro valor -- debe coincidir con lo que mande la Lambda/Chirpstack. */
#define CALIB_BYTE_CONFIRMACION   0xA5U

/* Categorías de parámetro, para decidir si se permite cambiar mientras
 * el motor está operando (ver CalibFlash_ProcesarParametroConEstado):
 *   CALIBRACION    -- siempre permitido, incluso operando (SET_RATIO,
 *                      ALPHA -- se necesitan ajustar en caliente para
 *                      calibrar contra un tacómetro externo real; y
 *                      PID_KP/KI -- la sintonización en lazo cerrado,
 *                      README sección 9, exige subir Kp de a poco CON
 *                      el motor operando, observando la respuesta real).
 *   CONFIGURACION  -- solo permitido con el motor detenido (límites de
 *                      seguridad, límites del servo, etc.) -- cambiarlos
 *                      en caliente podría causar un comportamiento
 *                      impredecible del lazo de control.
 *   PROCESO        -- siempre permitido, es su función normal (SET_RPM,
 *                      PRESION llegan constantemente mientras opera).
 *   COMANDO        -- casos especiales (RESTAURAR_DEFAULTS, etc.), se
 *                      evalúan aparte, no bloqueados por esta regla.
 */
typedef enum {
    CALIB_CATEGORIA_CALIBRACION = 0,
    CALIB_CATEGORIA_CONFIGURACION = 1,
    CALIB_CATEGORIA_PROCESO = 2,
    CALIB_CATEGORIA_COMANDO = 3
} CalibFlash_Categoria_t;

/* Valores válidos de MODO -- gobiernan de dónde sale el setpoint de RPM
 * que usa pid.c (ver main.c, bloque "MODO: fuente del setpoint de RPM").
 * Nombres corregidos 2026-09-07 (antes CALIB_MODO_MANUAL/AUTOMATICO/
 * MANTENIMIENTO, que no coincidían con la terminología real del README
 * ni de la tabla de parámetros -- mismos valores numéricos, solo se
 * renombró el enum al conectarlo al lazo de control real).
 *
 * ⚠️ RENUMERADO 2026-09-14 (branch main, ver README sección 12): el
 * valor 1 pasó de "SET_RPM directo (uso de banco/sintonización)" a
 * "presión local" -- se ajustó para que coincida con la terminología
 * que ya usa el cliente en campo (0=ralentí, 1=presión LOCAL con el
 * sensor propio del TID, 2=presión REMOTA del nodo aspersor). El modo
 * de SET_RPM directo (el que se usó TODA la sesión de sintonización del
 * PID interno, Kp=0.047/Ki=0.11) se corrió al valor 3
 * (CALIB_MODO_MANUAL_BANCO). Como este parámetro solo corrió hasta
 * ahora en la unidad de banco (sin flota desplegada todavía), no hace
 * falta coordinar migración con nodos en campo -- pero si esto cambia,
 * cualquier unidad que haya quedado guardada en MODO=1 con el
 * significado VIEJO (SET_RPM directo) va a pasar a intentar control por
 * presión local en cuanto se actualice el firmware. Verificar/reasignar
 * MODO explícitamente en cada unidad antes de reflashear en el futuro. */
typedef enum {
    CALIB_MODO_RALENTI       = 0,  /* sin control activo, sin importar SET_RPM/PRESION/presión local */
    CALIB_MODO_PRESION_LOCAL = 1,  /* setpoint = salida de PresionPid_CalcularSetpointRpm() contra
                                     * PRESION_OBJETIVO, usando el sensor de presión propio del TID
                                     * (hoy PresionV_GetPresionPsi(), temporal -- ver presion_voltaje.h).
                                     * Llena la tubería/gobierna presión de salida de la motobomba. */
    CALIB_MODO_REMOTO        = 2,  /* setpoint = salida de PresionPidRemoto_CalcularSetpointRpm() contra
                                     * PRESION_OBJETIVO_REMOTO, con PRESION recibida por downlink de un
                                     * nodo aspersor remoto (no del sensor local -- ver
                                     * CalibFlash_SetPresion()). PID en cascada evento-driven, agregado
                                     * 2026-09-23 en reemplazo de la formula lineal abierta que tenia
                                     * antes (ver presion_pid_remoto.h). */
    CALIB_MODO_MANUAL_BANCO  = 3   /* setpoint = SET_RPM (downlink/serial directo) -- uso de banco:
                                     * sintonización del PID interno, pruebas sin presión de bombeo
                                     * real (antes era el valor 1, ver nota de renumeración arriba). */
} CalibFlash_Modo_t;

/* Códigos de STATUS del protocolo Quick-Set (ver Tabla 2 acordada con
 * el equipo). Se devuelven en el ACK de aplicación, junto con el
 * valor que quedó realmente vigente (no necesariamente el solicitado,
 * si fue rechazado). */
typedef enum {
    CALIB_STATUS_OK                     = 0,
    CALIB_STATUS_OUT_OF_RANGE           = 1,
    CALIB_STATUS_UNKNOWN_PARAMETER_ID   = 2,
    CALIB_STATUS_STORAGE_ERROR          = 3,
    CALIB_STATUS_APPLY_ERROR            = 4,  /* ej. SET_RPM con MODO!=3
                                                 * (comando valido pero sin
                                                 * efecto en el estado actual) */
    CALIB_STATUS_REJECTED_ENGINE_RUNNING  = 5, /* motor operando -- parametro
                                                 * de configuracion/seguridad
                                                 * no se puede cambiar hasta
                                                 * que el motor se detenga */
    CALIB_STATUS_REJECTED_ENGINE_STOPPED  = 6, /* motor DETENIDO -- agregado
                                                 * 2026-09-22, decision
                                                 * explicita del usuario: no
                                                 * se puede pasar MODO a un
                                                 * modo de control automatico
                                                 * (1/2) con el motor apagado.
                                                 * Motivo: si MODO=1/2 queda
                                                 * "armado" desde antes de
                                                 * arrancar, el PID de presion
                                                 * entra a controlar desde el
                                                 * primer instante que el
                                                 * motor arranca, sin que el
                                                 * operador haya podido
                                                 * confirmar a mano que la
                                                 * tuberia ya esta llena (aire
                                                 * adentro -> el PID pediria
                                                 * RPM alta persiguiendo una
                                                 * lectura de presion irreal,
                                                 * mismo riesgo que ya se
                                                 * resolvio para el arranque
                                                 * en CalibFlash_Init()). */
    CALIB_STATUS_REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO = 7 /* agregado
                                                 * 2026-09-23, decision
                                                 * explicita del usuario: no
                                                 * se puede pasar MODO a 2
                                                 * (Remoto) con
                                                 * PRESION_OBJETIVO_REMOTO en
                                                 * 0 (sin configurar). No es
                                                 * un riesgo de seguridad
                                                 * como REJECTED_ENGINE_STOPPED
                                                 * (el PID persigue 0 PSI,
                                                 * el motor cae solo cerca de
                                                 * ralenti, nunca se satura
                                                 * en RPM_MAX) -- es para
                                                 * evitar que MODO=2 quede
                                                 * "andando" sin hacer nada
                                                 * util, en silencio, sin que
                                                 * el supervisor de abajo
                                                 * llegue a dispararse nunca
                                                 * (nunca se satura en
                                                 * RPM_MAX si el objetivo es
                                                 * 0). Ver presion_pid_remoto.h. */
} CalibFlash_ProtocoloStatus_t;

/* ==================== API DE INICIALIZACIÓN ==================== */

/** Carga la calibración guardada en flash, o valores por defecto si
 * no hay ninguna guardada todavía (primera vez que corre el firmware
 * en este chip, o flash corrupta/borrada). */
void CalibFlash_Init(void);

/* ==================== PUNTO DE ENTRADA CENTRALIZADO PARA DOWNLINKS ==================== */

/**
 * Versión detallada de CalibFlash_ProcesarParametro(), pensada para
 * armar el Application ACK del protocolo Quick-Set (4 bytes:
 * [PARAMETER_ID][STATUS][VALUE_H][VALUE_L]).
 *
 * A diferencia de la versión simple (bool), esta:
 *   - Devuelve el STATUS exacto (ver CalibFlash_ProtocoloStatus_t).
 *   - Entrega en 'valorAplicadoRaw' el valor de 2 bytes que quedó
 *     REALMENTE vigente tras procesar el downlink -- si se rechazó,
 *     es el valor anterior (el que ya tenía); si se aplicó, es el
 *     nuevo valor. Ya viene codificado en la misma escala/formato de
 *     2 bytes que usa el protocolo (big-endian, listo para partir en
 *     VALUE_H/VALUE_L).
 *
 * @param id                 Uno de los CALIB_ID_*.
 * @param datos              Bytes del valor recibido (2 bytes, big-endian).
 * @param longitudDatos      Cantidad de bytes disponibles en 'datos'.
 * @param motorOperando      true si el motor está operando (no detenido).
 *                           Los parámetros de categoría CONFIGURACION se
 *                           rechazan con CALIB_STATUS_REJECTED_ENGINE_RUNNING
 *                           si este valor es true -- normalmente se pasa
 *                           !Tacometro_EstaDetenido() desde el llamador
 *                           (rak3172.c), sin que este módulo dependa
 *                           directamente de tacometro.h.
 * @param frecuenciaHzActual Frecuencia actual del tacómetro (Hz), tal
 *                           cual la reporta Tacometro_GetFrecuenciaHz().
 *                           Se pasa desde el llamador por la misma razón
 *                           que motorOperando (mantener este módulo
 *                           desacoplado de tacometro.h). Solo la usa
 *                           CALIB_ID_SET_RATIO_AUTO -- cualquier otro
 *                           parámetro la ignora.
 * @param valorAplicadoRaw   [salida] valor vigente, codificado en 2 bytes.
 * @return El STATUS correspondiente.
 */
CalibFlash_ProtocoloStatus_t CalibFlash_ProcesarParametroConEstado(uint8_t id,
                                                                     const uint8_t *datos,
                                                                     uint8_t longitudDatos,
                                                                     bool motorOperando,
                                                                     float frecuenciaHzActual,
                                                                     uint16_t *valorAplicadoRaw);

/**
 * Procesa un parámetro recibido por downlink, ya separado en ID +
 * bytes de payload (sin el byte de ID). Decodifica según el tipo de
 * cada parámetro (ver tabla), valida rango, y aplica (persistiendo en
 * flash si corresponde). Es el único punto que rak3172.c necesita
 * llamar -- no requiere que rak3172.c conozca los detalles de escala
 * o validación de cada parámetro.
 *
 * Wrapper simple sobre CalibFlash_ProcesarParametroConEstado(), para
 * quien solo necesite saber si se aplicó o no, sin armar un ACK.
 *
 * @param id             Uno de los CALIB_ID_* de arriba.
 * @param datos          Puntero a los bytes del valor (big-endian).
 * @param longitudDatos  Cantidad de bytes disponibles en 'datos'.
 * @param frecuenciaHzActual Ver CalibFlash_ProcesarParametroConEstado().
 * @return true si el ID fue reconocido Y el valor se aplicó
 *         correctamente (rango válido, o comando con confirmación
 *         correcta). false si el ID no existe, el valor está fuera de
 *         rango, o un comando llegó sin la confirmación esperada.
 */
bool CalibFlash_ProcesarParametro(uint8_t id, const uint8_t *datos, uint8_t longitudDatos, bool motorOperando, float frecuenciaHzActual);

/* ==================== GETTERS DE PARÁMETROS PERSISTENTES ==================== */

float    CalibFlash_GetPulsosPorRevolucion(void);
float    CalibFlash_GetAlphaFiltro(void);
float    CalibFlash_GetRpmMax(void);
float    CalibFlash_GetRpmMin(void);
float    CalibFlash_GetPidKp(void);
float    CalibFlash_GetPidKi(void);
uint16_t CalibFlash_GetServoPulsoMinUs(void);
uint16_t CalibFlash_GetServoPulsoMaxUs(void);
uint32_t CalibFlash_GetTimeoutSinComandoS(void);
float    CalibFlash_GetTasaMaxCambioRpmS(void);
/** ⚠️ RENUMERADO 2026-09-23, SEGUNDA PASADA (ver README sección 12,
 * decisión explícita del usuario para que el número siga el orden real
 * de uso -- 4ta vez que se renumera este parámetro, ver historial en
 * README sección 8): 0=desactivado, 1=modo calibración manual (el
 * servo se mantiene quieto salvo que llegue un downlink nuevo de
 * SERVO_PULSO_MIN/MAX, en cuyo caso se mueve directo a ese valor),
 * 2=modo calibración con barrido automático MIN<->MAX, 3=auto-
 * calibración de ALPHA (mide el ruido real de RPM y calcula el
 * filtro), 4=auto-calibración de SERVO_PULSO_MIN (zona muerta del
 * acelerador -- antes era 5), 5=mapeo de curva de ganancia (pulso vs
 * RPM en lazo abierto, solo mide/loguea, no aplica corrección todavía
 * -- antes era 6), 6=auto-escalón del lazo INTERNO (PID#1) --
 * automatiza mandar un escalón de SET_RPM en MODO=3 y esperar un
 * tiempo fijo, logueando PID_TEST igual que en modo 10 (ver main.c,
 * INTERNO_CAL_*), 7=auto-calibración de ralentí (RPM_MIN -- antes era
 * 4, movido después de zona-muerta/ganancia/PID#1 porque en campo se
 * confirmó que el ralentí SÍ cambia con carga real conectada, a
 * diferencia de zona muerta/ganancia/PID#1 que dan igual con o sin
 * carga), 8=auto-escalón del lazo de presión LOCAL (MODO=1, antes era
 * 7) -- automatiza solo mandar un escalón de PRESION_OBJETIVO y
 * esperar un tiempo fijo, logueando PRESION_PID_TEST igual que en modo
 * 10 (ver main.c, PRESION_CAL_*); el cálculo de ganancias sigue siendo
 * con tools/pid_tuning/pid_tuning.py, esto no aplica nada solo,
 * 9=auto-escalón del lazo REMOTO (MODO=2, antes era 8) -- mismo
 * espíritu que 6/8 pero con SET_RPM en MODO=3 y REMOTO_PID_TEST,
 * terminando por reportes recibidos o timeout (ver main.c,
 * REMOTO_CAL_*), 10=modo sintonización de PID (antes era 9 -- el servo
 * se maneja igual que en modo 0 -- PID normal si corresponde -- pero a
 * diferencia de 1/2 el motor SÍ puede seguir operando sin que se
 * fuerce de vuelta a 0; único modo en el que PID_KP/PID_KI se aceptan,
 * y activa el log de alta frecuencia "PID_TEST,..." en main.c),
 * 11=barrido de ganancia de PRESIÓN (PID#2, agregado 2026-09-24) --
 * equivalente de 5 pero para el lazo de presión, ver
 * PRESION_GANANCIA_CAL_* en main.c. 3 a 9 y 11 solo se pueden pedir
 * viniendo de modo 0, y se auto-revierten a 0 solos al terminar (6/8/9/11
 * también se auto-revierten si el motor se detiene o si MODO cambia a
 * mitad de prueba). Ver README sección 4.4/9/12. */
uint8_t  CalibFlash_GetControlHabilitado(void);
uint16_t CalibFlash_GetIntervaloEnvioOperativoS(void);
uint16_t CalibFlash_GetIntervaloEnvioStandbyS(void);
CalibFlash_Modo_t CalibFlash_GetModo(void);
uint8_t  CalibFlash_GetNodeId(void);
uint16_t CalibFlash_GetHisteresisModoS(void);
float    CalibFlash_GetPresionObjetivo(void);
float    CalibFlash_GetTasaMaxCambioRpmLlenadoS(void);

/** Presion que SE ESPERA que reporte el aspersor remoto en MODO=2 --
 * ver CALIB_ID_PRESION_OBJETIVO_REMOTO arriba. Es el setpoint real que
 * recibe PresionPidRemoto_CalcularSetpointRpm() (ver presion_pid_remoto.h),
 * y ademas lo sigue usando el supervisor de "no se alcanza el
 * objetivo" (main.c). 0.0f = sin configurar (el PID perseguiria 0 PSI). */
float    CalibFlash_GetPresionObjetivoRemoto(void);

/** Tasa de subida (PSI/segundo) de la rampa de llenado de MODO=1 -- ver
 * CALIB_ID_TASA_LLENADO_PSI_S arriba. 0.0f = sin rampa, setpoint
 * directo a PRESION_OBJETIVO (comportamiento por default). */
float    CalibFlash_GetTasaLlenadoPsiS(void);

/** Ganancias del lazo EXTERNO de presión REMOTA (cascada, MODO=2 /
 * CALIB_MODO_REMOTO, ver presion_pid_remoto.h) -- independientes de
 * PID_KP/KI (lazo interno RPM->servo) y de PRESION_PID_KP/KI (lazo
 * externo de presión LOCAL, MODO=1, un mecanismo distinto). Reemplazan
 * desde 2026-09-23 a la fórmula lineal abierta que tenía antes MODO=2
 * (PRESION_OFFSET_RPM + PRESION_GANANCIA_RPM * PRESION) -- un PID no
 * necesita conocer de antemano la relación presión->RPM de cada
 * instalación, se autoajusta al error real. Solo P+I (sin derivativo),
 * mismo criterio que los otros dos lazos. Se calibran EN CAMPO -- solo
 * se aceptan con CONTROL_HABILITADO=10 (ver calibracion_flash.c). */
float CalibFlash_GetPresionRemotoPidKp(void);
float CalibFlash_GetPresionRemotoPidKi(void);

/** Ganancias del lazo EXTERNO de presión local (cascada, MODO=1 /
 * CALIB_MODO_PRESION_LOCAL, ver presion_pid.h) -- independientes de
 * PID_KP/KI (lazo interno RPM->servo) y de PRESION_REMOTO_PID_KP/KI
 * (fórmula/PID de MODO_REMOTO, un mecanismo distinto). Solo P+I (sin
 * derivativo), mismo criterio que el lazo interno. Se calibran EN
 * CAMPO igual que las demás ganancias -- solo se aceptan con
 * CONTROL_HABILITADO=10 (ver calibracion_flash.c). */
float CalibFlash_GetPresionPidKp(void);
float CalibFlash_GetPresionPidKi(void);

/** HAL_GetTick() del último downlink de PRESION válido (o del arranque,
 * si nunca llegó ninguno) -- usado por el watchdog de
 * TIMEOUT_SIN_COMANDO_S en MODO_REMOTO (ver main.c): si pasa más de ese
 * tiempo sin una PRESION fresca, el motor cae a ralentí en vez de seguir
 * el último valor indefinidamente. */
uint32_t CalibFlash_GetPresionUltimoTickMs(void);

/** Ultima hora UTC (epoch unix) confirmada por la red, persistida en
 * flash -- 0 si nunca se sincronizo todavia. Se usa como mejor
 * estimacion inicial del RTC en el arranque (ver
 * Reloj_CargarHoraAproximada() en rtc_reloj.h), mientras se espera la
 * resincronizacion real via DeviceTimeReq. Distinto de SET_RPM/PRESION:
 * este campo SI se persiste (no llega por downlink de parametro, lo
 * escribe main.c directo tras cada sincronizacion exitosa). */
uint32_t CalibFlash_GetUltimaHoraUtcConocida(void);
bool     CalibFlash_SetUltimaHoraUtcConocida(uint32_t epochUtc);

/** Última posición GPS válida conocida, persistida en flash -- 0.0f en
 * ambos campos si nunca hubo fix todavía. Mismo criterio que
 * CalibFlash_GetUltimaHoraUtcConocida(): se usa como respaldo cuando
 * el GPS no tiene fix en el momento de armar el uplink (ver gps.h
 * GPS_TieneFix()), en vez de caer directo a la coordenada fija
 * hardcodeada en main.c. Se persiste una sola vez por arranque, la
 * primera vez que el GPS consigue fix (no en cada actualización cada
 * 10s -- desgastaría la flash sin necesidad, ya que para este nodo
 * fijo la posición prácticamente no cambia). */
float CalibFlash_GetUltimaLatitudConocida(void);
float CalibFlash_GetUltimaLongitudConocida(void);
bool  CalibFlash_SetUltimaPosicionConocida(float latitud, float longitud);

/* ==================== SETTERS INDIVIDUALES (validan + persisten) ==================== */
/* Se exponen también individualmente por si se necesitan llamar
 * directo (ej. desde una futura pantalla local, o pruebas), aunque el
 * camino normal desde un downlink es CalibFlash_ProcesarParametro(). */

bool CalibFlash_SetPulsosPorRevolucion(float nuevoValor);
bool CalibFlash_SetAlphaFiltro(float nuevoValor);
bool CalibFlash_SetRpmMax(float nuevoValor);
bool CalibFlash_SetRpmMin(float nuevoValor);
bool CalibFlash_SetPidKp(float nuevoValor);
bool CalibFlash_SetPidKi(float nuevoValor);
bool CalibFlash_SetServoPulsoMinUs(uint16_t nuevoValor);
bool CalibFlash_SetServoPulsoMaxUs(uint16_t nuevoValor);
bool CalibFlash_SetTimeoutSinComandoS(uint32_t nuevoValor);
bool CalibFlash_SetTasaMaxCambioRpmS(float nuevoValor);
bool CalibFlash_SetControlHabilitado(uint8_t modo); /* válido: 0-11 -- ver getter */
bool CalibFlash_SetIntervaloEnvioOperativoS(uint16_t nuevoValor);
bool CalibFlash_SetIntervaloEnvioStandbyS(uint16_t nuevoValor);
bool CalibFlash_SetModo(CalibFlash_Modo_t nuevoModo);
bool CalibFlash_SetNodeId(uint8_t nuevoValor);
bool CalibFlash_SetHisteresisModoS(uint16_t nuevoValor);
bool CalibFlash_SetPresionObjetivo(float nuevoValor);
bool CalibFlash_SetPresionObjetivoRemoto(float nuevoValor);
bool CalibFlash_SetTasaLlenadoPsiS(float nuevoValor);
bool CalibFlash_SetTasaMaxCambioRpmLlenadoS(float nuevoValor);
bool CalibFlash_SetPresionRemotoPidKp(float nuevoValor);
bool CalibFlash_SetPresionRemotoPidKi(float nuevoValor);
bool CalibFlash_SetPresionPidKp(float nuevoValor);
bool CalibFlash_SetPresionPidKi(float nuevoValor);

/* ==================== PARÁMETROS NO PERSISTENTES (solo RAM) ==================== */

float CalibFlash_GetSetRpm(void);
void  CalibFlash_SetSetRpm(float nuevoValor); /* sin validación de rango propia --
                                                 * el módulo de control debe recortarla
                                                 * contra RPM_MIN/RPM_MAX antes de usarla */

float CalibFlash_GetPresion(void);
void  CalibFlash_SetPresion(float nuevoValor);

/* ==================== COMANDOS ==================== */

/**
 * true si llegó un FORZAR_REPORTE pendiente de atender. El loop
 * principal debe consultarlo, mandar el uplink correspondiente, y
 * llamar CalibFlash_LimpiarReporteForzado().
 */
bool CalibFlash_HayReporteForzado(void);
void CalibFlash_LimpiarReporteForzado(void);

/**
 * Arma internamente la misma bandera que un FORZAR_REPORTE por
 * downlink -- para que main.c pueda pedir un uplink inmediato como
 * "ACK" de una acción propia del firmware (ej. al completar la
 * auto-calibración de ralentí, CONTROL_HABILITADO=7) sin duplicar la
 * lógica de armado del uplink que ya vive en el loop principal.
 */
void CalibFlash_ForzarReporte(void);

/**
 * true si llegó un RESET_REMOTO válido (con confirmación correcta).
 * El loop principal debe consultarlo y, cuando sea seguro hacerlo
 * (ej. servo en una posición conocida/segura), ejecutar el reset
 * real (NVIC_SystemReset()).
 */
bool CalibFlash_HayResetPendiente(void);


/**
 * Objetivo pendiente para el modo manual de calibración del servo
 * (CONTROL_HABILITADO=1, ver README 4.4/2.4). Se marca cuando un
 * downlink de SERVO_PULSO_MIN o SERVO_PULSO_MAX se aplica con éxito
 * estando en ese modo -- por bandera, no por comparar contra el valor
 * anterior, para que un downlink que repite el valor ya guardado (ej.
 * los defaults de fábrica) igual mueva el servo. El loop principal debe
 * consultar CalibFlash_HayObjetivoManualServo(), y si es true, mover el
 * servo hacia CalibFlash_GetObjetivoManualServoUs() y llamar
 * CalibFlash_LimpiarObjetivoManualServo().
 */
bool     CalibFlash_HayObjetivoManualServo(void);
uint16_t CalibFlash_GetObjetivoManualServoUs(void);
void     CalibFlash_LimpiarObjetivoManualServo(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRACION_FLASH_H */
