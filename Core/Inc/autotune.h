/**
 * @file    autotune.h
 * @brief   Autosintonia de los lazos PID (identificacion + SIMC + validacion)
 *          corriendo DENTRO del firmware, sin laptop.
 *
 * Reemplaza, para el operador de campo, el flujo manual
 *   CALIB=6/10 (log) -> pid_tuning.py identify/simc -> bajar ganancias
 *   por downlink -> CALIB=7/11 (validar) -> mirar el log
 * por un solo CALIB:
 *   CALIB=13  autosintonia del lazo INTERNO RPM->servo (PID#1)
 *   CALIB=14  autosintonia del lazo de PRESION LOCAL (PID#2)
 *
 * Este archivo es el MOTOR COMPARTIDO (funciones puras, sin acceso a
 * hardware ni a flash -- pensadas para poder probarse igual en una PC):
 *   - identificacion FOPDT (K, tau, L) de una respuesta a escalon, con el
 *     MISMO algoritmo que tools/pid_tuning/pid_tuning.py (mismos umbrales,
 *     mismas formulas) para que el resultado de la placa y el de la PC
 *     sobre el mismo ensayo sean comparables;
 *   - formulas SIMC (Skogestad 2003) para PI;
 *   - metricas de una respuesta a escalon en lazo cerrado (sobre-impulso,
 *     tiempo de asentamiento, error final, cruces de signo);
 *   - vigia de oscilacion sostenida (Schmitt trigger sobre el error).
 * Las maquinas de estado de cada lazo viven en autotune_rpm.c /
 * autotune_psi.c.
 *
 * Solo P+I (sin derivativo), igual que el resto del proyecto.
 */

#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Capacidad del buffer de muestras de cada ensayo (floats, ~2.4KB). */
#define AUTOTUNE_MAX_MUESTRAS   600U

/** Resultado de la ultima corrida de una autosintonia (para el codigo de
 *  alerta del uplink, ver main.c). */
typedef enum {
    AUTOTUNE_SIN_RESULTADO = 0,
    AUTOTUNE_RESULTADO_OK,
    AUTOTUNE_RESULTADO_FALLO
} Autotune_Resultado_t;

/** Buffer de muestras compartido (AUTOTUNE_MAX_MUESTRAS floats). PID#1 y
 *  PID#2 nunca corren a la vez (son CALIB distintos), asi que comparten
 *  el mismo para no duplicar RAM. */
float *Autotune_Buffer(void);

/* ==================== IDENTIFICACION FOPDT ==================== */

typedef struct {
    float k;    /* ganancia estatica de la planta (unidad_resp / unidad_entrada) */
    float tau;  /* constante de tiempo (s) */
    float l;    /* tiempo muerto (s) */
    float gcl;  /* ganancia estatica medida del lazo cerrado (solo lazo cerrado; 0 en lazo abierto) */
} Autotune_Planta_t;

typedef enum {
    AUTOTUNE_ID_OK = 0,
    AUTOTUNE_ID_DATOS_INSUFICIENTES,
    AUTOTUNE_ID_NO_ASENTO,          /* la respuesta seguia moviendose al final de la ventana */
    AUTOTUNE_ID_GCL_FUERA_RANGO,    /* (lazo cerrado) la ganancia de lazo cerrado no esta en (0.02, 0.999) */
    AUTOTUNE_ID_K_INVALIDA,         /* (lazo abierto) la respuesta no cambio en la direccion esperada */
    AUTOTUNE_ID_NO_SE_MOVIO,
    AUTOTUNE_ID_NO_LLEGO_63
} Autotune_IdResultado_t;

/** Nombre corto para el log serie. */
const char *Autotune_IdNombre(Autotune_IdResultado_t r);

/**
 * Identifica (K, tau, L) de un lazo CERRADO con Kp puro (Ki=0) y Kp
 * conocida -- mismo metodo que _identify_step_generico() de
 * pid_tuning.py: G_cl = delta_resp_ss/delta_cmd, K = G_cl/(Kp*(1-G_cl)),
 * tau = tau_cl/(1-G_cl), L = L_cl.
 *
 * @param y            respuesta muestreada a paso fijo dtS; y[0] es el
 *                      instante del escalon.
 * @param n            cantidad de muestras.
 * @param y0           valor de la respuesta ANTES del escalon (promedio).
 * @param deltaCmd     salto efectivo del comando respecto de y0
 *                      (setpoint_nuevo - y0 -- ver autotune_rpm.c).
 * @param kpUsado      Kp que estaba activa durante el ensayo (> 0).
 * @param ventanaFinalS ventana (s) al final de la captura con la que se
 *                      promedia el valor final.
 * @param umbralMin    piso absoluto de "ya se movio" (unidades de y).
 * @param semiVentana  semi-ancho (muestras) del promedio movil centrado
 *                      usado para decidir cuando se movio / llego al 63%.
 */
Autotune_IdResultado_t Autotune_IdentificarLazoCerrado(const float *y, uint16_t n,
        float dtS, float y0, float deltaCmd, float kpUsado, float ventanaFinalS,
        float umbralMin, uint16_t semiVentana, Autotune_Planta_t *out);

/**
 * Identifica (K, tau, L) en LAZO ABIERTO: el comando aplicado a la
 * planta es directamente 'deltaCmd' (ej. un escalon de SET_RPM, con el
 * lazo interno ya sintonizado de por medio) -- K = delta_resp_ss/deltaCmd.
 * Mismo metodo que _identify_step_abierto_generico() de pid_tuning.py.
 */
Autotune_IdResultado_t Autotune_IdentificarLazoAbierto(const float *y, uint16_t n,
        float dtS, float y0, float deltaCmd, float ventanaFinalS,
        float umbralMin, float umbralFrac, uint16_t semiVentana, bool dosPuntos,
        Autotune_Planta_t *out);
/* dosPuntos: false = mismo metodo que pid_tuning.py (L = cuando se movio mas
 * del umbral, tau = de ahi al 63%); true = metodo de dos puntos de Smith
 * (28% y 63%), sin sesgo en tau. PID#1 usa true (reproduce el tau=0.43 con
 * el que se sacaron 0.047/0.11); PID#2 usa false (1.72/0.39 salio con
 * `identify-presion`, que usa el metodo de umbral). */

/* ==================== GRABADOR DE MUESTRAS ==================== */

/** Graba en Autotune_Buffer() una muestra cada dtMs (paso fijo, aunque el
 *  loop se atrase: si una vuelta tarda, se ponen al dia varias muestras). */
typedef struct {
    uint16_t n;          /* muestras grabadas */
    uint16_t objetivo;   /* muestras a grabar */
    uint16_t dtMs;
    uint32_t proximaMs;
} Autotune_Grabador_t;

void Autotune_GrabadorIniciar(Autotune_Grabador_t *g, uint32_t ahoraMs, uint16_t cuantas, uint16_t dtMs);
/** leer = funcion que devuelve la variable a grabar (RPM filtrada, PSI...). */
void Autotune_GrabadorMuestrear(Autotune_Grabador_t *g, uint32_t ahoraMs, float (*leer)(void));
bool Autotune_GrabadorLleno(const Autotune_Grabador_t *g);

/* ==================== DETECTOR DE "SEÑAL ESTABLE" ==================== */

/** Muestras a 1 Hz en una ventana de AUTOTUNE_ESTABLE_N s: estable cuando el
 *  promedio de las 5 mas nuevas y el de las 5 mas viejas difieren menos de
 *  'tolerancia'. Usado por el autotune 2 (asentar/volver a ralenti) y el 3
 *  (presion local estable antes de mirar el aspersor). */
#define AUTOTUNE_ESTABLE_N   20U

typedef struct {
    float    ring[AUTOTUNE_ESTABLE_N];
    uint8_t  cuenta;
    uint32_t proximoMs;
} Autotune_Estable_t;

void Autotune_EstableIniciar(Autotune_Estable_t *e, uint32_t ahoraMs);
/** Devuelve true cuando ya es estable; en ese caso *promedio10 = promedio de
 *  los ultimos 10 s. */
bool Autotune_EstableActualizar(Autotune_Estable_t *e, uint32_t ahoraMs, float valor,
                                float tolerancia, float *promedio10);

/* ==================== SIMC ==================== */

/** Kc = (1/K) * tau/(tau_c+L);  Ti = min(tau, 4*(tau_c+L));  Ki = Kc/Ti. */
void Autotune_Simc(const Autotune_Planta_t *p, float tauC, float *kp, float *ki);

/** SIMC + recorte: Kp en [kpMin, gananciaMax], Ki en [0, gananciaMax]. */
void Autotune_CandidataSimc(const Autotune_Planta_t *p, float tauC, float kpMin, float gananciaMax,
                            float *kp, float *ki);

/* ==================== METRICAS DE LA RESPUESTA ==================== */

typedef struct {
    float   sobrepicoPct;   /* (pico - sp)/|sp - y0| * 100, 0 si no paso de sp */
    float   errFinal;       /* promedio de la ventana final - sp */
    float   tAsentS;        /* ultimo instante fuera de banda (s); = duracion si no asento */
    float   banda;          /* banda de asentamiento usada (unidades de y) */
    uint8_t cruces;         /* cruces de signo de (y - sp) con histeresis +-banda */
    bool    asento;         /* termino dentro de banda */
} Autotune_Metricas_t;

/**
 * @param bandaMin  piso absoluto de la banda (unidades de y, anti-ruido);
 *                  la banda real es max(bandaMin, bandaFrac*|sp-y0|).
 */
void Autotune_EvaluarEscalon(const float *y, uint16_t n, float dtS, float sp, float y0,
        float bandaMin, float bandaFrac, float ventanaFinalS, uint16_t semiVentana,
        Autotune_Metricas_t *m);

/** Veredicto de una validacion: "oscila" (mas de crucesMax cruces),
 *  "sobreimpulso", "no_asienta" u "ok" (*aprobada = true). En ese orden. */
const char *Autotune_Veredicto(const Autotune_Metricas_t *m, uint8_t crucesMax, float sobrepicoMaxPct,
                               bool *aprobada);

/* ==================== VIGIA DE OSCILACION EN VIVO ==================== */

#define AUTOTUNE_OSC_CRUCES     4U

typedef struct {
    int8_t   estado;                        /* -1/0/+1, lado actual del error (Schmitt) */
    uint32_t t[AUTOTUNE_OSC_CRUCES];        /* instantes (ms) de los ultimos cruces */
    uint8_t  cuenta;                        /* cruces guardados (<= AUTOTUNE_OSC_CRUCES) */
    uint32_t total;                         /* cruces totales desde el reinicio (el log de CALIB=7/11 lo imprime) */
} Autotune_Osc_t;

void Autotune_OscReiniciar(Autotune_Osc_t *o);

/**
 * Alimentar con el error (y - sp) en cada pasada. Devuelve true cuando se
 * acumularon AUTOTUNE_OSC_CRUCES cruces dentro de 'ventanaMs'. El error
 * debe superar +-umbral para que cuente (banda muerta anti-ruido).
 */
bool Autotune_OscActualizar(Autotune_Osc_t *o, float error, float umbral,
        uint32_t ahoraMs, uint32_t ventanaMs);

#ifdef __cplusplus
}
#endif

#endif /* AUTOTUNE_H */
