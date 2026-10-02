/**
 * @file    autotune_asp.c
 * @brief   Autosintonia del PID#3 (MODO=2, triple cascada) -- ver autotune_asp.h.
 *
 * ⚠️ NO PROBADA CON MOTOBOMBA NI ASPERSOR REAL.
 */

/* -Os tambien en Debug: mismo motivo que autotune.c (que Debug quepa en 126 KB). */
#pragma GCC optimize ("Os")

#include "autotune_asp.h"
#include "presion_pid_remoto.h"
#include "presion_voltaje.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

/* ==================== PARAMETROS ==================== */

#define AUTOTUNE3_LOCAL_TOL_PSI          1.0f    /* |presion local - objetivo| para dar por llegado */
#define AUTOTUNE3_ESTABLE_TOL_PSI        0.5f    /* detector de presion local estable (Autotune_Estable_t) */
#define AUTOTUNE3_MARGEN_TIEMPO_MS     600000U   /* margen sobre lo que deberia tardar cada cambio de presion al ritmo de llenado */
#define AUTOTUNE3_REPORTES_ESTABLES        3U    /* reportes seguidos del aspersor para darlo por estable */
#define AUTOTUNE3_REMOTA_TOL_PSI         1.0f    /* max - min de esos reportes */
#define AUTOTUNE3_BASE_TIMEOUT_MS      720000U   /* 12 min: en su objetivo el aspersor reporta cada 90 s */
#define AUTOTUNE3_ESCALON_PSI            5.0f    /* mismo escalon que el autotune PID#2; mueve el aspersor > 2 PSI -> reporta cada 20 s */
#define AUTOTUNE3_RESPUESTA_MIN_PSI      1.5f
#define AUTOTUNE3_K_MIN                  0.1f    /* PSI aspersor por PSI local: fuera de esto el ensayo no es creible */
#define AUTOTUNE3_K_MAX                  2.0f
#define AUTOTUNE3_L_MIN_S               20.0f    /* un intervalo de reporte: no se puede resolver menos */
#define AUTOTUNE3_TAU_C_MIN_S           40.0f    /* ~2 intervalos de reporte */
#define AUTOTUNE3_TAU_C_FACTOR_L         2.0f
#define AUTOTUNE3_TAU_C_ESCALA           2.0f
#define AUTOTUNE3_INTENTOS_MAX             3U
#define AUTOTUNE3_GANANCIA_MAX          30.0f
#define AUTOTUNE3_TECHO_FRAC             0.95f   /* el objetivo local nunca pasa de este % de PRESION_MAX */
#define AUTOTUNE3_VAL_TIMEOUT_MS      1200000U   /* 20 min */
#define AUTOTUNE3_VAL_REPORTES_MAX        40U
#define AUTOTUNE3_BANDA_MIN_PSI          1.0f    /* con 0.5 el ruido del aspersor contaba como cruces (simulado) */
#define AUTOTUNE3_BANDA_FRAC             0.05f
#define AUTOTUNE3_CRUCES_MAX               2U
#define AUTOTUNE3_HISTORIAL_MAX          120U    /* reportes guardados durante el escalon (tiempo, valor) */

/* ==================== ESTADO ==================== */

typedef enum {
    ST_ESPERA = 0,
    ST_LLENADO,       /* presion local hasta la base (solo tarda si venia de MODO=4) */
    ST_BASE,          /* reportes estables del aspersor en la base */
    ST_ESCALON,       /* objetivo local +-5 PSI, esperando la respuesta del aspersor */
    ST_VOLVER,        /* de regreso a la base antes de validar */
    ST_VALIDAR,       /* triple cascada con la candidata */
    ST_FINALIZANDO    /* solo desde MODO=1: bajar el objetivo local a la base antes de soltar */
} Estado_t;

static Estado_t  s_estado = ST_ESPERA;
static bool      s_desdeModo1 = false;
static uint32_t  s_t0Ms = 0U;
static uint32_t  s_timeoutMs = 0U;
static Autotune_Estable_t s_estable;
static bool      s_localLlego = false;
static float     s_base = 0.0f;
static float     s_techo = 0.0f;
static float     s_deseado = 0.0f;       /* objetivo local deseado (antes de la rampa) */
static float     s_aplicado = 0.0f;      /* objetivo local aplicado (con rampa) */
static uint32_t  s_ultimoTick = 0U;
static float     s_rep[AUTOTUNE3_REPORTES_ESTABLES];
static uint8_t   s_nRep = 0U;

/* Escalon */
static float     s_delta = 0.0f;
static float     s_r0 = 0.0f, s_l0 = 0.0f, s_rf = 0.0f, s_lf = 0.0f;
static uint32_t  s_tEscalonMs = 0U;
static float     s_tL50S = -1.0f;
static uint16_t  s_nHist = 0U;

/* Calculo / validacion */
static float     s_k = 0.0f, s_l = 0.0f, s_tauC = 0.0f, s_ki = 0.0f;
static uint8_t   s_intento = 0U;
static float     s_objRem = 0.0f;
static float     s_banda = 0.0f;
static int8_t    s_lado = 0;
static uint8_t   s_cruces = 0U;
static uint8_t   s_nRepVal = 0U;
static Autotune_Resultado_t s_pendiente = AUTOTUNE_SIN_RESULTADO;
static Autotune_Resultado_t s_ultimo = AUTOTUNE_SIN_RESULTADO;

/* ==================== AUXILIARES ==================== */

static float absf(float x) { return (x < 0.0f) ? -x : x; }

static void terminar(Autotune_Resultado_t r)
{
    AutotuneAsp_Limpiar();
    s_ultimo = r;
    CalibFlash_SetCalib(0U);
}

static void abortar(const char *motivo)
{
    printf("AUTOTUNE_PID3,ABORTADO,motivo=%s\r\n", motivo);
    terminar(AUTOTUNE_RESULTADO_FALLO);
}

/* Desde MODO=1 no se suelta de golpe: primero se regresa el objetivo local a
 * la base (al ritmo de llenado) y recien ahi se deja CALIB=0. */
static void finalizar(Autotune_Resultado_t r)
{
    if (s_desdeModo1) {
        s_pendiente = r;
        s_deseado = s_base;
        s_estado = ST_FINALIZANDO;
    } else {
        terminar(r);
    }
}

/* Tiempo razonable para un cambio de 'psi' al ritmo de llenado. */
static uint32_t tiempoParaCambio(float psi)
{
    float tasa = CalibFlash_GetTasaLlenadoPsiS();
    uint32_t ms = (tasa > 0.0f) ? (uint32_t)(absf(psi) / tasa * 1000.0f) : 0U;
    return ms + AUTOTUNE3_MARGEN_TIEMPO_MS;
}

/* true si llego un reporte nuevo del aspersor; *valor = PRESION_REMOTO. */
static bool nuevoReporte(float *valor)
{
    uint32_t tick = CalibFlash_GetPresionRemotoUltimoTickMs();
    if (tick == s_ultimoTick) return false;
    s_ultimoTick = tick;
    *valor = CalibFlash_GetPresionRemoto();
    return true;
}

/* Agrega un reporte a la ventana de 3; true cuando los 3 caben en la tolerancia. */
static bool remotaEstable(float v, float *promedio)
{
    if (s_nRep < AUTOTUNE3_REPORTES_ESTABLES) {
        s_rep[s_nRep++] = v;
    } else {
        for (uint8_t i = 0U; i + 1U < AUTOTUNE3_REPORTES_ESTABLES; i++) s_rep[i] = s_rep[i + 1U];
        s_rep[AUTOTUNE3_REPORTES_ESTABLES - 1U] = v;
    }
    if (s_nRep < AUTOTUNE3_REPORTES_ESTABLES) return false;
    float mn = s_rep[0], mx = s_rep[0], acc = 0.0f;
    for (uint8_t i = 0U; i < AUTOTUNE3_REPORTES_ESTABLES; i++) {
        if (s_rep[i] < mn) mn = s_rep[i];
        if (s_rep[i] > mx) mx = s_rep[i];
        acc += s_rep[i];
    }
    if (mx - mn > AUTOTUNE3_REMOTA_TOL_PSI) return false;
    *promedio = acc / (float)AUTOTUNE3_REPORTES_ESTABLES;
    return true;
}

/* Presion local estable y dentro de la tolerancia del objetivo deseado. */
static bool localLlego(uint32_t ahora, float psi, float *promedio)
{
    bool estable = Autotune_EstableActualizar(&s_estable, ahora, psi, AUTOTUNE3_ESTABLE_TOL_PSI, promedio);
    return estable && absf(*promedio - s_deseado) <= AUTOTUNE3_LOCAL_TOL_PSI
           && absf(s_aplicado - s_deseado) < 0.01f;
}

static void irA(Estado_t e, uint32_t ahora, uint32_t timeoutMs)
{
    s_estado = e;
    s_t0Ms = ahora;
    s_timeoutMs = timeoutMs;
    s_nRep = 0U;
    s_localLlego = false;
    Autotune_EstableIniciar(&s_estable, ahora);
    s_ultimoTick = CalibFlash_GetPresionRemotoUltimoTickMs(); /* solo cuentan reportes posteriores */
}

static void calcularCandidata(void)
{
    /* SIMC con tau ~ 0: Kc = tau/(K(tau_c+L)) -> 0, Ti = tau -> control
     * integral puro, Ki = 1/(K(tau_c+L)). */
    s_ki = 1.0f / (s_k * (s_tauC + s_l));
    if (s_ki > AUTOTUNE3_GANANCIA_MAX) s_ki = AUTOTUNE3_GANANCIA_MAX;
    printf("AUTOTUNE_PID3,CANDIDATA,intento=%u,tau_c=%.1f,kp=0,ki=%.5f\r\n",
           (unsigned)(s_intento + 1U), s_tauC, s_ki);
}

static void volverABase(uint32_t ahora)
{
    PresionPidRemoto_LimpiarGananciasTemporales();
    s_deseado = s_base;
    irA(ST_VOLVER, ahora, tiempoParaCambio(s_delta) + AUTOTUNE3_BASE_TIMEOUT_MS);
}

static void analizarEscalon(uint32_t ahora)
{
    float dR = s_rf - s_r0;
    float dL = s_lf - s_l0;
    if (absf(dR) < AUTOTUNE3_RESPUESTA_MIN_PSI || absf(dL) < 0.5f) {
        printf("AUTOTUNE_PID3,ID_FALLIDA,delta_local=%.2f,delta_aspersor=%.2f\r\n", dL, dR);
        abortar("respuesta_del_aspersor_insuficiente");
        return;
    }
    s_k = dR / dL;
    if (s_k < AUTOTUNE3_K_MIN || s_k > AUTOTUNE3_K_MAX) {
        printf("AUTOTUNE_PID3,ID_FALLIDA,K=%.3f\r\n", s_k);
        abortar("k_fuera_de_rango");
        return;
    }
    /* Retardo: desde que el OBJETIVO local cruzo la mitad de su cambio hasta
     * el primer reporte del aspersor que cruzo la mitad del suyo. */
    float *h = Autotune_Buffer();
    float tR50 = -1.0f;
    for (uint16_t i = 0U; i < s_nHist; i++) {
        float v = h[2U * i + 1U];
        if ((dR > 0.0f && v >= s_r0 + 0.5f * dR) || (dR < 0.0f && v <= s_r0 + 0.5f * dR)) {
            tR50 = h[2U * i];
            break;
        }
    }
    s_l = (tR50 >= 0.0f && s_tL50S >= 0.0f) ? (tR50 - s_tL50S) : AUTOTUNE3_L_MIN_S;
    if (s_l < AUTOTUNE3_L_MIN_S) s_l = AUTOTUNE3_L_MIN_S;

    printf("AUTOTUNE_PID3,ID,K=%.3f,L=%.1f,local=%.2f->%.2f,aspersor=%.2f->%.2f,reportes=%u\r\n",
           s_k, s_l, s_l0, s_lf, s_r0, s_rf, (unsigned)s_nHist);

    s_intento = 0U;
    s_tauC = AUTOTUNE3_TAU_C_FACTOR_L * s_l;
    if (s_tauC < AUTOTUNE3_TAU_C_MIN_S) s_tauC = AUTOTUNE3_TAU_C_MIN_S;
    calcularCandidata();
    volverABase(ahora);
}

static void empezarValidacion(uint32_t ahora, float rInicio)
{
    s_objRem = CalibFlash_GetPresionObjetivoRemoto();
    s_banda = s_objRem * AUTOTUNE3_BANDA_FRAC;
    if (s_banda < AUTOTUNE3_BANDA_MIN_PSI) s_banda = AUTOTUNE3_BANDA_MIN_PSI;
    float e = rInicio - s_objRem;
    s_lado = (e > s_banda) ? 1 : ((e < -s_banda) ? -1 : 0);
    s_cruces = 0U;
    s_nRepVal = 0U;
    PresionPidRemoto_Init();
    PresionPidRemoto_SetGananciasTemporales(0.0f, s_ki);
    irA(ST_VALIDAR, ahora, AUTOTUNE3_VAL_TIMEOUT_MS);
    printf("AUTOTUNE_PID3,VALIDANDO,intento=%u,aspersor_inicio=%.2f,objetivo_remoto=%.2f,ki=%.5f%s\r\n",
           (unsigned)(s_intento + 1U), rInicio, s_objRem, s_ki,
           (s_lado == 0) ? ",nota=ya_estaba_en_el_objetivo_sin_excitacion" : "");
}

static void rechazar(const char *motivo, uint32_t ahora)
{
    printf("AUTOTUNE_PID3,INTENTO_RECHAZADO,intento=%u,motivo=%s,cruces=%u,reportes=%u\r\n",
           (unsigned)(s_intento + 1U), motivo, (unsigned)s_cruces, (unsigned)s_nRepVal);
    s_intento++;
    if (s_intento >= AUTOTUNE3_INTENTOS_MAX) {
        printf("AUTOTUNE_PID3,FALLO,motivo=ninguna_candidata_paso,ganancias_sin_cambios=1\r\n");
        PresionPidRemoto_LimpiarGananciasTemporales();
        finalizar(AUTOTUNE_RESULTADO_FALLO);
        return;
    }
    s_tauC *= AUTOTUNE3_TAU_C_ESCALA;
    calcularCandidata();
    volverABase(ahora);
}

/* ==================== API ==================== */

void AutotuneAsp_Iniciar(void)
{
    AutotuneAsp_Limpiar();
    s_ultimo = AUTOTUNE_SIN_RESULTADO;
    printf("AUTOTUNE_PID3,INICIO,pid_asp_kp_actual=%.4f,pid_asp_ki_actual=%.5f\r\n",
           CalibFlash_GetPidAspKp(), CalibFlash_GetPidAspKi());
}

void AutotuneAsp_Limpiar(void)
{
    PresionPidRemoto_LimpiarGananciasTemporales();
    s_estado = ST_ESPERA;
}

Autotune_Resultado_t AutotuneAsp_UltimoResultado(void) { return s_ultimo; }
bool  AutotuneAsp_CascadaActiva(void) { return s_estado != ST_ESPERA; }
float AutotuneAsp_ObjetivoLocal(void) { return s_aplicado; }

void AutotuneAsp_Paso(bool motorOperando, CalibFlash_Modo_t modo)
{
    uint32_t ahora = HAL_GetTick();
    float psi = PresionV_GetPresionPsi();
    float v = 0.0f, prom = 0.0f;

    if (s_estado != ST_ESPERA) {
        CalibFlash_Modo_t modoEsperado = s_desdeModo1 ? CALIB_MODO_PRESION_LOCAL : CALIB_MODO_CALIBRACION;
        if (!motorOperando)            { abortar("motor_se_detuvo"); return; }
        if (modo != modoEsperado)      { abortar("modo_cambio");     return; }
        if (!PresionV_SensorValido())  { abortar("sensor_invalido"); return; }
        /* El objetivo local aplicado sigue al deseado al ritmo de llenado,
         * en las dos direcciones (regla de la tuberia). */
        s_aplicado = PresionPidRemoto_ObjetivoAplicado(s_deseado, CalibFlash_GetTasaLlenadoPsiS());
        if (s_estado != ST_FINALIZANDO && ahora - s_t0Ms >= s_timeoutMs) {
            if (s_estado == ST_VALIDAR) {
                rechazar("no_asienta", ahora);   /* candidata mas suave, no abortar */
            } else {
                abortar("timeout");
            }
            return;
        }
    }

    switch (s_estado) {
    case ST_ESPERA:
        if (!(motorOperando && (modo == CALIB_MODO_CALIBRACION || modo == CALIB_MODO_PRESION_LOCAL))) break;
        if (CalibFlash_GetPresionObjetivoRemoto() <= 0.0f) { abortar("PRESION_OBJETIVO_REMOTO_sin_configurar"); break; }
        if (CalibFlash_GetPidPsiKp() <= 0.0f) { abortar("pid2_sin_calibrar_correr_CALIB_14_primero"); break; }
        s_desdeModo1 = (modo == CALIB_MODO_PRESION_LOCAL);
        s_base = CalibFlash_GetPresionObjetivoLocal();
        s_techo = CalibFlash_GetPresionMax() * AUTOTUNE3_TECHO_FRAC;
        s_deseado = s_base;
        PresionPidRemoto_ReiniciarRampa(psi);
        s_aplicado = psi;
        irA(ST_LLENADO, ahora, tiempoParaCambio(s_base - psi));
        printf("AUTOTUNE_PID3,INICIO_CORRIDA,desde=%s,base_local=%.2f,presion_local=%.2f,objetivo_remoto=%.2f\r\n",
               s_desdeModo1 ? "MODO1" : "MODO4", s_base, psi, CalibFlash_GetPresionObjetivoRemoto());
        break;

    case ST_LLENADO:
        if (localLlego(ahora, psi, &prom)) {
            printf("AUTOTUNE_PID3,BASE_LOCAL,presion_local=%.2f\r\n", prom);
            irA(ST_BASE, ahora, AUTOTUNE3_BASE_TIMEOUT_MS);
        }
        break;

    case ST_BASE:
        if (nuevoReporte(&v) && remotaEstable(v, &prom)) {
            s_r0 = prom;
            s_l0 = psi;
            s_delta = (s_base + AUTOTUNE3_ESCALON_PSI <= s_techo) ? AUTOTUNE3_ESCALON_PSI : -AUTOTUNE3_ESCALON_PSI;
            s_deseado = s_base + s_delta;
            s_tEscalonMs = ahora;
            s_tL50S = -1.0f;
            s_nHist = 0U;
            irA(ST_ESCALON, ahora, tiempoParaCambio(s_delta) + AUTOTUNE3_BASE_TIMEOUT_MS);
            printf("AUTOTUNE_PID3,ESCALON,aspersor_base=%.2f,local_base=%.2f,objetivo_local_nuevo=%.2f\r\n",
                   s_r0, s_l0, s_deseado);
        }
        break;

    case ST_ESCALON: {
        float tS = (float)(ahora - s_tEscalonMs) / 1000.0f;
        /* Referencia del retardo = el OBJETIVO local (lo que mueve el PID#3),
         * no la presion local medida: el lazo del PID#3 incluye lo que tarda
         * el PID#2 en seguir su objetivo (~1 min), mas tuberia y reportes. */
        if (s_tL50S < 0.0f && ((s_delta > 0.0f) ? (s_aplicado >= s_base + 0.5f * s_delta)
                                                : (s_aplicado <= s_base + 0.5f * s_delta))) {
            s_tL50S = tS;
        }
        if (!s_localLlego && localLlego(ahora, psi, &prom)) {
            s_localLlego = true;
            s_lf = prom;
        }
        if (nuevoReporte(&v)) {
            if (s_nHist < AUTOTUNE3_HISTORIAL_MAX) {
                float *h = Autotune_Buffer();
                h[2U * s_nHist] = tS;
                h[2U * s_nHist + 1U] = v;
                s_nHist++;
            }
            printf("AUTOTUNE_PID3,REPORTE,t_s=%.0f,aspersor=%.2f,local=%.2f\r\n", tS, v, psi);
            /* El final solo cuenta cuando la presion local ya llego. */
            if (s_localLlego && remotaEstable(v, &prom)) {
                s_rf = prom;
                analizarEscalon(ahora);
            }
        }
        break;
    }

    case ST_VOLVER:
        if (!s_localLlego && localLlego(ahora, psi, &prom)) s_localLlego = true;
        if (nuevoReporte(&v) && s_localLlego && remotaEstable(v, &prom)) {
            empezarValidacion(ahora, prom);
        }
        break;

    case ST_VALIDAR:
        if (nuevoReporte(&v)) {
            s_nRepVal++;
            float ajuste = PresionPidRemoto_CalcularAjustePsi(s_objRem, v, s_base, s_techo);
            s_deseado = s_base + ajuste;
            float e = v - s_objRem;
            int8_t lado = (e > s_banda) ? 1 : ((e < -s_banda) ? -1 : s_lado);
            if (lado != s_lado && s_lado != 0) s_cruces++;
            if (lado != 0) s_lado = lado;
            printf("AUTOTUNE_PID3,VAL_REPORTE,n=%u,aspersor=%.2f,ajuste=%.2f,objetivo_local=%.2f\r\n",
                   (unsigned)s_nRepVal, v, ajuste, s_deseado);

            bool dentro = true;
            (void)remotaEstable(v, &prom);
            for (uint8_t i = 0U; i < s_nRep; i++) {
                if (absf(s_rep[i] - s_objRem) > s_banda) dentro = false;
            }
            if (s_cruces > AUTOTUNE3_CRUCES_MAX) {
                rechazar("oscila", ahora);
            } else if (dentro && s_nRep >= AUTOTUNE3_REPORTES_ESTABLES) {
                bool okGan = CalibFlash_SetPidAspGanancias(0.0f, s_ki);
                PresionPidRemoto_LimpiarGananciasTemporales();
                if (!okGan) { abortar("error_al_guardar_en_flash"); break; }
                printf("AUTOTUNE_PID3,OK,PID_ASP_KP=0.0000,PID_ASP_KI=%.5f,K=%.3f,L=%.1f,intentos=%u\r\n",
                       s_ki, s_k, s_l, (unsigned)(s_intento + 1U));
                finalizar(AUTOTUNE_RESULTADO_OK);
            } else if (s_nRepVal >= AUTOTUNE3_VAL_REPORTES_MAX) {
                rechazar("no_asienta", ahora);
            }
        }
        break;

    case ST_FINALIZANDO:
        if (absf(s_aplicado - s_base) < 0.01f) {
            terminar(s_pendiente);
        }
        break;
    }
}
