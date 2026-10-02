/**
 * @file    autotune_psi.c
 * @brief   Autosintonia del lazo de presion local (PID#2) -- ver autotune_psi.h.
 *
 * ⚠️ NO PROBADA CON MOTOBOMBA REAL -- ver autotune_psi.h.
 */

/* Compilado siempre con -Os, tambien en Debug (-O0), para que el binario
 * Debug quepa en los 126 KB que declara el .ld (la ultima pagina, 0x0801F800,
 * es la de calibracion: si el programa crece de mas, falla al enlazar). */
#pragma GCC optimize ("Os")

#include "autotune_psi.h"
#include "calibracion_flash.h"
#include "presion_voltaje.h"
#include "presion_pid.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <math.h>

/* ==================== PARAMETROS ==================== */

#define AUTOTUNE2_BASE_MS                5000U   /* promedio de la presion base en RPM_MIN */
#define AUTOTUNE2_OBJETIVO_MIN_SOBRE_BASE_PSI 5.0f /* menos rango que esto no da una identificacion util. Bajado de 8 a 5 tras ver el log real COM3_2026_09_21: a ralenti la presion NO es 0 (~12.5 PSI) y el objetivo de esa sesion era 15-20 PSI, es decir 3 a 8 PSI sobre la base */

/* SUBIDA al punto de operacion: rampa de SET_RPM en lazo abierto PAUSADA
 * POR PSI REAL al ritmo que el operador define con TIEMPO_LLENADO_S: la
 * presion nunca sube mas rapido que ese ritmo, sin importar la ganancia real
 * de la instalacion. Es una regla de seguridad de la tuberia (decision del
 * usuario) -- NO se acelera aunque este llena. */
#define AUTOTUNE2_SUBIDA_TIMEOUT_MARGEN  1.5f    /* timeout de ultimo recurso = duracion esperada de la subida ((objetivo - base) / TASA_LLENADO_PSI_S) x esto */
#define AUTOTUNE2_SUBIDA_ESPERA_TECHO_MS 30000U  /* al tocar RPM_MAX_CARGA, esperar ~3 tiempos muertos a que la presion llegue antes de rendirse */
#define AUTOTUNE2_CURVA_PASO_LOG_PSI     1.0f    /* PRESION_GANANCIA_CAL,PASO cada vez que la PSI avanza esto */

/* K de la subida por TRAMOS ANCHOS (2026-10-01): SOLO para calcular la Kp de
 * prueba del escalon cerrado (como `sugerir-kp-presion`), NO entra en SIMC.
 * El rango base->objetivo se parte en N tramos iguales y se registra el RPM
 * al que se cruza cada umbral. Entre puntos cercanos no sirve: con ~11 s de
 * tiempo muerto, cuando la rampa pausa la presion sigue subiendo, y sin el
 * EMA de PresionV (PRESIONV_FILTROS_ACTIVOS=0) el ruido pesa mas. En un tramo
 * ancho ese desfase mueve los dos extremos por igual y se cancela. */
#define AUTOTUNE2_CURVA_TRAMOS           4U
#define AUTOTUNE2_CURVA_MIN_DRPM         3.0f

/* Kp de PRUEBA del escalon cerrado: la que da G_cl = K*Kp/(1+K*Kp) = 0.3 con
 * la K del PRIMER tramo (donde ocurre el escalon, cerca de ralenti) -- misma
 * formula que `pid_tuning.py sugerir-kp-presion`. Con la K de campo
 * (0.0746 PSI/RPM) da 5.7; en campo se uso 5 a mano. */
#define AUTOTUNE2_GCL_PRUEBA             0.3f
#define AUTOTUNE2_KP_PRUEBA_MIN          0.01f
#define AUTOTUNE2_KP_PRUEBA_MAX         30.0f

/* Detector de "presion estable" (compartido, Autotune_Estable_t en autotune.c):
 * muestras a 1 Hz, promedio de las 5 mas nuevas contra las 5 mas viejas de 20 s.
 * ⚠️ 0.5 PSI se eligio con el ruido del log del 21/09 (con EMA); sin filtros
 * puede tardar mas en declarar estable -- revisar en la primera prueba. */
#define AUTOTUNE2_ESTABLE_TOL_PSI        0.5f
#define AUTOTUNE2_VOLVER_TIMEOUT_MS    300000U

/* Escalon de IDENTIFICACION (2026-10-01): el metodo de campo -- escalon
 * CERRADO de +5 PSI desde ralenti con la Kp de prueba y Ki=0 (log
 * COM3_2026_09_21: 15->20 PSI con Kp=5), identificado como `identify-presion`.
 * 0.3 s x 600 = 180 s (600 es el tope del buffer). Pasando el log real por
 * este calculo, 0.2-0.3 s da Kp 1.60-1.82 / Ki 0.34-0.41; 0.4-0.5 s mide mal
 * el tiempo muerto. */
#define AUTOTUNE2_ID_ESCALON_PSI         5.0f
#define AUTOTUNE2_ID_DT_MS               300U
#define AUTOTUNE2_ID_MUESTRAS            600U
#define AUTOTUNE2_ID_SEMIVENTANA         5U      /* promedio movil de 11 muestras (3.3 s) ~ el suavizado_s=3.0 de identify-presion */
/* Muestreo de la VALIDACION: 0.5 s x 480 = 240 s. Para juzgar
 * oscilacion/asentamiento alcanza, y 240 s a 0.3 s no cabrian en el buffer. */
#define AUTOTUNE2_VAL_DT_MS              500U
#define AUTOTUNE2_VAL_MUESTRAS           480U
#define AUTOTUNE2_VAL_SEMIVENTANA        3U      /* promedio movil de 7 muestras (3.5 s) */
#define AUTOTUNE2_VENTANA_FINAL_S        20.0f
#define AUTOTUNE2_UMBRAL_MOV_PSI         0.3f    /* mismo piso que identify-presion */

/* SIMC "Medio" (tau_c = 2L), igual que la 1.72/0.39 de campo. L de este lazo
 * es de ~10 s; el piso solo protege de una L mal medida. */
#define AUTOTUNE2_TAU_C_FACTOR_L         2.0f
#define AUTOTUNE2_TAU_C_MIN_S            4.0f
#define AUTOTUNE2_TAU_C_ESCALA_REINTENTO 2.0f
#define AUTOTUNE2_INTENTOS_MAX           3U
#define AUTOTUNE2_KP_MIN                 0.001f
#define AUTOTUNE2_GANANCIA_MAX          30.0f

/* Validacion en la cascada real: un solo escalon de +5 PSI desde ralenti
 * (o hasta PRESION_OBJETIVO_LOCAL si queda mas cerca). Un escalon grande
 * hasta el objetivo iria sin rampa de llenado, justo lo que la regla de
 * TIEMPO_LLENADO_S prohibe (la "zona 2" se elimino el 2026-10-01). */
#define AUTOTUNE2_VAL_ESCALON_PSI        5.0f
#define AUTOTUNE2_VAL_PRESION_MAX_FRAC   0.95f
#define AUTOTUNE2_VAL_MIN_SOBRE_BASE_PSI 5.0f    /* tras volver a ralenti debe haber bajado al menos esto bajo el objetivo */
#define AUTOTUNE2_BANDA_MIN_PSI          2.5f    /* piso anti-ruido, sale del ruido real del log COM3_2026_09_21 */
#define AUTOTUNE2_BANDA_FRAC             0.05f
#define AUTOTUNE2_SOBREPICO_MAX_PCT     15.0f
#define AUTOTUNE2_CRUCES_MAX             2U
#define AUTOTUNE2_OSC_VENTANA_MS    120000U      /* con L~10 s el periodo de una oscilacion es >= ~40 s: 4 cruces en 2 min son 2 ciclos */

/* ==================== ESTADO ==================== */

typedef enum {
    ST_ESPERA = 0,
    ST_BASE,
    ST_SUBIDA,
    ST_VOLVER,      /* RPM_MIN, esperando presion estable; despues ID o validacion */
    ST_ID,          /* escalon cerrado con la Kp de prueba, grabando */
    ST_VAL          /* escalon con la candidata, grabando + vigia */
} Estado_t;

static uint8_t   s_etapa = CALIB_AUTOTUNE_PID2; /* CALIB que se esta corriendo, ver autotune_psi.h */
static Estado_t  s_estado = ST_ESPERA;
static Estado_t  s_siguienteTrasVolver = ST_VAL;
static uint32_t  s_t0Ms = 0U;
static Autotune_Grabador_t s_grab;   /* muestras de la fase en curso (ID o validacion) */
static float     s_acumBase = 0.0f;
static uint32_t  s_nBase = 0U;
static float     s_psiBase = 0.0f;

/* Subida */
static float     s_tasaPsiS = 0.0f;
static float     s_timeoutSubidaS = 0.0f;
static float     s_tasaRpmS = 0.0f;
static float     s_avanceS = 0.0f;
static uint32_t  s_inicioSubidaMs = 0U;
static uint32_t  s_ultimoTickMs = 0U;
static uint32_t  s_techoDesdeMs = 0U;  /* 0 = todavia no toco RPM_MAX_CARGA */
static float     s_rpmCmd = 0.0f;
static float     s_ultimoPsiLog = 0.0f;
static float     s_tramoRpm[AUTOTUNE2_CURVA_TRAMOS + 1U]; /* RPM al cruzar el umbral i (0 = base) */
static uint8_t   s_tramosCruzados = 0U;

/* Kp de prueba que deja CALIB=9 (o la subida de CALIB=14) para el escalon
 * cerrado. Solo RAM: se pierde al reiniciar, y entonces CALIB=10 pide correr
 * CALIB=9 de nuevo. */
static bool      s_kpPruebaValida = false;
static float     s_kpPrueba = 0.0f;

/* Detector de estabilidad */
static Autotune_Estable_t s_estable;

/* ID / SIMC / VAL */
static float     s_y0 = 0.0f;
static Autotune_Planta_t s_planta;
static uint8_t   s_intento = 0U;
static float     s_tauC = 0.0f;
static float     s_kpCand = 0.0f;
static float     s_kiCand = 0.0f;
static float     s_yInicioVal = 0.0f;
static float     s_objetivoVal = 0.0f;
static float     s_bandaVal = 0.0f;
static bool      s_cascada = false;
static Autotune_Osc_t s_osc;
static Autotune_Resultado_t s_ultimo = AUTOTUNE_SIN_RESULTADO;

/* ==================== AUXILIARES ==================== */

static void terminar(Autotune_Resultado_t r)
{
    AutotunePsi_Limpiar();
    s_ultimo = r;
    s_estado = ST_ESPERA;
    CalibFlash_SetCalib(0U);
}

static void abortar(const char *motivo)
{
    printf("AUTOTUNE_PID2,ABORTADO,motivo=%s\r\n", motivo);
    terminar(AUTOTUNE_RESULTADO_FALLO);
}

static void empezarGrabacion(uint32_t ahora, uint16_t cuantas, uint16_t dtMs)
{
    Autotune_GrabadorIniciar(&s_grab, ahora, cuantas, dtMs);
}

/* Alimentar en cada pasada; devuelve true cuando la presion ya se asento.
 * Al devolver true, *promedio queda con el promedio de los ultimos 10 s. */
static bool estableActualizar(uint32_t ahora, float *promedio)
{
    return Autotune_EstableActualizar(&s_estable, ahora, PresionV_GetPresionPsi(),
                                      AUTOTUNE2_ESTABLE_TOL_PSI, promedio);
}

static void calcularCandidata(void)
{
    Autotune_CandidataSimc(&s_planta, s_tauC, AUTOTUNE2_KP_MIN, AUTOTUNE2_GANANCIA_MAX, &s_kpCand, &s_kiCand);
    printf("AUTOTUNE_PID2,CANDIDATA,intento=%u,tau_c=%.2f,kp=%.4f,ki=%.4f\r\n",
           (unsigned)(s_intento + 1U), s_tauC, s_kpCand, s_kiCand);
}

/* Motor en RPM_MIN (sin cascada) y esperar a que la presion se asiente;
 * despues arranca 'proximo' (ST_ID o ST_VAL). RPM_MIN y no SET_RPM=0: es el
 * piso de presion_pid.c (salida = RPM_MIN + Kp*e), asi el escalon cerrado
 * arranca desde la misma entrada de planta que tenia la base. */
static void irAVolver(uint32_t ahora, Estado_t proximo)
{
    s_cascada = false;
    CalibFlash_SetSetRpm(CalibFlash_GetRpmMin());
    s_siguienteTrasVolver = proximo;
    s_estado = ST_VOLVER;
    s_t0Ms = ahora;
    Autotune_EstableIniciar(&s_estable, ahora);
}

static void siguienteIntento(const char *motivo, uint32_t ahora)
{
    printf("AUTOTUNE_PID2,INTENTO_RECHAZADO,intento=%u,motivo=%s\r\n",
           (unsigned)(s_intento + 1U), motivo);
    if (s_etapa == CALIB_PID2_VALIDAR) {
        /* Las ganancias guardadas son fijas: no hay candidata mas conservadora que probar. */
        printf("AUTOTUNE_PID2,VALIDAR_FALLO,motivo=%s\r\n", motivo);
        terminar(AUTOTUNE_RESULTADO_FALLO);
        return;
    }
    s_intento++;
    if (s_intento >= AUTOTUNE2_INTENTOS_MAX) {
        printf("AUTOTUNE_PID2,FALLO,motivo=ninguna_candidata_paso,ganancias_sin_cambios=1\r\n");
        terminar(AUTOTUNE_RESULTADO_FALLO);
        return;
    }
    s_tauC *= AUTOTUNE2_TAU_C_ESCALA_REINTENTO;
    calcularCandidata();
    irAVolver(ahora, ST_VAL);
}

/* Arranca la cascada real con 'kp'/'ki' en RAM y el objetivo de prueba en
 * RAM (AutotunePsi_ObjetivoPsi), y graba 'cuantas' muestras. */
static void arrancarCascada(uint32_t ahora, float kp, float ki, uint16_t cuantas, uint16_t dtMs)
{
    PresionPid_SetGananciasTemporales(kp, ki);
    empezarGrabacion(ahora, cuantas, dtMs);
    s_t0Ms = ahora;
    s_cascada = true; /* main.c corre presion_pid.c desde la proxima vuelta */
}

/* Escalon CERRADO de identificacion: +5 PSI desde la presion estable en
 * RPM_MIN, Kp de prueba, Ki=0 -- el mismo ensayo con el que salio la
 * 1.72/0.39 de campo. */
static void empezarIdentificacion(uint32_t ahora, float y0)
{
    s_y0 = y0;
    s_objetivoVal = y0 + AUTOTUNE2_ID_ESCALON_PSI;
    if (s_objetivoVal >= CalibFlash_GetPresionMax() * AUTOTUNE2_VAL_PRESION_MAX_FRAC) {
        abortar("objetivo_cerca_de_presion_max");
        return;
    }
    arrancarCascada(ahora, s_kpPrueba, 0.0f, AUTOTUNE2_ID_MUESTRAS, AUTOTUNE2_ID_DT_MS);
    s_estado = ST_ID;
    printf("AUTOTUNE_PID2,ESCALON,y0=%.2f,objetivo=%.2f,kp_prueba=%.4f,ki=0,duracion_s=%lu\r\n",
           s_y0, s_objetivoVal, s_kpPrueba,
           (unsigned long)((uint32_t)AUTOTUNE2_ID_MUESTRAS * AUTOTUNE2_ID_DT_MS / 1000U));
}

static void empezarValidacion(uint32_t ahora, float yInicio)
{
    float objetivo = CalibFlash_GetPresionObjetivoLocal();
    float t0 = yInicio + AUTOTUNE2_VAL_ESCALON_PSI;

    if (objetivo - yInicio < AUTOTUNE2_VAL_MIN_SOBRE_BASE_PSI) {
        abortar("la_presion_no_bajo_en_ralenti");
        return;
    }
    /* +5 PSI desde ralenti; si eso ya pasa el objetivo, usa el objetivo. */
    s_objetivoVal = (t0 < objetivo) ? t0 : objetivo;
    if (s_objetivoVal >= CalibFlash_GetPresionMax() * AUTOTUNE2_VAL_PRESION_MAX_FRAC) {
        abortar("objetivo_cerca_de_presion_max");
        return;
    }

    s_yInicioVal = yInicio;
    s_bandaVal = (s_objetivoVal - yInicio) * AUTOTUNE2_BANDA_FRAC;
    if (s_bandaVal < AUTOTUNE2_BANDA_MIN_PSI) s_bandaVal = AUTOTUNE2_BANDA_MIN_PSI;
    Autotune_OscReiniciar(&s_osc);
    arrancarCascada(ahora, s_kpCand, s_kiCand, AUTOTUNE2_VAL_MUESTRAS, AUTOTUNE2_VAL_DT_MS);
    s_estado = ST_VAL;
    printf("AUTOTUNE_PID2,VALIDANDO,intento=%u,psi_inicio=%.2f,objetivo=%.2f,kp=%.4f,ki=%.4f\r\n",
           (unsigned)(s_intento + 1U), yInicio, s_objetivoVal, s_kpCand, s_kiCand);
}

static void evaluarValidacion(uint32_t ahora)
{
    s_cascada = false;
    CalibFlash_SetSetRpm(CalibFlash_GetRpmMin());

    Autotune_Metricas_t m;
    Autotune_EvaluarEscalon(Autotune_Buffer(), s_grab.n, AUTOTUNE2_VAL_DT_MS / 1000.0f, s_objetivoVal, s_yInicioVal,
            AUTOTUNE2_BANDA_MIN_PSI, AUTOTUNE2_BANDA_FRAC, AUTOTUNE2_VENTANA_FINAL_S,
            AUTOTUNE2_VAL_SEMIVENTANA, &m);

    bool aprobada = false;
    const char *motivo = Autotune_Veredicto(&m, AUTOTUNE2_CRUCES_MAX, AUTOTUNE2_SOBREPICO_MAX_PCT, &aprobada);

    printf("AUTOTUNE_PID2,VALIDACION,intento=%u,sobreimpulso_pct=%.1f,t_asent_s=%.1f,err_final_psi=%.2f,cruces=%u,veredicto=%s\r\n",
           (unsigned)(s_intento + 1U), m.sobrepicoPct, m.tAsentS,
           m.errFinal, (unsigned)m.cruces, motivo);

    if (!aprobada) {
        siguienteIntento(motivo, ahora);
        return;
    }

    if (s_etapa == CALIB_PID2_VALIDAR) {
        printf("AUTOTUNE_PID2,VALIDAR_OK,PID_PSI_KP=%.4f,PID_PSI_KI=%.4f\r\n", s_kpCand, s_kiCand);
        terminar(AUTOTUNE_RESULTADO_OK);
        return;
    }

    if (!CalibFlash_SetPidPsiGanancias(s_kpCand, s_kiCand)) {
        abortar("error_al_guardar_en_flash");
        return;
    }
    printf("AUTOTUNE_PID2,OK,PID_PSI_KP=%.4f,PID_PSI_KI=%.4f,intentos=%u\r\n",
           s_kpCand, s_kiCand, (unsigned)(s_intento + 1U));
    terminar(AUTOTUNE_RESULTADO_OK);
}

static void analizarIdentificacion(uint32_t ahora)
{
    s_cascada = false;
    CalibFlash_SetSetRpm(CalibFlash_GetRpmMin());
    PresionPid_LimpiarGananciasTemporales();

    Autotune_IdResultado_t r = Autotune_IdentificarLazoCerrado(
            Autotune_Buffer(), s_grab.n, AUTOTUNE2_ID_DT_MS / 1000.0f, s_y0,
            AUTOTUNE2_ID_ESCALON_PSI, s_kpPrueba, AUTOTUNE2_VENTANA_FINAL_S,
            AUTOTUNE2_UMBRAL_MOV_PSI, AUTOTUNE2_ID_SEMIVENTANA, &s_planta);
    if (r != AUTOTUNE_ID_OK) {
        printf("AUTOTUNE_PID2,ID_FALLIDA,motivo=%s\r\n", Autotune_IdNombre(r));
        abortar("identificacion_fallida");
        return;
    }
    printf("AUTOTUNE_PID2,ID,K=%.5f,tau=%.2f,L=%.2f,G_cl=%.3f,y0=%.2f,kp_prueba=%.4f\r\n",
           s_planta.k, s_planta.tau, s_planta.l, s_planta.gcl, s_y0, s_kpPrueba);

    /* Fila "Medio" PURA con la K del escalon, igual que la 1.72/0.39 de campo. */
    s_intento = 0U;
    s_tauC = AUTOTUNE2_TAU_C_FACTOR_L * s_planta.l;
    if (s_tauC < AUTOTUNE2_TAU_C_MIN_S) s_tauC = AUTOTUNE2_TAU_C_MIN_S;
    calcularCandidata();

    if (s_etapa == CALIB_PID2_ID) {
        printf("AUTOTUNE_PID2,ID_COMPLETO,kp=%.4f,ki=%.4f,guardado=0\r\n", s_kpCand, s_kiCand);
        terminar(AUTOTUNE_RESULTADO_OK);
        return;
    }
    irAVolver(ahora, ST_VAL);
}

/* K (PSI/RPM) de cada tramo de la subida -> Kp de prueba. Usa la K del
 * PRIMER tramo (donde ocurre el escalon); si ese tramo no sirve, la K global
 * de toda la subida. Devuelve false si ninguna sirve. */
static bool calcularKpPrueba(float psiObjetivo)
{
    float dPsi = (psiObjetivo - s_psiBase) / (float)AUTOTUNE2_CURVA_TRAMOS;
    float kMin = 0.0f, kMax = 0.0f;
    uint8_t validos = 0U;
    for (uint8_t i = 1U; i < s_tramosCruzados; i++) {
        float dRpm = s_tramoRpm[i] - s_tramoRpm[i - 1U];
        if (dRpm < AUTOTUNE2_CURVA_MIN_DRPM) continue;
        float k = dPsi / dRpm;
        if (validos == 0U || k < kMin) kMin = k;
        if (validos == 0U || k > kMax) kMax = k;
        validos++;
    }
    float dRpm1 = (s_tramosCruzados > 1U) ? (s_tramoRpm[1] - s_tramoRpm[0]) : 0.0f;
    float dRpmTotal = s_rpmCmd - s_tramoRpm[0];
    float k;
    const char *fuente;
    if (dRpm1 >= AUTOTUNE2_CURVA_MIN_DRPM) {
        k = dPsi / dRpm1;
        fuente = "tramo1";
    } else if (dRpmTotal >= AUTOTUNE2_CURVA_MIN_DRPM) {
        k = (psiObjetivo - s_psiBase) / dRpmTotal;
        fuente = "global";
    } else {
        return false;
    }
    if (!(k > 0.0f)) return false;

    s_kpPrueba = AUTOTUNE2_GCL_PRUEBA / (k * (1.0f - AUTOTUNE2_GCL_PRUEBA));
    if (s_kpPrueba < AUTOTUNE2_KP_PRUEBA_MIN) s_kpPrueba = AUTOTUNE2_KP_PRUEBA_MIN;
    if (s_kpPrueba > AUTOTUNE2_KP_PRUEBA_MAX) s_kpPrueba = AUTOTUNE2_KP_PRUEBA_MAX;
    s_kpPruebaValida = true;
    printf("AUTOTUNE_PID2,CURVA,tramos_validos=%u,k_min=%.5f,k_max=%.5f,k_usada=%.5f,fuente=%s,kp_prueba=%.4f\r\n",
           (unsigned)validos, kMin, kMax, k, fuente, s_kpPrueba);
    return true;
}

/* ==================== API ==================== */

void AutotunePsi_Iniciar(uint8_t etapa)
{
    AutotunePsi_Limpiar();
    s_etapa = etapa;
    s_estado = ST_ESPERA;
    s_intento = 0U;
    s_ultimo = AUTOTUNE_SIN_RESULTADO;
    printf("AUTOTUNE_PID2,INICIO,calib=%u,pid_psi_kp_actual=%.4f,pid_psi_ki_actual=%.4f\r\n",
           (unsigned)etapa, CalibFlash_GetPidPsiKp(), CalibFlash_GetPidPsiKi());
}

void AutotunePsi_Limpiar(void)
{
    PresionPid_LimpiarGananciasTemporales();
    s_cascada = false;
    if (s_estado != ST_ESPERA) {
        CalibFlash_SetSetRpm(0.0f);
        s_estado = ST_ESPERA; /* cancelada desde afuera a mitad de ensayo */
    }
}

Autotune_Resultado_t AutotunePsi_UltimoResultado(void) { return s_ultimo; }
bool  AutotunePsi_CascadaActiva(void) { return s_cascada; }
float AutotunePsi_ObjetivoPsi(void)   { return s_objetivoVal; }

void AutotunePsi_Paso(bool motorOperando, bool modoCalibracion)
{
    uint32_t ahora = HAL_GetTick();

    if (s_estado != ST_ESPERA) {
        if (!motorOperando)            { abortar("motor_se_detuvo");   return; }
        if (!modoCalibracion)          { abortar("modo_cambio");       return; }
        if (!PresionV_SensorValido())  { abortar("sensor_invalido");   return; }
    }

    switch (s_estado) {
    case ST_ESPERA:
        if (motorOperando && modoCalibracion) {
            if (CalibFlash_GetPidRpmKp() <= 0.0f) {
                abortar("pid1_sin_calibrar_correr_CALIB_13_primero");
                break;
            }
            if (s_etapa == CALIB_PID2_VALIDAR && CalibFlash_GetPidPsiKp() <= 0.0f) {
                abortar("PID_PSI_KP_sin_cargar_no_hay_que_validar");
                break;
            }
            if (s_etapa == CALIB_PID2_ID && !s_kpPruebaValida) {
                abortar("sin_kp_de_prueba_correr_CALIB_9_primero");
                break;
            }
            CalibFlash_SetSetRpm(CalibFlash_GetRpmMin()); /* motor posado en RPM_MIN */
            s_acumBase = 0.0f;
            s_nBase = 0U;
            s_t0Ms = ahora;
            s_estado = ST_BASE;
            printf("AUTOTUNE_PID2,BASE,duracion_s=%lu\r\n", (unsigned long)(AUTOTUNE2_BASE_MS / 1000U));
        }
        break;

    case ST_BASE:
        s_acumBase += PresionV_GetPresionPsi();
        s_nBase++;
        if (ahora - s_t0Ms >= AUTOTUNE2_BASE_MS) {
            s_psiBase = s_acumBase / (float)s_nBase;
            float objetivo = CalibFlash_GetPresionObjetivoLocal();
            float tiempoLlenadoS = (float)CalibFlash_GetTiempoLlenadoS();
            float rpmMin = CalibFlash_GetRpmMin();
            float rpmMaxCarga = CalibFlash_GetRpmMaxCarga();
            printf("AUTOTUNE_PID2,BASE_MEDIDA,psi_base=%.2f,objetivo=%.2f\r\n", s_psiBase, objetivo);

            if (s_etapa == CALIB_PID2_ID) {
                /* Ya hay Kp de prueba (CALIB=9): directo al escalon cerrado. */
                irAVolver(ahora, ST_ID);
                break;
            }
            if (objetivo - s_psiBase < AUTOTUNE2_OBJETIVO_MIN_SOBRE_BASE_PSI) {
                abortar("PRESION_OBJETIVO_LOCAL_muy_cerca_de_la_presion_base");
                break;
            }
            if (s_etapa == CALIB_PID2_VALIDAR) {
                /* Valida lo guardado: sin subida (la validacion es +5 PSI desde
                 * ralenti), no necesita TIEMPO_LLENADO_S. */
                s_kpCand = CalibFlash_GetPidPsiKp();
                s_kiCand = CalibFlash_GetPidPsiKi();
                irAVolver(ahora, ST_VAL);
                break;
            }

            /* Ritmo de subida = el MISMO que usa la rampa de MODO=1
             * (PRESION_OBJETIVO_LOCAL / TIEMPO_LLENADO_S, ver
             * CalibFlash_GetTasaLlenadoPsiS): duracion dinamica, y nunca mas
             * rapido que el ritmo elegido por el operador. */
            float tasaPsiS = CalibFlash_GetTasaLlenadoPsiS();
            if (tiempoLlenadoS <= 0.0f || tasaPsiS <= 0.0f || rpmMaxCarga <= rpmMin) {
                abortar("config_invalida_TIEMPO_LLENADO_S_o_RPM_MAX_CARGA");
                break;
            }
            s_tasaPsiS = tasaPsiS;
            s_timeoutSubidaS = ((objetivo - s_psiBase) / tasaPsiS) * AUTOTUNE2_SUBIDA_TIMEOUT_MARGEN;
            s_tasaRpmS = (rpmMaxCarga - rpmMin) / tiempoLlenadoS;
            s_avanceS = 0.0f;
            s_rpmCmd = rpmMin;
            s_inicioSubidaMs = ahora;
            s_ultimoTickMs = ahora;
            s_techoDesdeMs = 0U;
            s_ultimoPsiLog = s_psiBase;
            s_tramoRpm[0] = rpmMin;
            s_tramosCruzados = 1U;
            s_kpPruebaValida = false; /* se recalcula con esta subida */
            s_estado = ST_SUBIDA;
            printf("AUTOTUNE_PID2,SUBIDA,tasa_psi_s=%.4f,tasa_rpm_s=%.4f,rpm_max_carga=%.1f,timeout_s=%.0f\r\n",
                   s_tasaPsiS, s_tasaRpmS, rpmMaxCarga, s_timeoutSubidaS);
        }
        break;

    case ST_SUBIDA: {
        /* Rampa de SET_RPM pausada por PSI real: mientras el PSI vaya por
         * debajo de la linea ideal de llenado sigue avanzando; al alcanzarla,
         * sostiene. El PSI nunca sube mas rapido que el ritmo que el operador
         * definio como seguro para esa tuberia. */
        float dtS = (float)(ahora - s_ultimoTickMs) / 1000.0f;
        s_ultimoTickMs = ahora;
        float tRampaS = (float)(ahora - s_inicioSubidaMs) / 1000.0f;
        float psi = PresionV_GetPresionPsi();
        float rpmMaxCarga = CalibFlash_GetRpmMaxCarga();
        float objetivo = CalibFlash_GetPresionObjetivoLocal();

        if (psi < s_psiBase + s_tasaPsiS * tRampaS) {
            s_avanceS += dtS;
            s_rpmCmd = CalibFlash_GetRpmMin() + s_tasaRpmS * s_avanceS;
            if (s_rpmCmd >= rpmMaxCarga) {
                s_rpmCmd = rpmMaxCarga;
                if (s_techoDesdeMs == 0U) s_techoDesdeMs = ahora;
            }
            CalibFlash_SetSetRpm(s_rpmCmd);
        }

        if (fabsf(psi - s_ultimoPsiLog) >= AUTOTUNE2_CURVA_PASO_LOG_PSI) {
            /* Formato del viejo CALIB=9: sirve para `pid_tuning.py --ganancia-log`. */
            printf("PRESION_GANANCIA_CAL,PASO,rpm=%.1f,psi=%.2f,salto=%.2f\r\n", s_rpmCmd, psi, psi - s_ultimoPsiLog);
            s_ultimoPsiLog = psi;
        }
        /* Cruce de cada umbral de presion: guarda el RPM en ese instante. */
        while (s_tramosCruzados <= AUTOTUNE2_CURVA_TRAMOS
               && psi >= s_psiBase + (objetivo - s_psiBase) * (float)s_tramosCruzados / (float)AUTOTUNE2_CURVA_TRAMOS) {
            s_tramoRpm[s_tramosCruzados] = s_rpmCmd;
            s_tramosCruzados++;
        }

        if (psi >= objetivo) {
            printf("AUTOTUNE_PID2,SUBIDA_FIN,rpm_operacion=%.1f,psi=%.2f,duracion_s=%lu\r\n",
                   s_rpmCmd, psi, (unsigned long)((ahora - s_inicioSubidaMs) / 1000U));
            if (!calcularKpPrueba(objetivo)) {
                abortar("curva_sin_datos_para_kp_de_prueba");
                break;
            }
            if (s_etapa == CALIB_PID2_CURVA) {
                terminar(AUTOTUNE_RESULTADO_OK); /* SET_RPM=0: vuelve a ralenti; la Kp de prueba queda en RAM para CALIB=10 */
                break;
            }
            irAVolver(ahora, ST_ID);
        } else if (s_techoDesdeMs != 0U && ahora - s_techoDesdeMs >= AUTOTUNE2_SUBIDA_ESPERA_TECHO_MS) {
            abortar("objetivo_no_alcanzable_con_RPM_MAX_CARGA");
        } else if (tRampaS >= s_timeoutSubidaS) {
            abortar("timeout_en_la_subida");
        }
        break;
    }

    case ST_VOLVER: {
        float prom = 0.0f;
        if (estableActualizar(ahora, &prom)) {
            if (s_siguienteTrasVolver == ST_ID) {
                empezarIdentificacion(ahora, prom);
            } else {
                empezarValidacion(ahora, prom);
            }
        } else if (ahora - s_t0Ms >= AUTOTUNE2_VOLVER_TIMEOUT_MS) {
            abortar("presion_no_se_estabiliza_en_ralenti");
        }
        break;
    }

    case ST_ID:
        Autotune_GrabadorMuestrear(&s_grab, ahora, PresionV_GetPresionPsi);
        if (Autotune_GrabadorLleno(&s_grab)) {
            analizarIdentificacion(ahora);
        }
        break;

    case ST_VAL:
        Autotune_GrabadorMuestrear(&s_grab, ahora, PresionV_GetPresionPsi);
        /* Vigia en vivo: oscilacion sostenida -> cortar ya, sin esperar los 240 s. */
        if (Autotune_OscActualizar(&s_osc, PresionV_GetPresionPsi() - s_objetivoVal,
                                   s_bandaVal, ahora, AUTOTUNE2_OSC_VENTANA_MS)) {
            s_cascada = false;
            CalibFlash_SetSetRpm(CalibFlash_GetRpmMin());
            siguienteIntento("oscila_en_vivo", ahora);
        } else if (Autotune_GrabadorLleno(&s_grab)) {
            evaluarValidacion(ahora);
        }
        break;
    }
}
