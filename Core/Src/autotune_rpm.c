/**
 * @file    autotune_rpm.c
 * @brief   Autosintonia del lazo interno RPM -> servo (PID#1) -- ver autotune_rpm.h.
 */

/* Compilado siempre con -Os, tambien en Debug (-O0), para que el binario
 * Debug quepa en los 126 KB que declara el .ld (la ultima pagina, 0x0801F800,
 * es la de calibracion: si el programa crece de mas, falla al enlazar). */
#pragma GCC optimize ("Os")

#include "autotune_rpm.h"
#include "autotune.h"
#include "calibracion_flash.h"
#include "tacometro.h"
#include "servo.h"
#include "pid.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

/* ==================== PARAMETROS ==================== */

/* Escalon y base: +200 RPM sobre RPM_MIN, el mismo escalon que usaba el
 * flujo manual (viejos CALIB 6/7), para que los logs sigan siendo comparables. */
#define AUTOTUNE1_ESCALON_RPM           200.0f
#define AUTOTUNE1_ESCALON_MIN_UTIL_RPM   50.0f   /* si sp - y0 queda por debajo, el ensayo no dice nada */
#define AUTOTUNE1_BASE_MS               3000U    /* promedio de RPM en ralenti antes de cualquier ensayo */

/* Barrido de curva de ganancia en lazo ABIERTO (mismo metodo y mismos
 * valores que el viejo CALIB=5 manual, que ahora es esta misma etapa): pulsos crecientes desde
 * SERVO_PULSO_MIN, (pulso, RPM) al final de cada paso, hasta RPM_MAX. De
 * ahi sale la K LOCAL maxima (peor caso), que reemplaza a la K del
 * escalon en SIMC -- igual que `pid_tuning.py auto --ganancia-log`: un
 * escalon da la K de UN punto del rango, y unas ganancias buenas en la
 * zona de poca ganancia pueden oscilar en la de mucha. */
#define AUTOTUNE1_CURVA_PASO_US             4U
#define AUTOTUNE1_CURVA_DWELL_MS         2500U
#define AUTOTUNE1_CURVA_SALTO_ANORMAL_RPM 150.0f
#define AUTOTUNE1_CURVA_PASOS_MIN           4U   /* menos puntos que esto = curva inutil */

/* Muestreo de los escalones: 50 ms x 600 = 30 s. */
#define AUTOTUNE1_DT_MS                   50U
#define AUTOTUNE1_MUESTRAS              AUTOTUNE_MAX_MUESTRAS
#define AUTOTUNE1_VENTANA_FINAL_S         5.0f
#define AUTOTUNE1_UMBRAL_MOV_RPM          5.0f   /* mismo piso que `identify` (--umbral-movimiento-rpm) */
#define AUTOTUNE1_SEMIVENTANA               2U   /* promedio movil de 5 muestras (0.25 s) para decidir "se movio"/"63%" */

/* Identificacion en LAZO ABIERTO: escalon directo del pulso del servo (sin
 * PID), del tamaño que en la curva dio +200 RPM sobre la base. NO se usa el
 * escalon con Kp=1/Ki=0 de CALIB=6: en una semi-simulacion calibrada contra
 * lo que paso en campo (Kp=0.25/Ki=0.57 oscilaba en 1300 RPM, 0.047/0.11 no
 * -- README seccion 9), ese lazo P con Kp=1 OSCILA (K de 2 a 7 RPM/us con
 * ~0.5-0.8 s de retardo no modelado), y la identificacion salia invalida o
 * de pura suerte. En lazo abierto no hay nada que pueda oscilar. */
#define AUTOTUNE1_ID_SUBIDA_RPM         200.0f
#define AUTOTUNE1_ID_SUBIDA_MIN_RPM      50.0f   /* si la curva nunca subio al menos esto, no hay escalon util */

/* SIMC: tau_c = max(2L, 3*tau). El 3*tau es la eleccion que YA se valido en
 * el motor real (README seccion 9: Kp=0.047/Ki=0.11 salieron de
 * tau_c = 3*tau sobre la K peor caso, despues de que 0.25/0.57 oscilara en
 * 1300 RPM) -- asi la autosintonia parte de lo conocido y debe dar
 * ganancias cercanas a esas. El 2L (fila "Medio") solo manda si el tiempo
 * muerto medido es grande. Hace falta ese piso porque con L casi cero
 * (como dio `identify` en este motor) el Medio puro no se puede calcular /
 * quedaba mucho mas agresivo que lo validado. Cada reintento duplica tau_c. */
#define AUTOTUNE1_TAU_C_FACTOR_L          2.0f
#define AUTOTUNE1_TAU_C_FACTOR_TAU        3.0f
/* Piso de L = 0.15 s: el MISMO que se puso a mano al sacar 0.047/0.11 (README
 * seccion 9: `identify` dio L=0 y se uso 0.15 s, del orden del periodo de
 * muestreo). Con K=6.3, tau=0.43, L=0.15 y tau_c=3*tau, SIMC da exactamente
 * Kp=0.047 / Ki=0.110 -- la receta del campo, paso por paso. */
#define AUTOTUNE1_L_MIN_S                 0.15f
#define AUTOTUNE1_TAU_C_MIN_S             0.30f
#define AUTOTUNE1_TAU_C_ESCALA_REINTENTO  2.0f
#define AUTOTUNE1_INTENTOS_MAX              3U
#define AUTOTUNE1_KP_MIN                  0.001f
#define AUTOTUNE1_GANANCIA_MAX           30.0f   /* el protocolo codifica x1000 en int16: tope 32.767 */

/* Volver a la base entre ensayos (SET_RPM=0 -> el servo baja a MIN solo). */
#define AUTOTUNE1_VOLVER_BANDA_RPM       15.0f
#define AUTOTUNE1_VOLVER_ESTABLE_MS     2000U
#define AUTOTUNE1_VOLVER_TIMEOUT_MS    60000U   /* tras el barrido el motor baja desde RPM_MAX */

/* Validacion 2 (zona de maxima ganancia): escalon hasta la RPM donde el
 * barrido encontro el peor caso. Solo se hace si queda al menos esto por
 * encima del escalon 1 -- si no, es la misma prueba. */
#define AUTOTUNE1_VAL2_MIN_SEPARACION_RPM 100.0f

/* Criterio de aprobacion de cada validacion en el motor real. Que la
 * respuesta sea lenta NO reprueba (K_peor_caso > K del escalon hace que el
 * lazo sea mas lento en la zona de poca ganancia, por diseño): se reprueba
 * por oscilar, pasarse de largo, o no asentar dentro de la ventana. */
#define AUTOTUNE1_BANDA_MIN_RPM          10.0f   /* piso anti-ruido de la banda de asentamiento */
#define AUTOTUNE1_BANDA_FRAC              0.05f  /* banda = max(piso, 5% del escalon) */
#define AUTOTUNE1_SOBREPICO_MAX_PCT      25.0f
#define AUTOTUNE1_CRUCES_MAX                2U   /* mas de 2 cruces de la banda = esta oscilando, no asentando */
#define AUTOTUNE1_OSC_VENTANA_MS        15000U   /* vigia en vivo: 4 cruces en 15 s aborta el intento de inmediato */

/* ==================== ESTADO ==================== */

typedef enum {
    ST_ESPERA = 0,  /* esperando motor operando + MODO=CALIBRACION */
    ST_BASE,        /* midiendo RPM base en ralenti */
    ST_CURVA,       /* barrido de ganancia, lazo abierto (el servo lo mueve ESTA rutina) */
    ST_VOLVER,      /* SET_RPM=0, esperando volver a la base */
    ST_ID,          /* escalon del pulso en lazo abierto, grabando */
    ST_VAL          /* escalon con la candidata, grabando + vigia */
} Estado_t;

static uint8_t   s_etapa = CALIB_AUTOTUNE_PID1; /* CALIB que se esta corriendo, ver autotune_rpm.h */
static Estado_t  s_estado = ST_ESPERA;
static Estado_t  s_siguienteTrasVolver = ST_ID;
static uint32_t  s_t0Ms = 0U;
static uint32_t  s_estableDesdeMs = 0U;
static Autotune_Grabador_t s_grab;
static float     s_acumBase = 0.0f;
static uint32_t  s_nBase = 0U;
static float     s_y0 = 0.0f;
static float     s_sp = 0.0f;       /* escalon 1 (ID y validacion 1) */
static float     s_sp2 = 0.0f;      /* escalon 2 (validacion en zona de maxima ganancia), 0 = no aplica */
static float     s_spVal = 0.0f;    /* escalon de la validacion en curso */
static uint8_t   s_valFase = 0U;    /* 0 = validacion 1, 1 = validacion 2 */
static uint16_t  s_pulsoId = 0U;    /* pulso del escalon de identificacion (0 = todavia no encontrado en la curva) */

/* Barrido */
static uint16_t  s_pulsoPaso = 0U;
static uint16_t  s_pulsoPrev = 0U;
static float     s_rpmPrev = 0.0f;
static uint16_t  s_pasos = 0U;
static float     s_kPeorCurva = 0.0f;
static float     s_rpmEnPeor = 0.0f;
/* Curva valida que deja CALIB=5 (o la de CALIB=13) para CALIB=6: solo RAM,
 * se pierde al reiniciar. Se invalida si cambio SERVO_PULSO_MIN (el escalon
 * se mide desde ahi). */
static bool      s_curvaValida = false;
static uint16_t  s_curvaPulsoMin = 0U;

static Autotune_Planta_t s_planta;
static uint8_t   s_intento = 0U;
static float     s_tauC = 0.0f;
static float     s_kpCand = 0.0f;
static float     s_kiCand = 0.0f;
static float     s_bandaVal = 0.0f;
static Autotune_Osc_t s_osc;
static Autotune_Resultado_t s_ultimo = AUTOTUNE_SIN_RESULTADO;

/* ==================== AUXILIARES ==================== */

static void terminar(Autotune_Resultado_t r)
{
    AutotuneRpm_Limpiar();
    s_ultimo = r;
    s_estado = ST_ESPERA;
    CalibFlash_SetCalib(0U);
}

static void abortar(const char *motivo)
{
    printf("AUTOTUNE_PID1,ABORTADO,motivo=%s\r\n", motivo);
    terminar(AUTOTUNE_RESULTADO_FALLO);
}

static void empezarGrabacion(uint32_t ahora)
{
    Autotune_GrabadorIniciar(&s_grab, ahora, AUTOTUNE1_MUESTRAS, AUTOTUNE1_DT_MS);
}

/* Calcula la candidata SIMC con el tau_c vigente y la deja en s_kpCand/s_kiCand. */
static void calcularCandidata(void)
{
    Autotune_CandidataSimc(&s_planta, s_tauC, AUTOTUNE1_KP_MIN, AUTOTUNE1_GANANCIA_MAX, &s_kpCand, &s_kiCand);
    printf("AUTOTUNE_PID1,CANDIDATA,intento=%u,tau_c=%.3f,kp=%.4f,ki=%.4f\r\n",
           (unsigned)(s_intento + 1U), s_tauC, s_kpCand, s_kiCand);
}

/* Apaga el escalon y espera a que la RPM regrese a la base; 'proximo' dice
 * que ensayo arranca despues. */
static void irAVolver(uint32_t ahora, Estado_t proximo)
{
    CalibFlash_SetSetRpm(0.0f);
    s_siguienteTrasVolver = proximo;
    s_estado = ST_VOLVER;
    s_t0Ms = ahora;
    s_estableDesdeMs = 0U;
}

static void empezarValidacion(uint32_t ahora)
{
    float delta = s_spVal - s_y0;
    s_bandaVal = delta * AUTOTUNE1_BANDA_FRAC;
    if (s_bandaVal < AUTOTUNE1_BANDA_MIN_RPM) s_bandaVal = AUTOTUNE1_BANDA_MIN_RPM;
    Autotune_OscReiniciar(&s_osc);
    PID_SetGananciasTemporales(s_kpCand, s_kiCand);
    empezarGrabacion(ahora);
    s_estado = ST_VAL;
    printf("AUTOTUNE_PID1,VALIDANDO,intento=%u,zona=%u,sp=%.1f,kp=%.4f,ki=%.4f\r\n",
           (unsigned)(s_intento + 1U), (unsigned)(s_valFase + 1U), s_spVal, s_kpCand, s_kiCand);
    CalibFlash_SetSetRpm(s_spVal);
}

static void siguienteIntento(const char *motivo, uint32_t ahora)
{
    printf("AUTOTUNE_PID1,INTENTO_RECHAZADO,intento=%u,zona=%u,motivo=%s\r\n",
           (unsigned)(s_intento + 1U), (unsigned)(s_valFase + 1U), motivo);
    if (s_etapa == CALIB_PID1_VALIDAR) {
        /* Las ganancias guardadas son fijas: no hay candidata mas conservadora que probar. */
        printf("AUTOTUNE_PID1,VALIDAR_FALLO,motivo=%s\r\n", motivo);
        terminar(AUTOTUNE_RESULTADO_FALLO);
        return;
    }
    s_intento++;
    if (s_intento >= AUTOTUNE1_INTENTOS_MAX) {
        printf("AUTOTUNE_PID1,FALLO,motivo=ninguna_candidata_paso,ganancias_sin_cambios=1\r\n");
        terminar(AUTOTUNE_RESULTADO_FALLO);
        return;
    }
    s_tauC *= AUTOTUNE1_TAU_C_ESCALA_REINTENTO;
    s_valFase = 0U;
    calcularCandidata();
    irAVolver(ahora, ST_VAL);
    s_spVal = s_sp;
}

static void analizarIdentificacion(uint32_t ahora)
{
    float deltaPulso = (float)(s_pulsoId - CalibFlash_GetServoPulsoMinUs());
    Autotune_IdResultado_t r = Autotune_IdentificarLazoAbierto(
            Autotune_Buffer(), s_grab.n, AUTOTUNE1_DT_MS / 1000.0f, s_y0, deltaPulso,
            AUTOTUNE1_VENTANA_FINAL_S, AUTOTUNE1_UMBRAL_MOV_RPM, 0.2f,
            AUTOTUNE1_SEMIVENTANA, true /* dos puntos: tau sin sesgo, como el 0.43 de campo */, &s_planta);
    if (r != AUTOTUNE_ID_OK) {
        printf("AUTOTUNE_PID1,ID_FALLIDA,motivo=%s\r\n", Autotune_IdNombre(r));
        abortar("identificacion_fallida");
        return;
    }
    float kEscalon = s_planta.k;
    if (s_kPeorCurva > s_planta.k) s_planta.k = s_kPeorCurva; /* peor caso del barrido, como `auto --ganancia-log` */
    printf("AUTOTUNE_PID1,ID,K_escalon=%.6f,K_peor_curva=%.6f,K_usada=%.6f,tau=%.3f,L=%.3f,y0=%.1f,pulso_escalon=%u\r\n",
           kEscalon, s_kPeorCurva, s_planta.k, s_planta.tau, s_planta.l, s_y0, s_pulsoId);

    if (s_planta.l < AUTOTUNE1_L_MIN_S) {
        s_planta.l = AUTOTUNE1_L_MIN_S; /* misma correccion manual que en campo */
        printf("AUTOTUNE_PID1,L_PISO,L_usada=%.3f\r\n", s_planta.l);
    }

    s_intento = 0U;
    s_tauC = AUTOTUNE1_TAU_C_FACTOR_L * s_planta.l;
    if (s_tauC < AUTOTUNE1_TAU_C_FACTOR_TAU * s_planta.tau) s_tauC = AUTOTUNE1_TAU_C_FACTOR_TAU * s_planta.tau;
    if (s_tauC < AUTOTUNE1_TAU_C_MIN_S) s_tauC = AUTOTUNE1_TAU_C_MIN_S;
    calcularCandidata();

    if (s_etapa == CALIB_PID1_ID) {
        printf("AUTOTUNE_PID1,ID_COMPLETO,kp=%.4f,ki=%.4f,guardado=0\r\n", s_kpCand, s_kiCand);
        terminar(AUTOTUNE_RESULTADO_OK);
        return;
    }

    s_valFase = 0U;
    s_spVal = s_sp;
    irAVolver(ahora, ST_VAL);
}

static void evaluarValidacion(uint32_t ahora)
{
    CalibFlash_SetSetRpm(0.0f);

    Autotune_Metricas_t m;
    Autotune_EvaluarEscalon(Autotune_Buffer(), s_grab.n, AUTOTUNE1_DT_MS / 1000.0f, s_spVal, s_y0,
            AUTOTUNE1_BANDA_MIN_RPM, AUTOTUNE1_BANDA_FRAC, AUTOTUNE1_VENTANA_FINAL_S,
            AUTOTUNE1_SEMIVENTANA, &m);

    bool aprobada = false;
    const char *motivo = Autotune_Veredicto(&m, AUTOTUNE1_CRUCES_MAX, AUTOTUNE1_SOBREPICO_MAX_PCT, &aprobada);

    printf("AUTOTUNE_PID1,VALIDACION,intento=%u,zona=%u,sobreimpulso_pct=%.1f,t_asent_s=%.2f,err_final_rpm=%.1f,cruces=%u,veredicto=%s\r\n",
           (unsigned)(s_intento + 1U), (unsigned)(s_valFase + 1U), m.sobrepicoPct, m.tAsentS,
           m.errFinal, (unsigned)m.cruces, motivo);

    if (!aprobada) {
        siguienteIntento(motivo, ahora);
        return;
    }

    /* Zona 1 aprobada: falta la zona de maxima ganancia (si aplica). */
    if (s_valFase == 0U && s_sp2 > 0.0f) {
        s_valFase = 1U;
        s_spVal = s_sp2;
        irAVolver(ahora, ST_VAL);
        return;
    }

    if (s_etapa == CALIB_PID1_VALIDAR) {
        printf("AUTOTUNE_PID1,VALIDAR_OK,PID_RPM_KP=%.4f,PID_RPM_KI=%.4f\r\n", s_kpCand, s_kiCand);
        terminar(AUTOTUNE_RESULTADO_OK);
        return;
    }

    /* Todas las zonas aprobadas: unica escritura a flash de toda la rutina. */
    if (!CalibFlash_SetPidRpmGanancias(s_kpCand, s_kiCand)) {
        abortar("error_al_guardar_en_flash");
        return;
    }
    printf("AUTOTUNE_PID1,OK,PID_RPM_KP=%.4f,PID_RPM_KI=%.4f,intentos=%u\r\n",
           s_kpCand, s_kiCand, (unsigned)(s_intento + 1U));
    terminar(AUTOTUNE_RESULTADO_OK);
}

/* Cierra el barrido: valida la curva, fija el escalon 2 y vuelve a la base. */
static void terminarCurva(uint32_t ahora, const char *motivoFin)
{
    if (s_pasos < AUTOTUNE1_CURVA_PASOS_MIN || s_kPeorCurva <= 0.0f) {
        printf("AUTOTUNE_PID1,CURVA_INVALIDA,pasos=%u,k_peor=%.6f\r\n", (unsigned)s_pasos, s_kPeorCurva);
        abortar("curva_de_ganancia_invalida");
        return;
    }
    if (s_pulsoId == 0U) {
        /* Nunca llego a +200 RPM (techo bajo): usar el ultimo pulso, si al menos subio algo util. */
        if (s_rpmPrev - s_y0 < AUTOTUNE1_ID_SUBIDA_MIN_RPM) {
            abortar("curva_sin_subida_suficiente_para_identificar");
            return;
        }
        s_pulsoId = s_pulsoPrev;
    }
    /* Escalon 2 = RPM donde estuvo el peor caso, dentro del techo y con
     * suficiente separacion del escalon 1. */
    s_sp2 = s_rpmEnPeor;
    if (s_sp2 > CalibFlash_GetRpmMax()) s_sp2 = CalibFlash_GetRpmMax();
    if (s_sp2 < s_sp + AUTOTUNE1_VAL2_MIN_SEPARACION_RPM) s_sp2 = 0.0f;
    printf("AUTOTUNE_PID1,CURVA,fin=%s,pasos=%u,k_peor=%.6f,rpm_en_peor=%.1f,sp2=%.1f,pulso_escalon=%u\r\n",
           motivoFin, (unsigned)s_pasos, s_kPeorCurva, s_rpmEnPeor, s_sp2, s_pulsoId);
    s_curvaValida = true;
    s_curvaPulsoMin = CalibFlash_GetServoPulsoMinUs();
    if (s_etapa == CALIB_PID1_CURVA) {
        terminar(AUTOTUNE_RESULTADO_OK);
        return;
    }
    irAVolver(ahora, ST_ID);
}

/* ==================== API ==================== */

void AutotuneRpm_Iniciar(uint8_t etapa)
{
    AutotuneRpm_Limpiar();
    s_etapa = etapa;
    s_estado = ST_ESPERA;
    s_intento = 0U;
    s_ultimo = AUTOTUNE_SIN_RESULTADO;
    printf("AUTOTUNE_PID1,INICIO,calib=%u,pid_kp_actual=%.4f,pid_ki_actual=%.4f\r\n",
           (unsigned)etapa, CalibFlash_GetPidRpmKp(), CalibFlash_GetPidRpmKi());
}

void AutotuneRpm_Limpiar(void)
{
    PID_LimpiarGananciasTemporales();
    if (s_estado != ST_ESPERA) {
        CalibFlash_SetSetRpm(0.0f);
        s_estado = ST_ESPERA; /* cancelada desde afuera (CALIB!=13) a mitad de ensayo */
    }
}

Autotune_Resultado_t AutotuneRpm_UltimoResultado(void)
{
    return s_ultimo;
}

bool AutotuneRpm_ServoDirecto(uint16_t *pulsoDestinoUs)
{
    if (s_estado == ST_CURVA) {
        *pulsoDestinoUs = s_pulsoPaso;
        return true;
    }
    if (s_estado == ST_ID) {
        *pulsoDestinoUs = s_pulsoId;
        return true;
    }
    return false;
}

void AutotuneRpm_Paso(bool motorOperando, bool modoCalibracion)
{
    uint32_t ahora = HAL_GetTick();

    if (s_estado != ST_ESPERA) {
        if (!motorOperando)   { abortar("motor_se_detuvo"); return; }
        if (!modoCalibracion) { abortar("modo_cambio");     return; }
    }

    switch (s_estado) {
    case ST_ESPERA:
        if (motorOperando && modoCalibracion) {
            CalibFlash_SetSetRpm(0.0f);
            s_acumBase = 0.0f;
            s_nBase = 0U;
            s_t0Ms = ahora;
            s_estado = ST_BASE;
            printf("AUTOTUNE_PID1,BASE,duracion_s=%lu\r\n", (unsigned long)(AUTOTUNE1_BASE_MS / 1000U));
        }
        break;

    case ST_BASE:
        s_acumBase += Tacometro_GetRPMFiltrada();
        s_nBase++;
        if (ahora - s_t0Ms >= AUTOTUNE1_BASE_MS) {
            s_y0 = s_acumBase / (float)s_nBase;
            s_sp = CalibFlash_GetRpmMin() + AUTOTUNE1_ESCALON_RPM;
            if (s_sp > CalibFlash_GetRpmMax()) s_sp = CalibFlash_GetRpmMax();
            /* La curva sola no usa el escalon: no se le exige. */
            if (s_etapa != CALIB_PID1_CURVA && s_sp - s_y0 < AUTOTUNE1_ESCALON_MIN_UTIL_RPM) {
                printf("AUTOTUNE_PID1,BASE_INVALIDA,y0=%.1f,sp=%.1f\r\n", s_y0, s_sp);
                abortar("escalon_insuficiente_sobre_la_base");
                break;
            }
            if (s_etapa == CALIB_PID1_VALIDAR) {
                /* Valida lo que ya esta guardado: zona 1, sin curva (no hay sp2). */
                s_kpCand = CalibFlash_GetPidRpmKp();
                s_kiCand = CalibFlash_GetPidRpmKi();
                s_sp2 = 0.0f;
                s_valFase = 0U;
                s_spVal = s_sp;
                empezarValidacion(ahora);
                break;
            }
            if (s_etapa == CALIB_PID1_ID) {
                /* Usa la curva de CALIB=5: no la repite. */
                if (!s_curvaValida || s_curvaPulsoMin != CalibFlash_GetServoPulsoMinUs()) {
                    abortar("sin_curva_correr_CALIB_5_primero");
                    break;
                }
                printf("AUTOTUNE_PID1,CURVA_REUSADA,k_peor=%.6f,pulso_escalon=%u\r\n", s_kPeorCurva, s_pulsoId);
                irAVolver(ahora, ST_ID);
                break;
            }
            /* Arranca el barrido de curva de ganancia. */
            s_curvaValida = false; /* se reemplaza por la de este barrido */
            s_pulsoPaso = CalibFlash_GetServoPulsoMinUs();
            s_pulsoPrev = s_pulsoPaso;
            s_rpmPrev = Tacometro_GetRPMFiltrada();
            s_pasos = 0U;
            s_pulsoId = 0U;
            s_kPeorCurva = 0.0f;
            s_rpmEnPeor = 0.0f;
            s_t0Ms = ahora;
            s_estado = ST_CURVA;
            printf("AUTOTUNE_PID1,CURVA_INICIO,y0=%.1f,pulso_inicial=%u,rpm_techo=%.0f\r\n",
                   s_y0, s_pulsoPaso, CalibFlash_GetRpmMax());
        }
        break;

    case ST_CURVA:
        if (ahora - s_t0Ms >= AUTOTUNE1_CURVA_DWELL_MS) {
            float rpm = Tacometro_GetRPMFiltrada();
            float salto = rpm - s_rpmPrev;
            /* Mismo formato que CALIB=5: el log sigue sirviendo para `pid_tuning.py --ganancia-log`. */
            printf("GANANCIA_CAL,PASO,pulso=%u,rpm=%.1f,salto=%.1f\r\n", s_pulsoPaso, rpm, salto);

            if (s_pulsoId == 0U && rpm >= s_y0 + AUTOTUNE1_ID_SUBIDA_RPM) {
                s_pulsoId = s_pulsoPaso; /* primer pulso que dio +200 RPM: tamaño del escalon de identificacion */
            }
            if (s_pasos > 0U && s_pulsoPaso > s_pulsoPrev) {
                float kLocal = (rpm - s_rpmPrev) / (float)(s_pulsoPaso - s_pulsoPrev);
                if (kLocal > s_kPeorCurva) {
                    s_kPeorCurva = kLocal;
                    s_rpmEnPeor = rpm;
                }
            }
            s_rpmPrev = rpm;
            s_pulsoPrev = s_pulsoPaso;
            s_pasos++;

            if (s_pasos > 1U && salto >= AUTOTUNE1_CURVA_SALTO_ANORMAL_RPM) {
                abortar("salto_rpm_anormal_en_barrido");
            } else if (rpm >= CalibFlash_GetRpmMax()) {
                terminarCurva(ahora, "techo_rpm");
            } else if ((uint32_t)s_pulsoPaso + AUTOTUNE1_CURVA_PASO_US > CalibFlash_GetServoPulsoMaxUs()) {
                terminarCurva(ahora, "servo_pulso_max");
            } else {
                s_pulsoPaso = (uint16_t)(s_pulsoPaso + AUTOTUNE1_CURVA_PASO_US);
                s_t0Ms = ahora;
            }
        }
        break;

    case ST_VOLVER: {
        float desvio = Tacometro_GetRPMFiltrada() - s_y0;
        if (desvio < 0.0f) desvio = -desvio;
        if (desvio <= AUTOTUNE1_VOLVER_BANDA_RPM) {
            if (s_estableDesdeMs == 0U) s_estableDesdeMs = ahora;
            if (ahora - s_estableDesdeMs >= AUTOTUNE1_VOLVER_ESTABLE_MS) {
                if (s_siguienteTrasVolver == ST_ID) {
                    printf("AUTOTUNE_PID1,ESCALON_ABIERTO,y0=%.1f,pulso_min=%u,pulso_escalon=%u,duracion_s=%lu\r\n",
                           s_y0, CalibFlash_GetServoPulsoMinUs(), s_pulsoId,
                           (unsigned long)(AUTOTUNE1_MUESTRAS * AUTOTUNE1_DT_MS / 1000U));
                    empezarGrabacion(ahora);
                    s_estado = ST_ID; /* el servo va a s_pulsoId via AutotuneRpm_ServoDirecto() */
                } else {
                    empezarValidacion(ahora);
                }
            }
        } else {
            s_estableDesdeMs = 0U;
        }
        if (s_estado == ST_VOLVER && ahora - s_t0Ms >= AUTOTUNE1_VOLVER_TIMEOUT_MS) {
            abortar("no_vuelve_a_la_base");
        }
        break;
    }

    case ST_ID:
        Autotune_GrabadorMuestrear(&s_grab, ahora, Tacometro_GetRPMFiltrada);
        if (Autotune_GrabadorLleno(&s_grab)) {
            s_estado = ST_VOLVER; /* suelta el servo (vuelve a SERVO_PULSO_MIN) antes de analizar */
            analizarIdentificacion(ahora);
        }
        break;

    case ST_VAL:
        Autotune_GrabadorMuestrear(&s_grab, ahora, Tacometro_GetRPMFiltrada);
        /* Vigia en vivo: si la candidata hace oscilar el lazo, cortar ya sin
         * esperar a completar los 30 s (mismo criterio que CALIB=7). */
        if (Autotune_OscActualizar(&s_osc, Tacometro_GetRPMFiltrada() - s_spVal,
                                   s_bandaVal, ahora, AUTOTUNE1_OSC_VENTANA_MS)) {
            CalibFlash_SetSetRpm(0.0f);
            siguienteIntento("oscila_en_vivo", ahora);
        } else if (Autotune_GrabadorLleno(&s_grab)) {
            evaluarValidacion(ahora);
        }
        break;
    }
}
