/**
 * @file    autotune.c
 * @brief   Motor compartido de autosintonia -- ver autotune.h.
 *
 * Todo aca es aritmetica pura sobre buffers: sin HAL, sin flash, sin
 * printf. Las rutinas de identificacion replican a proposito los mismos
 * pasos y umbrales de tools/pid_tuning/pid_tuning.py
 * (_identify_step_generico / _identify_step_abierto_generico), para que un
 * mismo ensayo dé el mismo (K, tau, L) en la placa y en la PC.
 */

/* Compilado siempre con -Os, tambien en Debug (-O0), para que el binario
 * Debug quepa en los 126 KB que declara el .ld (la ultima pagina, 0x0801F800,
 * es la de calibracion: si el programa crece de mas, falla al enlazar). */
#pragma GCC optimize ("Os")

#include "autotune.h"

/* ==================== UTILIDADES ==================== */

static float s_buffer[AUTOTUNE_MAX_MUESTRAS];

float *Autotune_Buffer(void)
{
    return s_buffer;
}

static float absf(float x) { return (x < 0.0f) ? -x : x; }

/* Promedio movil CENTRADO de semi-ancho 'semi' muestras en el indice i
 * (la ventana se recorta en los bordes en vez de rellenar -- el efecto es
 * minimo frente al padding por reflexion de numpy y no necesita buffer). */
static float suavizado(const float *y, uint16_t n, uint16_t i, uint16_t semi)
{
    uint16_t lo = (i > semi) ? (uint16_t)(i - semi) : 0U;
    uint16_t hi = (uint16_t)(i + semi);
    if (hi >= n) hi = (uint16_t)(n - 1U);
    float acc = 0.0f;
    for (uint16_t j = lo; j <= hi; j++) acc += y[j];
    return acc / (float)(hi - lo + 1U);
}

static float promedioTramo(const float *y, uint16_t desde, uint16_t hasta /* exclusivo */)
{
    float acc = 0.0f;
    for (uint16_t j = desde; j < hasta; j++) acc += y[j];
    return acc / (float)(hasta - desde);
}

const char *Autotune_IdNombre(Autotune_IdResultado_t r)
{
    switch (r) {
        case AUTOTUNE_ID_OK:                   return "ok";
        case AUTOTUNE_ID_DATOS_INSUFICIENTES:  return "datos_insuficientes";
        case AUTOTUNE_ID_NO_ASENTO:            return "no_asento";
        case AUTOTUNE_ID_GCL_FUERA_RANGO:      return "gcl_fuera_de_rango";
        case AUTOTUNE_ID_K_INVALIDA:           return "k_invalida";
        case AUTOTUNE_ID_NO_SE_MOVIO:          return "no_se_movio";
        case AUTOTUNE_ID_NO_LLEGO_63:          return "no_llego_al_63";
        default:                               return "desconocido";
    }
}

/* Parte comun a lazo cerrado y abierto: dado el valor final y el salto
 * estacionario, busca "cuando se movio" (L) y "cuando llego al 63.2%"
 * (tau de la respuesta observada). t = indice*dtS. */
static Autotune_IdResultado_t medirLyTau(const float *y, uint16_t n, float dtS, float y0,
        float deltaSs, float umbralMin, float umbralFrac, uint16_t semi,
        float *lOut, float *tauObsOut)
{
    float umbral = absf(deltaSs) * umbralFrac;
    if (umbral < umbralMin) umbral = umbralMin;

    int32_t idxMovio = -1;
    for (uint16_t i = 0U; i < n; i++) {
        if (absf(suavizado(y, n, i, semi) - y0) > umbral) { idxMovio = (int32_t)i; break; }
    }
    if (idxMovio < 0) return AUTOTUNE_ID_NO_SE_MOVIO;

    float obj63 = y0 + 0.632f * deltaSs;
    int32_t idx63 = -1;
    for (uint16_t i = (uint16_t)idxMovio; i < n; i++) {
        float ys = suavizado(y, n, i, semi);
        if ((deltaSs > 0.0f && ys >= obj63) || (deltaSs < 0.0f && ys <= obj63)) { idx63 = (int32_t)i; break; }
    }
    if (idx63 < 0) return AUTOTUNE_ID_NO_LLEGO_63;

    *lOut = (float)idxMovio * dtS;
    float tauObs = (float)(idx63 - idxMovio) * dtS;
    if (tauObs < dtS) tauObs = dtS;
    *tauObsOut = tauObs;
    return AUTOTUNE_ID_OK;
}

/* Metodo de DOS PUNTOS (Smith): con los cruces del 28.3% y del 63.2% de la
 * respuesta, tau = 1.5*(t63 - t28) y L = t63 - tau. Para una planta de
 * primer orden con tiempo muerto es exacto (sin sesgo). El metodo de
 * medirLyTau() empieza a contar cuando la respuesta ya subio 20%, y por eso
 * mide tau ~22% por debajo del real (simulado: 0.35 s para un tau real de
 * 0.43 s). */
static Autotune_IdResultado_t medirDosPuntos(const float *y, uint16_t n, float dtS, float y0,
        float deltaSs, uint16_t semi, float *lOut, float *tauOut)
{
    float obj28 = y0 + 0.283f * deltaSs;
    float obj63 = y0 + 0.632f * deltaSs;
    int32_t i28 = -1, i63 = -1;
    for (uint16_t i = 0U; i < n && i63 < 0; i++) {
        float ys = suavizado(y, n, i, semi);
        bool pasa28 = (deltaSs > 0.0f) ? (ys >= obj28) : (ys <= obj28);
        bool pasa63 = (deltaSs > 0.0f) ? (ys >= obj63) : (ys <= obj63);
        if (i28 < 0 && pasa28) i28 = (int32_t)i;
        if (pasa63) i63 = (int32_t)i;
    }
    if (i28 < 0) return AUTOTUNE_ID_NO_SE_MOVIO;
    if (i63 < 0) return AUTOTUNE_ID_NO_LLEGO_63;

    float t28 = (float)i28 * dtS;
    float t63 = (float)i63 * dtS;
    float tau = 1.5f * (t63 - t28);
    if (tau < dtS) tau = dtS;
    float l = t63 - tau;
    if (l < 0.0f) l = 0.0f;
    *tauOut = tau;
    *lOut = l;
    return AUTOTUNE_ID_OK;
}

/* Valor final de la respuesta y verificacion de que YA asento: compara el
 * promedio de la ventana final contra el de la ventana inmediatamente
 * anterior -- si siguen difiriendo mas que una fraccion del salto, la
 * captura termino antes de tiempo y K/G_cl saldrian subestimadas (en
 * pid_tuning.py esto lo decide el operador mirando el log). */
static Autotune_IdResultado_t valorFinal(const float *y, uint16_t n, float dtS, float y0,
        float ventanaFinalS, float umbralMin, float *yfOut)
{
    uint16_t nv = (uint16_t)(ventanaFinalS / dtS);
    if (nv < 2U) nv = 2U;
    if ((uint32_t)nv * 2U > n) return AUTOTUNE_ID_DATOS_INSUFICIENTES;
    float yf   = promedioTramo(y, (uint16_t)(n - nv), n);
    float yAnt = promedioTramo(y, (uint16_t)(n - 2U * nv), (uint16_t)(n - nv));
    float deriva = absf(yf - yAnt);
    /* Tolerancia: 10% del salto, con piso 2.5 x el umbral de movimiento. El
     * piso sale de datos reales: en un tramo ya estable del log de presion
     * COM3_2026_09_21, las medias de 20 s consecutivas difieren 0.19 PSI en
     * promedio y hasta 0.36 PSI (fluctuacion lenta del sensor/hidraulica);
     * con 1x el umbral (0.3 PSI) habria dado falsos "no asento". */
    float tol = 0.10f * absf(yf - y0);
    if (tol < 2.5f * umbralMin) tol = 2.5f * umbralMin;
    if (deriva > tol) return AUTOTUNE_ID_NO_ASENTO;
    *yfOut = yf;
    return AUTOTUNE_ID_OK;
}

/* ==================== IDENTIFICACION ==================== */

Autotune_IdResultado_t Autotune_IdentificarLazoCerrado(const float *y, uint16_t n,
        float dtS, float y0, float deltaCmd, float kpUsado, float ventanaFinalS,
        float umbralMin, uint16_t semiVentana, Autotune_Planta_t *out)
{
    if (n < 20U || dtS <= 0.0f || kpUsado <= 0.0f || deltaCmd == 0.0f) {
        return AUTOTUNE_ID_DATOS_INSUFICIENTES;
    }

    float yf = 0.0f;
    Autotune_IdResultado_t r = valorFinal(y, n, dtS, y0, ventanaFinalS, umbralMin, &yf);
    if (r != AUTOTUNE_ID_OK) return r;

    float deltaSs = yf - y0;
    float gcl = deltaSs / deltaCmd;
    if (!(gcl > 0.02f && gcl < 0.999f)) return AUTOTUNE_ID_GCL_FUERA_RANGO;

    float l = 0.0f, tauCl = 0.0f;
    r = medirLyTau(y, n, dtS, y0, deltaSs, umbralMin, 0.2f, semiVentana, &l, &tauCl);
    if (r != AUTOTUNE_ID_OK) return r;

    out->gcl = gcl;
    out->k   = gcl / (kpUsado * (1.0f - gcl));
    out->tau = tauCl / (1.0f - gcl);
    out->l   = l;
    return AUTOTUNE_ID_OK;
}

Autotune_IdResultado_t Autotune_IdentificarLazoAbierto(const float *y, uint16_t n,
        float dtS, float y0, float deltaCmd, float ventanaFinalS,
        float umbralMin, float umbralFrac, uint16_t semiVentana, bool dosPuntos,
        Autotune_Planta_t *out)
{
    if (n < 20U || dtS <= 0.0f || deltaCmd == 0.0f) {
        return AUTOTUNE_ID_DATOS_INSUFICIENTES;
    }

    float yf = 0.0f;
    Autotune_IdResultado_t r = valorFinal(y, n, dtS, y0, ventanaFinalS, umbralMin, &yf);
    if (r != AUTOTUNE_ID_OK) return r;

    float deltaSs = yf - y0;
    float k = deltaSs / deltaCmd;
    if (!(k > 0.0f)) return AUTOTUNE_ID_K_INVALIDA; /* mas comando tiene que dar mas respuesta */

    float l = 0.0f, tau = 0.0f;
    r = dosPuntos
        ? medirDosPuntos(y, n, dtS, y0, deltaSs, semiVentana, &l, &tau)
        : medirLyTau(y, n, dtS, y0, deltaSs, umbralMin, umbralFrac, semiVentana, &l, &tau);
    if (r != AUTOTUNE_ID_OK) return r;

    out->gcl = 0.0f;
    out->k   = k;
    out->tau = tau;
    out->l   = l;
    return AUTOTUNE_ID_OK;
}

/* ==================== GRABADOR DE MUESTRAS ==================== */

void Autotune_GrabadorIniciar(Autotune_Grabador_t *g, uint32_t ahoraMs, uint16_t cuantas, uint16_t dtMs)
{
    if (cuantas > AUTOTUNE_MAX_MUESTRAS) cuantas = AUTOTUNE_MAX_MUESTRAS;
    g->n = 0U;
    g->objetivo = cuantas;
    g->dtMs = dtMs;
    g->proximaMs = ahoraMs;
}

void Autotune_GrabadorMuestrear(Autotune_Grabador_t *g, uint32_t ahoraMs, float (*leer)(void))
{
    while (g->n < g->objetivo && (int32_t)(ahoraMs - g->proximaMs) >= 0) {
        s_buffer[g->n++] = leer();
        g->proximaMs += g->dtMs;
    }
}

bool Autotune_GrabadorLleno(const Autotune_Grabador_t *g)
{
    return g->n >= g->objetivo;
}

/* ==================== DETECTOR DE "SEÑAL ESTABLE" ==================== */

void Autotune_EstableIniciar(Autotune_Estable_t *e, uint32_t ahoraMs)
{
    e->cuenta = 0U;
    e->proximoMs = ahoraMs;
}

bool Autotune_EstableActualizar(Autotune_Estable_t *e, uint32_t ahoraMs, float valor,
                                float tolerancia, float *promedio10)
{
    if ((int32_t)(ahoraMs - e->proximoMs) >= 0) {
        e->proximoMs += 1000U;
        if (e->cuenta < AUTOTUNE_ESTABLE_N) {
            e->ring[e->cuenta++] = valor;
        } else {
            for (uint8_t i = 0U; i + 1U < AUTOTUNE_ESTABLE_N; i++) e->ring[i] = e->ring[i + 1U];
            e->ring[AUTOTUNE_ESTABLE_N - 1U] = valor;
        }
    }
    if (e->cuenta < AUTOTUNE_ESTABLE_N) return false;

    float viejo = 0.0f, nuevo = 0.0f, ultimos10 = 0.0f;
    for (uint8_t i = 0U; i < 5U; i++) {
        viejo += e->ring[i];
        nuevo += e->ring[AUTOTUNE_ESTABLE_N - 1U - i];
    }
    for (uint8_t i = 0U; i < 10U; i++) ultimos10 += e->ring[AUTOTUNE_ESTABLE_N - 1U - i];
    if (absf(nuevo / 5.0f - viejo / 5.0f) >= tolerancia) return false;
    *promedio10 = ultimos10 / 10.0f;
    return true;
}

/* ==================== SIMC ==================== */

void Autotune_Simc(const Autotune_Planta_t *p, float tauC, float *kp, float *ki)
{
    float tauCL = tauC + p->l;
    float kc = (1.0f / p->k) * (p->tau / tauCL);
    float ti = 4.0f * tauCL;
    if (p->tau < ti) ti = p->tau;
    *kp = kc;
    *ki = kc / ti;
}

static float recortar(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void Autotune_CandidataSimc(const Autotune_Planta_t *p, float tauC, float kpMin, float gananciaMax,
                            float *kp, float *ki)
{
    float kpCrudo = 0.0f, kiCrudo = 0.0f;
    Autotune_Simc(p, tauC, &kpCrudo, &kiCrudo);
    *kp = recortar(kpCrudo, kpMin, gananciaMax);
    *ki = recortar(kiCrudo, 0.0f, gananciaMax);
}

const char *Autotune_Veredicto(const Autotune_Metricas_t *m, uint8_t crucesMax, float sobrepicoMaxPct,
                               bool *aprobada)
{
    *aprobada = false;
    if (m->cruces > crucesMax)              return "oscila";
    if (m->sobrepicoPct > sobrepicoMaxPct)  return "sobreimpulso";
    if (!m->asento)                         return "no_asienta";
    *aprobada = true;
    return "ok";
}

/* ==================== METRICAS ==================== */

void Autotune_EvaluarEscalon(const float *y, uint16_t n, float dtS, float sp, float y0,
        float bandaMin, float bandaFrac, float ventanaFinalS, uint16_t semiVentana,
        Autotune_Metricas_t *m)
{
    float delta = sp - y0;
    float adelta = absf(delta);
    float banda = adelta * bandaFrac;
    if (banda < bandaMin) banda = bandaMin;

    uint16_t nv = (uint16_t)(ventanaFinalS / dtS);
    if (nv < 1U) nv = 1U;
    if (nv > n) nv = n;

    m->banda = banda;
    m->errFinal = promedioTramo(y, (uint16_t)(n - nv), n) - sp;

    /* Pico (en la direccion del escalon), sobre la senal suavizada para que
     * un solo pico de ruido no cuente como sobre-impulso. */
    float pico = 0.0f; /* exceso maximo sobre sp en la direccion del escalon */
    float sgn = (delta >= 0.0f) ? 1.0f : -1.0f;
    int32_t ultimoFuera = -1;
    int8_t  estado = (int8_t)(-sgn); /* arranca del lado de y0 */
    uint8_t cruces = 0U;

    for (uint16_t i = 0U; i < n; i++) {
        float ys = suavizado(y, n, i, semiVentana);
        float e = sgn * (ys - sp); /* >0 = pasado de sp en la direccion del escalon */
        if (e > pico) pico = e;
        if (absf(e) > banda) ultimoFuera = (int32_t)i;

        if (e > banda) {
            if (estado == -1) { cruces++; }
            estado = 1;
        } else if (e < -banda) {
            if (estado == 1) { cruces++; }
            estado = -1;
        }
    }

    m->sobrepicoPct = (adelta > 0.0f) ? (pico / adelta) * 100.0f : 0.0f;
    m->cruces = cruces;
    m->asento = (ultimoFuera < (int32_t)n - 1) && (absf(m->errFinal) <= banda);
    m->tAsentS = (ultimoFuera < 0) ? 0.0f : (float)(ultimoFuera + 1) * dtS;
}

/* ==================== VIGIA DE OSCILACION ==================== */

void Autotune_OscReiniciar(Autotune_Osc_t *o)
{
    o->estado = 0;
    o->cuenta = 0U;
    o->total = 0U;
    for (uint8_t i = 0U; i < AUTOTUNE_OSC_CRUCES; i++) o->t[i] = 0U;
}

bool Autotune_OscActualizar(Autotune_Osc_t *o, float error, float umbral,
        uint32_t ahoraMs, uint32_t ventanaMs)
{
    int8_t nuevo = o->estado;
    if (error > umbral)       nuevo = 1;
    else if (error < -umbral) nuevo = -1;

    if (nuevo != o->estado && o->estado != 0 && nuevo != 0) {
        o->total++;
        if (o->cuenta < AUTOTUNE_OSC_CRUCES) {
            o->t[o->cuenta++] = ahoraMs;
        } else {
            for (uint8_t i = 0U; i + 1U < AUTOTUNE_OSC_CRUCES; i++) o->t[i] = o->t[i + 1U];
            o->t[AUTOTUNE_OSC_CRUCES - 1U] = ahoraMs;
        }
    }
    o->estado = nuevo;

    return (o->cuenta >= AUTOTUNE_OSC_CRUCES)
        && ((o->t[AUTOTUNE_OSC_CRUCES - 1U] - o->t[0]) <= ventanaMs);
}
