#include "presion_voltaje.h"
#include "presion.h"

static ADC_HandleTypeDef *s_hadc = NULL;

static float s_voltaje = 0.0f;
static float s_presionPsiFiltrada = 0.0f;
static bool  s_sensorValido = false;

/* Confirmacion de falla -- ver PRESIONV_FALLA_CONFIRMAR_MS. */
static bool     s_fallaPendiente = false;
static uint32_t s_inicioFallaMs = 0;

/* Acumuladores de diagnostico -- ver PresionV_TomarDiagnostico(). */
static uint16_t s_diagTandasInvalidas = 0;
static float    s_diagVMin = 99.0f;
static float    s_diagVMax = 0.0f;

/* Ancla de tiempo del limitador de velocidad de cambio -- ver
 * PRESIONV_TASA_MAX_CAMBIO_PSI_S en presion_voltaje.h. */
static uint32_t s_ultimaTandaMs = 0;
static bool     s_primeraTandaProcesada = false;

/* Lectura rapida -- ver PRESIONV_VENTANA_RAPIDA_MS en presion_voltaje.h. */
static float    s_presionRapidaPsi = 0.0f;
static float    s_rapidaSuma = 0.0f;
static uint32_t s_rapidaTandas = 0;
static uint32_t s_rapidaInicioMs = 0;

/* Buffer del promedio recortado -- ver PRESIONV_MUESTRAS_POR_PROMEDIO/
 * PRESIONV_RECORTE_EXTREMOS en presion_voltaje.h. Cada llamada a
 * PresionV_Update() agrega una cuenta cruda del ADC; cuando el buffer se
 * llena, se ordena, se descartan los extremos y se promedia el resto --
 * ese resultado (no la cuenta cruda individual) es el que alimenta el
 * EMA y la deteccion de falla. */
static uint32_t s_muestras[PRESIONV_MUESTRAS_POR_PROMEDIO];
static uint8_t  s_indiceMuestra = 0;

static void OrdenarMuestras(uint32_t *valores, uint8_t cantidad)
{
    for (uint8_t i = 1; i < cantidad; i++) {
        uint32_t clave = valores[i];
        int16_t j = (int16_t)i - 1;
        while (j >= 0 && valores[j] > clave) {
            valores[j + 1] = valores[j];
            j--;
        }
        valores[j + 1] = clave;
    }
}

static void ProcesarTanda(void)
{
    uint32_t copia[PRESIONV_MUESTRAS_POR_PROMEDIO];
    for (uint8_t i = 0; i < PRESIONV_MUESTRAS_POR_PROMEDIO; i++) {
        copia[i] = s_muestras[i];
    }
    OrdenarMuestras(copia, PRESIONV_MUESTRAS_POR_PROMEDIO);

    uint32_t suma = 0;
    uint8_t  usadas = 0;
    for (uint8_t i = PRESIONV_RECORTE_EXTREMOS;
         i < (PRESIONV_MUESTRAS_POR_PROMEDIO - PRESIONV_RECORTE_EXTREMOS);
         i++) {
        suma += copia[i];
        usadas++;
    }
    uint32_t cuentasRecortadas = suma / usadas;

    s_voltaje = ((float)cuentasRecortadas / PRESIONV_ADC_CUENTAS_MAX) * Presion_GetVdda();

    /* Fallo del sensor se evalua sobre el voltaje CRUDO (sin offset),
     * para no confundir un cable suelto/corto con un simple desfase de
     * calibracion. */
    bool tandaValida = (s_voltaje >= PRESIONV_FALLA_BAJA_V)
                    && (s_voltaje <= PRESIONV_FALLA_ALTA_V);
    uint32_t ahora = HAL_GetTick();

    if (s_voltaje < s_diagVMin) {
        s_diagVMin = s_voltaje;
    }
    if (s_voltaje > s_diagVMax) {
        s_diagVMax = s_voltaje;
    }

    if (tandaValida) {
        s_sensorValido = true;
        s_fallaPendiente = false;
    } else {
        if (s_diagTandasInvalidas < UINT16_MAX) {
            s_diagTandasInvalidas++;
        }
        if (!s_fallaPendiente) {
            s_fallaPendiente = true;
            s_inicioFallaMs = ahora;
        }
        if ((ahora - s_inicioFallaMs) >= PRESIONV_FALLA_CONFIRMAR_MS) {
            s_sensorValido = false;
        }
    }

    if (tandaValida) {
        float voltajeCalibrado = s_voltaje - PRESIONV_OFFSET_V;
        float pendiente = (PRESIONV_PSI_EN_VOLTAJE_MAX - PRESIONV_PSI_EN_VOLTAJE_MIN)
                         / (PRESIONV_VOLTAJE_MAX - PRESIONV_VOLTAJE_MIN);
        float presionInstantanea = PRESIONV_PSI_EN_VOLTAJE_MIN
                                  + (voltajeCalibrado - PRESIONV_VOLTAJE_MIN) * pendiente;
        if (presionInstantanea < 0.0f) {
            presionInstantanea = 0.0f;
        }

        /* Lectura rapida: se acumula ANTES del limitador. */
        s_rapidaSuma += presionInstantanea;
        s_rapidaTandas++;

        if (!s_primeraTandaProcesada || !PRESIONV_FILTROS_ACTIVOS) {
            /* Primera tanda desde el arranque -- no hay referencia previa
             * contra la cual limitar, se acepta tal cual (mismo criterio
             * que PID_Init()/PresionPid_Init() con su primer dt). */
            s_presionPsiFiltrada = presionInstantanea;
            s_primeraTandaProcesada = true;
        } else {
            float dtS = (ahora - s_ultimaTandaMs) / 1000.0f;
            float maxDelta = PRESIONV_TASA_MAX_CAMBIO_PSI_S * dtS;
            float delta = presionInstantanea - s_presionPsiFiltrada;
            if (delta > maxDelta) {
                presionInstantanea = s_presionPsiFiltrada + maxDelta;
            } else if (delta < -maxDelta) {
                presionInstantanea = s_presionPsiFiltrada - maxDelta;
            }
            s_presionPsiFiltrada = (PRESIONV_ALPHA_FILTRO * presionInstantanea)
                                  + ((1.0f - PRESIONV_ALPHA_FILTRO) * s_presionPsiFiltrada);
        }
        s_ultimaTandaMs = ahora;
    }
    if ((ahora - s_rapidaInicioMs) >= PRESIONV_VENTANA_RAPIDA_MS) {
        if (s_rapidaTandas > 0U) {
            s_presionRapidaPsi = s_rapidaSuma / (float)s_rapidaTandas;
        }
        s_rapidaSuma = 0.0f;
        s_rapidaTandas = 0U;
        s_rapidaInicioMs = ahora;
    }
    /* Si la tanda salio fuera de rango (falla confirmada o todavia
     * pendiente), se conserva la ultima presion filtrada valida en vez de
     * contaminarla con una lectura fuera de rango. */
}

void PresionV_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    HAL_ADCEx_Calibration_Start(s_hadc, ADC_SINGLE_ENDED);
}

void PresionV_Update(void)
{
    if (s_hadc == NULL) {
        return;
    }

    HAL_ADC_Start(s_hadc);
    if (HAL_ADC_PollForConversion(s_hadc, 10U) != HAL_OK) {
        HAL_ADC_Stop(s_hadc);
        return;
    }
    uint32_t cuentas = HAL_ADC_GetValue(s_hadc);
    HAL_ADC_Stop(s_hadc);

    s_muestras[s_indiceMuestra] = cuentas;
    s_indiceMuestra++;
    if (s_indiceMuestra >= PRESIONV_MUESTRAS_POR_PROMEDIO) {
        ProcesarTanda();
        s_indiceMuestra = 0;
    }
}

float PresionV_GetPresionPsi(void)
{
    return s_presionPsiFiltrada;
}

float PresionV_GetPresionRapidaPsi(void)
{
    return s_presionRapidaPsi;
}

float PresionV_GetVoltaje(void)
{
    return s_voltaje;
}

bool PresionV_SensorValido(void)
{
    return s_sensorValido;
}

void PresionV_TomarDiagnostico(uint16_t *tandasInvalidas, float *vMin, float *vMax)
{
    *tandasInvalidas = s_diagTandasInvalidas;
    *vMin = s_diagVMin;
    *vMax = s_diagVMax;
    s_diagTandasInvalidas = 0;
    s_diagVMin = 99.0f;
    s_diagVMax = 0.0f;
}
