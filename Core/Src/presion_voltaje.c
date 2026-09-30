#include "presion_voltaje.h"
#include "presion.h"

static ADC_HandleTypeDef *s_hadc = NULL;

static float s_voltaje = 0.0f;
static float s_presionPsiFiltrada = 0.0f;
static bool  s_sensorValido = false;

/* Ancla de tiempo del limitador de velocidad de cambio -- ver
 * PRESIONV_TASA_MAX_CAMBIO_PSI_S en presion_voltaje.h. */
static uint32_t s_ultimaTandaMs = 0;
static bool     s_primeraTandaProcesada = false;

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
    s_sensorValido = (s_voltaje >= PRESIONV_FALLA_BAJA_V)
                   && (s_voltaje <= PRESIONV_FALLA_ALTA_V);

    if (s_sensorValido) {
        float voltajeCalibrado = s_voltaje - PRESIONV_OFFSET_V;
        float pendiente = (PRESIONV_PSI_EN_VOLTAJE_MAX - PRESIONV_PSI_EN_VOLTAJE_MIN)
                         / (PRESIONV_VOLTAJE_MAX - PRESIONV_VOLTAJE_MIN);
        float presionInstantanea = PRESIONV_PSI_EN_VOLTAJE_MIN
                                  + (voltajeCalibrado - PRESIONV_VOLTAJE_MIN) * pendiente;
        if (presionInstantanea < 0.0f) {
            presionInstantanea = 0.0f;
        }

        uint32_t ahora = HAL_GetTick();
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
    /* Si el sensor esta en falla, se conserva la ultima presion filtrada
     * valida en vez de contaminarla con una lectura fuera de rango. */
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

float PresionV_GetVoltaje(void)
{
    return s_voltaje;
}

bool PresionV_SensorValido(void)
{
    return s_sensorValido;
}
