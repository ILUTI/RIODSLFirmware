#include "presion.h"

static ADC_HandleTypeDef *s_hadc = NULL;

static float s_corrienteMa = 0.0f;
static float s_presionPsiFiltrada = 0.0f;
static bool  s_sensorValido = false;
static float s_vddaVoltios = PRESION_VREF_ADC_V_DEFAULT;
static uint32_t s_ultimoRefrescoVrefintMs = 0;
static bool s_primerRefrescoHecho = false;

/* Reconfigura momentaneamente el canal activo del ADC1 a VREFINT, lee
 * el VDDA real con la calibracion de fabrica del chip, y actualiza
 * s_vddaVoltios -- luego regresa el canal a ADC_CHANNEL_2 (presion)
 * para que Presion_Update() siga leyendo el sensor normalmente. No usa
 * un segundo "rank" fijo en el .ioc -- solo reconfigura el mismo canal
 * unico en tiempo de ejecucion (VREFINT varia lento, no hace falta
 * medirlo en cada vuelta del loop, ver PRESION_VREFINT_INTERVALO_MS). */
static void ActualizarVdda(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(s_hadc, &sConfig) == HAL_OK) {
        HAL_ADC_Start(s_hadc);
        if (HAL_ADC_PollForConversion(s_hadc, 10U) == HAL_OK) {
            uint32_t vrefintCrudo = HAL_ADC_GetValue(s_hadc);
            uint32_t vddaMv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefintCrudo, ADC_RESOLUTION_12B);
            s_vddaVoltios = vddaMv / 1000.0f;
            s_primerRefrescoHecho = true;
        }
        HAL_ADC_Stop(s_hadc);
    }

    /* Regresa el canal activo al de presion -- si esto llegara a fallar,
     * Presion_Update() seguiria leyendo VREFINT por error, asi que es
     * tan importante como la lectura misma. */
    sConfig.Channel = ADC_CHANNEL_2;
    HAL_ADC_ConfigChannel(s_hadc, &sConfig);
}

void Presion_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    HAL_ADCEx_Calibration_Start(s_hadc, ADC_SINGLE_ENDED);
    ActualizarVdda();
}

void Presion_Update(void)
{
    if (s_hadc == NULL) {
        return;
    }

    if (!s_primerRefrescoHecho
        || (HAL_GetTick() - s_ultimoRefrescoVrefintMs) >= PRESION_VREFINT_INTERVALO_MS) {
        s_ultimoRefrescoVrefintMs = HAL_GetTick();
        ActualizarVdda();
    }

    HAL_ADC_Start(s_hadc);
    if (HAL_ADC_PollForConversion(s_hadc, 10U) != HAL_OK) {
        HAL_ADC_Stop(s_hadc);
        return;
    }
    uint32_t cuentas = HAL_ADC_GetValue(s_hadc);
    HAL_ADC_Stop(s_hadc);

    float vAdc = ((float)cuentas / PRESION_ADC_CUENTAS_MAX) * s_vddaVoltios;
    float vShunt = vAdc / PRESION_GANANCIA_OPAMP;
    s_corrienteMa = (vShunt / PRESION_R_SHUNT_OHM) * 1000.0f;

    s_sensorValido = (s_corrienteMa >= PRESION_CORRIENTE_FALLA_BAJA_MA)
                   && (s_corrienteMa <= PRESION_CORRIENTE_FALLA_ALTA_MA);

    if (s_sensorValido) {
        float pendiente = (PRESION_PSI_EN_CORRIENTE_MAX - PRESION_PSI_EN_CORRIENTE_MIN)
                         / (PRESION_CORRIENTE_MAX_MA - PRESION_CORRIENTE_MIN_MA);
        float presionInstantanea = PRESION_PSI_EN_CORRIENTE_MIN
                                  + (s_corrienteMa - PRESION_CORRIENTE_MIN_MA) * pendiente;
        if (presionInstantanea < 0.0f) {
            presionInstantanea = 0.0f;
        }

        s_presionPsiFiltrada = (PRESION_ALPHA_FILTRO * presionInstantanea)
                              + ((1.0f - PRESION_ALPHA_FILTRO) * s_presionPsiFiltrada);
    }
    /* Si el sensor esta en falla, se conserva la ultima presion filtrada
     * valida en vez de contaminarla con una lectura fuera de rango. */
}

float Presion_GetPresionPsi(void)
{
    return s_presionPsiFiltrada;
}

float Presion_GetCorrienteMa(void)
{
    return s_corrienteMa;
}

bool Presion_SensorValido(void)
{
    return s_sensorValido;
}

float Presion_GetVdda(void)
{
    return s_vddaVoltios;
}
