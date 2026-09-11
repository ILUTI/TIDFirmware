#include "presion.h"

static ADC_HandleTypeDef *s_hadc = NULL;

static float s_corrienteMa = 0.0f;
static float s_presionPsiFiltrada = 0.0f;
static bool  s_sensorValido = false;

void Presion_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    HAL_ADCEx_Calibration_Start(s_hadc, ADC_SINGLE_ENDED);
}

void Presion_Update(void)
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

    float vAdc = ((float)cuentas / PRESION_ADC_CUENTAS_MAX) * PRESION_VREF_ADC_V;
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
