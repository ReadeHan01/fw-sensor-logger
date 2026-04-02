/**
 * @file sensors_analog.c
 * @brief Thermistor (beta equation) and LDR percent from ADC1 sequential ranks.
 */
#include "app/sensors_analog.h"

#include "app/app_extern.h"

#include <math.h>

/* Thermistor network and beta model (must match hardware). */
#define R_FIXED 10000
#define BETA 3950
#define R0 10000

HAL_StatusTypeDef adc_poll_get(uint16_t *out)
{
  if (HAL_ADC_PollForConversion(&hadc1, 200) != HAL_OK) {
    return HAL_TIMEOUT; /* Conversion did not finish in time. */
  }
  *out = (uint16_t)HAL_ADC_GetValue(&hadc1); /* Latest DR for current rank. */
  return HAL_OK; /* Success. */
}

bool read_therm_and_ldr(float *therm_c, float *ldr_pct)
{
  uint16_t therm_adc = 0; /* Raw thermistor divider tap. */
  uint16_t ldr_adc = 0;   /* Raw photoresistor tap. */

  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return false; /* Cannot start injected/regular sequence. */
  }

  if (adc_poll_get(&therm_adc) != HAL_OK) {
    HAL_ADC_Stop(&hadc1); /* Release ADC on error. */
    return false; /* Rank1 timeout. */
  }
  if (adc_poll_get(&ldr_adc) != HAL_OK) {
    HAL_ADC_Stop(&hadc1); /* Stop before returning. */
    return false; /* Rank2 timeout. */
  }

  HAL_ADC_Stop(&hadc1); /* Manual mode: stop until next explicit Start. */

  if (therm_adc == 0 || therm_adc == 4095) {
    return false; /* Saturated reading; avoid divide-by-zero in beta calc. */
  }

  float v = (therm_adc * 3.3f) / 4095.0f; /* Assuming nominal 3.3 V reference scaling. */
  if (v < 0.001f) {
    v = 0.001f; /* Clamp away from zero for division stability. */
  }
  if (v > 3.299f) {
    v = 3.299f; /* Clamp below rail. */
  }

  float rth = R_FIXED * (3.3f / v - 1.0f); /* Series resistor divider -> NTC resistance. */
  float t0 = 298.15f; /* 25 C in Kelvin (reference for beta parameter). */
  float tempK = 1.0f / ((1.0f / t0) + (1.0f / BETA) * logf(rth / R0)); /* Beta equation. */
  *therm_c = tempK - 273.15f; /* Kelvin to Celsius for output. */

  *ldr_pct = ((float)ldr_adc / 4095.0f) * 100.0f; /* Simple linear light level proxy. */
  return true; /* Analog path succeeded. */
}
