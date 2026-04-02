/**
 * @file sensors_analog.h
 * @brief ADC polling helpers and thermistor/LDR conversion for one sample tick.
 */
#ifndef SENSORS_ANALOG_H
#define SENSORS_ANALOG_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

HAL_StatusTypeDef adc_poll_get(uint16_t *out); /**< Poll one rank conversion result. */

bool read_therm_and_ldr(float *therm_c, float *ldr_pct); /**< Start ADC, read ranks, convert. */

#endif /* SENSORS_ANALOG_H */
