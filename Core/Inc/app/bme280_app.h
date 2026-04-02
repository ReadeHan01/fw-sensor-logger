/**
 * @file bme280_app.h
 * @brief Bosch BME280 integration: init, read, fault recovery, watchdog escalation.
 */
#ifndef BME280_APP_H
#define BME280_APP_H

#include "app_types.h"
#include "bme280.h"
#include <stdbool.h>

/** BME280 7-bit I2C address (0x76 primary strap; use SEC 0x77 if SDO tied high). */
#define BME280_I2C_ADDR_7BIT BME280_I2C_ADDR_PRIM

/** HAL I2C mem read/write timeout in milliseconds (blocking calls). */
#define BME280_I2C_TIMEOUT_MS 100u

void bme280_app_init(void); /**< Full driver setup; sets internal ready flag. */

void read_bme280_into_sample(Sample *s); /**< Forced-mode read with retry policy. */

bool bme280_app_is_fault_latched(void); /**< True after fatal escalation (IWDG stop refresh). */

#endif /* BME280_APP_H */
