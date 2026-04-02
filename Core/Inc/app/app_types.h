/**
 * @file app_types.h
 * @brief Shared data shapes for the sensor logger application layer.
 */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/** One logged sample: analog channels plus optional BME280 fields. */
typedef struct {
  uint32_t time_ms;       /**< Host tick when sample was taken (HAL_GetTick). */
  float thermistor_deg_c; /**< Thermistor temperature (deg C). */
  float ldr_pct;          /**< Photoresistor as percent of full scale. */
  float bme_temp_c;       /**< BME280 temperature (deg C), valid if bme_ok. */
  float bme_press_hpa;    /**< BME280 pressure (hPa), valid if bme_ok. */
  float bme_hum_pct;      /**< BME280 RH (%), valid if bme_ok. */
  bool bme_ok;            /**< True when last BME read in this sample succeeded. */
} Sample;

/** Power-of-two ring buffer of samples (overwrite-oldest when full). */
typedef struct {
  Sample buf[512u]; /**< Fixed-capacity storage. */
  uint16_t head;    /**< Next slot index to write. */
  uint16_t tail;    /**< Oldest valid sample index. */
  uint16_t count;   /**< Number of samples currently stored. */
} Ring;

/** Centralized debug / fault counters. */
typedef struct {
  uint32_t bme_i2c_fail;   /**< HAL I2C transaction failures in BME glue. */
  uint32_t bme_read_fail;  /**< Reserved for high-level BME read failures. */
  uint32_t ring_overwrite; /**< Ring dropped oldest sample due to overflow. */
} ErrorStats;

#define RB_CAPACITY 512u
#define RB_MASK (RB_CAPACITY - 1u)

#endif /* APP_TYPES_H */
