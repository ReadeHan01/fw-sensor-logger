/**
 * @file bme280_app.c
 * @brief BME280 I2C glue, initialization, sampling, and fault escalation policy.
 */
#include "app/bme280_app.h"

#include "app/app_extern.h"
#include "app/cli_uart.h"
#include "app/ring_buffer.h"

#include "bme280.h"

#include <string.h>

static uint8_t s_bme280_i2c_addr_7bit = BME280_I2C_ADDR_7BIT; /* 7-bit addr passed to Bosch API. */
static struct bme280_dev s_bme280_dev; /* Driver instance (callbacks + settings ptr). */
static struct bme280_settings s_bme280_settings; /* OSR, filter, standby selections. */
static bool s_bme280_ready; /* False until init path completes successfully. */
static uint32_t s_bme280_meas_delay_us; /* Conversion wait time from current OSR settings. */
static uint8_t s_bme_consecutive_fail; /* Runtime consecutive API failures (saturating). */
static uint32_t s_bme_last_reinit_ms; /* HAL_GetTick of last bme280_app_init retry. */
static uint8_t s_bme_reinit_fail_count; /* Count of re-inits that left sensor not-ready. */
static bool s_fault_latched; /* When true, app layer stops IWDG refresh until reset. */

/* Low-level I2C register read for Bosch driver (intf_ptr -> 7-bit address byte). */
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  const uint8_t *addr7 = (const uint8_t *)intf_ptr; /* Recover strap-selected address. */
  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)((uint16_t)(*addr7) << 1u), reg_addr, /* Shift to 8-bit HAL addr. */
                        I2C_MEMADD_SIZE_8BIT, reg_data, (uint16_t)len,
                        BME280_I2C_TIMEOUT_MS) != HAL_OK) {
    error_stats.bme_i2c_fail++; /* Count raw bus failures for diagnostics. */
    return BME280_E_COMM_FAIL; /* Tell Bosch layer transaction failed. */
  }
  return BME280_INTF_RET_SUCCESS; /* All bytes read OK. */
}

/* Low-level I2C register write for Bosch driver. */
static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  const uint8_t *addr7 = (const uint8_t *)intf_ptr; /* Same address byte as read path. */
  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)((uint16_t)(*addr7) << 1u), reg_addr,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, (uint16_t)len,
                         BME280_I2C_TIMEOUT_MS) != HAL_OK) {
    error_stats.bme_i2c_fail++; /* Bus write failure counter. */
    return BME280_E_COMM_FAIL; /* Propagate comm error to driver. */
  }
  return BME280_INTF_RET_SUCCESS; /* Write completed. */
}

/* Bosch API expects microsecond delays; HAL_Delay is milliseconds only. */
static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
  (void)intf_ptr; /* Unused; required by driver callback signature. */
  uint32_t ms = (period + 999u) / 1000u; /* Round up partial milliseconds. */
  if (ms == 0u) {
    ms = 1u; /* HAL_Delay(0) would be invalid for non-zero us waits. */
  }
  HAL_Delay(ms); /* Block for at least requested approximate time. */
}

void bme280_app_init(void)
{
  int8_t rslt; /* Bosch API signed status (negative = error). */
  uint8_t settings_sel; /* Bitmask of which settings fields to commit. */

  memset(&s_bme280_dev, 0, sizeof(s_bme280_dev)); /* Clear device struct before use. */
  memset(&s_bme280_settings, 0, sizeof(s_bme280_settings)); /* Clear settings defaults. */

  s_bme280_dev.intf = BME280_I2C_INTF; /* Select I2C transport. */
  s_bme280_dev.intf_ptr = &s_bme280_i2c_addr_7bit; /* Pass address through opaque pointer. */
  s_bme280_dev.read = bme280_i2c_read; /* Wire HAL read wrapper. */
  s_bme280_dev.write = bme280_i2c_write; /* Wire HAL write wrapper. */
  s_bme280_dev.delay_us = bme280_delay_us; /* Wire coarse delay implementation. */

  rslt = bme280_init(&s_bme280_dev); /* Chip ID read + trim load; may I2C-fail. */
  if (rslt < 0) {
    s_bme280_ready = false; /* Block reads until wiring/address fixed. */
    uart_print("BME280 init failed (check I2C wiring / address)\r\n"); /* User-visible hint. */
    return; /* Abort further configuration. */
  }

  s_bme280_settings.osr_h = BME280_OVERSAMPLING_1X; /* Humidity 1x oversampling. */
  s_bme280_settings.osr_p = BME280_OVERSAMPLING_1X; /* Pressure 1x. */
  s_bme280_settings.osr_t = BME280_OVERSAMPLING_1X; /* Temperature 1x. */
  s_bme280_settings.filter = BME280_FILTER_COEFF_OFF; /* No IIR (forced mode typical). */
  s_bme280_settings.standby_time = BME280_STANDBY_TIME_1000_MS; /* Used if switched to normal mode later. */

  settings_sel = (uint8_t)(BME280_SEL_OSR_HUM | BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP |
                           BME280_SEL_FILTER | BME280_SEL_STANDBY); /* Program all selected fields. */
  rslt = bme280_set_sensor_settings(settings_sel, &s_bme280_settings, &s_bme280_dev); /* Write config regs. */
  if (rslt < 0) {
    s_bme280_ready = false; /* Do not attempt measurements. */
    uart_print("BME280 set_sensor_settings failed\r\n"); /* UART breadcrumb. */
    return; /* Stop init. */
  }

  rslt = bme280_cal_meas_delay(&s_bme280_meas_delay_us, &s_bme280_settings); /* Compute wait after forced meas. */
  if (rslt < 0) {
    s_bme280_ready = false; /* Cannot schedule conversion delay safely. */
    uart_print("BME280 cal_meas_delay failed\r\n"); /* Rare internal API failure. */
    return; /* Stop init. */
  }

  s_bme280_ready = true; /* Allow read_bme280_into_sample() to run. */
  uart_print("BME280 ready\r\n"); /* Success banner on serial console. */
}

bool bme280_app_is_fault_latched(void)
{
  return s_fault_latched; /* Exposed so main/app_loop can gate IWDG refresh. */
}

void read_bme280_into_sample(Sample *s)
{
  struct bme280_data data; /* Bosch output structure (compensated floats). */
  int8_t rslt; /* Per-call API status. */
  uint32_t now_ms; /* Single tick snapshot for throttle comparisons. */

  s->bme_ok = false; /* Default: invalidate until a good read completes. */
  s->bme_temp_c = 0.0f; /* Clear fields so stale values are not mistaken as valid. */
  s->bme_press_hpa = 0.0f; /* Pressure in hPa after scaling. */
  s->bme_hum_pct = 0.0f; /* Relative humidity percent. */

  now_ms = HAL_GetTick(); /* FreRTOS-less timebase for re-init spacing. */

  if (!s_bme280_ready) {
    if (s_bme_consecutive_fail < 255u) {
      s_bme_consecutive_fail++; /* Count "not ready" as a failure streak step. */
    }
    if ((s_bme_consecutive_fail >= 3u) && ((now_ms - s_bme_last_reinit_ms) >= 5000u)) {
      s_bme_last_reinit_ms = now_ms; /* Record throttle timestamp before heavy init. */
      bme280_app_init(); /* Try full re-init (may print errors on UART). */
      if (!s_bme280_ready) {
        if (s_bme_reinit_fail_count < 255u) {
          s_bme_reinit_fail_count++; /* Track failed recovery attempts. */
        }
        if (s_bme_reinit_fail_count >= 3u) {
          s_fault_latched = true; /* Escalate: let IWDG reset by stopping refresh. */
        }
      } else {
        s_bme_reinit_fail_count = 0u; /* Recovery OK; clear escalation counter. */
      }
      s_bme_consecutive_fail = 0u; /* Reset streak after an init attempt. */
    }
    return; /* Cannot read data while not ready. */
  }

  rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &s_bme280_dev); /* Trigger one T/P/H cycle. */
  if (rslt < 0) {
    if (s_bme_consecutive_fail < 255u) {
      s_bme_consecutive_fail++; /* Mode set failed (comm or driver state). */
    }
    if ((s_bme_consecutive_fail >= 3u) && ((now_ms - s_bme_last_reinit_ms) >= 5000u)) {
      s_bme_last_reinit_ms = now_ms; /* Throttle re-init. */
      bme280_app_init(); /* Same recovery path as not-ready case. */
      if (!s_bme280_ready) {
        if (s_bme_reinit_fail_count < 255u) {
          s_bme_reinit_fail_count++;
        }
        if (s_bme_reinit_fail_count >= 3u) {
          s_fault_latched = true; /* Fatal escalation. */
        }
      } else {
        s_bme_reinit_fail_count = 0u;
      }
      s_bme_consecutive_fail = 0u;
    }
    return; /* No valid conversion started. */
  }

  s_bme280_dev.delay_us(s_bme280_meas_delay_us, s_bme280_dev.intf_ptr); /* Wait for ADC conversion end. */

  rslt = bme280_get_sensor_data(BME280_ALL, &data, &s_bme280_dev); /* Read compensated results. */
  if (rslt < 0) {
    if (s_bme_consecutive_fail < 255u) {
      s_bme_consecutive_fail++; /* Data read failed. */
    }
    if ((s_bme_consecutive_fail >= 3u) && ((now_ms - s_bme_last_reinit_ms) >= 5000u)) {
      s_bme_last_reinit_ms = now_ms; /* Throttle. */
      bme280_app_init(); /* Retry init path. */
      if (!s_bme280_ready) {
        if (s_bme_reinit_fail_count < 255u) {
          s_bme_reinit_fail_count++;
        }
        if (s_bme_reinit_fail_count >= 3u) {
          s_fault_latched = true;
        }
      } else {
        s_bme_reinit_fail_count = 0u;
      }
      s_bme_consecutive_fail = 0u;
    }
    return; /* Leave bme_ok false. */
  }

  s->bme_temp_c = (float)data.temperature; /* Deg C from driver. */
  s->bme_press_hpa = (float)(data.pressure / 100.0); /* Pa -> hPa for display/logging. */
  s->bme_hum_pct = (float)data.humidity; /* Percent RH. */
  s->bme_ok = true; /* Mark this sample’s BME fields valid. */
  s_bme_consecutive_fail = 0u; /* Clear failure streak on success. */
  s_bme_reinit_fail_count = 0u; /* Clear escalation streak on success. */
}
