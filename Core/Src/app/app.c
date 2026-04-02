/**
 * @file app.c
 * @brief Orchestrates sampling, sensor fusion, ring storage, and HAL callback hooks.
 */
#include "app/app.h"

#include "app/app_extern.h"
#include "app/bme280_app.h"
#include "app/cli_uart.h"
#include "app/ring_buffer.h"
#include "app/sensors_analog.h"

volatile uint8_t interrupt_flag = 0; /* TIM2 posts work; cleared in app_loop_tick. */
Sample sample; /* Latest construction area for thermistor/LDR/BME fields. */

void app_init(void)
{
  rb_init(&ringbuffer); /* Empty ring before first push. */
  bme280_app_init(); /* Configure BME280 or leave not-ready on failure. */
  HAL_UART_Receive_IT(&huart2, &rx_char, 1); /* Arm first UART RX interrupt byte. */
}

void app_loop_tick(void)
{
  if (interrupt_flag) {
    interrupt_flag--; /* Consume one pending timer tick (saturating uint8 in ISR). */
    sample.time_ms = HAL_GetTick(); /* Timestamp for this sample bundle. */
    if (read_therm_and_ldr(&sample.thermistor_deg_c, &sample.ldr_pct)) {
      read_bme280_into_sample(&sample); /* Fill BME fields when possible. */
      rb_push_overwrite(&ringbuffer, &sample); /* Store or overwrite oldest. */
    }
  }
  if (!bme280_app_is_fault_latched()) {
    HAL_IWDG_Refresh(&hiwdg); /* Keep watchdog happy unless fatal fault latched. */
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    interrupt_flag++; /* Defer heavy work to main loop; ISR stays O(1). */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    process_char(rx_char); /* Parse one byte toward CLI lines. */
    HAL_UART_Receive_IT(&huart2, &rx_char, 1); /* Re-arm for next byte. */
  }
}
