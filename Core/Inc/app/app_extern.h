/**
 * @file app_extern.h
 * @brief HAL handles defined in main.c, shared with application modules.
 */
#ifndef APP_EXTERN_H
#define APP_EXTERN_H

#include "main.h"

extern ADC_HandleTypeDef hadc1;   /**< ADC1: thermistor + LDR ranks. */
extern I2C_HandleTypeDef hi2c1;   /**< I2C1: BME280. */
extern UART_HandleTypeDef huart2; /**< USART2: CLI and prints. */
extern TIM_HandleTypeDef htim2;   /**< TIM2: periodic sample trigger ISR. */
extern IWDG_HandleTypeDef hiwdg;  /**< Independent watchdog. */

#endif /* APP_EXTERN_H */
