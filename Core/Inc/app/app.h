/**
 * @file app.h
 * @brief Application entry points from main: init, periodic loop, ISR glue declarations.
 */
#ifndef APP_H
#define APP_H

#include "app/app_types.h"

extern volatile uint8_t interrupt_flag; /**< TIM2 ISR increments; main loop decrements. */
extern Sample sample;                   /**< Working sample for acquisition + CLI status. */

void app_init(void); /**< After MX_*: ring, BME280, UART RX IT. */

void app_loop_tick(void); /**< One main-loop iteration: sample + IWDG policy. */

#endif /* APP_H */
