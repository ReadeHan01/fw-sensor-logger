/**
 * @file cli_uart.c
 * @brief UART CLI: echo characters, accumulate lines, run simple text commands.
 */
#include "app/cli_uart.h"

#include "app/app_extern.h"
#include "app/ring_buffer.h"
#include "app/app_types.h"

#include <stdio.h>
#include <string.h>

uint8_t rx_char; /* Global so HAL_UART_Receive_IT can take &rx_char. */
char rx_buffer[64]; /* Line buffer; bounded by rx_index checks. */
uint8_t rx_index; /* Current line length index. */

extern Sample sample; /* Defined in app.c; CLI status command reads/writes it. */

void uart_print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100); /* Poll TX until done. */
}

static void print_sample(const Sample *s)
{
  char line[256]; /* Assembled single line for UART. */
  int n = snprintf(line, sizeof(line),
                   "time_ms=%lu, thermistor=%.2f C, Photoresistor=%.2f%%",
                   (unsigned long)s->time_ms, s->thermistor_deg_c, s->ldr_pct); /* Base fields. */
  if (n < 0 || (size_t)n >= sizeof(line)) {
    uart_print("(sample line truncated)\r\n"); /* snprintf failure or overflow guard. */
    return; /* Abort print. */
  }
  if (s->bme_ok) {
    snprintf(line + (size_t)n, sizeof(line) - (size_t)n,
             " | BME280: T=%.2f C P=%.2f hPa RH=%.2f%%\r\n",
             s->bme_temp_c, s->bme_press_hpa, s->bme_hum_pct); /* Append BME when valid. */
  } else {
    snprintf(line + (size_t)n, sizeof(line) - (size_t)n, " | BME280: n/a\r\n"); /* No BME this sample. */
  }
  uart_print(line); /* Send full line to user terminal. */
}

void parse_command(char *cmd)
{
  if (strcmp(cmd, "start") == 0) {
    HAL_TIM_Base_Start_IT(&htim2); /* Enable periodic TIM2 interrupts -> sampling. */
    uart_print("Starting...\r\n"); /* Acknowledge. */
  } else if (strcmp(cmd, "stop") == 0) {
    HAL_TIM_Base_Stop_IT(&htim2); /* Halt timer ISR posts. */
    uart_print("Stopping...\r\n"); /* Acknowledge. */
  } else if (strcmp(cmd, "reset") == 0) {
    rb_init(&ringbuffer); /* Drop all buffered samples. */
    sample_init(&sample); /* Clear last displayed sample snapshot. */
    uart_print("Resetting...\r\n"); /* Acknowledge. */
  } else if (strcmp(cmd, "status") == 0) {
    if (rb_peek_latest(&ringbuffer, &sample)) {
      print_sample(&sample); /* Show most recent ring contents. */
    } else {
      uart_print("No data yet. Type 'start' and wait.\r\n"); /* Empty buffer hint. */
    }
  } else if (strcmp(cmd, "help") == 0) {
    uart_print("Commands:\r\n"); /* Help banner. */
    uart_print("  start  - Start sensor sampling\r\n"); /* Explain start. */
    uart_print("  stop   - Stop sensor sampling\r\n"); /* Explain stop. */
    uart_print("  reset  - Clear ring buffer\r\n"); /* Explain reset. */
    uart_print("  status - Latest thermistor/LDR + BME280 (if OK)\r\n"); /* Explain status. */
    uart_print("  help   - Show this message\r\n"); /* Explain help. */
  } else {
    uart_print("Unknown Command\r\n"); /* Unrecognized token. */
  }
}

void process_char(char c)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 10); /* Local echo for interactive typing. */
  if (c == '\r' || c == '\n') {
    rx_buffer[rx_index] = '\0'; /* Terminate accumulated string. */
    parse_command(rx_buffer); /* Run command on full line. */
    rx_index = 0; /* Reset for next line. */
  } else {
    if (rx_index < (sizeof(rx_buffer) - 1u)) {
      rx_buffer[rx_index++] = c; /* Append printable char if room remains. */
    } else {
      rx_index = 0; /* Prevent overflow: drop line if too long (simple policy). */
    }
  }
}
