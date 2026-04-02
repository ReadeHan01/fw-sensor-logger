/**
 * @file cli_uart.h
 * @brief Simple UART line discipline: echo, buffer RX, dispatch commands.
 */
#ifndef CLI_UART_H
#define CLI_UART_H

#include <stdint.h>

extern uint8_t rx_char;   /**< Last byte from UART IT (re-armed after each RX). */
extern char rx_buffer[64]; /**< NUL-terminated command line buffer. */
extern uint8_t rx_index;   /**< Next write index into rx_buffer. */

void uart_print(const char *msg); /**< Blocking transmit of a C string to USART2. */

void parse_command(char *cmd); /**< Dispatch one complete line (no CR/LF). */

void process_char(char c); /**< Feed one RX byte; flushes line on newline. */

#endif /* CLI_UART_H */
