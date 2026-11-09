#pragma once

#include "stm32g474xx.h"
#include <stdbool.h>
#include <stdint.h>


/*
 * @brief Initializes a USART peripheral.
 *
 * Configures the desired USART peripheral using the supplied baud rate.
 * Uses 8N1 configuration (1 start bit, 8 data bits, no parity bit, 1 stop bit).
 *
 * @param uart       USART peripheral to initialize.
 * @param baud_rate  Communication speed in bps (bits/sec).
 *
 * @returns true if initialization was successful, false otherwise.
 */
bool usart_init(USART_TypeDef *uart, int baud_rate);

/*
 * @brief Transmits a null-terminated string over UART.
 *
 * Blocks until all characters have been transmitted.
 *
 * @param uart USART peripheral to use.
 * @param msg  String to transmit.
 *
 * @note Assumes the string is null-terminated.
 */
void usart_transmitStr(USART_TypeDef *uart, const char *msg);

/*
 * @brief Transmits an array of bytes over UART.
 *
 * Blocks until all bytes have been transmitted.
 *
 * @param uart     USART peripheral to use.
 * @param data     Buffer containing the bytes to transmit.
 * @param nbytes   Number of bytes to transmit.
 *
 * @note Assumes nbytes > length of data buffer.
 */
void usart_transmitBytes(USART_TypeDef *uart, const uint8_t data[], int nbytes);

/*
 * @brief Receive bytes over UART.
 *
 * Reads bytes from an internal receive buffer and stores those
 * bytes inside the input buffer. This is a non-blocking operation.
 *
 * @param uart    USART peripheral to use.
 * @param buffer  Buffer to store the received bytes.
 * @param nbytes  Number of bytes to read.
 *
 * @return Number of bytes read.
 *
 * @note User should check nbytes vs number of bytes actually read.
 */
int usart_receiveBytes(USART_TypeDef *uart, uint8_t buffer[], int nbytes);
