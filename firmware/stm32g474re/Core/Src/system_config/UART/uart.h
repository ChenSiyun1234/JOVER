#pragma once

#include "stm32g474xx.h"
#include <stdbool.h>
#include <stdint.h>


/*
 * @brief Initializes a USART peripheral.
 *
 * Configures the desired USART peripheral using the supplied baud rate.
 * Uses 8N1 configuration (1 start bit, 8 data bits, no parity bit, 1 stop bit).
 * Assumes USART will use the CPU clock frequency (no prescalers).
 * Look at pin-out for specific RX/TX pins.
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
void usart_transmitString(USART_TypeDef *uart, const char *msg);

/*
 * @brief Transmits an array of bytes over UART.
 *
 * Blocks until all bytes have been transmitted.
 *
 * @param uart     USART peripheral to use.
 * @param bytes    Array containing the bytes to transmit.
 * @param nbytes   Number of bytes to transmit.
 *
 * @note Assumes nbytes <= size of bytes
 */
void usart_transmitBytes(USART_TypeDef *uart, const uint8_t bytes[], int nbytes);

/*
 * @brief Receive bytes over UART.
 *
 * Reads bytes from an internal RX buffer and stores those
 * bytes inside the input buffer. This is a non-blocking operation.
 *
 * @param uart    USART peripheral to use.
 * @param buffer  Buffer to store the received bytes.
 * @param nbytes  Number of bytes to read.
 *
 * @return Number of bytes read.
 *
 * @note User should check nbytes vs return value.
 */
int usart_receiveBytes(USART_TypeDef *uart, uint8_t buffer[], int nbytes);

/*
 * @brief Check if the internal RX buffer is full.
 *
 * Total capacity of the RX buffer is (RX_CAP - 1).
 * One slot is reserved to check if the buffer is full.
 *
 * @param uart USART peripheral to check.
 *
 * @return true if receive buffer is full, otherwise false.
 *
 * @note returns true if an invalid uart handle is used.
 * @note Bytes are discarded if HW FIFO or internal RX buffer is overrun.
 */
bool usart_rxIsFull(USART_TypeDef *uart);

/*
 * @brief Check if the internal receive buffer is empty.
 *
 * @param uart USART peripheral to check.
 *
 * @return true if receive buffer is empty, otherwise false.
 *
 * @note returns true if an invalid uart handle is used.
 */
bool usart_rxIsEmpty(USART_TypeDef *uart);

/*
 * @brief Flush the internal receive buffer.
 *
 * @param uart USART peripheral to target.
 *
 * @note returns without flushing if an invalid uart handle is used.
 */
void usart_rxFlush(USART_TypeDef *uart);
