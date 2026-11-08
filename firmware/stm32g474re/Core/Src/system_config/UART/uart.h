#ifndef UART_H
#define UART_H

#include "stm32g474xx.h"
#include <stdbool.h>
#include <stdint.h>
#include "GPIO/gpio.h"


/*
 * Initializes the UART hardware to conduct UART communication with
 * 1 start bit, 8 data bits, 1 stop bit, no parity bit.
 *
 * @param uart       The USART device that needs initialization
 * @param baud_rate  The desired baud rate
 *
 * @returns The initialization status (true = success | false = failure)
 */
bool usart_init(USART_TypeDef *uart, int baud_rate);

/*
 * Transmit a null-terminated string via UART
 *
 * @param uart The USART device doing the transmission
 * @param msg The string being transmitted
 *
 * @note This is a blocking operation
 */
void usart_transmitStr(USART_TypeDef *uart, const char *msg);

/*
 * Transmit n-number of bytes via UART
 *
 * @param uart      The USART device doing the transmission
 * @param message  The array of bytes being transmitted
 * @param nbytes   The number of bytes being transmitted
 *
 * @note This is a blocking operation
 */
void usart_transmitBytes(USART_TypeDef *uart, uint8_t msg[], int nbytes);

int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size);

#endif
