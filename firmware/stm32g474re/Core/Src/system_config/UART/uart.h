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
 * @param bus        The USART bus that needs initialization
 * @param baud_rate  The desired baud rate
 *
 * @returns The initialization status (true = success | false = failure)
 */
bool usart_init(USART_TypeDef *bus, int baud_rate);

/*
 * Utilizes USART hardware transmitter to send a variable length string
 *
 * @param bus       The USART Bus doing the transmission
 * @param message   The string (character array) being sent
 *
 * @returns None
 */
void usart_transmitStr(USART_TypeDef *bus, uint8_t message[]);

/*
 * Utilizes USART hardware transmitter to send a fixed length byte stream
 *
 * @param bus       The USART Bus doing the transmission
 * @param message   The string (character array) being sent
 *
 * @returns None
 */
void usart_transmitBytes(USART_TypeDef *bus, uint8_t message[], int nbytes);

bool usart_receiveBufferNotEmpty(USART_TypeDef *bus);

int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size);

void usart_flushrx(USART_TypeDef* bus);

#endif
