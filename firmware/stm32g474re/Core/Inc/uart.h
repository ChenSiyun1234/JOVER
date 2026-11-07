#ifndef UART_H
#define UART_H

#include "stm32g474xx.h"
#include <stdint.h>
#include <stdbool.h>

<<<<<<< Updated upstream
#define GPIO_AFRX_AF7 0b0111UL
#define GPIO_AFRX_AF5 0b0101UL

/*
 * Initializes the UART hardware to conduct UART communication with
 * 1 start bit, 8 data bits, 1 stop bits, No parity
 *
 * @param bus        The USART Bus that needs initialization
 * @param baud_rate  The desired Baud Rate of the UART system
 *
 * @returns None
=======
/*
 * Initializes the UART hardware to conduct UART communication with
 * 1 start bit, 8 data bits, 1 stop bit, no parity bit.
 *
 * @param bus        The USART bus that needs initialization
 * @param baud_rate  The desired baud rate
 *
 * @returns The initialization status (true = success | false = failure)
>>>>>>> Stashed changes
 */
bool usart_init(USART_TypeDef *bus, int baud_rate);

/*
<<<<<<< Updated upstream
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

/*
 * Returns the status of the USART receiver's FIFO buffer
 *
 * @param None
 * @returns Whether the buffer has
 */
bool usart_receiveBufferNotEmpty(USART_TypeDef *bus);

/*
 * Utilizes the USART hardware receiver to get `size` number of bytes from a FIFO buffer.
 * Will wait for more bytes in case enough aren't available.
 * Incorporates a timeout system for safety.
 *
 * @param bus		The USART Bus that will be receiving
 * @param buffer	The buffer where the received bytes must be stored. Length is assumed to be >= `size`
 * @param size		The number of bytes to receive and store
 *
 * @returns			The number of bytes actually received and stored ( <= `size`)
 */
=======
 * Transmits a string using the target USART bus
 *
 * @param bus The transmitting USART bus
 * @param msg The string to be transmitted
 *
 * @details
 * This is a blocking operation. After sending the last char
 * to the transmit data register, it waits for transmission
 * to complete before returning.
 */
void usart_transmitString(USART_TypeDef *bus, const char *msg);

/*
 * Transmits an array of bytes using the target USART bus
 *
 * @param bus    The transmitting USART bus
 * @param bytes  The array of bytes to be transmitted
 * @param nbytes The number of bytes to be transmitted
 *
 * @details
 * This is a blocking operation. After sending the last byte
 * to the transmit data register, it waits for transmission
 * to complete before returning.
 */
void usart_transmitBytes(USART_TypeDef *bus, uint8_t bytes[], int nbytes);


bool usart_receiveBufferNotEmpty(USART_TypeDef *bus);



>>>>>>> Stashed changes
int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size);

/*
 * Empties the receive buffer.
 *
 * @param bus		The USART Bus that will be receiving
 */
void usart_flushrx(USART_TypeDef* bus);


#endif
