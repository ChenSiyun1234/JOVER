#ifndef UART_H
#define UART_H

#include "stm32g474xx.h"
#include <stdbool.h>
#include <stdint.h>
#include "GPIO/gpio.h"


bool usart_init(USART_TypeDef *bus, int baud_rate);
void usart_writeString(USART_TypeDef *bus, uint8_t message[]);
void usart_writeBytes(USART_TypeDef *bus, uint8_t message[], int nbytes);
bool usart_receiveBufferNotEmpty(USART_TypeDef *bus);
int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size);
void usart_flushrx(USART_TypeDef* bus);

#endif
