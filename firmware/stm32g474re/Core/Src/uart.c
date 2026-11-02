#include "uart.h"
#include "gpio.h"

bool usart_init(USART_TypeDef *bus, int baud_rate) {
	GPIO_TypeDef *rx_port;
	GPIO_TypeDef *tx_port;
	gpio_pin_t rx_pin;
	gpio_pin_t tx_pin;

	// USART1_RX = PA10 | USART1_TX = PA9
	if (bus == USART1) {
		rx_port = GPIOA;
		tx_port = GPIOA;
		rx_pin = GPIO_PIN_10;
		tx_pin = GPIO_PIN_9;
	// USART2_RX = PD6 | USART2_TX = PD5
	} else if (bus == USART2) {
		rx_port = GPIOD;
		tx_port = GPIOD;
		rx_pin = GPIO_PIN_6;
		tx_pin = GPIO_PIN_5;
	// USART3_RX = PC11 | USART3_TX = PC10
	} else if (bus == USART3) {
		rx_port = GPIOC;
		tx_port = GPIOC;
		rx_pin = GPIO_PIN_11;
		tx_pin = GPIO_PIN_10;
	} else {
		return false;
	}

	gpio_config_t config = gpio_getDefaultConfig();
	config.alternate = GPIO_AF_7;
	config.mode = GPIO_MODE_AF;
	gpio_init(rx_port, rx_pin, &config);
	gpio_init(tx_port, tx_pin, &config);




	return true;
}

void usart_transmitStr(USART_TypeDef *bus, uint8_t message[]) {

}

void usart_transmitBytes(USART_TypeDef *bus, uint8_t message[], int nbytes) {

}

bool usart_receiveBufferNotEmpty(USART_TypeDef *bus) {
	return false;
}

int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size) {
	return 0;
}

void usart_flushrx(USART_TypeDef* bus) {

}
