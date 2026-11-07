#include "uart.h"

static void usart_transmitByte(USART_TypeDef *bus, uint8_t byte);

bool usart_init(USART_TypeDef *bus, uint32_t baud_rate) {
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
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// USART2_RX = PD6 | USART2_TX = PD5
	} else if (bus == USART2) {
		rx_port = GPIOD;
		tx_port = GPIOD;
		rx_pin = GPIO_PIN_6;
		tx_pin = GPIO_PIN_5;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	// USART3_RX = PC11 | USART3_TX = PC10
	} else if (bus == USART3) {
		rx_port = GPIOC;
		tx_port = GPIOC;
		rx_pin = GPIO_PIN_11;
		tx_pin = GPIO_PIN_10;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	} else {
		return false;
	}

	gpio_config_t config = gpio_getDefaultConfig();
	config.alternate = GPIO_AF_7;
	config.mode = GPIO_MODE_AF;
	gpio_init(rx_port, rx_pin, &config);
	gpio_init(tx_port, tx_pin, &config);

	SystemCoreClockUpdate();
	uint32_t usart_div = (SystemCoreClock + (baud_rate / 2)) / baud_rate;

	// 1 start bit, 8 data bits
	bus->CR1 &= ~(USART_CR1_M0_Msk | USART_CR1_M1_Msk);
	// Set baud rate
	bus->BRR = usart_div;
	// 1 stop bit
	bus->CR2 &= ~USART_CR2_STOP_Msk;
	// Enable USART
	bus->CR1 |= USART_CR1_UE;
	// Enable transmitter and receiver
	bus->CR1 |= USART_CR1_TE | USART_CR1_RE;

	return true;
}

void usart_transmitBytes(USART_TypeDef *bus, const uint8_t msg[], int nbytes) {
	for (int i = 0; i < nbytes; i++) {
		usart_transmitByte(bus, msg[i]);
	}
	while (!(bus->ISR & USART_ISR_TC)) {}
}

void usart_transmitStr(USART_TypeDef *bus, const char *msg) {
	for (const char *p = msg; *p != '\0'; p++) {
		usart_transmitByte(bus, (uint8_t)*p);
	}
	while (!(bus->ISR & USART_ISR_TC)) {}


bool usart_receiveBufferNotEmpty(USART_TypeDef *bus) {
	return false;
}

int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size) {
	return 0;
}

void usart_flushrx(USART_TypeDef* bus) {

}

static void usart_transmitByte(USART_TypeDef *bus, uint8_t byte) {
	while (!(bus->ISR & USART_ISR_TXE)) {}
	bus->TDR = byte;
}
