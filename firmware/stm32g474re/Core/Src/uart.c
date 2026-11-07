#include "uart.h"


static void usart1_gpio_init();
static void usart2_gpio_init();
static void usart3_gpio_init();
static void uart4_gpio_init();
static void uart5_gpio_init();
static void lpuart1_gpio_init();
static void uart_8bit_1stop(USART_TypeDef *bus, int baud_rate, bool rts_cts_control);
static void usart_transmitChar(USART_TypeDef *bus, char c);


<<<<<<< Updated upstream
bool usart_init(USART_TypeDef *bus, int baud_rate) {
	return true;

}

void usart_transmitStr(USART_TypeDef *bus, uint8_t message[]) {
}

void usart_transmitBytes(USART_TypeDef *bus, uint8_t message[], int nbytes) {
=======

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

void usart_transmitByte(USART_TypeDef *bus, uint8_t byte) {
	while (!(bus->ISR & USART_ISR_TXE)) {}
	bus->TDR = byte;
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
>>>>>>> Stashed changes
}

bool usart_receiveBufferNotEmpty(USART_TypeDef *bus) {
	return true;
}

int usart_receiveBytes(USART_TypeDef *bus, uint8_t buffer[], uint16_t size) {
	return 0;
}

void usart_flushrx(USART_TypeDef* bus) {
}

// TX = PC4-AF7 | RX = PC5-AF7
static void usart1_gpio_init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOC->MODER &= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk);
	GPIOC->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1);

	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL4_Msk | GPIO_AFRL_AFSEL5_Msk);
	GPIOC->AFR[0] |= ((GPIO_AFRX_AF7 << GPIO_AFRL_AFSEL4_Pos) | (GPIO_AFRX_AF7 << GPIO_AFRL_AFSEL5_Pos));
}

// TX = PA2-AF7 | RX = PA3-AF7
static void usart2_gpio_init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk);
	GPIOA->AFR[0] |= ((GPIO_AFRX_AF7 << GPIO_AFRL_AFSEL2_Pos) | (GPIO_AFRX_AF7 << GPIO_AFRL_AFSEL3_Pos));
}

// TX = PB10-AF7 | PB11-AF7
static void usart3_gpio_init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk);
	GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
	GPIOB->AFR[1] |= ((GPIO_AFRX_AF7 << GPIO_AFRH_AFSEL10_Pos) | (GPIO_AFRX_AF7 << GPIO_AFRH_AFSEL11_Pos));
}

// TX = PC10-AF5 | RX = PC11-AF5
static void uart4_gpio_init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOC->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk);
	GPIOC->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
	GPIOC->AFR[1] |= ((GPIO_AFRX_AF5 << GPIO_AFRH_AFSEL10_Pos) | (GPIO_AFRX_AF5 << GPIO_AFRH_AFSEL11_Pos));
}

// TX = PC12-AF5 | RX = PD2-AF5
static void uart5_gpio_init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;

	GPIOC->MODER &= ~GPIO_MODER_MODE12_Msk;
	GPIOC->MODER |= GPIO_MODER_MODE12_1;
	GPIOD->MODER &= ~GPIO_MODER_MODE2_Msk;
	GPIOD->MODER |= GPIO_MODER_MODE2_1;

	GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk;
	GPIOC->AFR[1] |= (GPIO_AFRX_AF5 << GPIO_AFRH_AFSEL12_Pos);
	GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL2;
	GPIOD->AFR[0] |= (GPIO_AFRX_AF5 << GPIO_AFRL_AFSEL2_Pos);
}

// TX=
static void lpuart1_gpio_init();
