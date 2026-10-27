#include "uart.h"


static void usart1_gpio_init();
static void usart2_gpio_init();
static void usart3_gpio_init();
static void uart4_gpio_init();
static void uart5_gpio_init();
static void lpuart1_gpio_init();
static void uart_8bit_1stop(USART_TypeDef *bus, int baud_rate, bool rts_cts_control);
static void usart_transmitChar(USART_TypeDef *bus, char c);


bool usart_init(USART_TypeDef *bus, int baud_rate) {




	return true;

}

void usart_transmitStr(USART_TypeDef *bus, uint8_t message[]) {
}

void usart_transmitBytes(USART_TypeDef *bus, uint8_t message[], int nbytes) {
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

static void uart4_gpio_init() {
	return;
}

static void uart5_gpio_init() {
	return;
}

static void lpuart1_gpio_init() {
	return;
};
