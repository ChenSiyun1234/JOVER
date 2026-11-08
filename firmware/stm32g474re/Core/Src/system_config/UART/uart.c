#include "uart.h"

#include <stddef.h>

#define RX_BUF_CAP 128

typedef struct {
	uint8_t buf[RX_BUF_CAP];
	uint32_t head;
	uint32_t tail;
} USART_RecieverBuffer;

static USART_RecieverBuffer usart1_rx_buffer;
static USART_RecieverBuffer usart2_rx_buffer;
static USART_RecieverBuffer usart3_rx_buffer;

static void rx_bufferPush(USART_RecieverBuffer *rx_buf, uint8_t data);
static uint8_t rx_bufferPop(USART_RecieverBuffer *rx_buf, uint8_t data);
static bool rx_bufferIsEmpty(USART_RecieverBuffer *rx_buf);
static USART_RecieverBuffer *rx_bufferFlush(USART_TypeDef *uart);
static void usart_transmitByte(USART_TypeDef *uart, uint8_t byte);

bool usart_init(USART_TypeDef *uart, uint32_t baud_rate) {
	GPIO_TypeDef *rx_port;
	GPIO_TypeDef *tx_port;
	gpio_pin_t rx_pin;
	gpio_pin_t tx_pin;

	// USART1_RX = PA10 | USART1_TX = PA9
	if (uart == USART1) {
		rx_port = GPIOA;
		tx_port = GPIOA;
		rx_pin = GPIO_PIN_10;
		tx_pin = GPIO_PIN_9;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// USART2_RX = PD6 | USART2_TX = PD5
	} else if (uart == USART2) {
		rx_port = GPIOD;
		tx_port = GPIOD;
		rx_pin = GPIO_PIN_6;
		tx_pin = GPIO_PIN_5;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	// USART3_RX = PC11 | USART3_TX = PC10
	} else if (uart == USART3) {
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
	uart->CR1 &= ~(USART_CR1_M0_Msk | USART_CR1_M1_Msk);
	// Set baud rate
	uart->BRR = usart_div;
	// 1 stop bit
	uart->CR2 &= ~USART_CR2_STOP_Msk;
	// Enable USART
	uart->CR1 |= USART_CR1_UE;
	// Enable transmitter and receiver
	uart->CR1 |= USART_CR1_TE | USART_CR1_RE;

	return true;
}

void usart_transmitBytes(USART_TypeDef *uart, const uint8_t msg[], int nbytes) {
	for (int i = 0; i < nbytes; i++) {
		usart_transmitByte(bus, msg[i]);
	}
	while (!(uart->ISR & USART_ISR_TC)) {}
}

void usart_transmitStr(USART_TypeDef *uart, const char *msg) {
	for (const char *p = msg; *p != '\0'; p++) {
		usart_transmitByte(bus, (uint8_t)*p);
	}
	while (!(uart->ISR & USART_ISR_TC)) {}


int usart_receiveBytes(USART_TypeDef *uart, uint8_t buffer[], uint16_t size) {

	return 0;
}


static void usart_transmitByte(USART_TypeDef *uart, uint8_t byte) {
	while (!(uart->ISR & USART_ISR_TXE)) {}
	uart->TDR = byte;
}

static void rx_bufferPush(USART_RecieverBuffer *rx_buf, uint8_t data) {
	rx_buf->buf[rx_buf->head] = data;
	rx_buf->head = (rx_buf->head + 1) % RX_BUF_CAP;
}


static uint8_t rx_bufferPop(USART_RecieverBuffer *rx_buf) {
	uint8_t data = rx_buf->buf[rx_buf->tail];
	rx_buf->tail = (rx_buf->tail + 1) % RX_BUF_CAP;

	return data;
}


static bool rx_bufferIsEmpty(USART_RecieverBuffer *rx_buf) {
	return true;
}

static void rx_bufferFlush(USART_RecieverBuffer *rx_buf) {
	return;
}

static USART_RecieverBuffer *rx_bufferGet(USART_TypeDef *uart) {
	if (uart == USART1) {
		return &usart1_rx_buffer;
	} else if (uart == USART2) {
		return &usart2_rx_buffer;
	} else if (uart == USART3) {
		return &usart3_rx_buffer;
	}

	return NULL;
}
