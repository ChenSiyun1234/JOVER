#include "UART/uart.h"
#include "GPIO/gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RX_CAP 128

typedef struct {
	uint8_t buf[RX_CAP];
	uint32_t head;
	uint32_t tail;
} USART_RecieverBuffer;

static USART_RecieverBuffer usart1_rx;
static USART_RecieverBuffer usart2_rx;
static USART_RecieverBuffer usart3_rx;

static void usart_rxPush(USART_RecieverBuffer *rx, uint8_t data);
static uint8_t usart_rxPop(USART_RecieverBuffer *rx);
static USART_RecieverBuffer * usart_rxGet(USART_TypeDef *uart);

bool usart_init(USART_TypeDef *uart, int baud_rate) {
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
		NVIC_EnableIRQ(USART1_IRQn);
	// USART2_RX = PD6 | USART2_TX = PD5
	} else if (uart == USART2) {
		rx_port = GPIOD;
		tx_port = GPIOD;
		rx_pin = GPIO_PIN_6;
		tx_pin = GPIO_PIN_5;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
		NVIC_EnableIRQ(USART2_IRQn);
	// USART3_RX = PC11 | USART3_TX = PC10
	} else if (uart == USART3) {
		rx_port = GPIOC;
		tx_port = GPIOC;
		rx_pin = GPIO_PIN_11;
		tx_pin = GPIO_PIN_10;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
		NVIC_EnableIRQ(USART3_IRQn);
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

	// Enable hardware RXFIFO and interrupt
	uart->CR1 |= USART_CR1_FIFOEN;
	uart->CR1 |= USART_CR1_RXNEIE;

	// Enable USART
	uart->CR1 |= USART_CR1_UE;
	// Enable transmitter and receiver
	uart->CR1 |= USART_CR1_TE | USART_CR1_RE;

	return true;
}

void usart_transmitStr(USART_TypeDef *uart, const char *msg) {
	for (const char *p = msg; *p != '\0'; p++) {
		while (!(uart->ISR & USART_ISR_TXE)) {}
		uart->TDR = (uint8_t)*p;
	}
	while (!(uart->ISR & USART_ISR_TC)) {}
}


void usart_transmitBytes(USART_TypeDef *uart, const uint8_t msg[], int nbytes) {
	for (int i = 0; i < nbytes; i++) {
		while (!(uart->ISR & USART_ISR_TXE)) {}
		uart->TDR = msg[i];
	}
	while (!(uart->ISR & USART_ISR_TC)) {}
}

int usart_receiveBytes(USART_TypeDef *uart, uint8_t buffer[], int nbytes) {
	USART_RecieverBuffer * rx = usart_rxGet(uart);
	if (rx == NULL) {
		return 0;
	}

	int read_count = 0;
	while (!usart_rxIsEmpty(uart) && read_count < nbytes) {
		buffer[read_count++] = usart_rxPop(rx);
	}

	return read_count;
}

bool usart_rxIsFull(USART_TypeDef *uart) {
	USART_RecieverBuffer *rx = usart_rxGet(uart);
	if (rx == NULL) {
		return true;
	}

	return (rx->head + 1) % RX_CAP == rx->tail;
}

bool usart_rxIsEmpty(USART_TypeDef *uart) {
	USART_RecieverBuffer *rx = usart_rxGet(uart);
	if (rx == NULL) {
		return false;
	}

	return rx->head == rx->tail;
}

void usart_rxFlush(USART_TypeDef *uart) {
	USART_RecieverBuffer *rx = usart_rxGet(uart);
	if (rx == NULL) {
		return;
	}

	rx->head = rx->tail;
}

static void usart_rxPush(USART_RecieverBuffer *rx, uint8_t data) {
	rx->buf[rx->head] = data;
	rx->head = (rx->head + 1) % RX_CAP;
}

static uint8_t usart_rxPop(USART_RecieverBuffer *rx) {
	uint8_t data = rx->buf[rx->tail];
	rx->tail = (rx->tail + 1) % RX_CAP;

	return data;
}

static USART_RecieverBuffer * usart_rxGet(USART_TypeDef *uart) {
	if (uart == USART1) {
		return &usart1_rx;
	} else if (uart == USART2) {
		return &usart2_rx;
	} else if (uart == USART3) {
		return &usart3_rx;
	}

	return NULL;
}


void USART1_IRQHandler() {
	if (USART1->ISR & USART_ISR_ORE) {
		USART1->ICR |= USART_ICR_ORECF;
		// TODO: handle hw fifo overruns in the future
		// For now, we just clear the overrun flag
		// The hardware discards incoming bytes
	}

	uint8_t data;
	while (USART1->ISR & USART_ISR_RXNE_RXFNE) {
		data = USART1->RDR;
		if (usart_rxIsFull(USART1)) {
			// TODO: handle rx buffer overruns in the future
			// For now, we just discard incoming bytes
		} else {
			usart_rxPush(&usart1_rx, data);
		}
	}
}

void USART2_IRQHandler() {
	if (USART2->ISR & USART_ISR_ORE) {
		USART2->ICR |= USART_ICR_ORECF;
		// TODO: handle hw fifo overruns in the future
		// For now, we just clear the overrun flag
		// The hardware discards incoming bytes
	}

	uint8_t data;
	while (USART2->ISR & USART_ISR_RXNE_RXFNE) {
		data = USART2->RDR;
		if (usart_rxIsFull(USART2)) {
			// TODO: handle rx buffer overruns in the future
			// For now, we just discard incoming bytes
		} else {
			usart_rxPush(&usart2_rx, data);
		}
	}
}

void USART3_IRQHandler() {
	if (USART3->ISR & USART_ISR_ORE) {
		USART3->ICR |= USART_ICR_ORECF;
		// TODO: handle hw fifo overruns in the future
		// For now, we just clear the overrun flag
		// The hardware discards incoming bytes
	}

	uint8_t data;
	while (USART3->ISR & USART_ISR_RXNE_RXFNE) {
		data = USART3->RDR;
		if (usart_rxIsFull(USART3)) {
			// TODO: handle rx buffer overruns in the future
			// For now, we just discard incoming bytes
		} else {
			usart_rxPush(&usart3_rx, data);
		}
	}
}
