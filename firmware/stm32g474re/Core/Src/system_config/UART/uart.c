#include "UART/uart.h"
#include "GPIO/gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RX_CAP 128

typedef struct {
	uint8_t buf[RX_CAP];
	volatile uint32_t head;
	volatile uint32_t tail;
} USART_RecieverBuffer;

static USART_RecieverBuffer usart1_rx;
static USART_RecieverBuffer usart2_rx;
static USART_RecieverBuffer usart3_rx;

static bool usart_rxPush(USART_RecieverBuffer *rx, uint8_t data);
static bool usart_rxPop(USART_RecieverBuffer *rx, uint8_t *data_buffer);
static USART_RecieverBuffer * usart_rxGet(USART_TypeDef *uart);

bool usart_init(USART_TypeDef *uart, int baud_rate) {
	if (baud_rate <= 0) {
		return false;
	}

	GPIO_TypeDef *rx_port;
	GPIO_TypeDef *tx_port;
	gpio_pin_t rx_pin;
	gpio_pin_t tx_pin;
	IRQn_Type irq;

	// USART1_RX = PA10 | USART1_TX = PA9
	if (uart == USART1) {
		rx_port = GPIOA;
		tx_port = GPIOA;
		rx_pin = GPIO_PIN_10;
		tx_pin = GPIO_PIN_9;
		irq = USART1_IRQn;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// USART2_RX = PD6 | USART2_TX = PD5
	} else if (uart == USART2) {
		rx_port = GPIOD;
		tx_port = GPIOD;
		rx_pin = GPIO_PIN_6;
		tx_pin = GPIO_PIN_5;
		irq = USART2_IRQn;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	// USART3_RX = PC11 | USART3_TX = PC10
	} else if (uart == USART3) {
		rx_port = GPIOC;
		tx_port = GPIOC;
		rx_pin = GPIO_PIN_11;
		tx_pin = GPIO_PIN_10;
		irq = USART3_IRQn;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	} else {
		return false;
	}

	// Disable UART before configuration (in case of re-initialization)
	NVIC_DisableIRQ(irq);
	uart->CR1 &= ~USART_CR1_RXNEIE;
	uart->CR1 &= ~USART_CR1_UE;

	gpio_config_t config = gpio_getDefaultConfig();
	config.alternate = GPIO_AF_7;
	config.mode = GPIO_MODE_AF;
	gpio_init(rx_port, rx_pin, &config);
	gpio_init(tx_port, tx_pin, &config);

	// Configure 8N1 and HW FIFO
	uart->CR1 &= ~(USART_CR1_M0
			     | USART_CR1_M1
				 | USART_CR1_PCE
				 | USART_CR1_OVER8);
	uart->CR2 &= ~USART_CR2_STOP_Msk;
	uart->CR1 |= USART_CR1_FIFOEN;

	// Assumes USART is using the CPU clock with no prescalers
	// TODO: if this changes in the future, need to make this code
	//       more robust
	SystemCoreClockUpdate();
	uint32_t usart_div = (SystemCoreClock + (baud_rate / 2)) / baud_rate;
	uart->BRR = usart_div;

	// Clear any stale data in HW FIFO and RX buffer
	uart->RQR = USART_RQR_RXFRQ;
	uart->ICR = USART_ICR_ORECF
	          | USART_ICR_FECF
	          | USART_ICR_NECF
	          | USART_ICR_PECF;
	usart_rxFlush(uart);

	// Enable UART after configuration
	uart->CR1 |= USART_CR1_TE | USART_CR1_RE;
	uart->CR1 |= USART_CR1_UE;
	uart->CR1 |= USART_CR1_RXNEIE;
	NVIC_ClearPendingIRQ(irq);
	NVIC_EnableIRQ(irq);

	return true;
}

void usart_transmitString(USART_TypeDef *uart, const char *msg) {
	for (const char *p = msg; *p != '\0'; p++) {
		while (!(uart->ISR & USART_ISR_TXE)) {}
		uart->TDR = (uint8_t)*p;
	}

	while (!(uart->ISR & USART_ISR_TC)) {}
}


void usart_transmitBytes(USART_TypeDef *uart, const uint8_t bytes[], int nbytes) {
	for (int i = 0; i < nbytes; i++) {
		while (!(uart->ISR & USART_ISR_TXE)) {}
		uart->TDR = bytes[i];
	}
	while (!(uart->ISR & USART_ISR_TC)) {}
}

int usart_receiveBytes(USART_TypeDef *uart, uint8_t buffer[], int nbytes) {
	USART_RecieverBuffer * rx = usart_rxGet(uart);
	if (rx == NULL) {
		return 0;
	}

	int read_count = 0;
	while (read_count < nbytes) {
		if (!usart_rxPop(rx, &buffer[read_count])) {
			return read_count;
		}
		read_count++;
	}

	return nbytes;
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
		return true;
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

static bool usart_rxPush(USART_RecieverBuffer *rx, uint8_t data) {
	uint32_t next = (rx->head + 1) % RX_CAP;
	if (next == rx->tail) {
		return false;
	}

	rx->buf[rx->head] = data;
	rx->head = next;

	return true;
}

static bool usart_rxPop(USART_RecieverBuffer *rx, uint8_t *data_buffer) {
	if (rx->head == rx->tail) {
		return false;
	}

	*data_buffer = rx->buf[rx->tail];
	rx->tail = (rx->tail + 1) % RX_CAP;

	return true;
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
		USART1->ICR = USART_ICR_ORECF;
		// TODO: handle hw fifo overruns in the future
		// For now, we just clear the overrun flag
		// The hardware discards incoming bytes
	}

	while (USART1->ISR & USART_ISR_RXNE_RXFNE) {
		if (!usart_rxPush(&usart1_rx, (uint8_t)USART1->RDR)) {
			// TODO: handle rx buffer overruns in the future
			// For now, we just discard incoming bytes
		}
	}
}

void USART2_IRQHandler() {
	if (USART2->ISR & USART_ISR_ORE) {
		USART2->ICR = USART_ICR_ORECF;
		// TODO: handle hw fifo overruns in the future
		// For now, we just clear the overrun flag
		// The hardware discards incoming bytes
	}

	while (USART2->ISR & USART_ISR_RXNE_RXFNE) {
		if (!usart_rxPush(&usart2_rx, (uint8_t)USART2->RDR)) {
			// TODO: handle rx buffer overruns in the future
			// For now, we just discard incoming bytes
		}
	}
}

void USART3_IRQHandler() {
	if (USART3->ISR & USART_ISR_ORE) {
		USART3->ICR = USART_ICR_ORECF;
		// TODO: handle hw fifo overruns in the future
		// For now, we just clear the overrun flag
		// The hardware discards incoming bytes
	}

	while (USART3->ISR & USART_ISR_RXNE_RXFNE) {
		if (!usart_rxPush(&usart3_rx, (uint8_t)USART3->RDR)) {
			// TODO: handle rx buffer overruns in the future
			// For now, we just discard incoming bytes
		}
	}
}
