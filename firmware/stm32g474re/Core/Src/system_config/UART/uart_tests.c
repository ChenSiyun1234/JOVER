#include "TestDefinition.h"
#include "UART/uart.h"
#include "Timers/systick.h"

#include <stdbool.h>
#include <string.h>

#define RECEIVE_TIMEOUT_MS 100

/*
 * This function tests transmission and reception via
 * a "loopback". USART1 will send data to itself, which
 * means we can inspect if the transaction was successful
 * without any external hardware.
 *
 * SETUP: Connect USART1 TX pin (PA9) to USART1 RX pin (PA10)
 */
TestStatus testFunction_UART_TXRX_Loopback() {
	if (!usart_init(USART1, 115200)) {
		return FAIL;
	}

	systick_init(1000);

	const char *tx_data = "it's JOVER";
	int tx_data_size = strlen(tx_data);

	usart_transmitString(USART1, tx_data);

	uint8_t rx_data[50];
	int bytes_received = 0;
	uint32_t start = systick_get_ticks();

	/*
	 * We cannot guarantee all bytes will be drained
	 * from hardware FIFO to software buffer when we call
	 * usart_receiveBytes for the first time. For example,
	 * 10 bytes total: 6 bytes in SW buffer, 4 bytes in HW FIFO
	 *
	 * We need to continually call usart_recieveBytes
	 * until we have received everything in the capture buffer.
	 *
	 * All bytes should be received (HW FIFO -> SW Buffer -> Capture Buffer)
	 * before the timeout.  If not, there was an issue and test should fail.
	 */
	while (bytes_received < tx_data_size) {
		if (systick_elapsed_ticks(start) >= RECEIVE_TIMEOUT_MS) {
			return FAIL;
		}

		// Adjusting our parameters based on number of bytes received
		bytes_received += usart_receiveBytes(
				USART1,
				rx_data + bytes_received,
				tx_data_size - bytes_received
		);

	}

	// Verify received data is same as transmitted data
	for (int i = 0; i < tx_data_size; i++) {
		if (tx_data[i] != rx_data[i]) {
			return FAIL;
		}
	}

	return PASS;
}

/*
 * This function tests USART transmission with external
 * hardware (Logic Analyzer, Serial Terminal, etc.).
 *
 * The result of the transmission must be inspected manually
 * on the other device.
 *
 * UART settings (configure on external device) : 115200 8N1
 *
 * Transmitted message (ASCII format) : "it's JOVERwe're BARACK"
 *
 * SETUP: Connect TX pin (PA9) to external device RX pin. Connect
 * 		  STM32 GND to external device GND. If using serial terminal
 * 		  on PC, use a UART to USB adapter. Setup capture on
 * 		  external device, then turn on STM32.
 */
TestStatus testFunction_UART_TX_External() {
	if (!usart_init(USART1, 115200)) {
		return FAIL;
	}

	const uint8_t bytes[10] = { 0x69, 0x74, 0x27, 0x73, 0x20, 0x4A, 0x4F, 0x56, 0x45, 0x52 };
	usart_transmitBytes(USART1, bytes, 10);

	const char *msg = "we're BARACK";
	usart_transmitString(USART1, msg);

	return NEEDS_VERIFICATION;
}

// TODO: finish this
TestStatus testFunction_UART_RX_External() {
	return FAIL;
}
