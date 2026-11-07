#include "gpio.h"
#include <stdint.h>

#define GPIO_MODER_BIT_SPAN 2U
#define GPIO_OSPEEDR_BIT_SPAN 2U
#define GPIO_PUPDR_BIT_SPAN 2U
#define GPIO_AFRX_BIT_SPAN 4U
#define GPIO_AFRH_PIN_OFFSET 8U

static void gpio_setMode(GPIO_TypeDef *port, gpio_pin_t pin, gpio_mode_t mode);
static void gpio_setOutputType(GPIO_TypeDef *port, gpio_pin_t pin, gpio_output_type_t output_type);
static void gpio_setOutputSpeed(GPIO_TypeDef *port, gpio_pin_t pin, gpio_speed_t speed);
static void gpio_setPull(GPIO_TypeDef *port, gpio_pin_t pin, gpio_pull_t pull_mode);
static void gpio_setAF(GPIO_TypeDef *port, gpio_pin_t pin, gpio_af_t af);
static void gpio_enableClock(GPIO_TypeDef *port);

void gpio_init(GPIO_TypeDef *port, gpio_pin_t pin, const gpio_config_t *config) {
	gpio_enableClock(port);

	gpio_setOutputType(port, pin, config->output_type);
	gpio_setOutputSpeed(port, pin, config->speed);
	gpio_setPull(port, pin, config->pull);
	gpio_setAF(port, pin, config->alternate);
	gpio_setMode(port, pin, config->mode);
}

void gpio_write(GPIO_TypeDef *port, gpio_pin_t pin, gpio_state_t state) {
	uint32_t mask;
	if (state == GPIO_STATE_LOW) {
		mask = 1U << ((uint32_t)pin + GPIO_BSRR_BR0_Pos);
	} else {
		mask = 1U << (uint32_t)pin;
	}

	port->BSRR = mask;
}

gpio_state_t gpio_read(GPIO_TypeDef *port, gpio_pin_t pin) {
	uint32_t mask = (1U << (uint32_t)pin);
	return (port->IDR & mask) ? GPIO_STATE_HIGH : GPIO_STATE_LOW;
}

void gpio_toggle(GPIO_TypeDef *port, gpio_pin_t pin) {
	uint32_t mask = 1U << (uint32_t)pin;
	port->ODR ^= mask;
}

gpio_config_t gpio_getDefaultConfig(void) {
	return (gpio_config_t) {
			.mode = GPIO_MODE_ANALOG,
			.output_type = GPIO_OUTPUT_PP,
			.pull = GPIO_PULL_NOPULL,
			.speed = GPIO_SPEED_LOW,
			.alternate = GPIO_AF_0
	};
}

static void gpio_setMode(GPIO_TypeDef *port, gpio_pin_t pin, gpio_mode_t mode) {
	uint32_t shift = GPIO_MODER_BIT_SPAN * (uint32_t)pin;
	port->MODER = (port->MODER & ~(GPIO_MODER_MODE0_Msk << shift)) | (uint32_t)mode << shift;
}

static void gpio_setOutputType(GPIO_TypeDef *port, gpio_pin_t pin, gpio_output_type_t output_type) {
	uint32_t shift = (uint32_t)pin;
	port->OTYPER = (port->OTYPER & ~(GPIO_OTYPER_OT0_Msk << shift)) | ((uint32_t)output_type << shift);
}

static void gpio_setOutputSpeed(GPIO_TypeDef *port, gpio_pin_t pin, gpio_speed_t speed) {
	uint32_t shift = GPIO_OSPEEDR_BIT_SPAN * (uint32_t)pin;
	port->OSPEEDR = (port->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED0_Msk << shift)) | ((uint32_t)speed << shift);
}

static void gpio_setPull(GPIO_TypeDef *port, gpio_pin_t pin, gpio_pull_t pull_mode) {
	uint32_t shift = GPIO_PUPDR_BIT_SPAN * (uint32_t)pin;
	port->PUPDR = (port->PUPDR & ~(GPIO_PUPDR_PUPD0_Msk << shift)) | ((uint32_t)pull_mode << shift);
}

static void gpio_setAF(GPIO_TypeDef *port, gpio_pin_t pin, gpio_af_t af) {
	uint32_t shift;
	if ((uint32_t)pin < GPIO_AFRH_PIN_OFFSET) {
		shift = GPIO_AFRX_BIT_SPAN * (uint32_t)pin;
		port->AFR[0] = (port->AFR[0] & ~(GPIO_AFRL_AFSEL0_Msk << shift)) | ((uint32_t)af << shift);
	} else {
		shift = GPIO_AFRX_BIT_SPAN * ((uint32_t)pin - GPIO_AFRH_PIN_OFFSET);
		port->AFR[1] = (port->AFR[1] & ~(GPIO_AFRH_AFSEL8_Msk << shift)) | ((uint32_t)af << shift);
	}
}

static void gpio_enableClock(GPIO_TypeDef *port) {
	if (port == GPIOA) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	} else if (port == GPIOB) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	} else if (port == GPIOC) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	} else if (port == GPIOD) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	} else if (port == GPIOE) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	} else if (port == GPIOF) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
	} else if (port == GPIOG) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
	}

	// Read-back for RCC clock-enable synchronization
	(void)RCC->AHB2ENR;
}
