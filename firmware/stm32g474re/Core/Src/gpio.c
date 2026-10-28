#include "gpio.h"
#include <stdint.h>

void gpio_init(GPIO_TypeDef *port, gpio_pin_t pin, const gpio_init_t *config) {
	gpio_enable_clock(port);

	gpio_set_mode(port, pin, config->mode);
	gpio_set_output_type(port, pin, config->output_type);
	gpio_set_output_speed(port, pin, config->speed);
	gpio_set_pull(port, pin, config->pull);
	gpio_set_af(port, pin, config->alternate);
}

gpio_init_t gpio_create_config(
		gpio_mode_t mode,
		gpio_output_type_t output_type,
		gpio_pull_t pull,
		gpio_speed_t speed,
		gpio_af_t alternate
)
{
	struct gpio_init_t config = {
			.mode = mode,
			.output_type = output_type,
			.pull = pull,
			.speed = speed,
			.alternate = alternate
	};

	return config;
}

void gpio_set_mode(GPIO_TypeDef *port, gpio_pin_t pin, gpio_mode_t mode) {
	uint32_t mask = (uint32_t)mode << (GPIO_MODER_BIT_SPAN * (uint32_t)pin);
	port->MODER &= ~mask;
	port->MODER |= mask;
}

void gpio_set_output_type(GPIO_TypeDef *port, gpio_pin_t pin, gpio_output_type_t output_type) {
	uint32_t mask = (uint32_t)output_type << (uint32_t)pin;
	port->OTYPER &= ~mask;
	port->OTYPER |= mask;
}

void gpio_set_output_speed(GPIO_TypeDef *port, gpio_pin_t pin, gpio_speed_t speed) {
	uint32_t mask = (uint32_t)speed << (GPIO_OSPEEDR_BIT_SPAN * (uint32_t)pin);
	port->OSPEEDR &= ~mask;
	port->OSPEEDR |= mask;
}

void gpio_set_pull(GPIO_TypeDef *port, gpio_pin_t pin, gpio_pull_t pull_mode) {
	uint32_t mask = (uint32_t)pull_mode << (GPIO_PUPDR_BIT_SPAN * (uint32_t)pin);
	port->PUPDR &= ~mask;
	port->PUPDR |= mask;
}

void gpio_set_af(GPIO_TypeDef *port, gpio_pin_t pin, gpio_af_t af) {
	uint32_t mask;
	if ((uint32_t)pin < GPIO_AFRH_PIN_OFFSET) {
		mask = (uint32_t)af << (GPIO_AFRX_BIT_SPAN * (uint32_t)pin);
		port->AFR[0] &= ~mask;
		port->AFR[0] |= mask;
	} else {
		mask = (uint32_t)af << (GPIO_AFRX_BIT_SPAN * ((uint32_t)pin - GPIO_AFRH_PIN_OFFSET));
		port->AFR[1] &= ~mask;
		port->AFR[1] |= mask;
	}
}

void gpio_enable_clock(GPIO_TypeDef *port) {
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
