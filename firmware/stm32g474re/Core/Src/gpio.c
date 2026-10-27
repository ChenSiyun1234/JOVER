#include "gpio.h"

void gpio_init(GPIO_TypeDef *port, gpio_init_t config) {

}

void gpio_set_mode(GPIO_TypeDef *port, gpio_pin_t pin, gpio_mode_t mode) {

}

void gpio_set_output_type(GPIO_TypeDef *port, gpio_pin_t pin, gpio_output_type_t output_type) {

}

void gpio_set_output_speed(GPIO_TypeDef *port, gpio_pin_t pin, gpio_speed_t speed) {

}
void gpio_set_pull(GPIO_TypeDef *port, uint32_t pin, gpio_pull_t pull_mode) {

}

void gpio_set_af(GPIO_TypeDef *port, uint32_t pin, gpio_af_t af) {

}

void gpio_enable_clock(GPIO_TypeDef *port) {

}

void gpio_write(GPIO_TypeDef *port, gpio_pin_t pin, gpio_state_t state) {

}

gpio_state_t gpio_read(GPIO_TypeDef *port, gpio_pin_t pin) {
	return GPIO_STATE_LOW;
}

void gpio_toggle(GPIO_TypeDef *port, gpio_pin_t pin) {

}
