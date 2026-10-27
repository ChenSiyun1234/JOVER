#ifndef GPIO_H
#define GPIO_H

#include "stm32g474xx.h"
#include <stdint.h>

typedef enum {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_AF,
	GPIO_MODE_ANALOG,
} gpio_mode_t;

typedef enum {
	GPIO_OUTPUT_PP,
	GPIO_OUTPUT_OD
} gpio_output_type_t;


typedef enum {
	GPIO_NOPULL,
	GPIO_PULLUP,
	GPIO_PULLDOWN
} gpio_pull_t;

typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH
} gpio_speed_t;

typedef enum {
	GPIO_AF_0,
	GPIO_AF_1,
	GPIO_AF_2,
	GPIO_AF_3,
	GPIO_AF_4,
	GPIO_AF_5,
	GPIO_AF_6,
	GPIO_AF_7,
	GPIO_AF_8,
	GPIO_AF_9,
	GPIO_AF_10,
	GPIO_AF_11,
	GPIO_AF_12,
	GPIO_AF_13,
	GPIO_AF_14,
	GPIO_AF_15
} gpio_af_t;

typedef enum {
	GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15
} gpio_pin_t;

typedef enum {
	GPIO_STATE_LOW = 0,
	GPIO_STATE_HIGH = 1
} gpio_state_t;

typedef struct {
	gpio_pin_t pin;
	gpio_mode_t mode;
	gpio_output_type_t output_type;
	gpio_pull_t pull;
	gpio_speed_t speed;
	gpio_af_t alternate;
} gpio_init_t;

void gpio_init(GPIO_TypeDef *port, gpio_init_t config);
void gpio_set_mode(GPIO_TypeDef *port, gpio_pin_t pin, gpio_mode_t mode);
void gpio_set_output_type(GPIO_TypeDef *port, gpio_pin_t pin, gpio_output_type_t output_type);
void gpio_set_output_speed(GPIO_TypeDef *port, gpio_pin_t pin, gpio_speed_t speed);
void gpio_set_pull(GPIO_TypeDef *port, uint32_t pin, gpio_pull_t pull_mode);
void gpio_set_af(GPIO_TypeDef *port, uint32_t pin, gpio_af_t af);
void gpio_enable_clock(GPIO_TypeDef *port);

void gpio_write(GPIO_TypeDef *port, gpio_pin_t pin, gpio_state_t state);
gpio_state_t gpio_read(GPIO_TypeDef *port, gpio_pin_t pin);
void gpio_toggle(GPIO_TypeDef *port, gpio_pin_t pin);

#endif
