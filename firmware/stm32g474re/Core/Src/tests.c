#include "stm32g474xx.h"
#include "tests.h"
#include "gpio.h"
#include "systick.h"

#include <stdint.h>

#define LED_PIN GPIO_PIN_5
#define BUTTON_PIN GPIO_PIN_13
#define DEBOUNCE_DELAY_MS 25


void gpio_test1(void){
    gpio_config_t config = gpio_getDefaultConfig();

    config.mode = GPIO_MODE_OUTPUT;
    gpio_init(GPIOA, LED_PIN, &config);

    config = gpio_getDefaultConfig();
    config.mode = GPIO_MODE_INPUT;
    gpio_init(GPIOC, BUTTON_PIN, &config);

    systick_init(1000);

    gpio_state_t button_state = gpio_read(GPIOC, BUTTON_PIN);
    gpio_state_t prev_button_state = button_state;
    uint32_t debounce_start_tick = systick_get_ticks();
    while (1) {
    	button_state = gpio_read(GPIOC, BUTTON_PIN);

    	if (button_state != prev_button_state) {
    		if (systick_elapsed_ticks(debounce_start_tick) >= DEBOUNCE_DELAY_MS) {
    			if (button_state == GPIO_STATE_HIGH) {
    				gpio_toggle(GPIOA, LED_PIN);
    			}
    			prev_button_state = button_state;
    			debounce_start_tick = systick_get_ticks();
    		}
    	} else {
    		debounce_start_tick = systick_get_ticks();
    	}
    }
}


void gpio_test2(void) {
	gpio_config_t config = gpio_getDefaultConfig();
	config.mode = GPIO_MODE_OUTPUT;
	gpio_init(GPIOA, LED_PIN, &config);

	systick_init(1000);
	while (1) {
		gpio_write(GPIOA, LED_PIN, GPIO_STATE_HIGH);
		systick_delay_ticks(500);
		gpio_write(GPIOA, LED_PIN, GPIO_STATE_LOW);
		systick_delay_ticks(500);
	}

}


