#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>


void systick_init(uint32_t tick_hz);
void systick_delay_ticks(uint32_t delay);
uint32_t systick_get_ticks(void);
uint32_t systick_elapsed_ticks(uint32_t start);


#endif
