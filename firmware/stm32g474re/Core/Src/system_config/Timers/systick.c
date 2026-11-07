#include "systick.h"
#include "stm32g474xx.h"

static volatile uint32_t system_ticks = 0;
static uint32_t tick_frequency = 0;

void SysTick_Handler(void){
    system_ticks++;
}

void systick_init(uint32_t tick_hz){
    if (tick_hz == 0) {
        return;
    }

    SystemCoreClockUpdate();

    uint32_t reload = SystemCoreClock / tick_hz;

    if (reload == 0 || reload > 0x1000000UL) {
        return;
    }

    SysTick->CTRL = 0;

    SysTick->LOAD = reload - 1;
    SysTick->VAL = 0;

    system_ticks = 0;
    tick_frequency = tick_hz;

    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_CLKSOURCE_Msk;
}

uint32_t systick_get_ticks(void){
    return system_ticks;
}

uint32_t systick_elapsed_ticks(uint32_t start){
    return systick_get_ticks() - start;
}

void systick_delay_ticks(uint32_t ticks){
    uint32_t start = systick_get_ticks();

    while (systick_elapsed_ticks(start) < ticks) {
    }
}
