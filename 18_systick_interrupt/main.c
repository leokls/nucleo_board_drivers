// BHM Engineering Academy
// https://cortex-m.com/lessons/
// Embedded Systems Bare-Metal Programming Ground Up™ (STM32)

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "systick.h"

#define GPIOAEN				   (1U << 0)
#define PIN5				   (1U << 5)

static void systick_callback(void);

int main(void)
{
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);

	uart2_tx_init();
	systick_1hz_interrupt();

	while(1) {}

}

static void systick_callback(void)
{
	printf("One second has passed.\n\r");
	GPIOA->ODR ^= PIN5;
}

void SysTick_Handler(void)
{
	systick_callback();
}
