// BHM Engineering Academy
// https://cortex-m.com/lessons/
// Embedded Systems Bare-Metal Programming Ground Up™ (STM32)

#include "stm32f4xx.h"

#define GPIOAEN (1U<<0)
#define PIN5    (1U<<5)
#define LED_PIN PIN5

int main(void)
{
	RCC->AHB1ENR |= GPIOAEN;

	for (;;)
	{
		GPIOA->MODER |= (1U << 10);
		GPIOA->MODER &= ~(1U << 11);
		GPIOA->ODR ^= LED_PIN;
		for (int i = 0; i < 1000000; ++i) {}
	}
}

