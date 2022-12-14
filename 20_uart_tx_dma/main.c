// BHM Engineering Academy
// https://cortex-m.com/lessons/
// Embedded Systems Bare-Metal Programming Ground Up™ (STM32)

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"

#define GPIOAEN				   (1U<<0)
#define GPIOA_5				   (1U<<5)

static void dma_callback(void);

int main(void)
{
	char message[25] = "DMA transfer completed.\n\r";

	/*1.Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*2.Set PA5 as output pin*/
	GPIOA->MODER |= (1U << 10);
	GPIOA->MODER &= ~(1U << 11);

	uart2_tx_init();
	dma1_stream6_init((uint32_t) message, (uint32_t)&USART2->DR, 25);

	while(1) {}
}

static void dma_callback(void)
{
	GPIOA->ODR |= GPIOA_5;
}

void DMA1_Stream6_IRQHandler(void)
{
	// Check whether the transfer complete interrupt occured.
	if(DMA1->HISR & HISR_TCIF6)
	{
		// Clear up the flag.
		DMA1->HIFCR |= HIFCR_CTCIF6;
		dma_callback();
	}
}
