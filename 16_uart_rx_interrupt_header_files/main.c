#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"

char targetKey = '1';
char keyPressed;

static void uart_callback(void);

int main(void)
{
	// 1.Enable (RCC) clock access to GPIOA.
	RCC->AHB1ENR |= (1U<<0);

	// 2. Set the PA5 pin as output.
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);

	uart2_rx_interrupt_init();

	while(1) {}
}

static void uart_callback(void)
{
	// Store the key pressed in the data register.
	keyPressed =  USART2->DR;

	if(keyPressed == targetKey)
	{
		// Turn the LED on.
		// Output data register: 1.
		GPIOA->ODR |= (1U<<5);
	}
	else
	{
		// Turn the LED off.
		// Output data register: 0.
		GPIOA->ODR &= ~(1U<<5);
	}
}

void USART2_IRQHandler(void)
{
	// When RXNE bit is set,..
	if(USART2->SR & SR_RXNE)
	{
		// ...execute the callback function.
		uart_callback();
	}
}
