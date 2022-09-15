#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"

#define GPIOAEN_BIT			   (1U<<0)
#define LED_PIN				   (1U<<5)

char keyPressed;

static void uart_callback(void);

int main(void)
{
	RCC->AHB1ENR |= GPIOAEN_BIT;
	GPIOA->MODER |= (1U<<10);     // Bit 10: 1
	GPIOA->MODER &= ~(1U<<11);    // Bit 11: 0

	uart2_rx_interrupt_init();

	while(1)
	{

	}

}

static void uart_callback(void)
{
	// Store the key pressed in the data register.
	keyPressed =  USART2->DR;

	if(keyPressed == '1')
	{
		// Turn the user LED on.
		GPIOA->ODR |= LED_PIN;
	}
	else
	{
		// Turn the user LED off.
		GPIOA->ODR &= ~LED_PIN;
	}
}

void USART2_IRQHandler(void)
{
	// See if the RXNE bit is set
	if(USART2->SR & SR_RXNE)
	{
		uart_callback();
	}
}
