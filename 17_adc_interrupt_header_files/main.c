// BHM Engineering Academy
// https://cortex-m.com/lessons/
// Embedded Systems Bare-Metal Programming Ground Up™ (STM32)

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "adc.h"

static void adc_callback(void);
uint32_t sensor_value;

int main(void)
{
	uart2_tx_init();
	pa1_adc_interrupt_init();
	start_conversion();

	while(1) {}
}

static void adc_callback(void)
{
	sensor_value = ADC1->DR;
	printf("Sensor reads: %ld\n\r", sensor_value);

}
void ADC_IRQHandler(void)
{
	// Check the status of End-of-Conversion in the status register.
	if((ADC1->SR & SR_EOC) !=0)
	{
		// Clear the End-of-Conversion register.
		ADC1->SR &= ~SR_EOC;
		adc_callback();
	}
}
