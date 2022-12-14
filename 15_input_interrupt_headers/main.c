#include <stdio.h>
#include "stm32f4xx.h"
#include "systick.h"

#define SYSCFGEN				(1U << 14)

#define BIT_0                   (1U << 0)
#define BIT_2                   (1U << 2)
#define BIT_5					(1U << 5)
#define BIT_10					(1U << 10)
#define BIT_11					(1U << 11)
#define BIT_13					(1U << 13)
#define BIT_16					(1U << 16)
#define BIT_26					(1U << 26)
#define BIT_27					(1U << 27)

#define GPIOAEN_BIT             BIT_0
#define GPIOCEN_BIT				BIT_0
#define PIN5    				BIT_5
#define LED_PIN 				PIN5

#define LINE_13					BIT_13

void pc13_exti_init(void);
void toggle_user_led_ODR(GPIO_TypeDef* GPIOx, int duration, int pin);

// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	// 1. Enable the RCC clock access to the GPIOA peripheral.
	RCC->AHB1ENR |= GPIOAEN_BIT;

	// 2. PA5 is output pin
	GPIOA->MODER |= BIT_10;   // bit 10 to 1
	GPIOA->MODER &= ~BIT_11;  // bit 11 to 0

	pc13_exti_init();

	while (1) {}
}

// ------- --------- --------- --------- --------- --------- --------- --------
void toggle_user_led_ODR(GPIO_TypeDef* GPIOx, int duration, int pin)
{
	// Output Data Register: exclusive or toggles the user LED.
	GPIOx->ODR ^= pin;
	systickDelayMs(duration);
	GPIOx->ODR ^= pin;
	systickDelayMs(duration);
}

static void exti_callback(void)
{
	toggle_user_led_ODR(GPIOA, 250, LED_PIN);
	toggle_user_led_ODR(GPIOA, 250, LED_PIN);
	EXTI->PR=EXTI_PR_PR13;
}

void EXTI15_10_IRQHandler(void)
{
	// startup_stm32f411retx.s -- g_pfnVectors -- the name of all interrupts.
	// find the interrupt's name and create a void-void function.
	// EXTI Line[15:10] interrupts; distinguish which interrupt was triggered.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// Pending register EXTI_PR
	// PR13, bit 13 (pending bit).
	// 0: no trigger request occurred, 1: selected trigger request occurred.
	if ((EXTI->PR & LINE_13) != 0)  // interrupt occurred.
	{
		// Clear the pending register (PR) flag.
		EXTI->PR |= LINE_13;
		exti_callback();
	}
}

void pc13_exti_init(void)
{
	// 0. Disable global interrupts.
	__disable_irq();

	// 1. Enable clock access for GPIOC.
	// GPIOC is connected to AHB1 bus.
	RCC->AHB1ENR |= GPIOCEN_BIT;

	// 2. Enable clock access to SYSCFG.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 6.3.12 RCC APB2 peripheral clock enable register.
	// Bit 14 SYSCFGRST: System configuration controller reset
	// 0: does not reset the System configuration controller.
	// 1: resets the System configuration controller.
	RCC->APB2ENR |= SYSCFGEN;

	// 3. Configure PC13 as input pin.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
    // 8.4.1 GPIO port mode register (GPIOx_MODER)
	// MODER13[1:0] bits 26 and 27.
	// 00: input (reset state)
	GPIOC->MODER &= ~BIT_26;
	GPIOC->MODER &= ~BIT_27;

	// 3. Select port C for EXTI13.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 7.2.6 SYSCFG external interrupt configuration register 4.
	// EXTIx[3:0] configuration: 0010: PC[x] pin.
	// EXTI13[3:0] bits 4 through 7.
	SYSCFG->EXTICR[3] |= BIT_5;   // EXTICR[3] means SYSCFG_EXTICR4.

	// 4. Un-mask EXT13.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 10.3.1 Interrupt mask register (EXTI_IMR).
	// MR13: interrupt mask on line 13. 0: masked, 1: unmasked.
	EXTI->IMR |= BIT_13;

	// 5. Select falling edge trigger.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 10.3.4 Falling trigger selection register (EXTI_FTSR).
	// TR13: 0: Falling edge disabled, 1: Falling trigger enabled.
	EXTI->FTSR |= BIT_13;

	// 6. Enable EXTI13 line in NVIC.
	// core_cm4.h
	// stm32f311xe.h:
    // EXTI15_10_IRQn = 40 External Line[15:10] Interrupts
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	// 7. Enable global interrupts.
	__enable_irq();
}
