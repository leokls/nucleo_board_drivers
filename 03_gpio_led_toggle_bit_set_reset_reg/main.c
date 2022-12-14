#include <stdint.h>

// ------- --------- --------- --------- --------- --------- --------- --------

#define __IO volatile

#define BLOCK2_PERIPH_BASE		(0x40000000UL)

#define AHB1_PERIPHERAL_OFFSET  (0x00020000UL)
#define AHB1_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + AHB1_PERIPHERAL_OFFSET)

#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPHERAL_BASE + RCC_OFFSET)

#define GPIOA_OFFSET			(0x0000UL)
#define GPIOA_BASE				(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

// ------- --------- --------- --------- --------- --------- --------- --------
// STM32F411CE_DATASHEET.PDF
// Table 10. STM32F411xC/xE register boundary addresses.
// GPIOC is connected to AHB1 bus.
// Address range of GPIOC peripheral: 0x4002.0800 - 0x4002.0BFF
#define GPIOC_OFFSET			(0x0800UL)
#define GPIOC_BASE				(AHB1_PERIPHERAL_BASE + GPIOC_OFFSET)

// GPIOx enable bit
#define GPIOAEN_BIT				(1U << 0)
#define GPIOCEN_BIT				(1U << 2)

#define BIT_5					(1U << 5)
#define BIT_10					(1U << 10)
#define BIT_11					(1U << 11)
#define BIT_21                  (1U << 21)

// ------- --------- --------- --------- --------- --------- --------- --------
// Define the GPIO peripheral as a structure and registers as its members.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 8.4 GPIO registers.
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               	     Address offset: 0x00      */

  volatile uint32_t PLACEHOLDER[5];
//  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        	 Address offset: 0x04      */
//  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       	 Address offset: 0x08      */
//  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,    	 Address offset: 0x0C      */
//  __IO uint32_t IDR;      /*!< GPIO port input data register,        		 Address offset: 0x10      */
//  __IO uint32_t ODR;      /*!< GPIO port output data register,       		 Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,     		 Address offset: 0x18      */
  // For this driver the registers defined below will not be needed.
  //  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register,	 Address offset: 0x1C      */
  //  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,    	 Address offset: 0x20-0x24 */
} GPIO_TypeDef;


// ------- --------- --------- --------- --------- --------- --------- --------
// Define the Reset and clock control (RCC) registers.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 6.3 RCC registers.
typedef struct
{
	// For this driver the registers above will not be needed; replace
	// them with a placeholder of 12 bytes:
    volatile uint32_t PLACEHOLDER[12];
    __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register, Address offset: 0x30 */
    // For this driver the registers defined below will not be needed.
} RCC_TypeDef;

#define RCC 	((RCC_TypeDef *)  RCC_BASE)
#define GPIOA	((GPIO_TypeDef *) GPIOA_BASE)

// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	// 1. Enable the RCC clock access to the GPIOA peripheral.
	RCC->AHB1ENR |= GPIOAEN_BIT;

	// 2. PA5 is output pin
	GPIOA->MODER |= BIT_10;   // bit 10 to 1
	GPIOA->MODER &= ~BIT_11;  // bit 11 to 0

	while (1)
	{

 	    // Turn PIN5 on, set BS5 to 1 of the Bit set/reset register.
		GPIOA->BSRR = BIT_5;
		for(int i=0;i<100000;i++){}


		// Turn PIN5 off, set BR5 to 1 of the Bit set/reset register.
		GPIOA->BSRR = BIT_21;
		for(int i=0;i<100000;i++){}
	}
}
