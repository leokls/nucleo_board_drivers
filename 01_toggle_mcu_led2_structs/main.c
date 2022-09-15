#include <stdint.h>

#define __IO volatile


#define BLOCK2_PERIPH_BASE		(0x40000000UL)

#define AHB1_PERIPHERAL_OFFSET  (0x00020000UL)
#define AHB1_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + AHB1_PERIPHERAL_OFFSET)

#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPHERAL_BASE + RCC_OFFSET)

#define GPIOA_OFFSET			(0x0000UL)
#define GPIOA_BASE				(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

#define GPIOAEN_BIT				(1U << 0)

#define BIT_10					(1U << 10)
#define BIT_11					(1U << 11)

#define PIN_5					(1U << 5)
#define LD2						PIN_5

// ------- --------- --------- --------- --------- --------- --------- --------
// Define the GPIO peripheral as a structure and registers as its members.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 8.4 GPIO registers.
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
//  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
//  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
//  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
//  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  // For this driver the above four registers above will not be needed;
  // replace them with a placeholder of 4 bytes:
  volatile uint32_t PLACEHOLDER[4];

  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */

  // For this driver the registers defined below will not be needed.
//  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
//  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
//  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


// ------- --------- --------- --------- --------- --------- --------- --------
// Define the Reset and clock control (RCC) registers.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 6.3 RCC registers.
typedef struct
{
//  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
//  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
//  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
//  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
//  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
//  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
//  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
//  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
//  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
//  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
//  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */

	// For this driver the registers above will not be needed; replace
	// them with a placeholder of 12 bytes:
	volatile uint32_t PLACEHOLDER[12];

  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */

    // For this driver the registers defined below will not be needed.
//  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
//  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
//  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
//  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
//  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
//  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
//  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
//  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
//  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
//  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
//  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
//  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
//  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
//  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
//  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
//  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
//  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
//  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
//  uint32_t      RESERVED7[1];  /*!< Reserved, 0x88                                                                    */
//  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;

#define RCC 	((RCC_TypeDef *)  RCC_BASE)
#define GPIOA	((GPIO_TypeDef *) GPIOA_BASE)

// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	// 1. Enable the RCC clock access to the GPIOA peripheral.
	RCC->AHB1ENR |= GPIOAEN_BIT;

	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 8.4.1 GPIO port mode register GPIOx_MODER
	// To configure PA5, set bits 10 to 1 and 11 to 0 in MODER5[1:0]:
	// 01: general purpose output mode.
	// Reset value for port A is 0xA800.0000; 32 bits of MODER register are in
	// the lower half of 0xA800.0000 -- set to 0, by default.
	GPIOA->MODER |= BIT_10;   // bit 10 to 1
	GPIOA->MODER &= ~BIT_11;  // bit 11 to 0

	while (1)
	{
		// Exclusive or toggles the user LED.
		GPIOA->ODR ^= LD2;

		// Set a delay to see the LED toggling.
		for(int i = 0; i < 1000000; ++i) {}
	}
}
