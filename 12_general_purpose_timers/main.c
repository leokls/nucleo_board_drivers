#include <stdint.h>
#include <stdio.h>

#define __IO volatile        // Read / Write permissions
#define __IOM volatile       // Read / Write structure member permissions
#define __IM volatile const  // Read only structure member permissions

// ------- --------- --------- --------- --------- --------- --------- --------
// ------ BLOCK2 PERIPHERAL, RCC, and GPIO
#define BLOCK2_PERIPH_BASE		(0x40000000UL)
#define AHB1_PERIPHERAL_OFFSET  (0x00020000UL)
#define AHB1_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + AHB1_PERIPHERAL_OFFSET)
#define APB1_PERIPHERAL_BASE	BLOCK2_PERIPH_BASE
#define APB2_PERIPHERAL_OFFSET	(0x10000UL)
#define APB2_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + APB2_PERIPHERAL_OFFSET)
#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPHERAL_BASE + RCC_OFFSET)
#define GPIOA_OFFSET			(0x0000UL)
#define GPIOA_BASE				(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)
#define GPIOAEN_BIT				(1U << 0)	   // GPIOx enable bit.
// ------- --------- --------- --------- --------- --------- --------- --------
// USART
#define USART2EN 				(1U << 17);
#define USART2_OFFSET			(0x4400UL)
#define USART2_BASE			    (APB1_PERIPHERAL_BASE + USART2_OFFSET)
#define SYS_FREQ 				16000000	   // The default system frequency.
#define APB1_CLK  				SYS_FREQ       // The same as the system clock.
#define UART_BAUDRATE    		115200		   // A popular Baudrate.
#define CR1_TE					(1U << 3)      // Bit 3: TE: Transmitter enable.
#define CR1_RE					(1U << 2)      // Bit 13 UE: USART enable.
#define CR1_UE					(1U << 13)     // Bit 13 UE: USART enable.
#define SR_TXE					(1U << 7)      // Transmit data register empty.
#define SR_RXNE					(1U << 5)      // Read data register not empty.
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ ADC
// STM32F411CE_DATASHEET.PDF, Table 10. STM32F411xC/xE register boundary addresses
// Figure 14. Memory map
#define ADC1_OFFSET				(0x2000UL)
#define ADC1_BASE				(APB2_PERIPHERAL_BASE + ADC1_OFFSET)
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SysTick
// CORTEX M4 DEVICES GENERIC USER GUIDE.PDF
// https://developer.arm.com/documentation/dui0553/a/
// Table 4-32 System timer registers summary
#define SCS_BASE          	 	(0xE000E000UL)         // System Control Space
#define SysTick_BASE        	(SCS_BASE +  0x0010UL) // SysTick Base Address
//
// ------ Timer 2 (TIM2)
#define TIM2_OFFSET			  (0x0000UL)
#define TIM2_BASE             (APB1_PERIPHERAL_BASE + TIM2_OFFSET)
// ------- --------- --------- --------- --------- --------- --------- --------
#define BIT_0                   (1U << 0)
#define BIT_1                   (1U << 1)
#define BIT_2                   (1U << 2)
#define BIT_3                   (1U << 3)
#define BIT_4                   (1U << 4)
#define BIT_5					(1U << 5)
#define BIT_6					(1U << 6)
#define BIT_7					(1U << 7)
#define BIT_8					(1U << 8)
#define BIT_9					(1U << 9)
#define BIT_10					(1U << 10)
#define BIT_11					(1U << 11)
#define BIT_12					(1U << 12)
#define BIT_13					(1U << 13)
#define BIT_14					(1U << 14)
#define BIT_15					(1U << 15)
#define BIT_16					(1U << 16)
#define BIT_30					(1U << 30)
// ------- --------- --------- --------- --------- --------- --------- --------
// MCU
#define LD2						BIT_5         // MCU's user led.
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ ADC
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 6.3.12 RCC_APB2ENR
#define ADC1EN  				BIT_8         // Bit 8: ADC1EN
#define ADC_CH1					BIT_0         // Binary number 1 at position 0.
#define ADC_SEQ_LEN_1			(0x00)
#define CR2_ADON                BIT_0
#define CR2_SWSTART				BIT_30        // Start conv of reg channels.
#define SR_EOC					BIT_1         // Regular channel end of conv.
#define CR2_CONT				BIT_1
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SYSTICK
// CORTEX-M4 DEVICES GENERIC USER GUIDE
// https://developer.arm.com/documentation/dui0553/a/
// 4.4 system timer, SysTick.
// Table 4-33 SysTick SYST_CSR register bit assignments.
#define SYSTICK_LOAD_VAL		16000
#define CTRL_ENABLE				BIT_0         // Enable the counter.
#define CTRL_CLKSRC				BIT_2		  // Indicates the clock source.
#define CTRL_COUNTFLAG			BIT_16		  // 1 if timer count to 0 last
											  // last time it was read.
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ TIMER 2
#define TIM2EN					BIT_0
#define CR1_CEN					BIT_0         // Bit 0 CEN: Counter enable
#define SR_UIF                  BIT_0         // UIF: Update interrupt flag
// ------- --------- --------- --------- --------- --------- --------- --------
// Define the GPIO peripheral as a structure and registers as its members.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 8.4 GPIO registers.
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,                 Address offset: 0x00      */
  volatile uint32_t PLACEHOLDER[4];
  __IO uint32_t ODR;      /*!< GPIO port output data register,          Address offset: 0x14      */
  volatile uint32_t PLACEHOLDER2[2];
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

// Define the Universal synchronous asynchronous receiver transmitter registers
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 19.6 USART registers
typedef struct
{
  __IO uint32_t SR;         /*!< USART Status register,	                							Address offset: 0x00 */
  __IO uint32_t DR;         /*!< USART Data register,                    							Address offset: 0x04 */
  __IO uint32_t BRR;        /*!< USART Baud rate register,                							Address offset: 0x08 */
  __IO uint32_t CR1;        /*!< USART Control register 1,                							Address offset: 0x0C */
  __IO uint32_t CR2;        /*!< USART Control register 2,                							Address offset: 0x10 */
  __IO uint32_t CR3;        /*!< USART Control register 3,               							Address offset: 0x14 */
  __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, 							Address offset: 0x18 */
} USART_TypeDef;

// ------- --------- --------- --------- --------- --------- --------- --------
// ------ Define the Reset and clock control (RCC) registers.
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

  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                            Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                            Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                            Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                     */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                     Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                     Address offset: 0x44 */
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

// ------ ADC Register
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
typedef struct
{
  __IO uint32_t SR;     /*!< ADC status register,                        						   Address offset: 0x00 */
  __IO uint32_t CR1;    /*!< ADC control register 1,                                               Address offset: 0x04 */
  __IO uint32_t CR2;    /*!< ADC control register 2,                                               Address offset: 0x08 */
  __IO uint32_t SMPR1;  /*!< ADC sample time register 1,                                           Address offset: 0x0C */
  __IO uint32_t SMPR2;  /*!< ADC sample time register 2,                                           Address offset: 0x10 */
  __IO uint32_t JOFR1;  /*!< ADC injected channel data offset register 1,                          Address offset: 0x14 */
  __IO uint32_t JOFR2;  /*!< ADC injected channel data offset register 2,                          Address offset: 0x18 */
  __IO uint32_t JOFR3;  /*!< ADC injected channel data offset register 3,                          Address offset: 0x1C */
  __IO uint32_t JOFR4;  /*!< ADC injected channel data offset register 4,                          Address offset: 0x20 */
  __IO uint32_t HTR;    /*!< ADC watchdog higher threshold register,                               Address offset: 0x24 */
  __IO uint32_t LTR;    /*!< ADC watchdog lower threshold register,                                Address offset: 0x28 */
  __IO uint32_t SQR1;   /*!< ADC regular sequence register 1,                                      Address offset: 0x2C */
  __IO uint32_t SQR2;   /*!< ADC regular sequence register 2,                                      Address offset: 0x30 */
  __IO uint32_t SQR3;   /*!< ADC regular sequence register 3,                                      Address offset: 0x34 */
  __IO uint32_t JSQR;   /*!< ADC injected sequence register,                                       Address offset: 0x38*/
  __IO uint32_t JDR1;   /*!< ADC injected data register 1,                                         Address offset: 0x3C */
  __IO uint32_t JDR2;   /*!< ADC injected data register 2,                                         Address offset: 0x40 */
  __IO uint32_t JDR3;   /*!< ADC injected data register 3,                                         Address offset: 0x44 */
  __IO uint32_t JDR4;   /*!< ADC injected data register 4,                                         Address offset: 0x48 */
  __IO uint32_t DR;     /*!< ADC regular data register,                                            Address offset: 0x4C */
} ADC_TypeDef;
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SysTick Core Register
typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IOM uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IOM uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IM  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ TIM Peripheral
typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

// ------- --------- --------- --------- --------- --------- --------- --------
// ------ TypeDef configuration structures
#define RCC 	((RCC_TypeDef *)  RCC_BASE)
#define GPIOA	((GPIO_TypeDef *) GPIOA_BASE)
#define USART2  ((USART_TypeDef *) USART2_BASE)
#define ADC1 	((ADC_TypeDef *) ADC1_BASE)
#define SysTick ((SysTick_Type *) SysTick_BASE)
#define TIM2	((TIM_TypeDef *) TIM2_BASE)
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ USART
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t Baudrate);
void uart2_rxtx_init(void);
void uart2_write(int ch);
char uart2_read(void);
int __io_putchar(int ch);
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ ADC
uint32_t adc_read(void);
void start_conversion(void);
void pa1_adc_init(void);
uint32_t sensor_value;
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SysTick
void systickDelayMs(int delay);
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ General Purpose Timers
void tim2_1hz_init(void);
// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	// 1. Enable the clock access to GPIOA.
	RCC->AHB1ENR |= GPIOAEN_BIT;
	// 2. Set PA5 pin as output.
	GPIOA->MODER |= BIT_10;
	GPIOA->MODER &= ~BIT_11;

	uart2_rxtx_init();
	tim2_1hz_init();

	while (1)
	{
		// 12.4.5 TIM1 status register (TIMx_SR), bit 0 UIF
		// Wait for the update interrupt flag.
		while (!(TIM2->SR & SR_UIF)) {}

		// Clear UIF.
		TIM2->SR &= ~SR_UIF;

		printf("One second has passed. \n\r");
		GPIOA->ODR ^= LD2;
		systickDelayMs(1000);
	}
}
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ GENERAL PURPOSE DRIVERS
void tim2_1hz_init(void)
{
	// 1. Enable clock access to TIM2, timer 2.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 6.3.11 RCC APB1 peripheral clock enable register (RCC_APB1ENR).
	// TIM2 EN, bit 0
	RCC->APB1ENR |= TIM2EN;

	// 2. Configure the pre-scaler register.
	// The default system timer is 16 Mhz.
	// Pres-caler register.
	TIM2->PSC = 1600 - 1;   // 16.000.000 / 1.600 = 10.000

	// 3. Configure the auto-reload register.
	// Reduce this system timer to 1Hz
	TIM2->ARR = 10000 - 1;  // 10.000 / 10.000 = 1

	// 4. Clear up the counter register.
	TIM2->CNT = 0;

	// 5. Enable timer.
	// 12.4.1 TIM1 control register 1.
	// Bit 0 CEN: Counter enable
	// Overwrite other values; only need bit 0.
	TIM2->CR1 = CR1_CEN;
}
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SYS TICK
void systickDelayMs(int delay)
{
	// ------ Configure the SysTick.

	// Reload with number of clocks per millisecond.
	SysTick->LOAD = SYSTICK_LOAD_VAL;

	// Clear SysTick current value register.
	SysTick->VAL = 0;

	// Enable SysTick and select internal clock source.
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for (int i = 0; i < delay; ++i)
	{
		// Wait before the count flag has been set.
		while ((SysTick->CTRL & CTRL_COUNTFLAG) == 0) {}
	}
	SysTick->CTRL = 0;
}
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ ADC
void pa1_adc_init(void)
{
    // ------ Configure the ADC GPIO pin.

	// 1. Enable clock access to GPIOA.
	RCC->AHB1ENR |= GPIOAEN_BIT;

	// 2. Set the mode of PA1 to analog mode.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 8.4.1 GPIO port mode register GPIOx_MODER
	// Bits 2 and 3. 11: Analog mode.
	GPIOA->MODER |= BIT_2;
	GPIOA->MODER |= BIT_3;

	// ------ Configure the ADC peripheral.

	// 1. Enable clock access to ADC.
	RCC->APB2ENR |= ADC1EN;

	// 2. Configure the ADC parameters.

	// 2a. Conversion sequence start.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 11.12.11 ADC regular sequence register 3 ADC_SQR3.
	// There's single channel - placed in SQ1[4:0].
	ADC1->SQR3 = ADC_CH1;

	// 2b. Conversion sequence end.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 11.12.9 ADC regular sequence register 1 (ADC_SQR1).
	// Regular channel sequence length L[3:0], bits 20 to 23.
	// 0000: 1 conversion.
	ADC1->SQR1 = ADC_SEQ_LEN_1;

	// 3. Enable the ADC peripheral.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 11.12.3 ADC control register 2 (ADC_CR2)
	// Bit 0 ADON (A/D converter ON/OFF):
	// 0 disable ADC conversion; 1: enable ADC.
	ADC1->CR2 |= CR2_ADON;
}

void start_conversion(void)
{
    // 1. Enable continuous conversion.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 11.12.3 ADC control register 2 (ADC_CR2)
	ADC1->CR2 |= CR2_CONT;

	// 2. Start the ADC conversion
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 11.12.3 ADC control register 2 (ADC_CR2)
	// SWSTART: Start conversion of regular channels (bit 30).
	// 0: reset state, 1: starts conversion of regular channels.
	ADC1->CR2 |= CR2_SWSTART;
}

uint32_t adc_read(void)
{
	// Wait before the conversion has completed.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	while (!(ADC1->SR & SR_EOC)) {}

	// Read the result of conversion.
	return (ADC1->DR);
}

// ------ UART
// ------- --------- --------- --------- --------- --------- --------- --------
int __io_putchar(int ch)
{
	uart2_write(ch);
	return(ch);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate)
{
	return ((PeriphClk + (Baudrate / 2U)) / Baudrate);
}


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t Baudrate)
{
	// Compute the UART Baudrate empirically, and store it in BRR register.
	USARTx->BRR = compute_uart_bd(PeriphClk, Baudrate);
}

// ------- --------- --------- --------- --------- --------- --------- --------
void uart2_rxtx_init(void)
{
	// ------ Configure the UART GPIO pin
	// 1. Enable the clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN_BIT;

	// TX init
	// 2. Set PA2 mode to alternate function mode.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 8.4.1 GPIO port mode register GPIOx_MODER
	// Pin 2 requires MODER2[1:0] at bits 4 and 5.
	// 10: Alternate function mode.
	GPIOA->MODER &= ~BIT_4;
	GPIOA->MODER |= BIT_5;

	// 3. Set PA2 alternate function type to UART_TX (AF07)
	// 8.4.9 GPIO alternate function low register (GPIOx_AFRL)
	// AFRL register contains AFRL0[3:0] through ARFL7[3:0].
	// Pin 2 uses ARFL2[3:0].
	// Alternate function selection 0111: AF7
	GPIOA->AFR[0] |= BIT_8;    // AFR[0] maps to AFR low.
	GPIOA->AFR[0] |= BIT_9;
	GPIOA->AFR[0] |= BIT_10;
	GPIOA->AFR[0] &= ~BIT_11;

	// RX init
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// MODER3[1:0] is located at bits 7 and 6.
	// Bits 2y: 2y+1: 3*2 : 3*2 + 1 = 6 : 7.
	// Set bit 6 to 0, bit 7 to 1: Alternate function mode
	// 4. Set PA3 to alternate function mode
	GPIOA->MODER &= ~BIT_6;
	GPIOA->MODER |= BIT_7;

	// 5. Set PA3 alternate function type to UART_RX
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 8.4.9. GPIOx_AFRL3[3:0] set to 0111: AF7
	GPIOA->AFR[0] |= BIT_12;    // AFR[0] maps to AFR low.
	GPIOA->AFR[0] |= BIT_13;
	GPIOA->AFR[0] |= BIT_14;
	GPIOA->AFR[0] &= ~BIT_15;

	// ------ Configure the UART module.
	// 1. Enable the clock access to UART2
	RCC->APB1ENR |= USART2EN;

	// 2. Configure the Baudrate.
    // Derive an equation empirically.
    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	// 3. Configure the transfer direction.
    // RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
    // 19.6.4 Control register 1 (USART_CR1).
    // Bit 3: TE. Transmitter enable: 0: disabled, 1: enabled.
    // Set the default state of the UART transmitter:
    // Data width: 8 bits. USART_CR1 bit 12: M, word length.
    // 0: 1 start bit, 8 data bits, n stop bit.
    // Default number of stop and end bits and parity.
    // The number of stop bits is configured in CR2:
    // 19.6.5 Control register 2 (USART_CR2)
    // Bits 13:12, 00: 1 stop bit.
    // Hardware stop control is disabled.
    // Parity is set to even.
    // Set TE to 1, and make every other bit 0 to use the default settings.
    // Don't need to configure the USART2_CR2 register.
    // Bit 2 RE: receiver enable.
    USART2->CR1 = (CR1_TE | CR1_RE);         // enable transmitter and receiver.

	// 4. Enable the UART module.
    // RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
    // 19.6.4 Control register 1 (USART_CR1).
    // Bit 13 UE: USART enable.
    USART2->CR1 |= CR1_UE;        // |= add the USART enable bit.
}

// ------- --------- --------- --------- --------- --------- --------- --------
void uart2_write(int ch)
{
	// 1. Check whether the transmit data register is empty.
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 19.6.1 USART_SR.
	// TXE: Transmit data register empty.
	while (!(USART2->SR & SR_TXE)) {}

	// 2. Write to the transmit data register.
	USART2->DR = (ch & (0xFF));
}

// ------- --------- --------- --------- --------- --------- --------- --------
char uart2_read(void)
{
	// 1. Check whether the receive data register is NOT empty..
	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 19.6.1 USART_SR.
	// RXNE: Read data register not empty
	while (!(USART2->SR & SR_RXNE)) {}

	// 2. Read data
	return (USART2->DR);
}
