#include <stdint.h>
#include <stdio.h>

#define __IO volatile        // Read / Write permissions
#define __IOM volatile       // Read / Write structure member permissions
#define __IM volatile const  // Read only structure member permissions

// ------- --------- --------- --------- --------- --------- --------- --------
// USART
#define USART2EN 				(1U << 17);
#define APB1_PERIPHERAL_BASE	BLOCK2_PERIPH_BASE
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
// ------ BLOCK2 PERIPHERAL, RCC, and GPIO
#define BLOCK2_PERIPH_BASE		(0x40000000UL)
#define AHB1_PERIPHERAL_OFFSET  (0x00020000UL)
#define AHB1_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + AHB1_PERIPHERAL_OFFSET)
#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPHERAL_BASE + RCC_OFFSET)
#define GPIOA_OFFSET			(0x0000UL)
#define GPIOA_BASE				(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)
#define GPIOAEN_BIT				(1U << 0)	   // GPIOx enable bit.
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SysTick
// CORTEX M4 DEVICES GENERIC USER GUIDE.PDF
// https://developer.arm.com/documentation/dui0553/a/
// Table 4-32 System timer registers summary
#define SCS_BASE          	 	(0xE000E000UL)         // System Control Space
#define SysTick_BASE        	(SCS_BASE +  0x0010UL) // SysTick Base Address
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
	// For this driver the registers above will not be needed; replace
	// them with a placeholder of 12 bytes:
	volatile uint32_t PLACEHOLDER[12];

  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                            Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                            Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                            Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                     */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                     Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                     Address offset: 0x44 */
} RCC_TypeDef;
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
// ------ TypeDef configuration structures
#define RCC 	((RCC_TypeDef *)  RCC_BASE)
#define GPIOA	((GPIO_TypeDef *) GPIOA_BASE)
#define USART2  ((USART_TypeDef *) USART2_BASE)
#define SysTick ((SysTick_Type *) SysTick_BASE)
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ USART
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t Baudrate);
void uart2_rxtx_init(void);
void uart2_write(int ch);
char uart2_read(void);
int __io_putchar(int ch);
// ------- --------- --------- --------- --------- --------- --------- --------
// ------ SysTick
void systickDelayMs(int delay);

// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	// 1. Enable the clock access to GPIOA.
	RCC->AHB1ENR |= GPIOAEN_BIT;
	// 2. Set PA5 pin as output.
	GPIOA->MODER |= BIT_10;
	GPIOA->MODER &= ~BIT_11;

	uart2_rxtx_init();

	while (1)
	{
		printf("One second has passed. \n\r");
		GPIOA->ODR ^= LD2;
		systickDelayMs(1000);
	}
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
