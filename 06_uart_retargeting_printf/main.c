#include <stdint.h>
#include <stdio.h>

// ------- --------- --------- --------- --------- --------- --------- --------
// ------ 1. Define the number of USART peripherals of the microcontroller.
// STM32F411CE_DATASHEET.PDF, Block Diagram.
// USART2 is hanging on APB1 bus.
// USART2 is connected to the USB through the debugger.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
// ------ 2. Check the RCC APB1 clock enable register.
// 6.3.11. Bit 17 for USART2 EN.
#define USART2EN 				(1U << 17);
// ------ 3. Find out which GPIO pins the USART2 peripheral uses.
// STM32F411CE_DATASHEET.PDF, Table 9. Alternate function mapping.
// (PA2, AF07): Enable and configure PA2 to use USART2 TX.

// Table 10. STM32F411xC/xE register boundary addresses.
#define APB1_PERIPHERAL_BASE	BLOCK2_PERIPH_BASE
#define USART2_OFFSET			(0x4400UL)
#define USART2_BASE			    (APB1_PERIPHERAL_BASE + USART2_OFFSET)

// ------- --------- --------- --------- --------- --------- --------- --------
#define SYS_FREQ 				16000000	   // The default system frequency.

// In the clock tree, the system clock value is divided by a value to be used
// as the clock for APB bus. In the default state, the divider for the busses
// equals 1.
#define APB1_CLK  				SYS_FREQ       // The same as the system clock.
#define UART_BAUDRATE    		115200		   // A popular Baudrate.

// ------- --------- --------- --------- --------- --------- --------- --------
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
// 19.6.4 Control register 1 (USART_CR1).
#define CR1_TE					(1U << 3)      // Bit 3: TE: Transmitter enable.
#define CR1_UE					(1U << 13)     // Bit 13 UE: USART enable.

// 19.6.1 USART_SR.
#define SR_TXE					(1U << 7)      // Transmit data register empty.

// ------- --------- --------- --------- --------- --------- --------- --------
#define __IO volatile

#define BLOCK2_PERIPH_BASE		(0x40000000UL)

#define AHB1_PERIPHERAL_OFFSET  (0x00020000UL)
#define AHB1_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + AHB1_PERIPHERAL_OFFSET)

#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPHERAL_BASE + RCC_OFFSET)

#define GPIOA_OFFSET			(0x0000UL)
#define GPIOA_BASE				(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)
#define GPIOAEN_BIT				(1U << 0)	   // GPIOx enable bit

#define BIT_4                   (1U << 4)
#define BIT_5					(1U << 5)

#define BIT_8					(1U << 8)
#define BIT_9					(1U << 9)
#define BIT_10					(1U << 10)
#define BIT_11					(1U << 11)


// ------- --------- --------- --------- --------- --------- --------- --------
// Define the GPIO peripheral as a structure and registers as its members.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 8.4 GPIO registers.
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,			           	    				   Address offset: 0x00      */
  volatile uint32_t PLACEHOLDER[7];  															   // unused registers
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,    	 						   Address offset: 0x20-0x24 */
} GPIO_TypeDef;

// ------- --------- --------- --------- --------- --------- --------- --------
// Define the Reset and clock control (RCC) registers.
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF, 6.3 RCC registers.
typedef struct
{
	volatile uint32_t PLACEHOLDER[12]; 															   // unused registers
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                            Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                            Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                            Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                      */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                     Address offset: 0x40 */
} RCC_TypeDef;

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


#define RCC 	((RCC_TypeDef *)  RCC_BASE)
#define GPIOA	((GPIO_TypeDef *) GPIOA_BASE)
#define USART2  ((USART_TypeDef *) USART2_BASE)

// ------- --------- --------- --------- --------- --------- --------- --------
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t Baudrate);
void uart2_tx_init(void);
void uart2_write(int ch);

int __io_putchar(int ch)
{
	uart2_write(ch);
	return(ch);
}

// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	uart2_tx_init();

	while (1)
	{
		printf("The Microcontroller Driver Development Project\n\r");
	}
}

// ------- --------- --------- --------- --------- --------- --------- --------
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
void uart2_tx_init(void)
{
	// ------ Configure the UART GPIO pin
	// 1. Enable the clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN_BIT;

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
    USART2->CR1 = CR1_TE;         // = means clear up all bits except bit 3.

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
