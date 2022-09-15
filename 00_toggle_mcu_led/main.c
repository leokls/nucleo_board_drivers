// ------- --------- --------- --------- --------- --------- --------- --------
// STM32F411CE_DATASHEET.PDF
// Figure 14. Memory Map.
// GPIO belongs to 512-Mbyte block 2 Peripherals: 0x4000.0000 - 0xFFFF.FFFF
#define BLOCK2_PERIPH_BASE		(0x40000000UL)

// AHB1 boundary addresses: 0x4002.0000 (start) - 0x4002.FFFF (end)
#define AHB1_PERIPHERAL_OFFSET  (0x00020000UL)
#define AHB1_PERIPHERAL_BASE	(BLOCK2_PERIPH_BASE + AHB1_PERIPHERAL_OFFSET)

// Figure 3. STM32F411xC/xE block diagram
// GPIOA is connected to AHB1 bus.
// Table 10. STM32F411xC/xE register boundary addresses
// Range of GPIOA peripheral's addresses: 0x4002.0000 - 0x4002.03FF
#define GPIOA_OFFSET			(0x0000UL)
#define GPIOA_BASE				(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

// Table 10. STM32F411xC/xE register boundary addresses
#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPHERAL_BASE + RCC_OFFSET)

// ------- --------- --------- --------- --------- --------- --------- --------
// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
// 6.3.9 RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
#define AHB1ENR_CLOCK_EN_REG_OFFSET (0x30UL)
#define RCC_AHB1ENR				(*(volatile unsigned int *)(RCC_BASE + \
								AHB1ENR_CLOCK_EN_REG_OFFSET))

// GPIO registers have at least two registers: direction and data.
// Direction: set the pin to execute input or output.
// Data: store in the data register or take from data register.
// 8.4.1 GPIO port mode register (GPIOx_MODER)
#define MODER_OFFSET			(0x00UL)
#define GPIOA_MODER				(*(volatile unsigned int *)(GPIOA_BASE + \
								MODER_OFFSET))

// OUTPUT DATA REGISTERS: write to these registers to turn the LED on and off
// 8.4.6 GPIO port output data register (GPIOx_ODR)
#define ODR_OFFSET				(0x14UL)
#define GPIOA_ODR_BASE			(*(volatile unsigned int *)(GPIOA_BASE + \
								ODR_OFFSET))

// GPIOAEN is at bit 0 within RCC_AHB1ENR peripheral clock enable register
#define GPIOAEN_BIT				(1U << 0)

// ------- --------- --------- --------- --------- --------- --------- --------
// UM1724-STM32-NUCLEO64-BOARDS-USER-GUIDE.PDF
// 6.4 LEDs
// User LD2: the green LED is a user LED connected to ARDUINO® signal D13
// corresponding to STM32 I/O PA5 (pin 21).
// The I/O is HIGH value, the LED is on; the I/O is LOW, the LED is off.
#define PIN_5					(1U << 5)
#define LD2						PIN_5

// ------- --------- --------- --------- --------- --------- --------- --------
int main(void)
{
	// 1. Enable the RCC clock access to the GPIOA peripheral.
	RCC_AHB1ENR |= GPIOAEN_BIT;

	// RM0383-STM32F411XCE-REFERENCE-MANUAL.PDF
	// 8.4.1 GPIO port mode register GPIOx_MODER
	// To configure PA5, set bits 10 to 1 and 11 to 0 in MODER5[1:0]:
	// 01: general purpose output mode.
	// Reset value for port A is 0xA800.0000; 32 bits of MODER register are in
	// the lower half of 0xA800.0000 -- set to 0, by default.
	GPIOA_MODER |= (1U<<10);    // bit 10 to 1
	GPIOA_MODER &= ~(1U<<11);   // bit 11 to 0

	while (1)
	{
		// Exclusive or toggles the user LED.
		GPIOA_ODR_BASE ^= LD2;

		// Set a delay to see the LED toggling.
		for(int i = 0; i < 1000000; ++i) {}
	}
}
