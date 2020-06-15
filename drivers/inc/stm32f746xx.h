#pragma once

#include <stdint.h>

#define FLASH_BASEADDR 			0x08000000U
#define SRAM1_BASEADDR 			0x20010000U
#define SRAM2_BASEADDR 			0x2004C000U
#define ROM_BASEADDR			0x1FF00000U
#define SRAM					SRAM1_BASEADDR

#define PERIPHERAL_BASEADDR		0x40000000U
#define APB1PERIPH_BASEADDR		PERIPHERAL_BASEADDR
#define APB2PERIPH_BASEADDR 	0x40010000U
#define AHB1PERIPH_BASEADDR 	0x40020000U
#define AHB2PERIPH_BASEADDR 	0x50000000U
#define AHB3PERIPH_BASEADDR		0x60000000U

// Base addresses for peripherals hanging on AHB1 bus

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800U)
#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2000U)
#define GPIOJ_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2400U)
#define GPIOK_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2800U)

// Base addresses for peripherals hanging on APB1 bus

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00U)
#define I2C4_BASEADDR			(APB1PERIPH_BASEADDR + 0x6000U)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000U)
#define UART7_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800U)
#define UART8_BASEADDR			(APB1PERIPH_BASEADDR + 0x7C00U)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00U)

// Base addresses for peripherals hanging on APB2 bus

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00U)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800U)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400U)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400U)
#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000U)
#define SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400U)

typedef struct {
	// 0x00 = GPIO port mode register
	volatile uint32_t MODER;
	// 0x04 = GPIO port output type register
	volatile uint32_t OTYPER;
	// 0x08 = GPIO port output speed register
	volatile uint32_t OSPEEDR;
	// 0x0C = GPIO port pull-up/pull-down register
	volatile uint32_t PUPDR;
	// 0x10 = GPIO port input data register
	volatile uint32_t IDR;
	// 0x14 = GPIO port output data register
	volatile uint32_t ODR;
	// 0x18 = GPIO port bit set/reset register
	volatile uint32_t BSRR;
	// 0x1C = GPIO port configuration lock register
	volatile uint32_t LCKR;
	// 0x20 = GPIO alternate function low register
	volatile uint32_t AFR[2]; // AFR[0] => AFRL, AFR[1] => AFRH
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t _reserved_1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t _reserved_2;
	volatile uint32_t _reserved_3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t _reserved_4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t _reserved_5;
	volatile uint32_t _reserved_6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t RCC_AHB3LPENR;
	volatile uint32_t _reserved_7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t _reserved_8;
	volatile uint32_t _reserved_9;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t _reserved_10;
	volatile uint32_t _reserved_11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR1;
	volatile uint32_t DCKCFGR2;
} RCC_RegDef_t;

// Peripheral definitions (Peripheral base addresses casted to GPIO_RegDef_t)
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 		((GPIO_RegDef_t*)GPIOK_BASEADDR)
#define RCC   		((RCC_RegDef_t*)RCC_BASEADDR)

// Clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 10) )

// Clock enable macros for I2C peripherals
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )
#define I2C4_PCLK_EN()		( RCC->APB1ENR |= (1 << 24) )

// Clock enable macros for SPI peripherals

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= (1 << 13) )
#define SPI5_PCLK_EN()		( RCC->APB2ENR |= (1 << 20) )
#define SPI6_PCLK_EN()		( RCC->APB2ENR |= (1 << 21) )

// Clock enable macros for U(S)ART peripherals
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()		( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5) )
#define UART7_PCLK_EN()		( RCC->APB1ENR |= (1 << 30) )
#define UART8_PCLK_EN()		( RCC->APB1ENR |= (1 << 31) )

// Clock enable macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )

// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI() 	( RCC->AHB1ENR &= ~(1 << 10) )

// Clock disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )
#define I2C4_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 24) )

// Clock disable macros for SPIx peripherals
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 13) )
#define SPI5_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 20) )
#define SPI6_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 21) )

// Clock disable macros for U(S)ARTx peripherals
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )
#define UART7_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 30) )
#define UART8_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 31) )

// Clock disable macros for SYSCFG peripheral
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )
