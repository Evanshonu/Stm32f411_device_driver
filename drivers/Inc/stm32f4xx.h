/*
 * @ stm32f4xx.h
 *
 *  	Created on: June 28, 2023
 *      Author: Evans Honu
 */
#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_
#include<stdint.h>


/**<!Memory Sections Macro's****/
#define 	__FLASH_BASE_ADDRESS_		(uint32_t)0x08000000 	/**<!The base address of main memory (flash)***/
#define 	__SRAM1_BASE_ADDRESS_		(uint32_t)0x20000000	/**<!The base address of system memory SRAM1***/
#define 	SRAM1						__SRAM1_BASE_ADDRESS_
#define		SRAM2						(uint32_t)0x20001C00	/**<!The base address of SRAM2***/
#define		__ROM_BASE_ADDRESS_			(uint32_t)0x1FFF0000	/**<!The base address of System memory!!!***/

/*<! AHBx and APBx address	>*/
#define		__PERIPH_BASE_ADDRESS		0x40000000UL
#define		__AHB1PERIPH_BASE_ADDRESS	0x40020000UL
#define 	__AHB2PERIPH_BASE_ADDRESS	0x50000000UL
#define 	__APB2PERIPH_BASE_ADDRESS	0x40010000UL
#define 	__APB1PERIPH_BASE_ADDRESS	__PERIPH_BASE_ADDRESS

/*<! GPIOx base address > where x stands for A,B,C,D,E, etc depending on the package*/

#define		GPIOA_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x0000UL)
#define		GPIOB_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x0400UL)
#define		GPIOC_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x0800UL)
#define		GPIOD_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x0C00UL)
#define		GPIOE_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x1000UL)

/*<!Complete Peripherals connected to AHB1 Bus*/
#define 	DMA1_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x6000UL)
#define		DMA2_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x6400UL)
#define 	RCC_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x3800UL)
#define 	CRC_BASE_ADDRESS			(__AHB1PERIPH_BASE_ADDRESS + 0x3000UL)

/*<!complete Peripherals connected to AHB2 BUS***/
#define		USB_OTG_FS_BASE_ADDRESS		(0x50000000UL)

/*<!complete Peripherals connected to APB1 BUS***/
#define 	TIM2_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x0000UL)
#define		TIM3_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x0400UL)
#define		TIM4_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x0800UL)
#define		TIM5_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x0C00UL)
#define		SPI2_I2S_BASE_ADDRESS		(__APB1PERIPH_BASE_ADDRESS + 0x3800UL)
#define		SPI3_I2S3_BASE_ADDRESS		(__APB1PERIPH_BASE_ADDRESS + 0x3C00UL)
#define 	USART2_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x4400UL)
#define 	I2C1_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x5400UL)
#define 	I2C2_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x5800UL)
#define 	I2C3_BASE_ADDRESS			(__APB1PERIPH_BASE_ADDRESS + 0x5C00UL)

/*<!complete Peripherals connected to APB2 BUS***/
#define 	TIM1_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x0000UL)
#define 	USART1_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x1000UL)
#define 	USART6_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x1400UL)
#define 	ADC1_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x2000UL)
#define 	SPI1_I2S1_BASE_ADDRESS		(__APB2PERIPH_BASE_ADDRESS + 0x3000UL)
#define 	SPI4_I2S4_BASE_ADDRESS		(__APB2PERIPH_BASE_ADDRESS + 0x3400UL)
#define 	SYSCFG_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x3800UL)
#define 	EXTI_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x3C00UL)
#define 	TIM9_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x4000UL)
#define 	TIM10_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x4400UL)
#define 	TIM11_BASE_ADDRESS			(__APB2PERIPH_BASE_ADDRESS + 0x4800UL)
#define 	SPI5_I2S5_BASE_ADDRESS		(__APB2PERIPH_BASE_ADDRESS + 0x5000UL)

/*******************************************************************************************************/
/*************************************************************************************************************
 ** 							PERIPHERIAL REGISTER STRUCTURE DEFINITIONS							        **
 ** 					A basic structure definition for most common used registers in Firmware Engineering **
 ** 							@Written and authored BY EVANS HONU											**
 ** 							#GPIO, #SP1 #UART #I2C #I2S, #CAN etc.....									**
 ***************************************************************************************************** *******/

#define			__IO	volatile

typedef struct{
__IO uint32_t MODER;
__IO uint32_t OTYPER;
__IO uint32_t OSPEEDR;
__IO uint32_t PUPDR;
__IO uint32_t IDR;
__IO uint32_t ODR;
__IO uint32_t BSRR;
__IO uint32_t AFRL;
__IO uint32_t AFRH;

}GPIO_REGISTER_TypeDef;

typedef struct{
__IO	uint32_t CR;
__IO	uint32_t DUMMY[11];
__IO	uint32_t AHB1ENR;
__IO	uint32_t AHB2ENR;
__IO	uint32_t DUMMY2[2];
__IO	uint32_t APB1ENR;
__IO	uint32_t APB2ENR;
}RCC_REGISTER_TypeDef;

typedef struct{
__IO uint32_t SR;
__IO uint32_t CR1;
__IO uint32_t CR2;
__IO uint32_t DUMMY1[8];
__IO uint32_t SQR1;
__IO uint32_t SQR2;
__IO uint32_t SQR3;
__IO uint32_t DUMMY2[5];
__IO uint32_t DR;

}ADC1_REGISTER_TypeDef;



/**ADC1 register cast**/
#define 		    ADC1			((ADC1_REGISTER_TypeDef*)ADC1_BASE_ADDRESS)

/*RCC cast register***/
#define 		    RCC			((RCC_REGISTER_TypeDef*)RCC_BASE_ADDRESS)

/*
 * GPIO peripheral typecast  register definitions ******/
#define				GPIOA		((GPIO_REGISTER_TypeDef *)GPIOA_BASE_ADDRESS)
#define				GPIOB		((GPIO_REGISTER_TypeDef *)GPIOB_BASE_ADDRESS)
#define				GPIOC		((GPIO_REGISTER_TypeDef *)GPIOC_BASE_ADDRESS)
#define				GPIOD		((GPIO_REGISTER_TypeDef *)GPIOD_BASE_ADDRESS)
#define				GPIOE		((GPIO_REGISTER_TypeDef *)GPIOE_BASE_ADDRESS)

/***Clock Enable macros for GPIOx x = A, B,C, D****/

#define		GPIOAEN				(RCC->AHB1ENR |=(1UL<<0))
#define		GPIOBEN				(RCC->AHB1ENR |=(1UL<<1))
#define		GPIOCEN				(RCC->AHB1ENR |=(1UL<<2))

/***Clock Enable macro for SPI***/
#define		SP11EN				(RCC->APB2ENR |=(1UL<<12))
#define		SPI2EN				(RCC->APB1ENR |=(1UL<<14))
#define		SPI3EN				(RCC->APB1ENR |=(1UL<<15))
#define		SPI4EN				(RCC->APB2ENR |=(1UL<<13))
#define		SPI5EN				(RCC->APB2ENR |=(1UL<<20))

/***Clock Enable macro for I2C***/



/*****Generic macros******/
#define		ENABLE				1
#define		DISABLE				0
#define		SET					ENABLE
#define 	RESET				DISABLE


























#endif /* INC_STM32F4XX_H_ */
