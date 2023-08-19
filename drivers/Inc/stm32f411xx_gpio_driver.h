/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: June 28, 2023
 *      Author: EVANS HONU
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_
#include"stm32f4xx.h"


/**This is a GPIO configuration structure*****/
typedef struct{
uint8_t GPIO_Pin_NUMBER;
uint8_t GPIO_Pin_MODE;
uint8_t GPIO_Pin_SPEED;
uint8_t GPIO_Pin_PULLDOWN_UP;
uint8_t GPIO_Pin_TYPE;
uint8_t GPIO_Pin_ALTERNATE;

}GPIO_Pin_Configuration;

typedef struct{

	GPIO_REGISTER_TypeDef *GPIOx;
	GPIO_Pin_Configuration GPIO_Pin_Config;
}GPIO_Handle_TypeDef;

/************************************************************************
 * 					API's supported by this driver file
 *			For more information about the API's check the function definitions in
 *			the @@@@ stm32f411xx_gpio_driver.h
 *
 *************************************************************************/

/**
 * Peripheral clock setup either disable or Enable
 * **/
void GPIO_ClkEnable(GPIO_REGISTER_TypeDef *GPIOx, uint8_t ENABLE_DISABLE);

/***
 * GPIO initialization and deInitialization
 * ****/
void GPIO_Init(GPIO_Handle_TypeDef *);
void GPIO_DeInit(void);

/***
 * GPIO readPin and Write Pin API's
 * ****/
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOuputPort(void);
void GPIO_WriteToOuputPin(void);
void GPIO_ToggleOutputPin(void);

/***
 * GPIO IRQ config (ISR handling)
 * ****/

void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);




#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
