/*
 * stm32f407xx_gpio.c
 *
 *  Created on: 6 בדצמ׳ 2020
 *      Author: Itsik Ben Atar
 */

#include "stm32f407xx_gpio_driver.h"

/*Periphral Clock setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*init and De-init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

}
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx){

}

/*Data read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber){

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t value){

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value){

}
void GPIO_ToggleOutPutPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber){

}

/*IRQ Configuration and ISR handling*/
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDi){

}
void GPIO_IRQHandling(uint8_t pinNumber){

}
