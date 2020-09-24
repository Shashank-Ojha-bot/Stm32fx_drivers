#include "stm32f4xx_gpio_driver.h"
#include<stdint.h>
/*For Gpio Initialization/Deinitialization*/

/*@fn                      -GPIO_Init


  @brief                   -This function initializes the given GPIO port

  @param[in]               -pointer variable of the handle structure of GPIO (which consists of GPIO peripheral base address and configuration structure of GPIO as its members)
  @param[in]               -
  @param[in]               -

  @return                  -none

  @Note                    -none
 */
void GPIO_Init(GPIO_Handle_t* pGpioHandle)
{
	/*Enabling the peripheral clock*/
	GPIO_PeriClockControl(pGpioHandle->pGPIOx, ENABLE);
	uint32_t temp;
	/*Initializing mode of GPIO*/
	if(pGpioHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{/*Non Interrupt modes*/

		temp=pGpioHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGpioHandle->pGPIOx->MODER&=~(0x3<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGpioHandle->pGPIOx->MODER|=temp;
	}
	else
	{/*Interrupt modes*/

	}
	temp=0;

	/*Initializing speed of GPIO*/

	temp=pGpioHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGpioHandle->pGPIOx->OSPEEDR&=~(0x3<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGpioHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	/*Initializing output type of GPIO*/

	if(pGpioHandle->pGPIOx->MODER==GPIO_MODE_OUT)
	{
		temp=pGpioHandle->GPIO_PinConfig.GPIO_PinOpType<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGpioHandle->pGPIOx->OTYPER&=~(0x1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGpioHandle->pGPIOx->OTYPER|=temp;
		temp=0;
	}

	/*Initializing internal pull up/pull down registers of GPIO*/

	temp=pGpioHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGpioHandle->pGPIOx->PUPDR&=~(0x3<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGpioHandle->pGPIOx->PUPDR|=temp;
	temp=0;

	/*Initializing Alternate functionality of GPIO*/

	if(pGpioHandle->pGPIOx->MODER==GPIO_MODE_ALTFN)
	{
		uint8_t temp1=pGpioHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		uint8_t temp2=pGpioHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGpioHandle->pGPIOx->AFR[temp1]&=~(0xF<<4*temp2);
		pGpioHandle->pGPIOx->AFR[temp1]|=pGpioHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2);
	}
}

/*@fn                      -GPIO_DeInit


  @brief                   -This function deinitializes the given GPIO port

  @param[in]               -base address of the GPIO port to be deinitialized
  @param[in]               -
  @param[in]               -

  @return                  -none

  @Note                    -Reset register of the AHB1 bus is used to reset the GPIO peripheral
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
       GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx==GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*For Gpio Clock Control*/

/*@fn                      -GPIO_PeriClockControl


  @brief                   -This function enables or disable the clock of the given GPIO port

  @param[in]               -base address of the GPIO port
  @param[in]               -Macro for set/reset(enable/disable)
  @param[in]               -

  @return                  -none

  @Note                    -none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{

if(EnorDi==ENABLE)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_PCLK_EN;
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_PCLK_EN;
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_PCLK_EN;
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_PCLK_EN;
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_PCLK_EN;
	}
	else if(pGPIOx==GPIOF)
	{
		GPIOF_PCLK_EN;
	}
	else if(pGPIOx==GPIOG)
	{
		GPIOG_PCLK_EN;
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_PCLK_EN;
	}
	else if(pGPIOx==GPIOI)
	{
		GPIOI_PCLK_EN;
	}
}
else
{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI;
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI;
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI;
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI;
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI;
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DI;
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DI;
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI;
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_DI;
		}
	}
}

/*For Gpio Read from pin/port*/

/*@fn                      -GPIO_ReadFromInputPin


  @brief                   -This function reads from given pin of a given GPIO port

  @param[in]               -base address of the GPIO port
  @param[in]               -pin number of the given port to read data from
  @param[in]               -

  @return                  -set/reset value(0/1)

  @Note                    -none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
uint8_t value;
value=(uint8_t)pGPIOx->IDR<<PinNumber & 0x1;
return value;
}

/*@fn                      -GPIO_ReadFromInputPort


  @brief                   -This function reads from a given GPIO port

  @param[in]               -base address of the GPIO port
  @param[in]               -
  @param[in]               -

  @return                  -set/reset value(0/1)

  @Note                    -none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}

/*For Gpio Write to output pin/port*/

/*@fn                      -GPIO_WriteToOutputPort


  @brief                   -This function writes on a given GPIO port

  @param[in]               -base address of the GPIO port
  @param[in]               -value to be written to that port
  @param[in]               -

  @return                  -none

  @Note                    -none
 */


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
pGPIOx->ODR=value;
}

/*@fn                      -GPIO_WriteToOutputPin


  @brief                   -This function writes on a given pin of a given GPIO port

  @param[in]               -base address of the GPIO port
  @param[in]               -pin number of the given GPIO port on which the value is to be written
  @param[in]               -value to be written to that port

  @return                  -none

  @Note                    -none
 */

void GPIO_WriteToOutpuPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value)
{
if(value==SET)
{
	//Write 1 to corresponding bit field of GPIO port
	pGPIOx->ODR|=(1<<PinNumber);
}
else
{
	//Write 0 to corresponding bit field of GPIO port
	pGPIOx->ODR &=~(1<<PinNumber);
}

/*@fn                      -GPIO_ToggleOutputPin


  @brief                   -This function toggles a pin on the GPIO port

  @param[in]               -base address of the GPIO port
  @param[in]               -pin number of the given GPIO port on which the pin is to be toggled.
  @param[in]               -

  @return                  -none

  @Note                    -none
 */

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
pGPIOx->ODR ^=(1<<PinNumber);
}

/*For Gpio Interrupt Handling*/

void GPIO_IRQConfig(uint8_t IRQ_number,uint8_t IRQ_priority,uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
