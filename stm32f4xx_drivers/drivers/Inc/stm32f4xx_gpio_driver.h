

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_
#include<stdint.h>
#include "stm32f407xx.h"

/*This is GPIO pin configuration structure(which is a member of Handle structure of GPIO)*/

typedef struct
{
	uint8_t GPIO_PinNumber;       /*<possible values from @GPIO_PIN_NO>*/
	uint8_t GPIO_PinMode;         /*<possible values from @GPIO_MODES>*/
	uint8_t GPIO_PinSpeed; 		  /*<possible values from @GPIO_SPEED>*/
	uint8_t GPIO_PinPuPdControl;  /*<possible values from @GPIO_PUPD_CONTROL>*/
	uint8_t GPIO_PinOpType;		  /*<possible values from @GPIO_OP_TYPES>*/
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*This is the handle structure for GPIO*/

typedef struct
{
	GPIO_RegDef_t *pGPIOx;  /*This holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;  /*This holds GPIO pin configuration settings*/
}GPIO_Handle_t;


/**************************APIs for handling the GPIO port********************************/

/*For Gpio Initialization/Deinitialization*/

void GPIO_Init(GPIO_Handle_t* pGpioHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*For Gpio Clock Control*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/*For Gpio Read from pin/port*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*For Gpio Write to output pin/port*/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_WriteToOutpuPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*For Gpio Interrupt Handling*/

void GPIO_IRQConfig(uint8_t IRQ_number,uint8_t IRQ_priority,uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

/*GPIO pin possible modes*/
//@GPIO_MODES

#define GPIO_MODE_IN     	0
#define GPIO_MODE_OUT    	1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_RT  	4
#define GPIO_MODE_IT_FT 	5
#define GPIO_MODE_IT_RFT 	6

/*GPIO pin possible o/p types*/
//@GPIO_OP_TYPES
#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1

/*GPIO pin possible o/p speeds*/
//@GPIO_SPEED
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST 	2
#define GPIO_SPEED_HIGH 	3

/*GPIO pin pull up and pull down register*/
//@GPIO_PUPD_CONTROL

#define GPIO_NO_PUPD 		0
#define GPIO_PU 			1
#define GPIO_PD 			2

/*GPIO possible pin numbers*/
//@GPIO_PIN_NO

#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15
#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
