/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Jul 14, 2020
 *      Author: Shashank
 */

#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

#include<stdint.h>
#include "stm32f407xx.h"

/*This is Configuration structure for I2C*/
typedef struct
{
	uint32_t I2C_SCL_SPEED;                 /*possible values from @I2C_SCL_SPEED*/
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACK_CONTROL;                /*possible values from @I2C_ACK_CONTROL*/
	uint8_t I2C_FM_DutyCycle;               /*possible values from @I2C_FM_DutyCycle*/
}I2C_Config_t;

/*This is the handle structure of I2C*/
typedef struct
{
	I2C_RegDef_t *pI2Cx;                /*base address of I2Cx peripheral*/
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/**************************APIs for handling the I2C peripheral********************************/

/*For I2C Initialization/Deinitialization*/

void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*For I2C SClock Control*/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*Data send and receive*/
/*Data sending and receiving is in Blocking mode i.e.non interrupt mode*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t Slave_addr);  //Slave addr is extra as compared to SPI
void I2C_ReceiveSendData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len,uint8_t Slave_addr);


/*I2C interrupt handling*/

void I2C_IRQConfig(uint8_t IRQ_number,uint8_t IRQ_priority,uint8_t EnorDi);
void I2C_IRQHandling(I2C_Handle_t* pI2CHandle);

/*peripheral control*/
void I2C_peripheral_control(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);


/*@I2C_SCL_SPEED*/
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM2K      200000
#define I2C_SCL_SPEED_FM4K      400000

/*@I2C_ACK_CONTROL*/
#define I2C_ACK_ENABLE            1
#define I2C_ACK_DISABLE           0

/*@I2C_FM_DutyCycle*/
#define I2C_FM_DUTY_CYCLE_2       0
#define I2C_FM_DUTY_CYCLE_16_9    1



#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
