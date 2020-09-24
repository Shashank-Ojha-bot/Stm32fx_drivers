/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Jul 12, 2020
 *      Author: Shashank
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include<stdint.h>
#include "stm32f407xx.h"

/*This is the configuration structure of SPI*/

typedef struct
{
	uint8_t SPI_DeviceMode;     /*possible values from @SPI_DeviceModes*/
	uint8_t SPI_BusConfig;      /*possible values from @SPI_BusConfig*/
	uint8_t SPI_DFF;            /*possible values from @DFF*/
	uint8_t SPI_CPHA;           /*possible values from @CPHA*/
	uint8_t SPI_CPOL;           /*possible values from @CPOL*/
	uint8_t SPI_SSM;            /*possible values from @SSM*/
	uint8_t SPI_SclkSpeed;      /*possible values from @SClk_Speed*/
}SPI_PinConfig_t;

/*This is the handle structure of SPI*/

typedef struct
{
	SPI_RegDef_t *pSPIx;   //this holds the base Address of given SPI peripheral
	SPI_PinConfig_t SPI_PinConfig;   //this holds the configuration settings
}SPI_Handle_t;

/**************************APIs for handling the SPI peripheral********************************/

/*For SPI Initialization/Deinitialization*/

void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*For SPI SClock Control*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*Data send and receive*/
/*Data sending and receiving is in Blocking mode i.e.non interrupt mode*/

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *TxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *RxBuffer,uint32_t Len);

/*SPI interrupt handling*/

void SPI_IRQConfig(uint8_t IRQ_number,uint8_t IRQ_priority,uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);

/*peripheral control*/
void SPI_peripheral_control(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

//SSI config
void SPI_SSI_config(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

//SSOE config
void SPI_SSOE_config(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*uint8_t get_FlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return Flag_Set();
	}
	return Flag_Reset();
}*/
/*@SPI_DeviceModes*/

#define SPI_DeviceMode_Master      0
#define SPI_DeviceMode_Slave       1

/*@SPI_BusConfig*/

#define SPI_BusConfig_FD                      0
#define SPI_BusConfig_HD                      1
//#define SPI_BusConfig_Simplex_TXonly          2
#define SPI_BusConfig_Simplex_RXonly          2

/*@DFF*/

#define SPI_DFF_8bits     0
#define SPI_DFF_16bits    1

/*@CPHA*/
#define SPI_CPHA_High  1
#define SPI_CPHA_Low   0

/*@CPOL*/

#define SPI_CPOL_High  1
#define SPI_CPOL_Low  0

/*@SSM*/

#define SPI_SSM_EN    1
#define SPI_SSM_DI    0

/*@SClk_Speed*/

#define SPI_SCLK_Speed_div2    0
#define SPI_SCLK_Speed_div4    1
#define SPI_SCLK_Speed_div8    2
#define SPI_SCLK_Speed_div16    3
#define SPI_SCLK_Speed_div32    4
#define SPI_SCLK_Speed_div64    5
#define SPI_SCLK_Speed_div128    6
#define SPI_SCLK_Speed_div256    7

/*Bits definations for SPI CR1 flag*/
//#define SPI_CR1_

#define SPI_TXE_FLAG       (1<<1)

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
