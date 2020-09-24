/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Jul 12, 2020
 *      Author: Shashank
 */

#include<stdint.h>
#include "stm32f4xx_spi_driver.h"


/*For SPI Initialization/Deinitialization*/

void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	/*Enabling the peripheral clock*/
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t tempreg=0;
/*SPI Device mode*/

	tempreg|=pSPIHandle->SPI_PinConfig.SPI_DeviceMode<<2;

/*SPI BusConfig*/

	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig==SPI_BusConfig_FD)
	{
		//bidi mode must be cleared
		tempreg&=~(1<<15);
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig==SPI_BusConfig_HD)
	{
		//bidi mode must be set
		tempreg|=(1<<15);
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig==SPI_BusConfig_Simplex_RXonly)
	{
		//bidi mode must be cleared
		tempreg&=~(1<<15);
		//rx only bit must be set
		tempreg|=(1<<10);
	}

/*SPI Data frame format*/
	tempreg|=pSPIHandle->SPI_PinConfig.SPI_DFF<<11;
/*SPI CPHA*/
	tempreg|=pSPIHandle->SPI_PinConfig.SPI_CPHA<<0;
/*SPI CPOL*/
	tempreg|=pSPIHandle->SPI_PinConfig.SPI_CPOL<<1;
/*SPI SSM*/
	tempreg|=pSPIHandle->SPI_PinConfig.SPI_SSM<<9;
/*SPI Sclk Speed*/
	tempreg|=pSPIHandle->SPI_PinConfig.SPI_SclkSpeed<<3;

	pSPIHandle->pSPIx->CR1=tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI2)
	{
		SPI3_REG_RESET();
	}
}

/*For SPI SClock Control*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
if(EnorDi==ENABLE)
{
	if(pSPIx==SPI1)
	{
		SPI1_PCLK_EN;
	}
	else if(pSPIx==SPI2)
	{
		SPI2_PCLK_EN;
	}
	else if(pSPIx==SPI2)
	{
		SPI3_PCLK_EN;
	}
}
else
{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_DI;
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN;
		}
		else if(pSPIx==SPI2)
		{
			SPI3_PCLK_DI;
		}
}
}

/*Data send and receive*/

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *TxBuffer,uint32_t Len)
{
while(Len)
{
	while(!(pSPIx->SR & (1<<1)));     //wait until TXE is 1 i.e empty.
	//while(get_FlagStatus(pSPIx,SPI_TXE_FLAG)==Flag_Reset());
	if(pSPIx->CR1 & (1<<11))      //DFF bit of CR1 register
	{
		//16 bit dff
		pSPIx->DR=*((uint16_t*)TxBuffer);
		Len--;
		Len--;
		(uint16_t*)TxBuffer++;
	}
	else
	{
		//8 bit dff
		pSPIx->DR=*(TxBuffer);
		Len--;
		TxBuffer++;
	}

}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *RxBuffer,uint32_t Len)
{//uint16_t data;
	while(Len)
	{
		while(!(pSPIx->SR & (1<<0)));     //wait until RXNE is 1 i.e not empty.
		if(pSPIx->CR1 & (1<<11))
		{
			//16 bit dff
			*(uint16_t*)RxBuffer=pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)RxBuffer++;
		}
		else
		{  //8 bit dff
		    *(RxBuffer)=pSPIx->DR;
			Len--;
			RxBuffer++;
		}
	}}

void SPI_peripheral_control(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1 |=(1<<6);
	}
	else
	{
		pSPIx->CR1 &=~(1<<6);
	}
}

void SPI_SSI_config(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			pSPIx->CR1 |=(1<<8);
		}
		else
		{
			pSPIx->CR1 &=~(1<<8);
		}
}

void SPI_SSOE_config(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			pSPIx->CR2 |=(1<<2);
		}
		else
		{
			pSPIx->CR2 &=~(1<<2);
		}
}
