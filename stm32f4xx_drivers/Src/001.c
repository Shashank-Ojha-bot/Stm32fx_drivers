/*
 * 001.c
 *
 *  Created on: Jul 13, 2020
 *      Author: Shashank
 */
/*GPIO Port=B
 * Alternate functionality=5
 * Pin number 15=MOSI
 * Pin number 14=MISO
 * Pin number 13=Sclk
 * Pin number 12=NSS
 */

#include "stm32f407xx.h"
#include<string.h>
void SPI2_GPIO_Inits()
{
	GPIO_Handle_t SPIpins;
	SPIpins.pGPIOx=GPIOB;
	SPIpins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIpins.GPIO_PinConfig.GPIO_PinOpType=GPIO_OP_TYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIpins);
	//MISO
	SPIpins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPIpins);
	//SCLk
	SPIpins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIpins);
}
void SPI2_Inits()
{
	SPI_Handle_t SPIpins;
	SPIpins.pSPIx=SPI2;
	SPIpins.SPI_PinConfig.SPI_BusConfig=SPI_BusConfig_FD;
	SPIpins.SPI_PinConfig.SPI_CPHA=SPI_CPHA_Low;
	SPIpins.SPI_PinConfig.SPI_CPOL=SPI_CPOL_Low;
	SPIpins.SPI_PinConfig.SPI_DFF=SPI_DFF_8bits;
	SPIpins.SPI_PinConfig.SPI_DeviceMode=SPI_DeviceMode_Master;
	SPIpins.SPI_PinConfig.SPI_SclkSpeed=SPI_SCLK_Speed_div2;
	SPIpins.SPI_PinConfig.SPI_SSM=SPI_SSM_EN;
	SPI_Init(&SPIpins);
}
int main()
{
	char user_data[]="hello world";
	//This function is used to make GPIO pins as SPI pins
	SPI2_GPIO_Inits();

	//This function is used to configure the SPI2 peripheral
	SPI2_Inits();

	//This function will set the ssi bit to avoid the modf error
	SPI_SSI_config(SPI2,ENABLE);

	//Enable the SPI2 peripheral
	SPI_peripheral_control(SPI2,ENABLE);

	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	//Disable the SPI2 peripheral after the data is sent

	SPI_peripheral_control(SPI2,DISABLE);

	while(1);

	return 0;
}
