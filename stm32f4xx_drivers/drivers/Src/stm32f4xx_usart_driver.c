/*
 * stm32f4xx_usart_driver.c
 *
 *  Created on: Jul 25, 2020
 *      Author: Shashank
 */
#include<stdint.h>
#include "stm32f4xx_usart_driver.h"

void USART_PeriClkControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pUSARTx==USART1)
		{
			USART1_PCLK_EN;
		}
		if(pUSARTx==USART6)
		{
			USART6_PCLK_EN;
		}
        if(pUSARTx==USART3)
		{
			USART3_PCLK_EN;
		}
		if(pUSARTx==UART4)
		{
			UART4_PCLK_EN;
		}
	}
	else
	{
		if(pUSARTx==USART1)
		{
			USART1_PCLK_DI;
		}
		if(pUSARTx==USART6)
		{
			USART6_PCLK_DI;
		}
		if(pUSARTx==USART3)
		{
			USART3_PCLK_DI;
		}
		if(pUSARTx==UART4)
		{
			UART4_PCLK_DI;
		}
	}
}


void USART_PeriControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi)
{
	if(EnorDi)
	{
		pUSARTx->CR1 |= (1<<13);
	}
	else
	{
		pUSARTx->CR1 &= ~(1<<13);
	}
}


/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClkControl(pUSARTHandle->pUSARTx,ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_PinConfig.USART_Mode == USART_Mode_Rx_only)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << 2);
	}else if (pUSARTHandle->USART_PinConfig.USART_Mode == USART_Mode_Tx_only)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << 3 );

	}else if (pUSARTHandle->USART_PinConfig.USART_Mode == USART_Mode_Tx_Rx)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << 2) | ( 1 << 3) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_PinConfig.USART_WordLen << 12 ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_PinConfig.USART_ParControl == USART_ParControl_Enable_Even)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << 10);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_PinConfig.USART_ParControl == USART_ParControl_Enable_Odd)
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << 10);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << 9);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_PinConfig.USART_NoOfStopBits <<12;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_PinConfig.USART_HwFlowControl == USART_HwFlowControl_Enable_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << 9);


	}else if (pUSARTHandle->USART_PinConfig.USART_HwFlowControl ==USART_HwFlowControl_Enable_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << 8);

	}else if (pUSARTHandle->USART_PinConfig.USART_HwFlowControl == USART_HwFlowControl_Enable_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (( 1 << 9) | (1 << 8));
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here

}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx==USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx==USART6)
	{
		USART6_REG_RESET();
	}
	else if(pUSARTx==UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx==USART3)
	{
		USART3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(!( pUSARTHandle->pUSARTx->SR & (1<<7)));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_PinConfig.USART_WordLen ==USART_WordLen_9bits )
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_PinConfig.USART_ParControl == USART_ParControl_Disable)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! (pUSARTHandle->pUSARTx->SR & (1 << 6)));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!(pUSARTHandle->pUSARTx->SR  & (1 << 5)));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_PinConfig.USART_WordLen == USART_WordLen_9bits)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_PinConfig.USART_ParControl == USART_ParControl_Disable)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_PinConfig.USART_ParControl == USART_ParControl_Disable)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}
