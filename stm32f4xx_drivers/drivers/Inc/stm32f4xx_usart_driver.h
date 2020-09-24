/*
 * stm32f4xx_usart_driver.h
 *
 *  Created on: Jul 25, 2020
 *      Author: Shashank
 */

#ifndef INC_STM32F4XX_USART_DRIVER_H_
#define INC_STM32F4XX_USART_DRIVER_H_

#include<stdint.h>
#include "stm32f407xx.h"

/*This is the configuration structure of USART Peripheral*/

typedef struct
{
	uint32_t USART_Mode;          /*possible values from @USART_Mode*/
	uint32_t USART_Baud;           /*possible values from @USART_Baud*/
	uint32_t USART_NoOfStopBits;    /*possible values from @USART_NoOfStopBits*/
	uint32_t USART_WordLen;         /*possible values from @USART_WordLen*/
	uint32_t USART_ParControl;       /*possible values from @USART_ParControl*/
	uint32_t USART_HwFlowControl;     /*possible values from @USART_HwFlowControl*/
}USART_PinConfig_t;

/*This is the handle structure of USART peripheral*/

typedef struct
{
    USART_RegDef_t *pUSARTx;      //base address of the USART peripheral
    USART_PinConfig_t USART_PinConfig;  //configurations of the USART peripheral(assigning values to different fields of Configuration register
}USART_Handle_t;

                  /*****************************************************************
                       APIs for various configuration on USARTx peripheral
 	 	 	 	  ********************************************************************/
/*peripheral clock control*/

void USART_PeriClkControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi);

/*Peripheral Control API*/

void USART_PeriControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi);

/*Initialization/DeInitialization*/

void USART_Init(USART_Handle_t *pUSART);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*USART send/receive data API*/

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*USART interrupt handling*/

void USART_IRQConfig(uint8_t IRQ_number,uint8_t IRQ_priority,uint8_t EnorDi);
void USART_IRQHandling(USART_Handle_t* pUSARTHandle);

/*possible values of configuration register */
/*@USART_Mode*/

#define USART_Mode_Tx_only      0
#define USART_Mode_Rx_only      1
#define USART_Mode_Tx_Rx        2

/*@USART_Baud*/

#define USART_Baud_1200        1200
#define USART_Baud_2400        2400
#define USART_Baud_9600        9600
#define USART_Baud_19200       19200
#define USART_Baud_38400       38400
#define USART_Baud_57600       57600

/*@USART_NoOfStopBits*/

#define USART_NoOfStopBits_0_5    0
#define USART_NoOfStopBits_1      1
#define USART_NoOfStopBits_2_5    2
#define USART_NoOfStopBits_2      3

/*@USART_WordLen*/

#define USART_WordLen_8bits       0
#define USART_WordLen_9bits       1

/*@USART_ParControl*/

#define USART_ParControl_Enable_Even    0
#define USART_ParControl_Enable_Odd     1
#define USART_ParControl_Disable        2

/*@USART_HwFlowControl*/

#define USART_HwFlowControl_None 			0
#define USART_HwFlowControl_Enable_CTS      1
#define USART_HwFlowControl_Enable_RTS      2
#define USART_HwFlowControl_Enable_CTS_RTS  3

#endif /* INC_STM32F4XX_USART_DRIVER_H_ */
