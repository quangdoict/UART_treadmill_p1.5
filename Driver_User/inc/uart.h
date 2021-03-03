/*
 *  uart.h
 *
 *  Created on: Dec 25, 2020
 *      Author: TuanNA
 */

#ifndef _UART_H_
#define _UART_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "stddef.h"

/******************************************************************************/
/*                              DEFINE                                        */
/******************************************************************************/
#define BLUETOOTH_COM        USART1
#define USB2COM_COM          USART3
#define BAUDRATE             115200
#define RX_BUF_SIZE          100

#define UART1_GPIO           GPIOA
#define UART1_RxPin          GPIO_Pin_10
#define UART1_TxPin          GPIO_Pin_9
#define UART1_GPIO_CLK       RCC_APB2Periph_GPIOA
#define UART1_CLK            RCC_APB2Periph_USART1

#define UART3_GPIO           GPIOB
#define UART3_RxPin          GPIO_Pin_11
#define UART3_TxPin          GPIO_Pin_10
#define UART3_GPIO_CLK       RCC_APB2Periph_GPIOB
#define UART3_CLK            RCC_APB1Periph_USART3

/******************************************************************************/
/*                              FUNCTION                                      */
/******************************************************************************/

void UART_PinInit(USART_TypeDef* USARTx);

void UART_ClockInit(USART_TypeDef* USARTx);

void UART_Init(USART_TypeDef* USARTx, u32 baudrate, u16 mode);

void UART_SendData(USART_TypeDef* USARTx, u8* buff, u8 len);

void USARTSend(USART_TypeDef* USARTx, char *pucBuffer);

u8 UART_GetData(USART_TypeDef* USARTx);

void UART_Init_All(void);

#endif /*_UART_H_*/