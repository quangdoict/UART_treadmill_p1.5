/*
 *  uart.c
 *
 *  Created on: Dec 25, 2020
 *      Author: Truong VV
 */

#include "uart.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/******************************************************************************/
/*                              DECLARE VARIABLE                              */
/******************************************************************************/
char RX_FLAG_END_LINE = 0;
char RXi;
char RXc;
char RX_BUF[RX_BUF_SIZE] = {'\0'};

/******************************************************************************/
/*                              FUNCTION                                      */
/******************************************************************************/

void clear_RXBuffer(void)
{
    for (RXi = 0; RXi < RX_BUF_SIZE; RXi++)
        RX_BUF[RXi] = '\0';
    RXi = 0;
}

void UART_PinInit(USART_TypeDef *USARTx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    if (USARTx == BLUETOOTH_COM)
    {
        /*Config USART1 Rx as input floating */
        GPIO_InitStructure.GPIO_Pin = UART1_RxPin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
        /*Config USART1 Tx as alternate function pp*/
        GPIO_InitStructure.GPIO_Pin = UART1_TxPin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
    }
    else if (USARTx == USB2COM_COM)
    {
        /*Config USART2 Rx as input floating */
        GPIO_InitStructure.GPIO_Pin = UART3_RxPin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
        /*Config USART2 Tx as alternate function pp*/
        GPIO_InitStructure.GPIO_Pin = UART3_TxPin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
    }
}

void UART_ClockInit(USART_TypeDef *USARTx)
{
    if (USARTx == BLUETOOTH_COM)
    {
        /*Enable GPIO clock*/
        RCC_APB2PeriphClockCmd(UART1_GPIO_CLK, ENABLE);
        /*Enable AFIO clock*/
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        /*Enable UART clock*/
        RCC_APB2PeriphClockCmd(UART1_CLK, ENABLE);
    }
    else if (USARTx == USB2COM_COM)
    {
        /*Enable GPIO clock*/
        RCC_APB2PeriphClockCmd(UART3_GPIO_CLK, ENABLE);
        /*Enable AFIO clock*/
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        /*Enable UART clock*/
        RCC_APB1PeriphClockCmd(UART3_CLK, ENABLE);
    }
}

void UART_Init(USART_TypeDef *USARTx, u32 baudrate, u16 mode)
{
    USART_InitTypeDef USART_InitStructure;
    UART_ClockInit(USARTx);
    UART_PinInit(USARTx);

    /*UARTx Config*/
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = mode;

    /*USART Config Init*/
    USART_Init(USARTx, &USART_InitStructure);

    /* Enable USARTx Receive interrupts */
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

    /*Enable UARTx*/
    USART_Cmd(USARTx, ENABLE);
}

void USARTSend(USART_TypeDef *USARTx, char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USARTx, *pucBuffer++);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
        {
        }
    }
}

void UART_SendData(USART_TypeDef *USARTx, u8 *buff, u8 len)
{
    for (u8 i = 0; i < len; i++)
    {
        USART_SendData(USARTx, buff[i]);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    }
}

u8 UART_GetData(USART_TypeDef *USARTx)
{
    u8 datarx;
    while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
        ;
    datarx = (u8)USART_ReceiveData(USARTx);
    return datarx;
}

void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
        RXc = USART_ReceiveData(BLUETOOTH_COM);
        USART_SendData(USB2COM_COM,RXc);
    }
}
void USART3_IRQHandler(void)
{
    if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
        RXc = USART_ReceiveData(USB2COM_COM);
        USART_SendData(BLUETOOTH_COM,RXc);
    }
}

void UART_Init_All(void)
{
    UART_Init(BLUETOOTH_COM, BAUDRATE, USART_Mode_Tx | USART_Mode_Rx);
    UART_Init(USB2COM_COM, BAUDRATE, USART_Mode_Tx | USART_Mode_Rx);
    UART_SendData(BLUETOOTH_COM,"BT+OK\r\n",7);
    UART_SendData(USB2COM_COM, "USB2COM+OK\n",11);
}