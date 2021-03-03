/*
 *  uart.c
 *
 *  Created on: Dec 25, 2020
 *      Author: Tuan NA
 */

#include "gpio.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "uart.h"
#include "string.h"
#include "delay.h"
/******************************************************************************/
/*                              DECLARE VARIABLE                              */
/******************************************************************************/
static uint8_t Pin_state = 0;
static uint8_t Pin_previous_state = 0;
char *motorStart = "MOTOR_START\n"; //12
char *motorStop = "MOTOR_STOP\n";   //11
/******************************************************************************/
/*                              FUNCTION                                      */
/******************************************************************************/

void GPIO_CHECK_PIN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(CHECK_GPIO_CLK, ENABLE);
    /* Configure the GPIO_BUTTON pin */
    GPIO_InitStructure.GPIO_Pin = CHECK_GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(CHECK_GPIO_PORT, &GPIO_InitStructure);
}

void GPIO_CHECK_PIN_PROC(void)
{
    //uint8_t Pin_state = 0x00;
    Pin_state = GPIO_ReadInputDataBit(CHECK_GPIO_PORT, CHECK_GPIO_Pin);
    if (Pin_state == 1 && Pin_previous_state == 0)
    {
        UART_SendData(USB2COM_COM, (u8 *)motorStart, strlen(motorStart));
        Delay_ms(500);
    }
    else if (Pin_state == 0 && Pin_previous_state == 1)
    {
        UART_SendData(USB2COM_COM, (u8 *)motorStop, strlen(motorStop));
        Delay_ms(500);
    }
    Pin_previous_state = Pin_state;
}
