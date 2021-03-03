/*
 *  uart.h
 *
 *  Created on: Dec 25, 2020
 *      Author: TuanNA
 */

#ifndef _GPIO_H_
#define _GPIO_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "stddef.h"

/******************************************************************************/
/*                              DEFINE                                        */
/******************************************************************************/

#define CHECK_GPIO_PORT      GPIOC
#define CHECK_GPIO_Pin       GPIO_Pin_13
#define CHECK_GPIO_CLK       RCC_APB2Periph_GPIOC

/******************************************************************************/
/*                              FUNCTION                                      */
/******************************************************************************/

void GPIO_CHECK_PIN_Init(void);

void GPIO_CHECK_PIN_PROC(void);


#endif /*_GPIO_H_*/