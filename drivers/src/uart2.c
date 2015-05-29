/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart2.c - uart2 driver
 */
#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

#include "config.h"
#include "uart2.h"
#include "cfassert.h"
#include "config.h"


static bool isInit = false;

void uart2Init(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART2_GPIO_PERIF, ENABLE);
  ENABLE_UART2_RCC(UART2_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART2_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART2_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UART2_GPIO_PORT, UART2_GPIO_AF_TX_PIN, UART2_GPIO_AF_TX);
  GPIO_PinAFConfig(UART2_GPIO_PORT, UART2_GPIO_AF_RX_PIN, UART2_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = UART2_BAUDRATE;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
  USART_Init(UART2_TYPE, &USART_InitStructure);

  //Enable UART
  USART_Cmd(UART2_TYPE, ENABLE);
  
  isInit = true;
}

bool uart2Test(void)
{
  uart2Printf("Hello UART2!\n");
  return isInit;
}

void uart2SendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
    while (!(UART2_TYPE->SR & USART_FLAG_TXE));
    UART2_TYPE->DR = (data[i] & 0x00FF);
  }
}

int uart2Putchar(int ch)
{
    uart2SendData(1, (uint8_t *)&ch);
    
    return (unsigned char)ch;
}
