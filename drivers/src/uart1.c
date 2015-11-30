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
 * uart1.c - uart1 driver
 */
#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

#include "config.h"
#include "nvic.h"
#include "uart1.h"
#include "cfassert.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static bool isInit = false;

static xQueueHandle uartDataDelivery;

void uart1Init(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART1_GPIO_PERIF, ENABLE);
  ENABLE_UART1_RCC(UART1_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART1_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART1_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UART1_GPIO_PORT, UART1_GPIO_AF_TX_PIN, UART1_GPIO_AF_TX);
  GPIO_PinAFConfig(UART1_GPIO_PORT, UART1_GPIO_AF_RX_PIN, UART1_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = UART1_BAUDRATE;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART1_TYPE, &USART_InitStructure);

  uartDataDelivery = xQueueCreate(40, sizeof(uint8_t));

  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Enable UART
  USART_Cmd(UART1_TYPE, ENABLE);
  
  USART_ITConfig(UART1_TYPE, USART_IT_RXNE, ENABLE);

  isInit = true;
}

bool uart1Test(void)
{
  return isInit;
}

void uart1SendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
    while (!(UART1_TYPE->SR & USART_FLAG_TXE));
    UART1_TYPE->DR = (data[i] & 0x00FF);
  }
}

int uart1Putchar(int ch)
{
    uart1SendData(1, (uint8_t *)&ch);
    
    return (unsigned char)ch;
}

void uart1Getchar(char * ch) {
  xQueueReceive(uartDataDelivery, ch, portMAX_DELAY);
}

static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint16_t rxDataInterrupt;

void __attribute__((used)) USART3_IRQHandler(void)
{
  if (USART_GetITStatus(UART1_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(UART1_TYPE) & 0xFF;
    xQueueSendFromISR(uartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}
