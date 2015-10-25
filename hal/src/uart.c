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
 * uart.c - uart CRTP link and raw access functions
 */
#include <string.h>

/*ST includes */
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "uart.h"
#include "crtp.h"
#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"

#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CRTP_START_BYTE 0xAA
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static bool isInit = false;

xSemaphoreHandle waitUntilSendDone = NULL;
static uint8_t outBuffer[64];
static uint8_t dataIndex;
static uint8_t dataSize;
static uint8_t crcIndex = 0;
static bool    isUartDmaInitialized;
static enum { notSentSecondStart, sentSecondStart} txState;
static xQueueHandle packetDelivery;
static xQueueHandle uartDataDelivery;
static DMA_InitTypeDef DMA_InitStructureShare;

void uartRxTask(void *param);

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
void uartDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // USART TX DMA Channel Config
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&UART_TYPE->DR;
  DMA_InitStructureShare.DMA_MemoryBaseAddr = (uint32_t)outBuffer;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructureShare.DMA_BufferSize = 0;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShare.DMA_M2M = DMA_M2M_Disable;

  NVIC_InitStructure.NVIC_IRQChannel = UART_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  isUartDmaInitialized = TRUE;
}

void uartInit(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_APB2PeriphClockCmd(UART_GPIO_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(UART_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
/* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
  USART_InitStructure.USART_BaudRate            = 2000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
#else
  USART_InitStructure.USART_BaudRate            = 115200;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
#endif
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART_TYPE, &USART_InitStructure);

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA)
  uartDmaInit();
#else
  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(UART_TYPE, USART_IT_RXNE, ENABLE);

  vSemaphoreCreateBinary(waitUntilSendDone);

  xTaskCreate(uartRxTask, (const signed char * const)UART_RX_TASK_NAME,
              UART_RX_TASK_STACKSIZE, NULL, UART_RX_TASK_PRI, NULL);

  packetDelivery = xQueueCreate(2, sizeof(CRTPPacket));
  uartDataDelivery = xQueueCreate(1024, sizeof(uint8_t));
#endif
  //Enable it
  USART_Cmd(UART_TYPE, ENABLE);

  isInit = true;
}

bool uartTest(void)
{
  return isInit;
}

void uartRxTask(void *param)
{
  enum {waitForFirstStart, waitForSecondStart,
        waitForPort, waitForSize, waitForData, waitForCRC } rxState;

  uint8_t c;
  uint8_t dataIndex = 0;
  uint8_t crc = 0;
  CRTPPacket p;
  rxState = waitForFirstStart;
  uint8_t counter = 0;
  while(1)
  {
    if (xQueueReceive(uartDataDelivery, &c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
    {
      counter++;
     /* if (counter > 4)
        ledSetRed(1);*/
      switch(rxState)
      {
        case waitForFirstStart:
          rxState = (c == CRTP_START_BYTE) ? waitForSecondStart : waitForFirstStart;
          break;
        case waitForSecondStart:
          rxState = (c == CRTP_START_BYTE) ? waitForPort : waitForFirstStart;
          break;
        case waitForPort:
          p.header = c;
          crc = c;
          rxState = waitForSize;
          break;
        case waitForSize:
          if (c < CRTP_MAX_DATA_SIZE)
          {
            p.size = c;
            crc = (crc + c) % 0xFF;
            dataIndex = 0;
            rxState = (c > 0) ? waitForData : waitForCRC;
          }
          else
          {
            rxState = waitForFirstStart;
          }
          break;
        case waitForData:
          p.data[dataIndex] = c;
          crc = (crc + c) % 0xFF;
          dataIndex++;
          if (dataIndex == p.size)
          {
            rxState = waitForCRC;
          }
          break;
        case waitForCRC:
          if (crc == c)
          {
            xQueueSend(packetDelivery, &p, 0);
          }
          rxState = waitForFirstStart;
          break;
        default:
          ASSERT(0);
          break;
      }
    }
    else
    {
      // Timeout
      rxState = waitForFirstStart;
    }
  }
}

static int uartReceiveCRTPPacket(CRTPPacket *p)
{
  if (xQueueReceive(packetDelivery, p, portMAX_DELAY) == pdTRUE)
  {
    return 0;
  }

  return -1;
}

static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint8_t rxDataInterrupt;

void uartIsr(void)
{
  if (USART_GetITStatus(UART_TYPE, USART_IT_TXE))
  {
    if (dataIndex < dataSize)
    {
      USART_SendData(UART_TYPE, outBuffer[dataIndex] & 0xFF);
      dataIndex++;
      if (dataIndex < dataSize - 1 && dataIndex > 1)
      {
        outBuffer[crcIndex] = (outBuffer[crcIndex] + outBuffer[dataIndex]) % 0xFF;
      }
    }
    else
    {
      USART_ITConfig(UART_TYPE, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  USART_ClearITPendingBit(UART_TYPE, USART_IT_TXE);
  if (USART_GetITStatus(UART_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(UART_TYPE) & 0xFF;
    xQueueSendFromISR(uartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}

static int uartSendCRTPPacket(CRTPPacket *p)
{
  outBuffer[0] = CRTP_START_BYTE;
  outBuffer[1] = CRTP_START_BYTE;
  outBuffer[2] = p->header;
  outBuffer[3] = p->size;
  memcpy(&outBuffer[4], p->data, p->size);
  dataIndex = 1;
  txState = notSentSecondStart;
  dataSize = p->size + 5;
  crcIndex = dataSize - 1;
  outBuffer[crcIndex] = 0;

  USART_SendData(UART_TYPE, outBuffer[0] & 0xFF);
  USART_ITConfig(UART_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);

  return 0;
}

static int uartSetEnable(bool enable)
{
  return 0;
}

static struct crtpLinkOperations uartOp =
{
  .setEnable         = uartSetEnable,
  .sendPacket        = uartSendCRTPPacket,
  .receivePacket     = uartReceiveCRTPPacket,
};

struct crtpLinkOperations * uartGetLink()
{
  return &uartOp;
}

void uartDmaIsr(void)
{
  DMA_ITConfig(UART_DMA_CH, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UART_DMA_IT_TC);
  USART_DMACmd(UART_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UART_DMA_CH, DISABLE);

}

void uartSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  for(i = 0; i < size; i++)
  {
    while (!(UART_TYPE->SR & USART_FLAG_TXE));
    UART_TYPE->DR = (data[i] & 0xFF);
  }
}

int uartPutchar(int ch)
{
    uartSendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}

void uartSendDataDma(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    memcpy(outBuffer, data, size);
    DMA_InitStructureShare.DMA_BufferSize = size;
    // Wait for DMA to be free
    while(UART_DMA_CH->CCR & CCR_ENABLE_SET);
    DMA_Init(UART_DMA_CH, &DMA_InitStructureShare);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART_DMA_CH, DMA_IT_TC, ENABLE);
    USART_DMACmd(UART_TYPE, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(UART_DMA_CH, ENABLE);
  }
}

void __attribute__((used)) USART3_IRQHandler(void)
{
  uartIsr();
}

void __attribute__((used)) DMA1_Channel2_IRQHandler(void)
{
#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA)
  uartDmaIsr();
#endif
}
