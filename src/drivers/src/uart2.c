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

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "debug.h"

#include "autoconf.h"
#include "config.h"
#include "nvic.h"
#include "uart2.h"
#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"
#include "static_mem.h"

static xSemaphoreHandle uartBusy;
static StaticSemaphore_t uartBusyBuffer;
static xSemaphoreHandle waitUntilSendDone;
static StaticSemaphore_t waitUntilSendDoneBuffer;

static bool isInit = false;

static DMA_InitTypeDef DMA_InitStructureShare;
static uint8_t dmaBuffer[UART2_DMA_BUFFER_SIZE];
static bool    isUartDmaInitialized;
static uint32_t initialDMACount;

static StreamBufferHandle_t rxStream;
static EventGroupHandle_t isrEvents;

static bool hasOverrun = false;

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
static void uart2DmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // USART TX DMA Channel Config
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&UART2_TYPE->DR;
  DMA_InitStructureShare.DMA_Memory0BaseAddr = (uint32_t)dmaBuffer;
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShare.DMA_BufferSize = 0;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShare.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructureShare.DMA_Channel = UART2_DMA_CH;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_Low;

  NVIC_InitStructure.NVIC_IRQChannel = UART2_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART2_DMA_PRI;
  NVIC_Init(&NVIC_InitStructure);

  isUartDmaInitialized = true;
}

void uart2Init(const uint32_t baudrate)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // initialize the FreeRTOS structures first, to prevent null pointers in interrupts
  waitUntilSendDone = xSemaphoreCreateBinaryStatic(&waitUntilSendDoneBuffer); // initialized as blocking
  uartBusy = xSemaphoreCreateBinaryStatic(&uartBusyBuffer); // initialized as blocking
  xSemaphoreGive(uartBusy); // but we give it because the uart isn't busy at initialization

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

  USART_InitStructure.USART_BaudRate            = baudrate;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART2_TYPE, &USART_InitStructure);

  uart2DmaInit();

  // Configure Rx buffer not empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART2_PRI;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(UART2_TYPE, USART_IT_RXNE, ENABLE);

  //Enable UART
  USART_Cmd(UART2_TYPE, ENABLE);

  USART_ITConfig(UART2_TYPE, USART_IT_RXNE, ENABLE);

  isrEvents = xEventGroupCreate();

  rxStream = xStreamBufferCreate( 200, 1);
  ASSERT(rxStream);

  isInit = true;
}

bool uart2Test(void)
{
  return isInit;
}

static const int TX_DONE = (1<<0);

static size_t txSize = 0;
static size_t txIdx = 0;
static uint8_t * txBuffer = 0;

void uart2SendData(uint32_t size, uint8_t* data)
{
  if (!isInit)
    return;

  txIdx = 0;
  txSize = size;
  txBuffer = data;

  USART_ITConfig(UART2_TYPE, USART_IT_TXE, ENABLE);

  xEventGroupWaitBits(isrEvents,
                      TX_DONE,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

}

void uart2SendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    xSemaphoreTake(uartBusy, portMAX_DELAY);
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UART2_DMA_STREAM) != DISABLE);
    //Copy data in DMA buffer
    memcpy(dmaBuffer, data, size);
    DMA_InitStructureShare.DMA_BufferSize = size;
    initialDMACount = size;
    // Init new DMA stream
    DMA_Init(UART2_DMA_STREAM, &DMA_InitStructureShare);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART2_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Enable USART DMA TX Requests */
    USART_DMACmd(UART2_TYPE, USART_DMAReq_Tx, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UART2_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UART2_DMA_STREAM, ENABLE);
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    xSemaphoreGive(uartBusy);
  }
}

int uart2Putchar(int ch)
{
  uart2SendData(1, (uint8_t *)&ch);

  return (unsigned char)ch;
}

bool uart2GetCharWithTimeout(uint8_t *c, const uint32_t timeoutTicks)
{
  if (uart2GetDataWithTimeout(1, c, timeoutTicks) > 0)
  {
    return true;
  }

  *c = 0;
  return false;
}

bool uart2GetCharWithDefaultTimeout(uint8_t *c)
{
  return uart2GetCharWithTimeout(c, UART2_DATA_TIMEOUT_TICKS);
}

void uart2Getchar(char * ch)
{
  uart2GetData(1, (uint8_t*) ch);
}

int uart2GetDataWithTimeout(size_t size, uint8_t * buffer, const uint32_t timeoutTicks) {
  size_t sizeLeft = size;
  uint32_t timeoutEnd = xTaskGetTickCount() + timeoutTicks;
  while (sizeLeft > 0 && timeoutEnd > xTaskGetTickCount()) {
    xStreamBufferSetTriggerLevel(rxStream, sizeLeft);
    uint32_t ticksToWait = timeoutEnd - xTaskGetTickCount();
    sizeLeft -= xStreamBufferReceive(rxStream, &buffer[size-sizeLeft], sizeLeft, ticksToWait);
  }

  return size - sizeLeft;
}

int uart2GetData(size_t size, uint8_t * buffer) {
  size_t sizeLeft = size;
  while (sizeLeft > 0) {
    xStreamBufferSetTriggerLevel(rxStream, sizeLeft);
    sizeLeft -= xStreamBufferReceive(rxStream, &buffer[size-sizeLeft], sizeLeft, portMAX_DELAY);
  }

  return size;
}

bool uart2DidOverrun()
{
  bool result = hasOverrun;
  hasOverrun = false;

  return result;
}

#ifndef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
void __attribute__((used)) DMA1_Stream6_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UART2_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UART2_DMA_STREAM, UART2_DMA_FLAG_TCIF);
  USART_DMACmd(UART2_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UART2_DMA_STREAM, DISABLE);

  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

void __attribute__((used)) USART2_IRQHandler(void)
{

  uint32_t status = UART2_TYPE->SR;
  if ((UART2_TYPE->SR & USART_FLAG_RXNE) != 0)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t rxData = USART_ReceiveData(UART2_TYPE) & 0x00FF;
    xStreamBufferSendFromISR(rxStream, &rxData, 1, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }

  if ((UART2_TYPE->SR & USART_FLAG_TXE) != 0)
  {
    if (txBuffer != 0) {
      uint8_t byteToWrite = txBuffer[txIdx++];
      UART2_TYPE->DR = (byteToWrite & 0x00FF);

      if (txSize == txIdx) {
        USART_ITConfig(UART2_TYPE, USART_IT_TXE, DISABLE);

        txBuffer = 0;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(isrEvents, TX_DONE, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
      }
    }
  }

  if ((status & USART_FLAG_ORE) != 0 ||
      (status & USART_FLAG_NE) != 0 ||
      (status & USART_FLAG_FE) != 0 ||
      (status & USART_FLAG_PE) != 0) {
    /** if we get here, the error is most likely caused by an overrun!
     * - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun error)
     * - and IDLE (Idle line detected) pending bits are cleared by software sequence:
     * - reading USART_SR register followed reading the USART_DR register.
     */
    asm volatile ("" : "=m" (UART2_TYPE->SR) : "r" (UART2_TYPE->SR)); // force non-optimizable reads
    asm volatile ("" : "=m" (UART2_TYPE->DR) : "r" (UART2_TYPE->DR)); // of these two registers

    hasOverrun = true;
  }
}
