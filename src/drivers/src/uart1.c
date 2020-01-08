/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2019 Bitcraze AB
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

#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/*ST includes */
#include "stm32fxxx.h"

#include "config.h"
#include "nvic.h"
#include "uart1.h"
#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"
#include "static_mem.h"

/** This uart is conflicting with SPI2 DMA used in sensors_bmi088_spi_bmp388.c
 *  which is used in CF-Bolt. So for other products this can be enabled.
 */
//#define ENABLE_UART1_DMA

static xQueueHandle uart1queue;
STATIC_MEM_QUEUE_ALLOC(uart1queue, 64, sizeof(uint8_t));

static bool isInit = false;
static bool hasOverrun = false;

#ifdef ENABLE_UART1_DMA
static xSemaphoreHandle uartBusy;
static StaticSemaphore_t uartBusyBuffer;
static xSemaphoreHandle waitUntilSendDone;
static StaticSemaphore_t waitUntilSendDoneBuffer;
static DMA_InitTypeDef DMA_InitStructureShare;
static uint8_t dmaBuffer[64];
static bool    isUartDmaInitialized;
static uint32_t initialDMACount;
#endif

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
static void uart1DmaInit(void)
{
#ifdef ENABLE_UART1_DMA
  NVIC_InitTypeDef NVIC_InitStructure;

  // initialize the FreeRTOS structures first, to prevent null pointers in interrupts
  waitUntilSendDone = xSemaphoreCreateBinaryStatic(&waitUntilSendDoneBuffer); // initialized as blocking
  uartBusy = xSemaphoreCreateBinaryStatic(&uartBusyBuffer); // initialized as blocking
  xSemaphoreGive(uartBusy); // but we give it because the uart isn't busy at initialization

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // USART TX DMA Channel Config
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&UART1_TYPE->DR;
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
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructureShare.DMA_Channel = UART1_DMA_CH;

  NVIC_InitStructure.NVIC_IRQChannel = UART1_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MID_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  isUartDmaInitialized = true;
#endif
}

void uart1Init(const uint32_t baudrate) {
  uart1InitWithParity(baudrate, uart1ParityNone);
}

void uart1InitWithParity(const uint32_t baudrate, const uart1Parity_t parity)
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

  USART_InitStructure.USART_BaudRate            = baudrate;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  if (parity == uart1ParityEven || parity == uart1ParityOdd) {
    USART_InitStructure.USART_WordLength        = USART_WordLength_9b;
  } else {
    USART_InitStructure.USART_WordLength        = USART_WordLength_8b;
  }

  USART_InitStructure.USART_StopBits            = USART_StopBits_1;

  if (parity == uart1ParityEven) {
    USART_InitStructure.USART_Parity            = USART_Parity_Even;
  } else if (parity == uart1ParityOdd) {
    USART_InitStructure.USART_Parity            = USART_Parity_Odd;
  } else {
    USART_InitStructure.USART_Parity            = USART_Parity_No;
  }

  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART1_TYPE, &USART_InitStructure);

  uart1DmaInit();

  NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MID_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  uart1queue = STATIC_MEM_QUEUE_CREATE(uart1queue);

  USART_ITConfig(UART1_TYPE, USART_IT_RXNE, ENABLE);

  //Enable UART
  USART_Cmd(UART1_TYPE, ENABLE);

  USART_ITConfig(UART1_TYPE, USART_IT_RXNE, ENABLE);

  isInit = true;
}

bool uart1Test(void)
{
  return isInit;
}

bool uart1GetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(uart1queue, c, UART1_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }

  *c = 0;
  return false;
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

#ifdef ENABLE_UART1_DMA
void uart1SendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    xSemaphoreTake(uartBusy, portMAX_DELAY);
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UART1_DMA_STREAM) != DISABLE);
    //Copy data in DMA buffer
    memcpy(dmaBuffer, data, size);
    DMA_InitStructureShare.DMA_BufferSize = size;
    initialDMACount = size;
    // Init new DMA stream
    DMA_Init(UART1_DMA_STREAM, &DMA_InitStructureShare);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART1_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Enable USART DMA TX Requests */
    USART_DMACmd(UART1_TYPE, USART_DMAReq_Tx, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UART1_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UART1_DMA_STREAM, ENABLE);
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    xSemaphoreGive(uartBusy);
  }
}
#endif

int uart1Putchar(int ch)
{
    uart1SendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}

void uart1Getchar(char * ch)
{
  xQueueReceive(uart1queue, ch, portMAX_DELAY);
}

bool uart1DidOverrun()
{
  bool result = hasOverrun;
  hasOverrun = false;

  return result;
}

#ifdef ENABLE_UART1_DMA
void __attribute__((used)) DMA1_Stream3_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UART1_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UART1_DMA_STREAM, UART1_DMA_FLAG_TCIF);
  USART_DMACmd(UART1_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UART1_DMA_STREAM, DISABLE);

  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}
#endif

void __attribute__((used)) USART3_IRQHandler(void)
{
  uint8_t rxData;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (USART_GetITStatus(UART1_TYPE, USART_IT_RXNE))
  {
    rxData = USART_ReceiveData(UART1_TYPE) & 0x00FF;
    xQueueSendFromISR(uart1queue, &rxData, &xHigherPriorityTaskWoken);
  } else {
    /** if we get here, the error is most likely caused by an overrun!
     * - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun error)
     * - and IDLE (Idle line detected) pending bits are cleared by software sequence:
     * - reading USART_SR register followed reading the USART_DR register.
     */
    asm volatile ("" : "=m" (UART1_TYPE->SR) : "r" (UART1_TYPE->SR)); // force non-optimizable reads
    asm volatile ("" : "=m" (UART1_TYPE->DR) : "r" (UART1_TYPE->DR)); // of these two registers

    hasOverrun = true;
  }
}
