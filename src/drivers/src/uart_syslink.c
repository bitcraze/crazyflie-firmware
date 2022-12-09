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
 * uart_syslink.c - Uart syslink to nRF51 and raw access functions
 */
#include <stdint.h>
#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "autoconf.h"
#include "uart_syslink.h"
#include "crtp.h"
#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"
#include "queuemonitor.h"
#include "static_mem.h"

#define DEBUG_MODULE "U-SLK"
#include "debug.h"

#define UARTSLK_DATA_TIMEOUT_MS 1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

#define UARTSLK_CLKSUM_SIZE   2

static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone;
static StaticSemaphore_t waitUntilSendDoneBuffer;
static xSemaphoreHandle uartBusy;
static StaticSemaphore_t uartBusyBuffer;
static xQueueHandle syslinkPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(syslinkPacketDelivery, 8, sizeof(SyslinkPacket));
static bool syslinkPacketDeliveryReadyToReceive = false;

#ifdef CONFIG_SYSLINK_RX_DMA
static uint8_t dmaRXBuffer[64];
static DMA_InitTypeDef DMA_InitStructureShareRX;
#endif
static uint8_t dmaTXBuffer[64];
static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;
static bool    isUartDmaInitialized;
static DMA_InitTypeDef DMA_InitStructureShareTX;
static uint16_t initialDMACount;
static uint16_t remainingDMACount;
static bool     dmaIsPaused;

static volatile SyslinkPacket slp = {0};
static volatile SyslinkRxState rxState = waitForFirstStart;
static volatile uint8_t dataIndex = 0;
static volatile uint8_t cksum[UARTSLK_CLKSUM_SIZE] = {0};
static void uartslkHandleDataFromISR(uint8_t c, BaseType_t * const pxHigherPriorityTaskWoken);

static void uartslkPauseDma();
static void uartslkResumeDma();

// Debug probe
static uint32_t dmaPausedCounter;
static uint32_t dmaTxStreamPausedCounter;
static uint32_t dmaResumedCounter;
static uint32_t dmaTxStreamResumedCounter;
static bool dmaNrfFlowControlBufferFull;
static uint32_t dmaSendWhileNrfBufferFull;

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
void uartslkDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // USART TX DMA Channel Config
  DMA_InitStructureShareTX.DMA_PeripheralBaseAddr = (uint32_t)&UARTSLK_TYPE->DR;
  DMA_InitStructureShareTX.DMA_Memory0BaseAddr = (uint32_t)dmaTXBuffer;
  DMA_InitStructureShareTX.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShareTX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShareTX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShareTX.DMA_BufferSize = 0;
  DMA_InitStructureShareTX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShareTX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShareTX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructureShareTX.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructureShareTX.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShareTX.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShareTX.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShareTX.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructureShareTX.DMA_Channel = UARTSLK_DMA_TX_CH;

  NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_DMA_TX_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SYSLINK_DMA_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#ifdef CONFIG_SYSLINK_RX_DMA
  // USART TX DMA Channel Config
  DMA_InitStructureShareRX.DMA_PeripheralBaseAddr = (uint32_t)&UARTSLK_TYPE->DR;
  DMA_InitStructureShareRX.DMA_Memory0BaseAddr = (uint32_t)dmaRXBuffer;
  DMA_InitStructureShareRX.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShareRX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShareRX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShareRX.DMA_BufferSize = 0;
  DMA_InitStructureShareRX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShareRX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShareRX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructureShareRX.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructureShareRX.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShareRX.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShareRX.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShareRX.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructureShareRX.DMA_Channel = UARTSLK_DMA_RX_CH;
  DMA_Init(UARTSLK_DMA_RX_STREAM, &DMA_InitStructureShareRX);

  NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_DMA_RX_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SYSLINK_DMA_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif

  isUartDmaInitialized = true;
}

void uartslkInit(void)
{
  // initialize the FreeRTOS structures first, to prevent null pointers in interrupts
  waitUntilSendDone = xSemaphoreCreateBinaryStatic(&waitUntilSendDoneBuffer); // initialized as blocking
  uartBusy = xSemaphoreCreateBinaryStatic(&uartBusyBuffer); // initialized as blocking
  xSemaphoreGive(uartBusy); // but we give it because the uart isn't busy at initialization

  syslinkPacketDelivery = STATIC_MEM_QUEUE_CREATE(syslinkPacketDelivery);
  DEBUG_QUEUE_MONITOR_REGISTER(syslinkPacketDelivery);

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef extiInit;

  // Enable GPIO and USART clock
  RCC_AHB1PeriphClockCmd(UARTSLK_GPIO_PERIF, ENABLE);
  ENABLE_UARTSLK_RCC(UARTSLK_PERIF, ENABLE);

  // Configure USART Rx as input floating
  GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

  // Configure USART Tx as alternate function
  GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

  // Map uartslk to alternate functions
  GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_TX_PIN, UARTSLK_GPIO_AF_TX);
  GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_RX_PIN, UARTSLK_GPIO_AF_RX);

#if defined(UARTSLK_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
  USART_InitStructure.USART_BaudRate            = 2000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
#else
  USART_InitStructure.USART_BaudRate            = 1000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
#endif
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UARTSLK_TYPE, &USART_InitStructure);

  uartslkDmaInit();

  // Configure Rx buffer not empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SYSLINK_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, ENABLE);

  // Setting up TXEN pin (NRF flow control)
  RCC_AHB1PeriphClockCmd(UARTSLK_TXEN_PERIF, ENABLE);

  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = UARTSLK_TXEN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UARTSLK_TXEN_PORT, &GPIO_InitStructure);

  extiInit.EXTI_Line = UARTSLK_TXEN_EXTI;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInit);
  EXTI_ClearITPendingBit(UARTSLK_TXEN_EXTI);

  NVIC_EnableIRQ(EXTI4_IRQn);

  //Enable UART
  USART_Cmd(UARTSLK_TYPE, ENABLE);
  isInit = true;
}

bool uartslkTest(void)
{
  return isInit;
}

void uartslkPauseRx(void)
{
  NVIC_DisableIRQ(UARTSLK_IRQ);
}

void uartslkResumeRx(void)
{
  rxState = waitForFirstStart;
  NVIC_EnableIRQ(UARTSLK_IRQ);
}

void uartslkEnableIncoming()
{
  syslinkPacketDeliveryReadyToReceive = true;
}

void uartslkGetPacketBlocking(SyslinkPacket* packet)
{
  xQueueReceive(syslinkPacketDelivery, packet, portMAX_DELAY);
}

void uartslkSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UARTSLK_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET);
#endif
    while (!(UARTSLK_TYPE->SR & USART_FLAG_TXE));
    UARTSLK_TYPE->DR = (data[i] & 0x00FF);
  }
}

void uartslkSendDataIsrBlocking(uint32_t size, uint8_t* data)
{
  xSemaphoreTake(uartBusy, portMAX_DELAY);
  outDataIsr = data;
  dataSizeIsr = (uint8_t)size;
  dataIndexIsr = 1;
  uartslkSendData(1, &data[0]);
  USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
  xSemaphoreGive(uartBusy);
}

int uartslkPutchar(int ch)
{
    uartslkSendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}

void uartslkSendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (dmaNrfFlowControlBufferFull) {
    dmaSendWhileNrfBufferFull++;
  }

  if (isUartDmaInitialized)
  {
    xSemaphoreTake(uartBusy, portMAX_DELAY);
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UARTSLK_DMA_TX_STREAM) != DISABLE);
    //Copy data in DMA buffer
    memcpy(dmaTXBuffer, data, size);
    DMA_InitStructureShareTX.DMA_BufferSize = size;
    initialDMACount = (uint16_t)size;
    // Init new DMA stream
    DMA_Init(UARTSLK_DMA_TX_STREAM, &DMA_InitStructureShareTX);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
    // Enable USART DMA TX Requests
    // Critical section is needed as the RX DMA that runs form interrupt can
    // change the same USARTx->CR3 register at the wrong point (not atomic).
    taskENTER_CRITICAL();
    USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, ENABLE);
    taskEXIT_CRITICAL();
    // Clear transfer complete
    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    // Enable DMA USART TX Stream
    DMA_Cmd(UARTSLK_DMA_TX_STREAM, ENABLE);
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    xSemaphoreGive(uartBusy);
  }
}

static void uartslkPauseDma()
{
  dmaNrfFlowControlBufferFull = true;
  dmaPausedCounter++;
  if (DMA_GetCmdStatus(UARTSLK_DMA_TX_STREAM) == ENABLE)
  {
    // Disable transfer complete interrupt
    DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, DISABLE);
    // Disable stream to pause it
    DMA_Cmd(UARTSLK_DMA_TX_STREAM, DISABLE);
    // Wait for it to be disabled
    while(DMA_GetCmdStatus(UARTSLK_DMA_TX_STREAM) != DISABLE);
    // Disable transfer complete
    DMA_ClearITPendingBit(UARTSLK_DMA_TX_STREAM, UARTSLK_DMA_TX_FLAG_TCIF);
    // Read remaining data count
    remainingDMACount = DMA_GetCurrDataCounter(UARTSLK_DMA_TX_STREAM);
    dmaIsPaused = true;
    dmaTxStreamPausedCounter++;
  }
}

static void uartslkResumeDma()
{
  dmaNrfFlowControlBufferFull = false;
  dmaResumedCounter++;
  if (dmaIsPaused)
  {
    // Update DMA counter
    DMA_SetCurrDataCounter(UARTSLK_DMA_TX_STREAM, remainingDMACount);
    // Update memory read address
    UARTSLK_DMA_TX_STREAM->M0AR = (uint32_t)&dmaTXBuffer[initialDMACount - remainingDMACount];
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
    // Clear transfer complete
    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    // Enable DMA USART TX Stream
    DMA_Cmd(UARTSLK_DMA_TX_STREAM, ENABLE);
    dmaIsPaused = false;
    dmaTxStreamResumedCounter++;
  }
}

static void uartslkDmaTXIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UARTSLK_DMA_TX_STREAM, UARTSLK_DMA_TX_FLAG_TCIF);
  USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UARTSLK_DMA_TX_STREAM, DISABLE);

  remainingDMACount = 0;
  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}

#ifdef CONFIG_SYSLINK_RX_DMA
static void uartslkReceiveDMA(uint32_t size)
{
  if (isUartDmaInitialized)
  {
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UARTSLK_DMA_RX_STREAM) != DISABLE);
    // Reload new DMA stream by loading number of bytes to be transferred
    UARTSLK_DMA_RX_STREAM->NDTR = size;
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UARTSLK_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
    // Enable USART DMA RX Requests
    USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Rx, ENABLE);
    // Clear transfer complete
    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    // Disable USART RX interrupt
    USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, DISABLE);
    // Enable DMA USART RX Stream
    DMA_Cmd(UARTSLK_DMA_RX_STREAM, ENABLE);
  }
}

static void uartslkDmaRXIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UARTSLK_DMA_RX_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UARTSLK_DMA_RX_STREAM, UARTSLK_DMA_RX_FLAG_TCIF);
  USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Rx, DISABLE);
  DMA_Cmd(UARTSLK_DMA_RX_STREAM, DISABLE);
  // Enable USART RX interrupt
  USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, ENABLE);

  for (int i = 0; i < slp.length; i++)
  {
    slp.data[i] = dmaRXBuffer[i];
    cksum[0] += dmaRXBuffer[i];
    cksum[1] += cksum[0];
  }
  // Check checksum and send to queue
  if ((cksum[0] == dmaRXBuffer[slp.length]) &&
      (cksum[1] == dmaRXBuffer[slp.length + 1]))
  {
    // Post the packet to the queue if there's room
    if (!xQueueIsQueueFullFromISR(syslinkPacketDelivery))
    {
      if (syslinkPacketDeliveryReadyToReceive)
      {
        xQueueSendFromISR(syslinkPacketDelivery, (void *)&slp, &xHigherPriorityTaskWoken);
      }
    }
    else if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
    {
      // Only assert if debugger is not connected
      ASSERT(0); // Queue overflow
    }
  }
  else
  { // Checksum error
    if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
    {
      // Only assert if debugger is not connected
      ASSERT(0);
    }
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

void uartslkHandleDataFromISR(uint8_t c, BaseType_t * const pxHigherPriorityTaskWoken)
{
  switch (rxState)
  {
  case waitForFirstStart:
    rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
    break;
  case waitForSecondStart:
    rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
    break;
  case waitForType:
    cksum[0] = c;
    cksum[1] = c;
    slp.type = c;
    rxState = waitForLength;
    break;
  case waitForLength:
    if (c <= SYSLINK_MTU)
    {
      slp.length = c;
      cksum[0] += c;
      cksum[1] += cksum[0];
      dataIndex = 0;
#ifdef CONFIG_SYSLINK_RX_DMA
      if (c >= 1)
      {
        rxState = waitForFirstStart;
        // For efficiency receive using DMA
        uartslkReceiveDMA(slp.length + UARTSLK_CLKSUM_SIZE);
      }
      else // zero length
      {
        rxState = waitForChksum1;
      }
#else
      rxState = (c > 0) ? waitForData : waitForChksum1;
#endif
    }
    else
    {
      rxState = waitForFirstStart;
    }
    break;
  case waitForData:
    slp.data[dataIndex] = c;
    cksum[0] += c;
    cksum[1] += cksum[0];
    dataIndex++;
    if (dataIndex == slp.length)
    {
      rxState = waitForChksum1;
    }
    break;
  case waitForChksum1:
    if (cksum[0] == c)
    {
      rxState = waitForChksum2;
    }
    else
    {
      rxState = waitForFirstStart; //Checksum error
      if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
      {
        // Only assert if debugger is not connected
        ASSERT(0);
      }
    }
    break;
  case waitForChksum2:
    if (cksum[1] == c)
    {
      // Post the packet to the queue if there's room
      if (!xQueueIsQueueFullFromISR(syslinkPacketDelivery))
      {
        if (syslinkPacketDeliveryReadyToReceive)
        {
          xQueueSendFromISR(syslinkPacketDelivery, (void *)&slp, pxHigherPriorityTaskWoken);
        }
      }
      else if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
      {
        // Only assert if debugger is not connected
        ASSERT(0); // Queue overflow
      }
    }
    else
    {
      rxState = waitForFirstStart; //Checksum error
      ASSERT(0);
    }
    rxState = waitForFirstStart;
    break;
  default:
    ASSERT(0);
    break;
  }
}

void uartslkIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // the following if statement replaces:
  // if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_RXNE) == SET)
  // we do this check as fast as possible to minimize the chance of an overrun,
  // which occasionally cause problems and cause packet loss at high CPU usage
  if ((UARTSLK_TYPE->SR & (1<<5)) != 0) // if the RXNE interrupt has occurred
  {
    uint8_t rxDataInterrupt = (uint8_t)(UARTSLK_TYPE->DR & 0xFF);
    uartslkHandleDataFromISR(rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
  else if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_TXE) == SET)
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(UARTSLK_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
    }
    else
    {
      USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, DISABLE);
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  else
  {
    /** if we get here, the error is most likely caused by an overrun!
     * - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun error)
     * - and IDLE (Idle line detected) pending bits are cleared by software sequence:
     * - reading USART_SR register followed reading the USART_DR register.
     */
    asm volatile ("" : "=m" (UARTSLK_TYPE->SR) : "r" (UARTSLK_TYPE->SR)); // force non-optimizable reads
    asm volatile ("" : "=m" (UARTSLK_TYPE->DR) : "r" (UARTSLK_TYPE->DR)); // of these two registers
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void uartslkTxenFlowctrlIsr()
{
  EXTI_ClearFlag(UARTSLK_TXEN_EXTI);
  if (GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET)
  {
    uartslkPauseDma();
    //ledSet(LED_GREEN_R, 1);
  }
  else
  {
    uartslkResumeDma();
    //ledSet(LED_GREEN_R, 0);
  }
}

void __attribute__((used)) EXTI4_Callback(void)
{
  uartslkTxenFlowctrlIsr();
}

void __attribute__((used)) USART6_IRQHandler(void)
{
  uartslkIsr();
}

void __attribute__((used)) DMA2_Stream7_IRQHandler(void)
{
  uartslkDmaTXIsr();
}

#ifdef CONFIG_SYSLINK_RX_DMA
void __attribute__((used)) DMA2_Stream1_IRQHandler(void)
{
  uartslkDmaRXIsr();
}
#endif

void uartSyslinkDumpDebugProbe() {
  DEBUG_PRINT("STM dmaPausedCounter: %ld\n",dmaPausedCounter);
  DEBUG_PRINT("STM dmaTxStreamPausedCounter: %ld\n", dmaTxStreamPausedCounter);
  DEBUG_PRINT("STM dmaResumedCounter: %ld\n", dmaResumedCounter);
  DEBUG_PRINT("STM dmaTxStreamResumedCounter: %ld\n", dmaTxStreamResumedCounter);
  DEBUG_PRINT("STM dmaSendWhileNrfBufferFull: %ld\n", dmaSendWhileNrfBufferFull);
}
