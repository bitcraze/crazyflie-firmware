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
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "uart_syslink.h"
#include "crtp.h"
#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"


#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static bool isInit = false;

xSemaphoreHandle waitUntilSendDone = NULL;
static xQueueHandle uartDataDelivery;

static uint8_t dmaBuffer[64];
static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;
static bool    isUartDmaInitialized;
static DMA_InitTypeDef DMA_InitStructureShare;
static uint32_t initialDMACount;
static uint32_t remainingDMACount;
static bool     dmaIsPaused;

static void uartPauseDma();
static void uartResumeDma();

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
void uartDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // USART TX DMA Channel Config
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&UART_TYPE->DR;
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
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructureShare.DMA_Channel = UART_DMA_CH;

  NVIC_InitStructure.NVIC_IRQChannel = UART_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  isUartDmaInitialized = true;
}

void uartInit(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef extiInit;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART_GPIO_PERIF, ENABLE);
  ENABLE_UART_RCC(UART_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UART_GPIO_PORT, UART_GPIO_AF_TX_PIN, UART_GPIO_AF_TX);
  GPIO_PinAFConfig(UART_GPIO_PORT, UART_GPIO_AF_RX_PIN, UART_GPIO_AF_RX);

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
  USART_InitStructure.USART_BaudRate            = 2000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
#else
  USART_InitStructure.USART_BaudRate            = 1000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
#endif
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
  USART_Init(UART_TYPE, &USART_InitStructure);

  uartDmaInit();

  // TODO: Enable
  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  vSemaphoreCreateBinary(waitUntilSendDone);
  uartDataDelivery = xQueueCreate(40, sizeof(uint8_t));

  USART_ITConfig(UART_TYPE, USART_IT_RXNE, ENABLE);

  //Setting up TXEN pin (NRF flow control)
  RCC_AHB1PeriphClockCmd(UART_TXEN_PERIF, ENABLE);

  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = UART_TXEN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART_TXEN_PORT, &GPIO_InitStructure);

  extiInit.EXTI_Line = UART_TXEN_EXTI;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInit);

  NVIC_EnableIRQ(EXTI4_IRQn);

  //Enable UART
  USART_Cmd(UART_TYPE, ENABLE);
  
  isInit = true;
}

bool uartTest(void)
{
  return isInit;
}

bool uartGetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(uartDataDelivery, c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }
  return false;
}

void uartSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UART_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UART_TXEN_PORT, UART_TXEN_PIN) == Bit_SET);
#endif
    while (!(UART_TYPE->SR & USART_FLAG_TXE));
    UART_TYPE->DR = (data[i] & 0x00FF);
  }
}

void uartSendDataIsrBlocking(uint32_t size, uint8_t* data)
{
  outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
  uartSendData(1, &data[0]);
  USART_ITConfig(UART_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
}

int uartPutchar(int ch)
{
    uartSendData(1, (uint8_t *)&ch);
    
    return (unsigned char)ch;
}

void uartSendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UART_DMA_STREAM) != DISABLE);
    //Copy data in DMA buffer
    memcpy(dmaBuffer, data, size);
    DMA_InitStructureShare.DMA_BufferSize = size;
    initialDMACount = size;
    // Init new DMA stream
    DMA_Init(UART_DMA_STREAM, &DMA_InitStructureShare);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Enable USART DMA TX Requests */
    USART_DMACmd(UART_TYPE, USART_DMAReq_Tx, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UART_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UART_DMA_STREAM, ENABLE);
  }
}

static void uartPauseDma()
{
  if (DMA_GetCmdStatus(UART_DMA_STREAM) == ENABLE)
  {
    // Disable transfer complete interrupt
    DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, DISABLE);
    // Disable stream to pause it
    DMA_Cmd(UART_DMA_STREAM, DISABLE);
    // Wait for it to be disabled
    while(DMA_GetCmdStatus(UART_DMA_STREAM) != DISABLE);
    // Disable transfer complete
    DMA_ClearITPendingBit(UART_DMA_STREAM, UART_DMA_FLAG_TCIF);
    // Read remaining data count
    remainingDMACount = DMA_GetCurrDataCounter(UART_DMA_STREAM);
    dmaIsPaused = true;
  }
}

static void uartResumeDma()
{
  if (dmaIsPaused)
  {
    // Update DMA counter
    DMA_SetCurrDataCounter(UART_DMA_STREAM, remainingDMACount);
    // Update memory read address
    UART_DMA_STREAM->M0AR = (uint32_t)&dmaBuffer[initialDMACount - remainingDMACount];
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UART_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UART_DMA_STREAM, ENABLE);
    dmaIsPaused = false;
  }
}

void uartDmaIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UART_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UART_DMA_STREAM, UART_DMA_FLAG_TCIF);
  USART_DMACmd(UART_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UART_DMA_STREAM, DISABLE);

  remainingDMACount = 0;
  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}

void uartIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint8_t rxDataInterrupt;

  if (USART_GetITStatus(UART_TYPE, USART_IT_TXE))
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(UART_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
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
    rxDataInterrupt = USART_ReceiveData(UART_TYPE) & 0x00FF;
    xQueueSendFromISR(uartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}

void uartTxenFlowctrlIsr()
{
  EXTI_ClearFlag(UART_TXEN_EXTI);
  if (GPIO_ReadInputDataBit(UART_TXEN_PORT, UART_TXEN_PIN) == Bit_SET)
  {
    uartPauseDma();
  }
  else
  {
    uartResumeDma();
  }
}

void __attribute__((used)) EXTI4_IRQHandler(void)
{
  uartTxenFlowctrlIsr();
}

void __attribute__((used)) USART2_IRQHandler(void)
{
  uartIsr();
}

void __attribute__((used)) UART4_IRQHandler(void)
{
  uartIsr();
}

void __attribute__((used)) USART6_IRQHandler(void)
{
  uartIsr();
}

void __attribute__((used)) DMA2_Stream7_IRQHandler(void)
{
  uartDmaIsr();
}

