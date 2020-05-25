/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * deck_spi3.c - Deck-API SPI3 communication implementation. Mainly used so that
 *               the uSD-deck can log e.g. the loco deck.
 */

#include "deck.h"

/*ST includes */
#include "stm32fxxx.h"
#include "config.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"

// Alternative SPI
#define SPI                    SPI3
#define SPI_CLK                RCC_APB1Periph_SPI3
#define SPI_CLK_INIT           RCC_APB1PeriphClockCmd
#define SPI_IRQ_HANDLER        SPI3_IRQHandler
#define SPI_IRQn               SPI3_IRQn

#define SPI_DMA_IRQ_PRIO       (NVIC_HIGH_PRI)
#define SPI_DMA                DMA1
#define SPI_DMA_CLK            RCC_AHB1Periph_DMA1
#define SPI_DMA_CLK_INIT       RCC_AHB1PeriphClockCmd

#define SPI_TX_DMA_STREAM      DMA1_Stream7
#define SPI_TX_DMA_IRQ         DMA1_Stream7_IRQn
#define SPI_TX_DMA_IRQHandler  DMA1_Stream7_IRQHandler
#define SPI_TX_DMA_CHANNEL     DMA_Channel_0
#define SPI_TX_DMA_FLAG_TCIF   DMA_FLAG_TCIF7

#define SPI_RX_DMA_STREAM      DMA1_Stream0
#define SPI_RX_DMA_IRQ         DMA1_Stream0_IRQn
#define SPI_RX_DMA_IRQHandler  DMA1_Stream0_IRQHandler
#define SPI_RX_DMA_CHANNEL     DMA_Channel_0
#define SPI_RX_DMA_FLAG_TCIF   DMA_FLAG_TCIF0

#define SPI_SCK_PIN            GPIO_Pin_10
#define SPI_SCK_GPIO_PORT      GPIOC
#define SPI_SCK_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define SPI_SCK_SOURCE         GPIO_PinSource10
#define SPI_SCK_AF             GPIO_AF_SPI3

#define SPI_MISO_PIN           GPIO_Pin_11
#define SPI_MISO_GPIO_PORT     GPIOC
#define SPI_MISO_GPIO_CLK      RCC_AHB1Periph_GPIOC
#define SPI_MISO_SOURCE        GPIO_PinSource11
#define SPI_MISO_AF            GPIO_AF_SPI3

#define SPI_MOSI_PIN           GPIO_Pin_12
#define SPI_MOSI_GPIO_PORT     GPIOC
#define SPI_MOSI_GPIO_CLK      RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE        GPIO_PinSource12
#define SPI_MOSI_AF            GPIO_AF_SPI3

#define DUMMY_BYTE         0xA5

static bool isInit = false;

static SemaphoreHandle_t txComplete;
static SemaphoreHandle_t rxComplete;
static SemaphoreHandle_t spiMutex;

// Mainly used of uSD deck that is patched to use alternative SPI
static void spi3DMAInit();
static void spi3ConfigureWithSpeed(uint16_t baudRatePrescaler);

void spi3Begin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // binary semaphores created using xSemaphoreCreateBinary() are created in a state
  // such that the the semaphore must first be 'given' before it can be 'taken'
  txComplete = xSemaphoreCreateBinary();
  rxComplete = xSemaphoreCreateBinary();
  spiMutex = xSemaphoreCreateMutex();

  /*!< Enable the SPI clock */
  SPI_CLK_INIT(SPI_CLK, ENABLE);

  /*!< Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MISO_GPIO_CLK |
                         SPI_MOSI_GPIO_CLK, ENABLE);

  /*!< Enable DMA Clocks */
  SPI_DMA_CLK_INIT(SPI_DMA_CLK, ENABLE);

  /*!< SPI pins configuration *************************************************/

  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  GPIO_PinAFConfig(SPI_MISO_GPIO_PORT, SPI_MISO_SOURCE, SPI_MISO_AF);
  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

  /*!< SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MISO_PIN;
  GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI DMA Initialization */
  spi3DMAInit();

  /*!< SPI configuration */
  spi3ConfigureWithSpeed(SPI_BAUDRATE_2MHZ);

  isInit = true;
}

static void spi3DMAInit()
{
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = SPI_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(SPI_RX_DMA_STREAM,DISABLE);
  DMA_Init(SPI_RX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = SPI_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);
}

static void spi3ConfigureWithSpeed(uint16_t baudRatePrescaler)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_I2S_DeInit(SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = baudRatePrescaler;
  SPI_Init(SPI, &SPI_InitStructure);
}

bool spi3Test(void)
{
  return isInit;
}

bool spi3Exchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx)
{
  ASSERT_DMA_SAFE(data_tx);
  ASSERT_DMA_SAFE(data_rx);

  // DMA already configured, just need to set memory addresses
  SPI_TX_DMA_STREAM->M0AR = (uint32_t)data_tx;
  SPI_TX_DMA_STREAM->NDTR = length;

  SPI_RX_DMA_STREAM->M0AR = (uint32_t)data_rx;
  SPI_RX_DMA_STREAM->NDTR = length;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  DMA_ClearFlag(SPI_TX_DMA_STREAM, DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7|DMA_FLAG_TCIF7);
  DMA_ClearFlag(SPI_RX_DMA_STREAM, DMA_FLAG_FEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_TEIF0|DMA_FLAG_HTIF0|DMA_FLAG_TCIF0);

  // Enable DMA Streams
  DMA_Cmd(SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral
  SPI_Cmd(SPI, ENABLE);

  // Wait for completion
  bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
             && (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

  // Disable peripheral
  SPI_Cmd(SPI, DISABLE);
  return result;
}

void spi3BeginTransaction(uint16_t baudRatePrescaler)
{
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  spi3ConfigureWithSpeed(baudRatePrescaler);
}

void spi3EndTransaction()
{
  xSemaphoreGive(spiMutex);
}

#ifdef USDDECK_USE_ALT_PINS_AND_SPI
void __attribute__((used)) SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(SPI_TX_DMA_STREAM, SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(SPI_TX_DMA_STREAM,SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) SPI_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(SPI_RX_DMA_STREAM, SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(SPI_RX_DMA_STREAM,SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_RX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(rxComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
#endif
