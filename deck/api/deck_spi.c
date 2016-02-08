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
 * deck_spi.c - Deck-API SPI communication implementation
 */

#include "deck.h"

#include "stm32fxxx.h"

#include <string.h>

/*ST includes */
#include "stm32fxxx.h"
#include "config.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"

#define SPI                           SPI1
#define SPI_CLK                       RCC_APB2Periph_SPI1
#define SPI_CLK_INIT                  RCC_APB2PeriphClockCmd
#define SPI_IRQ_HANDLER               SPI1_IRQHandler
#define SPI_IRQn                      SPI1_IRQn


#define SPI_SCK_PIN                   GPIO_Pin_5
#define SPI_SCK_GPIO_PORT             GPIOA
#define SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPI_SCK_SOURCE                GPIO_PinSource5
#define SPI_SCK_AF                    GPIO_AF_SPI1

#define SPI_MISO_PIN                  GPIO_Pin_6
#define SPI_MISO_GPIO_PORT            GPIOA
#define SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPI_MISO_SOURCE               GPIO_PinSource6
#define SPI_MISO_AF                   GPIO_AF_SPI1

#define SPI_MOSI_PIN                  GPIO_Pin_7
#define SPI_MOSI_GPIO_PORT            GPIOA
#define SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPI_MOSI_SOURCE               GPIO_PinSource7
#define SPI_MOSI_AF                   GPIO_AF_SPI1

#define DUMMY_BYTE         0xA5

static bool isInit = false;

static xSemaphoreHandle xferComplete;

void spiBegin(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  vSemaphoreCreateBinary(xferComplete);
  xSemaphoreTake(xferComplete, portMAX_DELAY);

  /*!< Enable the SPI clock */
  SPI_CLK_INIT(SPI_CLK, ENABLE);

  /*!< Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MISO_GPIO_CLK |
                         SPI_MOSI_GPIO_CLK, ENABLE);

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

  /*!< SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; //~2.6 MHz

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used
  SPI_Init(SPI, &SPI_InitStructure);

  /*!< Enable the SPI  */
  SPI_Cmd(SPI, ENABLE);

  /* Enable SPI Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SPI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //NVIC_EnableIRQ(SPI_IRQn);

  isInit = true;
}

void spiFast()
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //~10 MHz

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used
  SPI_Init(SPI, &SPI_InitStructure);
}

bool spiTest(void)
{
  return isInit;
}

static const uint8_t * txBuffer;
static uint8_t * rxBuffer;
static volatile int nextTxByte;
static volatile int byteTxLeft;
static volatile int nextRxByte;
static volatile int byteRxLeft;

void SPI_IRQ_HANDLER()
{
  volatile int dummy;

  if (SPI_I2S_GetITStatus(SPI, SPI_I2S_IT_RXNE) == SET) {
    if (rxBuffer) {
      rxBuffer[nextRxByte] = SPI_I2S_ReceiveData(SPI);
    } else {
      dummy = SPI_I2S_ReceiveData(SPI);
      dummy;  // To avoid GCC Warning
    }

    if (byteRxLeft > 1) {
      if (txBuffer) {
        SPI_I2S_SendData(SPI, txBuffer[nextTxByte]);
      } else {
        SPI_I2S_SendData(SPI, DUMMY_BYTE);
      }
    }

    byteRxLeft--;
    nextRxByte++;
    nextTxByte++;

    if (byteRxLeft == 0) {
      portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

      SPI_I2S_ITConfig(SPI, SPI_I2S_IT_RXNE, DISABLE);

      xSemaphoreGiveFromISR(xferComplete, &xHigherPriorityTaskWoken);

      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}


bool spiXfer(uint32_t length, const uint8_t * data_tx, uint8_t * data_rx)
{
  volatile int dummy;

  if (length < 1) {
    return true;
  }

  // Clear possible overrun error
  dummy = SPI->DR;
  dummy = SPI->SR;
  dummy;

  txBuffer = data_tx;
  rxBuffer = data_rx;

  SPI_I2S_SendData(SPI, txBuffer[0]);

  nextTxByte = 1;
  nextRxByte = 0;
  byteRxLeft = length;

  SPI_I2S_ITConfig(SPI, SPI_I2S_IT_RXNE, ENABLE);

  xSemaphoreTake(xferComplete, portMAX_DELAY);

  return true;
}

void spiTransfer(void *buffer, int length) {
  spiXfer(length, buffer, buffer);
}

bool spiWrite(uint32_t length, const uint8_t * data)
{
  return spiXfer(length, data, NULL);
}

bool spiRead(uint32_t length, uint8_t * data)
{
  return spiXfer(length, NULL, data);
}
