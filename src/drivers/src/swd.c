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
 * swd.c - Low level SWD functionality
 */
#include <stdbool.h>

#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "swd.h"

static bool isInit = false;

#define SWD_SPI                           SPI2
#define SWD_SPI_CLK                       RCC_APB1Periph_SPI2
#define SWD_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define SWD_SPI_SCK_PIN                   GPIO_Pin_13
#define SWD_SPI_SCK_GPIO_PORT             GPIOB
#define SWD_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define SWD_SPI_SCK_SOURCE                GPIO_PinSource13
#define SWD_SPI_SCK_AF                    GPIO_AF_SPI2

#define SWD_SPI_MISO_PIN                  GPIO_Pin_15
#define SWD_SPI_MISO_GPIO_PORT            GPIOB
#define SWD_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define SWD_SPI_MISO_SOURCE               GPIO_PinSource15
#define SWD_SPI_MISO_AF                   GPIO_AF_SPI2


static void initGPIO(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  SWD_SPI_CLK_INIT(SWD_SPI_CLK, ENABLE);

  RCC_AHB1PeriphClockCmd(SWD_SPI_SCK_GPIO_CLK | SWD_SPI_MISO_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(SWD_SPI_SCK_GPIO_PORT, SWD_SPI_SCK_SOURCE, SWD_SPI_SCK_AF);
  GPIO_PinAFConfig(SWD_SPI_MISO_GPIO_PORT, SWD_SPI_MISO_SOURCE, SWD_SPI_MISO_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = SWD_SPI_SCK_PIN;
  GPIO_Init(SWD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* In bi-directional mode only MISO is used */
  GPIO_InitStructure.GPIO_Pin =  SWD_SPI_MISO_PIN;
  GPIO_Init(SWD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

}

static void initSPI(void) {

  SPI_InitTypeDef  SPI_InitStructure;

  initGPIO();

  /* Set up SPI in bi-directional mode */
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SWD_SPI, &SPI_InitStructure);

  SPI_Cmd(SWD_SPI, ENABLE);
}

//Initialize the green led pin as output
void swdInit()
{
  if(isInit)
    return;

  initSPI();

  isInit = true;
}

/* Code for debugging and testing, should be removed later */

uint8_t tx_data[35];
uint8_t rx_data[35];

uint32_t counter = 0;
uint16_t number_of_txd_bytes = 0;
uint32_t transfer_size = 35;

uint8_t header = 0xA5;
uint8_t idx = 0;

bool swdTest(void) {
  // Start = 1
  // DP access = 0
  // Read access = 1
  // DP reg = 00 (IDCODE)
  // Parity = odd bits -> 1
  // Stop = 0
  // Park = 1 (not 0!)

  // First at least fifty ones, then the sequence below to switch from JTAG to SWD and then at least 50 ones again.
  // The the SWD is reset and ready to use. Note that once this is done the nRF will only go into simulated off mode since
  // a debugger is connected.
  // 111...01111001 11100111...111

  // The command for reading IDCODE is
  /// 10100101

  // Should read out (and does):0x0BB11477 (according to OpenOCD)
  //T ACK |------------DATA--------------|P
  //? 100 111011100010100010001101110100001

  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
#if 0
  tx_data[idx++] = 0x9E;
  tx_data[idx++] = 0xE7;
#else
  tx_data[idx++] = 0x79;
  tx_data[idx++] = 0xE7;
#endif
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFF;
  tx_data[idx++] = 0xFC;
  tx_data[idx++] = header;
  // Rest is 0x00

  /* As long as the SPI is enabled in bi-di mode the clock is enabled. So
   * we write data as normal, but when reading there's no need to output any
   * data to trigger the clock. Direction needs to be set accordingly.
   */

  while(number_of_txd_bytes < transfer_size)
  {
    if (number_of_txd_bytes < idx)
    {
      SPI_BiDirectionalLineConfig(SWD_SPI, SPI_Direction_Tx);

      /*!< Loop while DR register in not emplty */
      while (SPI_I2S_GetFlagStatus(SWD_SPI, SPI_I2S_FLAG_TXE) == RESET);

      SPI_I2S_SendData(SWD_SPI, tx_data[number_of_txd_bytes]);

      // Make sure that we have sent data in case next loop will be RX, the
      // mode is switched before we manage to send the data!
      while (SPI_I2S_GetFlagStatus(SWD_SPI, SPI_I2S_FLAG_BSY) == SET);
    }
    else
    {
      SPI_BiDirectionalLineConfig(SWD_SPI, SPI_Direction_Rx);
      while (SPI_I2S_GetFlagStatus(SWD_SPI, SPI_I2S_FLAG_RXNE) == RESET);

      /*!< Return the byte read from the SPI bus */
      rx_data[number_of_txd_bytes] = (uint8_t) SPI_I2S_ReceiveData(SWD_SPI);
    }
    number_of_txd_bytes++;
  }
  SPI_Cmd(SWD_SPI, DISABLE);
  return isInit;
}

