/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 *
 * sensors_bmi088_spi.c: SPI backend for the bmi088 sensor
 */

#include <string.h>

#include "stm32fxxx.h"

#include "bmi088.h"
#include "i2cdev.h"
#include "bstdr_types.h"
#include "nvicconf.h"

#include "sensors_bmi088_common.h"

#define DUMMY_BYTE    0x00

/* Defines and buffers for full duplex SPI DMA transactions */
/* The buffers must not be placed in CCM */
#define SPI_MAX_DMA_TRANSACTION_SIZE    15
static uint8_t spiTxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static uint8_t spiRxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static xSemaphoreHandle spiTxDMAComplete;
static StaticSemaphore_t spiTxDMACompleteBuffer;
static xSemaphoreHandle spiRxDMAComplete;
static StaticSemaphore_t spiRxDMACompleteBuffer;

#define BMI088_ACC_GPIO_CS             GPIO_Pin_1
#define BMI088_ACC_GPIO_CS_PORT        GPIOB
#define BMI088_ACC_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOB
#define BMI088_GYR_GPIO_CS             GPIO_Pin_0
#define BMI088_GYR_GPIO_CS_PORT        GPIOB
#define BMI088_GYR_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOB

#define BMI088_SPI                     SPI2
#define BMI088_SPI_AF                  GPIO_AF_SPI2
#define BMI088_SPI_CLK                 RCC_APB1Periph_SPI2
#define BMI088_GPIO_SPI_PORT           GPIOB
#define BMI088_GPIO_SPI_CLK            RCC_AHB1Periph_GPIOB
#define BMI088_GPIO_SPI_SCK            GPIO_Pin_13
#define BMI088_GPIO_SPI_SCK_SRC        GPIO_PinSource13
#define BMI088_GPIO_SPI_MISO           GPIO_Pin_14
#define BMI088_GPIO_SPI_MISO_SRC       GPIO_PinSource14
#define BMI088_GPIO_SPI_MOSI           GPIO_Pin_15
#define BMI088_GPIO_SPI_MOSI_SRC       GPIO_PinSource15

#define BMI088_SPI_DMA_IRQ_PRIO        NVIC_HIGH_PRI
#define BMI088_SPI_DMA                 DMA1
#define BMI088_SPI_DMA_CLK             RCC_AHB1Periph_DMA1
#define BMI088_SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define BMI088_SPI_RX_DMA_STREAM       DMA1_Stream3
#define BMI088_SPI_RX_DMA_IRQ          DMA1_Stream3_IRQn
#define BMI088_SPI_RX_DMA_IRQHandler   DMA1_Stream3_IRQHandler
#define BMI088_SPI_RX_DMA_CHANNEL      DMA_Channel_0
#define BMI088_SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define BMI088_SPI_TX_DMA_STREAM       DMA1_Stream4
#define BMI088_SPI_TX_DMA_IRQ          DMA1_Stream4_IRQn
#define BMI088_SPI_TX_DMA_IRQHandler   DMA1_Stream4_IRQHandler
#define BMI088_SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define BMI088_SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF4

#define ACC_EN_CS() GPIO_ResetBits(BMI088_ACC_GPIO_CS_PORT, BMI088_ACC_GPIO_CS)
#define ACC_DIS_CS() GPIO_SetBits(BMI088_ACC_GPIO_CS_PORT, BMI088_ACC_GPIO_CS)
#define GYR_EN_CS() GPIO_ResetBits(BMI088_GYR_GPIO_CS_PORT, BMI088_GYR_GPIO_CS)
#define GYR_DIS_CS() GPIO_SetBits(BMI088_GYR_GPIO_CS_PORT, BMI088_GYR_GPIO_CS)

static bool isInit;

static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI peripheral */
  SPI_I2S_SendData(BMI088_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(BMI088_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

static void spiDMATransaction(uint8_t reg_addr,
                              uint8_t *reg_data,
                              uint16_t len)
{
  ASSERT(len < SPI_MAX_DMA_TRANSACTION_SIZE);

  // Disable peripheral before setting up for duplex DMA
  SPI_Cmd(BMI088_SPI, DISABLE);

  // DMA already configured, just need to set memory addresses and read command byte
  spiTxBuffer[0] = reg_addr;
  BMI088_SPI_TX_DMA_STREAM->M0AR = (uint32_t)&spiTxBuffer[0];
  BMI088_SPI_TX_DMA_STREAM->NDTR = len + 1;

  BMI088_SPI_RX_DMA_STREAM->M0AR = (uint32_t)&spiRxBuffer[0];
  BMI088_SPI_RX_DMA_STREAM->NDTR = len + 1;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(BMI088_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(BMI088_SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  DMA_ClearFlag(BMI088_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|
                DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
  DMA_ClearFlag(BMI088_SPI_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|
                DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

  // Enable DMA Streams
  DMA_Cmd(BMI088_SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(BMI088_SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral to begin the transaction
  SPI_Cmd(BMI088_SPI, ENABLE);

  // Wait for completion
  // TODO: Better error handling rather than passing up invalid data
  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it higher up
  memcpy(reg_data, &spiRxBuffer[1], len);
}

static bstdr_ret_t spi_burst_read(uint8_t dev_id, uint8_t reg_addr,
                                  uint8_t *reg_data, uint16_t len)
{
  /**< Burst read code comes here */
  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_EN_CS();
  }
  else
  {
    GYR_EN_CS();
  }

  if (len <= 1 || len > SPI_MAX_DMA_TRANSACTION_SIZE)
  {
    spiSendByte(reg_addr);
    for (int i = 0; i < len; i++)
    {
      reg_data[i] = spiReceiveByte();
    }
  }
  else
  {
    spiDMATransaction(reg_addr, reg_data, len);
  }

  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_DIS_CS();
  }
  else
  {
    GYR_DIS_CS();
  }

  return BSTDR_OK;
}

static bstdr_ret_t spi_burst_write(uint8_t dev_id, uint8_t reg_addr,
                                   uint8_t *reg_data, uint16_t len)
{
  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_EN_CS();
  }
  else
  {
    GYR_EN_CS();
  }
  spiSendByte(reg_addr);
  for (int i = 0; i < len; i++)
  {
    spiSendByte(reg_data[i]);
  }

  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_DIS_CS();
  }
  else
  {
    GYR_DIS_CS();
  }

  return BSTDR_OK;
}

static void spiConfigure(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(BMI088_SPI, DISABLE);
  SPI_I2S_DeInit(BMI088_SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //~10.5 MHz
  SPI_Init(BMI088_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(BMI088_SPI, ENABLE);
}

/* Initialisation */
static void spiInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  isInit = true;

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(BMI088_GPIO_SPI_CLK | BMI088_ACC_GPIO_CS_PERIF, ENABLE);
  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(BMI088_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = BMI088_GPIO_SPI_SCK |  BMI088_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BMI088_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = BMI088_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(BMI088_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = BMI088_ACC_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(BMI088_ACC_GPIO_CS_PORT, &GPIO_InitStructure);
  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = BMI088_GYR_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(BMI088_GYR_GPIO_CS_PORT, &GPIO_InitStructure);


  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(BMI088_GPIO_SPI_PORT, BMI088_GPIO_SPI_SCK_SRC, BMI088_SPI_AF);
  GPIO_PinAFConfig(BMI088_GPIO_SPI_PORT, BMI088_GPIO_SPI_MISO_SRC, BMI088_SPI_AF);
  GPIO_PinAFConfig(BMI088_GPIO_SPI_PORT, BMI088_GPIO_SPI_MOSI_SRC, BMI088_SPI_AF);

  /* disable the chip select */
  ACC_DIS_CS();

  spiConfigure();
}

static void spiDMAInit(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*!< Enable DMA Clocks */
  BMI088_SPI_DMA_CLK_INIT(BMI088_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(BMI088_SPI->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = BMI088_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(BMI088_SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(BMI088_SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = BMI088_SPI_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(BMI088_SPI_RX_DMA_STREAM, DISABLE);
  DMA_Init(BMI088_SPI_RX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_BMI088_SPI_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = BMI088_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = BMI088_SPI_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  spiTxDMAComplete = xSemaphoreCreateBinaryStatic(&spiTxDMACompleteBuffer);
  spiRxDMAComplete = xSemaphoreCreateBinaryStatic(&spiRxDMACompleteBuffer);
}

void sensorsBmi088_SPI_deviceInit(struct bmi088_dev *device)
{
  spiInit();
  spiDMAInit();

  device->accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY;
  device->gyro_id = BMI088_GYRO_I2C_ADDR_PRIMARY;
  device->interface = BMI088_SPI_INTF;
  device->read = spi_burst_read;
  device->write = spi_burst_write;
  device->delay_ms = bmi088_ms_delay;
}

void __attribute__((used)) BMI088_SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(BMI088_SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(BMI088_SPI_TX_DMA_STREAM, BMI088_SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(BMI088_SPI_TX_DMA_STREAM,BMI088_SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(BMI088_SPI_TX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiTxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) BMI088_SPI_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(BMI088_SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(BMI088_SPI_RX_DMA_STREAM, BMI088_SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(BMI088_SPI_RX_DMA_STREAM, BMI088_SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(BMI088_SPI_RX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiRxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
