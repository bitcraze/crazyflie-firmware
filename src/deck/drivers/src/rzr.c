/*
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
 * rzr.c - Crazyflie RZR board.
 */
#define DEBUG_MODULE "RZR"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32fxxx.h"
#include "config.h"
#include "motors.h"
#include "debug.h"
#include "deck.h"
#include "extrx.h"
#include "pm.h"
#include "mpu6500.h"

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define RZR_GPIO_CS             GPIO_Pin_14
#define RZR_GPIO_CS_PORT        GPIOC
#define RZR_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOC

#define RZR_SPI                 SPI2
#define RZR_SPI_AF              GPIO_AF_SPI2
#define RZR_SPI_CLK             RCC_APB1Periph_SPI2
#define RZR_GPIO_SPI_PORT       GPIOB
#define RZR_GPIO_SPI_CLK        RCC_AHB1Periph_GPIOB
#define RZR_GPIO_SPI_SCK        GPIO_Pin_13
#define RZR_GPIO_SPI_SCK_SRC    GPIO_PinSource13
#define RZR_GPIO_SPI_MISO       GPIO_Pin_14
#define RZR_GPIO_SPI_MISO_SRC   GPIO_PinSource14
#define RZR_GPIO_SPI_MOSI       GPIO_Pin_15
#define RZR_GPIO_SPI_MOSI_SRC   GPIO_PinSource15

#define DUMMY_BYTE    0x00
/* Usefull macro */
#define RZR_EN_CS() GPIO_ResetBits(RZR_GPIO_CS_PORT, RZR_GPIO_CS)
#define RZR_DIS_CS() GPIO_SetBits(RZR_GPIO_CS_PORT, RZR_GPIO_CS)

#define RZR_GPIO_RCC_M1_OVERRIDE    RCC_AHB1Periph_GPIOA
#define RZR_GPIO_PORT_M1_OVERRIDE   GPIOA
#define RZR_GPIO_PIN_M1_OVERRIDE    GPIO_Pin_0
#define RZR_GPIO_RCC_M2_OVERRIDE    RCC_AHB1Periph_GPIOB
#define RZR_GPIO_PORT_M2_OVERRIDE   GPIOB
#define RZR_GPIO_PIN_M2_OVERRIDE    GPIO_Pin_12
#define RZR_GPIO_RCC_M3_OVERRIDE    RCC_AHB1Periph_GPIOC
#define RZR_GPIO_PORT_M3_OVERRIDE   GPIOC
#define RZR_GPIO_PIN_M3_OVERRIDE    GPIO_Pin_8
#define RZR_GPIO_RCC_M4_OVERRIDE    RCC_AHB1Periph_GPIOC
#define RZR_GPIO_PORT_M4_OVERRIDE   GPIOC
#define RZR_GPIO_PIN_M4_OVERRIDE    GPIO_Pin_15

//Hardware configuration
static bool isInit;

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RZR_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(RZR_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RZR_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(RZR_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

static void writeReg(uint8_t reg, uint8_t val)
{
  RZR_EN_CS();
  spiSendByte(reg);
  spiSendByte(val);
  RZR_DIS_CS();
}

static uint8_t readReg(uint8_t reg)
{
  uint8_t regval;

  RZR_EN_CS();
  spiSendByte(reg | 0x80);
  regval = spiReceiveByte();
  RZR_DIS_CS();

  return regval;
}

static void readAllReg(void)
{
  static uint8_t regs[128];

  RZR_EN_CS();
  spiSendByte(0x00 | 0x80);

  for (int i=0; i<10; i++)
  {
    regs[i] = spiReceiveByte();
  }
  RZR_DIS_CS();
}



/* Initialisation */
static void spiInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(RZR_GPIO_SPI_CLK | RZR_GPIO_CS_PERIF, ENABLE);
  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(RZR_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_SPI_SCK |  RZR_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RZR_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(RZR_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(RZR_GPIO_CS_PORT, &GPIO_InitStructure);

  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(RZR_GPIO_SPI_PORT, RZR_GPIO_SPI_SCK_SRC, RZR_SPI_AF);
  GPIO_PinAFConfig(RZR_GPIO_SPI_PORT, RZR_GPIO_SPI_MISO_SRC, RZR_SPI_AF);
  GPIO_PinAFConfig(RZR_GPIO_SPI_PORT, RZR_GPIO_SPI_MOSI_SRC, RZR_SPI_AF);

  /* disable the chip select */
  RZR_DIS_CS();

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(RZR_SPI, &SPI_InitStructure);

  /* Enable the SPI  */
  SPI_Cmd(RZR_SPI, ENABLE);

  isInit = true;
}

static void rzrInit(DeckInfo *info)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  if(isInit)
    return;

  DEBUG_PRINT("Switching to brushless.\n");

  spiInit();

  // Configure GPIO for power to BL motor connectors
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M1_OVERRIDE, ENABLE);
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M2_OVERRIDE, ENABLE);
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M3_OVERRIDE, ENABLE);
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M4_OVERRIDE, ENABLE);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M1_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M1_OVERRIDE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M2_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M2_OVERRIDE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M3_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M3_OVERRIDE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M4_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M4_OVERRIDE, &GPIO_InitStructure);

  // Enable for power to BL motor connectors
  GPIO_WriteBit(RZR_GPIO_PORT_M1_OVERRIDE, RZR_GPIO_PIN_M1_OVERRIDE, 1);
  GPIO_WriteBit(RZR_GPIO_PORT_M2_OVERRIDE, RZR_GPIO_PIN_M2_OVERRIDE, 1);
  GPIO_WriteBit(RZR_GPIO_PORT_M3_OVERRIDE, RZR_GPIO_PIN_M3_OVERRIDE, 1);
  GPIO_WriteBit(RZR_GPIO_PORT_M4_OVERRIDE, RZR_GPIO_PIN_M4_OVERRIDE, 1);

  // Remap motor PWM output
  motorsInit(motorMapRZRBrushless);

  isInit = true;
}

static bool rzrTest()
{
  bool status = true;

  if(!isInit)
    return false;

  status = motorsTest();

  readAllReg();

//  // reset the device
//  writeReg(MPU6500_RA_PWR_MGMT_1, 0x80);
//  vTaskDelay(M2T(100)); // page 42 - delay 100ms
//  // set SPI mode by setting I2C_IF_DIS
//  // reset DMP, FIFO, SIG
//  writeReg( MPU6500_RA_USER_CTRL, 0x10 | 0x8 | 0x4 | 0x1 );
//  vTaskDelay(M2T(100));
//
//  for (int i=0; i<10; i++)
//  {
//    readReg(MPU6500_RA_WHO_AM_I);
//    //DEBUG_PRINT("WHO:%X\n", readReg(MPU6500_RA_WHO_AM_I));
//    //vTaskDelay(M2T(1));
//  }


  return status;
}

static const DeckDriver rzr_deck = {
//  .vid = 0xBC,
//  .pid = 0x08,
  .name = "bcRZR",

  .usedPeriph = 0,
  .usedGpio = 0,
  .init = rzrInit,
  .test = rzrTest,
};

DECK_DRIVER(rzr_deck);
