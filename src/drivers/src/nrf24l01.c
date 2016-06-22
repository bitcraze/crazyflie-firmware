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
 * nrf24l01.c: nRF24L01(-p) PRX mode low level driver
 */

/* TODO:
 *  - Separate the SPI and GPIO driver from here.
 *  - Handle PTX mode
 */
#define DEBUG_MODULE "NRF"

#include "nrf24l01.h"

#include <stdbool.h>
#include <string.h>

#include "cfassert.h"

/* ST includes */
#include "stm32fxxx.h"

#include "debug.h"
#include "exti.h"

#include "nRF24L01reg.h"

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define RADIO_GPIO_CS             GPIO_Pin_12
#define RADIO_GPIO_CS_PORT        GPIOB
#define RADIO_GPIO_CS_PERIF       RCC_APB2Periph_GPIOB

#define RADIO_GPIO_CLK            GPIO_Pin_8
#define RADIO_GPIO_CLK_PORT       GPIOA
#define RADIO_GPIO_CLK_PERIF      RCC_APB2Periph_GPIOA

#define RADIO_GPIO_CE             GPIO_Pin_10
#define RADIO_GPIO_CE_PORT        GPIOA
#define RADIO_GPIO_CE_PERIF       RCC_APB2Periph_GPIOA

#define RADIO_GPIO_IRQ            GPIO_Pin_9
#define RADIO_GPIO_IRQ_PORT       GPIOA
#define RADIO_GPIO_IRQ_PERIF      RCC_APB2Periph_GPIOA
#define RADIO_GPIO_IRQ_SRC_PORT   GPIO_PortSourceGPIOA
#define RADIO_GPIO_IRQ_SRC        GPIO_PinSource9
#define RADIO_GPIO_IRQ_LINE       EXTI_Line9

#define RADIO_SPI                 SPI2
#define RADIO_SPI_CLK             RCC_APB1Periph_SPI2
#define RADIO_GPIO_SPI_PORT       GPIOB
#define RADIO_GPIO_SPI_CLK        RCC_APB2Periph_GPIOB
#define RADIO_GPIO_SPI_SCK        GPIO_Pin_13
#define RADIO_GPIO_SPI_MISO       GPIO_Pin_14
#define RADIO_GPIO_SPI_MOSI       GPIO_Pin_15

#define DUMMY_BYTE    0xA5

/* nRF24L SPI commands */
#define CMD_R_REG              0x00
#define CMD_W_REG              0x20
#define CMD_R_RX_PAYLOAD       0x61
#define CMD_W_TX_PAYLOAD       0xA0
#define CMD_FLUSH_TX           0xE1
#define CMD_FLUSH_RX           0xE2
#define CMD_REUSE_TX_PL        0xE3
#define CMD_ACTIVATE           0x50
#define CMD_RX_PL_WID          0x60
#define CMD_W_ACK_PAYLOAD(P)  (0xA8|(P&0x0F))
#define CMD_W_PAYLOAD_NO_ACK   0xD0
#define CMD_NOP                0xFF

/* Usefull macro */
#define RADIO_EN_CS() GPIO_ResetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CS() GPIO_SetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CE() GPIO_ResetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define RADIO_EN_CE() GPIO_SetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define ACTIVATE_DATA   0x73

/* Private variables */
static bool isInit;
static void (*interruptCb)(void) = NULL;

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(RADIO_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(RADIO_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

/****************************************************************
 * nRF SPI commands, Every commands return the status byte      *
 ****************************************************************/

/* Read len bytes from a nRF24L register. 5 Bytes max */
unsigned char nrfReadReg(unsigned char address, char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte( CMD_R_REG | (address&0x1F) );
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=spiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

/* Write len bytes a nRF24L register. 5 Bytes max */
unsigned char nrfWriteReg(unsigned char address, char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the write command with the address */
  status = spiSendByte( CMD_W_REG | (address&0x1F) );
  /* Write LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

/* Write only one byte (useful for most of the reg.) */
unsigned char nrfWrite1Reg(unsigned char address, char byte)
{
  return nrfWriteReg(address, &byte, 1);
}

/* Read only one byte (useful for most of the reg.) */
unsigned char nrfRead1Reg(unsigned char address) {
  char byte;

  nrfReadReg(address, &byte, 1);

  return byte;
}

/* Sent the NOP command. Used to get the status byte */
unsigned char nrfNop()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_NOP);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfFlushRx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_FLUSH_RX);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfFlushTx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_FLUSH_TX);
  RADIO_DIS_CS();

  return status;
}

// Return the payload length
unsigned char nrfRxLength(unsigned int pipe)
{
  unsigned char length;

  RADIO_EN_CS();
  spiSendByte(CMD_RX_PL_WID);
  length = spiReceiveByte();
  RADIO_DIS_CS();

  return length;
}

unsigned char nrfActivate()
{
  unsigned char status;
  
  RADIO_EN_CS();
  status = spiSendByte(CMD_ACTIVATE);
  spiSendByte(ACTIVATE_DATA);
  RADIO_DIS_CS();

  return status;
}

// Write the ack payload of the pipe 0
unsigned char nrfWriteAck(unsigned int pipe, char *buffer, int len)
{
  unsigned char status;
  int i;

  ASSERT(pipe<6);

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_W_ACK_PAYLOAD(pipe));
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

// Read the RX payload
unsigned char nrfReadRX(char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_R_RX_PAYLOAD);
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=spiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

/* Interrupt service routine, call the interrupt callback */
void __attribute__((used)) EXTI9_Callback(void)
{
  if (interruptCb)
  {
    interruptCb();
  }
}

void nrfSetInterruptCallback(void (*cb)(void))
{
  interruptCb = cb;
}

void nrfSetChannel(unsigned int channel)
{
  if (channel<126)
    nrfWrite1Reg(REG_RF_CH, channel);
}

void nrfSetDatarate(int datarate)
{
  switch(datarate)
  {
    case RADIO_RATE_250K:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_250K);
      break;
    case RADIO_RATE_1M:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_1M);
      break;
    case RADIO_RATE_2M:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_2M);
      break;
  }  
}

void nrfSetAddress(unsigned int pipe, char* address)
{
  int len = 5;

  ASSERT(pipe<6);

  if (pipe > 1)
    len = 1;

  nrfWriteReg(REG_RX_ADDR_P0 + pipe, address, len);
}

void nrfSetEnable(bool enable)
{
  if (enable)
  {
    RADIO_EN_CE();
  } 
  else
  {
    RADIO_DIS_CE();
  }
}

unsigned char nrfGetStatus()
{
  return nrfNop();
}

/* Initialisation */
void nrfInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable SPI and GPIO clocks */
  RCC_APB2PeriphClockCmd(RADIO_GPIO_SPI_CLK | RADIO_GPIO_CS_PERIF | 
                         RADIO_GPIO_CE_PERIF | RADIO_GPIO_IRQ_PERIF, ENABLE);

  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(RADIO_SPI_CLK, ENABLE);

  /* Configure main clock */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CLK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_CLK_PORT, &GPIO_InitStructure);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_SCK |  RADIO_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CS_PORT, &GPIO_InitStructure);

  /* Configure the interruption (EXTI Source) */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_IRQ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(RADIO_GPIO_IRQ_PORT, &GPIO_InitStructure);

  GPIO_EXTILineConfig(RADIO_GPIO_IRQ_SRC_PORT, RADIO_GPIO_IRQ_SRC);
  EXTI_InitStructure.EXTI_Line = RADIO_GPIO_IRQ_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Clock the radio with 16MHz
  RCC_MCOConfig(RCC_MCO_HSE);

  /* disable the chip select */
  RADIO_DIS_CS();

  /* Configure I/O for the Chip Enable */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CE_PORT, &GPIO_InitStructure);

  /* disable the chip enable */
  RADIO_DIS_CE();

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(RADIO_SPI, &SPI_InitStructure);

  /* Enable the SPI  */
  SPI_Cmd(RADIO_SPI, ENABLE);
  
  isInit = true;
}

bool nrfTest(void)
{
  //TODO implement real tests!
  return isInit & extiTest();
}
