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
 *
 * i2cdev.c - Functions to write to I2C devices
 */

#include <stdint.h>
#include <stdbool.h>

#include "i2cdev.h"

#include "nvicconf.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32f10x.h"

#include "i2croutines.h"

#define I2C_TIMEOUT 5
#define I2CDEV_CLK_TS (1000000 / 100000)

#define I2CDEV_I2C1_PIN_SDA GPIO_Pin_7
#define I2CDEV_I2C1_PIN_SCL GPIO_Pin_6

#define I2CDEV_I2C2_PIN_SDA GPIO_Pin_11
#define I2CDEV_I2C2_PIN_SCL GPIO_Pin_10

#define GPIO_WAIT_LOW(gpio, pin, timeoutcycles)\
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && i--);\
  }

#define GPIO_WAIT_HIGH(gpio, pin, timeoutcycles) \
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_SET && i--);\
  }

xSemaphoreHandle i2cdevDmaEventI2c1;
xSemaphoreHandle i2cdevDmaEventI2c2;
/* Buffer of data to be received by I2C1 */
uint8_t* Buffer_Rx1;
/* Buffer of data to be transmitted by I2C1 */
uint8_t* Buffer_Tx1;
__IO uint32_t I2CDirection;

static void i2cdevResetAndLowLevelInitBusI2c1(void);
static void i2cdevResetAndLowLevelInitBusI2c2(void);
static inline void i2cdevRuffLoopDelay(uint32_t us);


int i2cdevInit(I2C_Dev *I2Cx)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  if (I2Cx == I2C1)
  {
    i2cdevResetAndLowLevelInitBusI2c1();
    // Enable the DMA1 channel6 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the DMA1 channel7 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the I2C error interrupt
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
    vSemaphoreCreateBinary(i2cdevDmaEventI2c1);
  }
  else if (I2Cx == I2C2)
  {
    i2cdevResetAndLowLevelInitBusI2c2();
    // Enable the DMA1 channel4 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the DMA1 channel5 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the I2C error interrupt
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
    vSemaphoreCreateBinary(i2cdevDmaEventI2c2);
  }
  else
  {
    return FALSE;
  }
  
  return TRUE;
}

bool i2cdevReadByte(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
  return i2cdevRead(I2Cx, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data)
{
  uint8_t byte;
  bool status;
  
  status = i2cdevRead(I2Cx, devAddress, memAddress, 1, &byte);
  *data = byte & (1 << bitNum);

  return status;
}

bool i2cdevReadBits(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(I2Cx, devAddress, memAddress, &byte)) == TRUE)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      byte &= mask;
      byte >>= (bitStart - length + 1);
      *data = byte;
  }
  return status;
}

bool i2cdevRead(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data)
{
  bool status = TRUE;

  if (memAddress != I2CDEV_NO_MEM_ADDR)
  {
    status = I2C_Master_BufferWrite(I2Cx, &memAddress,  1, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
  }
  if (status)
  {
    //TODO: Fix DMA transfer if more then 3 bytes
    status = I2C_Master_BufferRead(I2Cx, (uint8_t*)data,  len, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
  }

  return status;
}

bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
               uint16_t len, uint8_t *data)
{
  // TODO: Implement
  return false;
}

bool i2cdevWriteByte(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t data)
{
  return i2cdevWrite(I2Cx, devAddress, memAddress, 1, &data);
}

bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                   uint16_t len, uint8_t *data)
{
  // TODO: Implement
  return false;
}

bool i2cdevWriteBit(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(I2Cx, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(I2Cx, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(I2Cx, devAddress, memAddress, &byte)) == TRUE)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      data <<= (bitStart - length + 1); // shift data into correct position
      data &= mask; // zero all non-important bits in data
      byte &= ~(mask); // zero all important bits in existing byte
      byte |= data; // combine data with existing byte
      status = i2cdevWriteByte(I2Cx, devAddress, memAddress, byte);
  }

  return status;
}

bool i2cdevWrite(I2C_Dev *I2Cx, uint8_t devAddress, uint8_t memAddress,
                uint16_t len, uint8_t *data)
{
  bool status;
  static uint8_t buffer[17];
  int i;

  if (memAddress != I2CDEV_NO_MEM_ADDR)
  {
    // Sorry ...
    if (len > 16) len = 16;

    if(len == 0) return 0;

    buffer[0] = memAddress;
    for(i = 0; i < len ; i++)
      buffer[i + 1] = data[i];

    status = I2C_Master_BufferWrite(I2Cx, buffer,  len + 1, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
  }
  else
  {
    status = I2C_Master_BufferWrite(I2Cx, data,  len, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
  }

  return status;
}

static inline void i2cdevRuffLoopDelay(uint32_t us)
{
  volatile uint32_t delay;

  for(delay = I2CDEV_LOOPS_PER_US * us; delay > 0; delay--);
}

static void i2cdevResetAndLowLevelInitBusI2c1(void)
{
  /* Reset the I2C block */
  I2C_DeInit(I2C1);

  /* Make sure the bus is free by clocking it until any slaves release the bus. */
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* I2C1 SDA configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C1_PIN_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* I2C1 SCL configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C1_PIN_SCL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(GPIOB, I2CDEV_I2C1_PIN_SDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C1_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
  GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C1_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_HIGH(GPIOB, I2CDEV_I2C1_PIN_SDA, 10 * I2CDEV_LOOPS_PER_MS);

  /* Initialize the I2C block */
  I2C_LowLevel_Init(I2C1);

  /* Reset if I2C device is busy */
  if (I2C1->SR2 & 0x20)
  {
    /* Reset the I2C block */
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
  }
}

static void i2cdevResetAndLowLevelInitBusI2c2(void)
{
  /* Reset the I2C block */
  I2C_DeInit(I2C2);

  /* Make sure the bus is free by clocking it until any slaves release the bus. */
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* I2C1 SDA configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C2_PIN_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* I2C1 SCL configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C2_PIN_SCL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(GPIOB, I2CDEV_I2C2_PIN_SDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C2_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
  GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C2_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_HIGH(GPIOB, I2CDEV_I2C2_PIN_SDA, 10 * I2CDEV_LOOPS_PER_MS);

  /* Initialize the I2C block */
  I2C_LowLevel_Init(I2C2);

  /* Reset if I2C device is busy */
  if (I2C2->SR2 & 0x20)
  {
    /* Reset the I2C block */
    I2C_SoftwareResetCmd(I2C2, ENABLE);
    I2C_SoftwareResetCmd(I2C2, DISABLE);
  }
}

void i2cDmaInterruptHandlerI2c1(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC6))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC6);

    xSemaphoreGive(i2cdevDmaEventI2c1);
  }
  if(DMA_GetITStatus(DMA1_IT_TC7))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC7);

    xSemaphoreGive(i2cdevDmaEventI2c1);
  }
}

void i2cDmaInterruptHandlerI2c2(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC4))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC4);

    xSemaphoreGive(i2cdevDmaEventI2c2);
  }
  if(DMA_GetITStatus(DMA1_IT_TC5))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC5);

    xSemaphoreGive(i2cdevDmaEventI2c2);
  }
}
