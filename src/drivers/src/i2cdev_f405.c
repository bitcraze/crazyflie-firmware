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
#define DEBUG_MODULE "I2CDEV"

#include <stdint.h>
#include <stdbool.h>

#include "i2cdev.h"

#include "nvicconf.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "debug.h"
#include "cpal.h"
#include "cpal_i2c.h"

#define OWN_ADDRESS        0x74
#define I2C_CLOCK_SPEED    400000;

#define I2CDEV_CLK_TS (10 * I2CDEV_LOOPS_PER_US)

#define GPIO_WAIT_FOR_HIGH(gpio, pin, timeoutcycles)\
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && i--);\
  }

#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles) \
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_SET && i--);\
  }

CPAL_TransferTypeDef  rxTransfer, txTransfer;

xSemaphoreHandle i2cdevDmaEventI2c1;
xSemaphoreHandle i2cdevDmaEventI2c2;
xSemaphoreHandle i2cdevDmaEventI2c3;

/* Private functions */
static bool i2cdevWriteTransfer(I2C_Dev *dev);
static bool i2cdevReadTransfer(I2C_Dev *dev);
static inline void i2cdevRoughLoopDelay(uint32_t us) __attribute__((optimize("O2")));

#define SEMAPHORE_TIMEOUT M2T(30)
static void semaphoreGiveFromISR(xSemaphoreHandle semaphore);
static BaseType_t i2cDevTakeSemaphore(CPAL_DevTypeDef CPAL_Dev);
static void i2cDevGiveSemaphore(CPAL_DevTypeDef CPAL_Dev);
static xSemaphoreHandle getSemaphore(CPAL_DevTypeDef CPAL_Dev);

int i2cdevInit(I2C_Dev *dev)
{
  CPAL_I2C_StructInit(dev);
  dev->CPAL_Mode = CPAL_MODE_MASTER;
  //I2C_DevStructure.wCPAL_Options =  CPAL_OPT_NO_MEM_ADDR;
  dev->CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  dev->CPAL_Direction = CPAL_DIRECTION_TXRX;
  dev->pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_CLOCK_SPEED;
  dev->pCPAL_I2C_Struct->I2C_OwnAddress1 = OWN_ADDRESS;
  dev->pCPAL_TransferRx = &rxTransfer;
  dev->pCPAL_TransferTx = &txTransfer;
  /* Initialize CPAL device with the selected parameters */
  CPAL_I2C_Init(dev);

  // binary semaphores created using xSemaphoreCreateBinary() are created in a state
  // such that the the semaphore must first be 'given' before it can be 'taken'
  i2cdevDmaEventI2c1 = xSemaphoreCreateBinary();
  i2cdevDmaEventI2c2 = xSemaphoreCreateBinary();
  i2cdevDmaEventI2c3 = xSemaphoreCreateBinary();

  return true;
}

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
  return i2cdevRead(dev, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data)
{
  uint8_t byte;
  bool status;

  status = i2cdevRead(dev, devAddress, memAddress, 1, &byte);
  *data = byte & (1 << bitNum);

  return status;
}

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      byte &= mask;
      byte >>= (bitStart - length + 1);
      *data = byte;
  }
  return status;
}

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data)
{
  dev->pCPAL_TransferRx->wNumData = len;
  dev->pCPAL_TransferRx->pbBuffer = data;
  dev->pCPAL_TransferRx->wAddr1 = devAddress << 1;
  dev->pCPAL_TransferRx->wAddr2 = memAddress;

  dev->wCPAL_Options = CPAL_OPT_I2C_NOSTOP_MODE;
  if (memAddress == I2CDEV_NO_MEM_ADDR)
  {
    dev->wCPAL_Options |= CPAL_OPT_NO_MEM_ADDR;
  }

  return i2cdevReadTransfer(dev);
}

bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
               uint16_t len, uint8_t *data)
{
  dev->pCPAL_TransferRx->wNumData = len;
  dev->pCPAL_TransferRx->pbBuffer = data;
  dev->pCPAL_TransferRx->wAddr1 = devAddress << 1;
  dev->pCPAL_TransferRx->wAddr2 = memAddress;
  dev->wCPAL_Options = CPAL_OPT_I2C_NOSTOP_MODE | CPAL_OPT_16BIT_REG;

  return i2cdevReadTransfer(dev);
}

static bool i2cdevReadTransfer(I2C_Dev *dev)
{
  uint32_t status;

  dev->CPAL_Mode = CPAL_MODE_MASTER;
  /* Force the CPAL state to ready (in case a read operation has been initiated) */
  dev->CPAL_State = CPAL_STATE_READY;
  /* Start reading data in master mode */
  vTaskSuspendAll();
  // we need to suspend task switching for this command: the timeout callback is called
  // from this command, and if the stabilizer task is not the highest priority, then
  // I2C can often timeout
  status = CPAL_I2C_Read(dev);
  xTaskResumeAll();

  if (status == CPAL_PASS)
  {
    if (pdTRUE == i2cDevTakeSemaphore(dev->CPAL_Dev)) {
      return (dev->CPAL_State == CPAL_STATE_READY); // if the semaphore was taken successfully
    }
  }

  return false;
}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t data)
{
  return i2cdevWrite(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      data <<= (bitStart - length + 1); // shift data into correct position
      data &= mask; // zero all non-important bits in data
      byte &= ~(mask); // zero all important bits in existing byte
      byte |= data; // combine data with existing byte
      status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
  }

  return status;
}

bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                uint16_t len, uint8_t *data)
{
  dev->pCPAL_TransferTx->wNumData = len;
  dev->pCPAL_TransferTx->pbBuffer = data;
  dev->pCPAL_TransferTx->wAddr1 = devAddress << 1;
  dev->pCPAL_TransferTx->wAddr2 = memAddress;

  if (memAddress != I2CDEV_NO_MEM_ADDR)
  {
    dev->wCPAL_Options &= !CPAL_OPT_NO_MEM_ADDR;
  }
  else
  {
    dev->wCPAL_Options |= CPAL_OPT_NO_MEM_ADDR;
  }

  return i2cdevWriteTransfer(dev);
}

bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                   uint16_t len, uint8_t *data)
{
  dev->pCPAL_TransferTx->wNumData = len;
  dev->pCPAL_TransferTx->pbBuffer = data;
  dev->pCPAL_TransferTx->wAddr1 = devAddress << 1;
  dev->pCPAL_TransferTx->wAddr2 = memAddress;
  dev->wCPAL_Options = CPAL_OPT_I2C_NOSTOP_MODE | CPAL_OPT_16BIT_REG;

  return i2cdevWriteTransfer(dev);
}

static bool i2cdevWriteTransfer(I2C_Dev *dev)
{
  bool status;

  dev->CPAL_Mode = CPAL_MODE_MASTER;
  /* Force the CPAL state to ready (in case a read operation has been initiated) */
  dev->CPAL_State = CPAL_STATE_READY;
  /* Start writing data in master mode */
  vTaskSuspendAll();
  // we need to suspend task switching for this command: the timeout callback is called
  // from this command, and if the stabilizer task is not the highest priority, then
  // I2C can often timeout
  status = CPAL_I2C_Write(dev);
  xTaskResumeAll();

  if (status == CPAL_PASS)
  {
    if (pdTRUE == i2cDevTakeSemaphore(dev->CPAL_Dev)) {
      return (dev->CPAL_State == CPAL_STATE_READY); // if the semaphore was taken successfully
    }
  }

  return false;
}

static inline void i2cdevRoughLoopDelay(uint32_t us)
{
  volatile uint32_t delay = 0;
  for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}

void i2cdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
  GPIO_SetBits(portSDA, pinSDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(portSCL, pinSCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdevRoughLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(portSCL, pinSCL);
    i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(portSCL, pinSCL);
    i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(portSCL, pinSCL);
  i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(portSDA, pinSDA);
  i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(portSDA, pinSDA);
  i2cdevRoughLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(portSDA, pinSDA);
  GPIO_SetBits(portSCL, pinSCL);
  GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, 10 * I2CDEV_LOOPS_PER_MS);
}


static void semaphoreGiveFromISR(xSemaphoreHandle semaphore)
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken)
  {
   portYIELD();
  }
}


static xSemaphoreHandle getSemaphore(CPAL_DevTypeDef CPAL_Dev) {
  xSemaphoreHandle result = NULL;

  if (CPAL_Dev == (I2C1_DEV)->CPAL_Dev) {
    result = i2cdevDmaEventI2c1;
  } else if (CPAL_Dev == (I2C2_DEV)->CPAL_Dev) {
    result = i2cdevDmaEventI2c2;
  } else if (CPAL_Dev == (I2C3_DEV)->CPAL_Dev) {
    result = i2cdevDmaEventI2c3;
  } else {
    ASSERT_FAILED();
  }

  return result;
}


static BaseType_t i2cDevTakeSemaphore(CPAL_DevTypeDef CPAL_Dev) {
  xSemaphoreHandle semaphore = getSemaphore(CPAL_Dev);
  return xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT);
}

static void i2cDevGiveSemaphore(CPAL_DevTypeDef CPAL_Dev)
{
  xSemaphoreHandle semaphore = getSemaphore(CPAL_Dev);
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    semaphoreGiveFromISR(semaphore);
  } else {
    xSemaphoreGive(semaphore);
  }
}

/**
  * @brief  User callback that manages the I2C device errors.
  * @note   Make sure that the define USE_SINGLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInitStruct.
  * @param  DeviceError.
  * @retval None
  */
void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)
{
  i2cDevGiveSemaphore(pDevInstance);
}

/**
  * @brief  User callback that manages the Timeout error.
  * NOTE: This method is called from both interrupts and tasks!
  * @param  pDevInitStruct .
  * @retval None.
  */
uint32_t CPAL_TIMEOUT_UserCallback(CPAL_InitTypeDef* pDevInitStruct) {
  return CPAL_FAIL;
}

/**
  * @brief  Manages the End of Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
  i2cDevGiveSemaphore(pDevInitStruct->CPAL_Dev);
}

/**
  * @brief  Manages the End of Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
  i2cDevGiveSemaphore(pDevInitStruct->CPAL_Dev);
}
