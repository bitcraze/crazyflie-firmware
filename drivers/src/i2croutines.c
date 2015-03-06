/**
 ******************************************************************************
 * @file OptimizedI2Cexamples/src/I2CRoutines.c
 * @author  MCD Application Team
 * @version  V4.0.0
 * @date  06/18/2010
 * @brief  Contains the I2Cx slave/Master read and write routines.
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32fxxx.h"
#include "i2croutines.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "i2cdev.h"
#include "nvicconf.h"

extern xSemaphoreHandle i2cdevDmaEventI2c1;
extern xSemaphoreHandle i2cdevDmaEventI2c2;
/* Buffer of data to be received by I2C1, I2C2 */
uint8_t* Buffer_Rx1;
uint8_t* Buffer_Tx1;
/* Buffer of data to be transmitted by I2C1, I2C2 */
uint8_t* Buffer_Rx2;
uint8_t* Buffer_Tx2;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DMA_InitTypeDef I2CDMA_InitStructure;
__IO uint32_t I2CDirection1 = I2C_DIRECTION_TX;
__IO uint32_t I2CDirection2 = I2C_DIRECTION_TX;
__IO uint32_t NumbOfBytes1;
__IO uint32_t NumbOfBytes2;
__IO uint8_t Address1;
__IO uint8_t Address2;
__IO uint8_t Tx_Idx1 = 0, Rx_Idx1 = 0;
__IO uint8_t Tx_Idx2 = 0, Rx_Idx2 = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Reads buffer of bytes  from the slave.
 * @param pBuffer: Buffer of bytes to be read from the slave.
 * @param NumByteToRead: Number of bytes to be read by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
ErrorStatus I2C_Master_BufferRead(I2C_TypeDef* I2Cx, uint8_t* pBuffer,
    uint32_t NumByteToRead, I2C_ProgrammingModel Mode, uint8_t SlaveAddress,
    uint32_t timeoutMs)

{
  __IO uint32_t temp = 0;
  __IO uint32_t Timeout = 0;

  /* Enable I2C errors interrupts (used in all modes: Polling, DMA and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;

  if (Mode == DMA) /* I2Cx Master Reception using DMA */
  {
    /* Configure I2Cx DMA channel */
    I2C_DMAConfig(I2Cx, pBuffer, NumByteToRead, I2C_DIRECTION_RX);
    /* Set Last bit to have a NACK on the last received byte */
    I2Cx->CR2 |= CR2_LAST_Set;
    /* Enable I2C DMA requests */
    I2Cx->CR2 |= CR2_DMAEN_Set;
    Timeout = 0xFFFF;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    /* Wait until SB flag is set: EV5  */
    while ((I2Cx->SR1 & 0x0001) != 0x0001)
    {
      if (Timeout-- == 0)
        return ERROR;
    }
    Timeout = 0xFFFF;
    /* Send slave address */
    /* Set the address bit0 for read */
    SlaveAddress |= OAR1_ADD0_Set;
    I2Cx->DR = SlaveAddress;
    /* Wait until ADDR is set: EV6 */
    while ((I2Cx->SR1 & 0x0002) != 0x0002)
    {
      if (Timeout-- == 0)
        return ERROR;
    }
    /* Clear ADDR flag by reading SR2 register */
    temp = I2Cx->SR2;
    if (I2Cx == I2C1)
    {
      /* Wait until DMA end of transfer */
      //while (!DMA_GetFlagStatus(DMA1_FLAG_TC7));
      xSemaphoreTake(i2cdevDmaEventI2c1, M2T(timeoutMs));
      /* Disable DMA Channel */
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, DISABLE);
      /* Clear the DMA Transfer Complete flag */
      DMA_ClearFlag(DMA1_FLAG_TC7);
    }
    else /* I2Cx = I2C2*/
    {
      /* Wait until DMA end of transfer */
      //while (!DMA_GetFlagStatus(DMA1_FLAG_TC5))
      xSemaphoreTake(i2cdevDmaEventI2c2, M2T(timeoutMs));
      /* Disable DMA Channel */
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
      /* Clear the DMA Transfer Complete flag */
      DMA_ClearFlag(DMA1_FLAG_TC5);
    }
    /* Program the STOP */
    I2Cx->CR1 |= CR1_STOP_Set;
    /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
    while ((I2Cx->CR1 & 0x200) == 0x200)
      ;
  }
  /* I2Cx Master Reception using Interrupts with highest priority in an application */
  else
  {
    /* Enable EVT IT*/
    I2Cx->CR2 |= I2C_IT_EVT;
    /* Enable BUF IT */
    I2Cx->CR2 |= I2C_IT_BUF;
    SlaveAddress |= OAR1_ADD0_Set;
    if (I2Cx== I2C1)
    {
      /* Set the I2C direction to reception */
      I2CDirection1 = I2C_DIRECTION_RX;
      Buffer_Rx1 = pBuffer;
      Address1 = SlaveAddress;
      NumbOfBytes1 = NumByteToRead;
    }
    else
    {
      /* Set the I2C direction to reception */
      I2CDirection2 = I2C_DIRECTION_RX;
      Buffer_Rx2 = pBuffer;
      Address2 = SlaveAddress;
      NumbOfBytes2 = NumByteToRead;
    }
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    Timeout = timeoutMs * I2CDEV_LOOPS_PER_MS;
    /* Wait until the START condition is generated on the bus: START bit is cleared by hardware */
    while ((I2Cx->CR1 & 0x100) == 0x100 && Timeout)
    {
      Timeout--;
    }
    /* Wait until BUSY flag is reset (until a STOP is generated) */
    while ((I2Cx->SR2 & 0x0002) == 0x0002 && Timeout)
    {
      Timeout--;
    }
    /* Enable Acknowledgement to be ready for another reception */
    I2Cx->CR1 |= CR1_ACK_Set;

    if (Timeout == 0)
      return ERROR;
  }

  return SUCCESS;

  temp++; //To avoid GCC warning!
}

/**
 * @brief  Writes buffer of bytes.
 * @param pBuffer: Buffer of bytes to be sent to the slave.
 * @param NumByteToWrite: Number of bytes to be sent by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
ErrorStatus I2C_Master_BufferWrite(I2C_TypeDef* I2Cx, uint8_t* pBuffer,
    uint32_t NumByteToWrite, I2C_ProgrammingModel Mode, uint8_t SlaveAddress,
    uint32_t timeoutMs)
{

  __IO uint32_t temp = 0;
  __IO uint32_t Timeout = 0;

  /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;
  if (Mode == DMA) /* I2Cx Master Transmission using DMA */
  {
    Timeout = 0xFFFF;
    /* Configure the DMA channel for I2Cx transmission */
    I2C_DMAConfig(I2Cx, pBuffer, NumByteToWrite, I2C_DIRECTION_TX);
    /* Enable the I2Cx DMA requests */
    I2Cx->CR2 |= CR2_DMAEN_Set;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    /* Wait until SB flag is set: EV5 */
    while ((I2Cx->SR1 & 0x0001) != 0x0001)
    {
      if (Timeout-- == 0)
        return ERROR;
    }
    Timeout = 0xFFFF;
    /* Send slave address */
    /* Reset the address bit0 for write */
    SlaveAddress &= OAR1_ADD0_Reset;
    I2Cx->DR = SlaveAddress;
    /* Wait until ADDR is set: EV6 */
    while ((I2Cx->SR1 & 0x0002) != 0x0002)
    {
      if (Timeout-- == 0)
        return ERROR;
    }

    /* Clear ADDR flag by reading SR2 register */
    temp = I2Cx->SR2;
    if (I2Cx == I2C1)
    {
      /* Wait until DMA end of transfer */
//            while (!DMA_GetFlagStatus(DMA1_FLAG_TC6));
      xSemaphoreTake(i2cdevDmaEventI2c1, M2T(5));
      /* Disable the DMA1 Channel 6 */
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, DISABLE);
      /* Clear the DMA Transfer complete flag */
      DMA_ClearFlag(DMA1_FLAG_TC6);
    }
    else /* I2Cx = I2C2 */
    {
      /* Wait until DMA end of transfer */
      //while (!DMA_GetFlagStatus(DMA1_FLAG_TC4))
      xSemaphoreTake(i2cdevDmaEventI2c2, M2T(5));
      /* Disable the DMA1 Channel 4 */
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
      /* Clear the DMA Transfer complete flag */
      DMA_ClearFlag(DMA1_FLAG_TC4);
    }

    /* EV8_2: Wait until BTF is set before programming the STOP */
    while ((I2Cx->SR1 & 0x00004) != 0x000004)
      ;
    /* Program the STOP */
    I2Cx->CR1 |= CR1_STOP_Set;
    /* Make sure that the STOP bit is cleared by Hardware */
    while ((I2Cx->CR1 & 0x200) == 0x200)
      ;
  }
  /* I2Cx Master Transmission using Interrupt with highest priority in the application */
  else
  {
    /* Enable EVT IT*/
    I2Cx->CR2 |= I2C_IT_EVT;
    /* Enable BUF IT */
    I2Cx->CR2 |= I2C_IT_BUF;
    /* Set the I2C direction to Transmission */
    SlaveAddress &= OAR1_ADD0_Reset;
    if (I2Cx== I2C1)
    {
      /* Set the I2C direction to reception */
      I2CDirection1 = I2C_DIRECTION_TX;
      Buffer_Tx1 = pBuffer;
      Address1 = SlaveAddress;
      NumbOfBytes1 = NumByteToWrite;
    }
    else
    {
      /* Set the I2C direction to reception */
      I2CDirection2 = I2C_DIRECTION_TX;
      Buffer_Tx2 = pBuffer;
      Address2 = SlaveAddress;
      NumbOfBytes2 = NumByteToWrite;
    }
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    Timeout = timeoutMs * I2CDEV_LOOPS_PER_MS;
    /* Wait until the START condition is generated on the bus: the START bit is cleared by hardware */
    while ((I2Cx->CR1 & 0x100) == 0x100 && Timeout)
    {
      Timeout--;
    }
    /* Wait until BUSY flag is reset: a STOP has been generated on the bus signaling the end
     of transmission */
    while ((I2Cx->SR2 & 0x0002) == 0x0002 && Timeout)
    {
      Timeout--;
    }

    if (Timeout == 0)
      return ERROR;
  }
  return SUCCESS;

  temp++; //To avoid GCC warning!
}

/**
 * @brief Prepares the I2Cx slave for transmission.
 * @param I2Cx: I2C1 or I2C2.
 * @param Mode: DMA or Interrupt having the highest priority in the application.
 * @retval : None.
 */
void I2C_Slave_BufferReadWrite(I2C_TypeDef* I2Cx, I2C_ProgrammingModel Mode)

{
  /* Enable Event IT needed for ADDR and STOPF events ITs */
  I2Cx->CR2 |= I2C_IT_EVT;
  /* Enable Error IT */
  I2Cx->CR2 |= I2C_IT_ERR;

  if (Mode == DMA) /* I2Cx Slave Transmission using DMA */
  {
    /* Enable I2Cx DMA requests */
    I2Cx->CR2 |= CR2_DMAEN_Set;
  }
  else /* I2Cx Slave Transmission using Interrupt with highest priority in the application */
  {
    /* Enable Buffer IT (TXE and RXNE ITs) */
    I2Cx->CR2 |= I2C_IT_BUF;
  }
}

/**
 * @brief  Initializes peripherals: I2Cx, GPIO, DMA channels .
 * @param  None
 * @retval None
 */
void I2C_LowLevel_Init(I2C_TypeDef* I2Cx)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Enable the DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  if (I2Cx == I2C1)
  {
    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Release I2C1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI + 1;
    NVIC_Init(&NVIC_InitStructure);
  }
  else /* I2Cx = I2C2 */
  {

    /* I2C2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C2 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* Release I2C2 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI + 1;
    NVIC_Init(&NVIC_InitStructure);
  }

  /* I2C1 and I2C2 configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = OwnAddress1;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
  I2C_Init(I2C1, &I2C_InitStructure);
  I2C_InitStructure.I2C_OwnAddress1 = OwnAddress2;
  I2C_Init(I2C2, &I2C_InitStructure);

  if (I2Cx == I2C1)

  { /* I2C1 TX DMA Channel configuration */
    DMA_DeInit(I2C1_DMA_CHANNEL_TX);
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C1_DR_Address;
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) 0; /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF; /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);

    /* I2C1 RX DMA Channel configuration */
    DMA_DeInit(I2C1_DMA_CHANNEL_RX);
    DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
  }

  else /* I2Cx = I2C2 */

  {
    /* I2C2 TX DMA Channel configuration */
    DMA_DeInit(I2C2_DMA_CHANNEL_TX);
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C2_DR_Address;
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) 0; /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF; /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);

    /* I2C2 RX DMA Channel configuration */
    DMA_DeInit(I2C2_DMA_CHANNEL_RX);
    DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);

  }
}

/**
 * @brief  Initializes DMA channel used by the I2C Write/read routines.
 * @param  None.
 * @retval None.
 */
void I2C_DMAConfig(I2C_TypeDef* I2Cx, uint8_t* pBuffer, uint32_t BufferSize,
    uint32_t Direction)
{
  /* Initialize the DMA with the new parameters */
  if (Direction == I2C_DIRECTION_TX)
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) pBuffer;
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    I2CDMA_InitStructure.DMA_BufferSize = (uint32_t) BufferSize;

    if (I2Cx == I2C1)
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C1_DR_Address;
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, DISABLE);
      DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, ENABLE);
    }
    else
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C2_DR_Address;
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
      DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
      DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, ENABLE);
    }
  }
  else /* Reception */
  {
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) pBuffer;
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    I2CDMA_InitStructure.DMA_BufferSize = (uint32_t) BufferSize;
    if (I2Cx == I2C1)
    {

      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C1_DR_Address;
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, DISABLE);
      DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, ENABLE);
    }

    else
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C2_DR_Address;
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
      DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, ENABLE);
    }

  }
}

/**
 * @brief  This function handles I2C Event interrupt request.
 * @param  None
 * @retval : None
 */
void __attribute__((used)) I2C1_EV_IRQHandler(void)
{

  __IO uint32_t SR1Register = 0;
  __IO uint32_t SR2Register = 0;

  /* Read the I2C SR1 and SR2 status registers */
  SR1Register = I2C1->SR1;
  SR2Register = I2C1->SR2;

  /* If SB = 1, I2C master sent a START on the bus: EV5) */
  if ((SR1Register & 0x0001) == 0x0001)
  {
    /* Send the slave address for transmssion or for reception (according to the configured value
     in the write master write routine */
    I2C1->DR = Address1;
    SR1Register = 0;
    SR2Register = 0;
  }
  /* If I2C is Master (MSL flag = 1) */

  if ((SR2Register & 0x0001) == 0x0001)
  {
    /* If ADDR = 1, EV6 */
    if ((SR1Register & 0x0002) == 0x0002)
    {
      /* Write the first data in case the Master is Transmitter */
      if (I2CDirection1 == I2C_DIRECTION_TX)
      {
        /* Initialize the Transmit counter */
        Tx_Idx1 = 0;
        /* Write the first data in the data register */
        I2C1->DR = Buffer_Tx1[Tx_Idx1++];
        /* Decrement the number of bytes to be written */
        NumbOfBytes1--;
        /* If no further data to be sent, disable the I2C BUF IT
         in order to not have a TxE  interrupt */
        if (NumbOfBytes1 == 0)
        {
          I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      /* Master Receiver */
      else
      {
        /* Initialize Receive counter */
        Rx_Idx1 = 0;
        /* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
        /* EV6_1: used for single byte reception. The ACK disable and the STOP
         Programming should be done just after ADDR is cleared. */
        if (NumbOfBytes1 == 1)
        {
          /* Clear ACK */
          I2C1->CR1 &= CR1_ACK_Reset;
          /* Program the STOP */
          I2C1->CR1 |= CR1_STOP_Set;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* Master transmits the remaing data: from data2 until the last one.  */
    /* If TXE is set */
    if ((SR1Register & 0x0084) == 0x0080)
    {
      /* If there is still data to write */
      if (NumbOfBytes1 != 0)
      {
        /* Write the data in DR register */
        I2C1->DR = Buffer_Tx1[Tx_Idx1++];
        /* Decrment the number of data to be written */
        NumbOfBytes1--;
        /* If  no data remains to write, disable the BUF IT in order
         to not have again a TxE interrupt. */
        if (NumbOfBytes1 == 0)
        {
          /* Disable the BUF IT */
          I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If BTF and TXE are set (EV8_2), program the STOP */
    if ((SR1Register & 0x0084) == 0x0084)
    {
      /* Program the STOP */
      I2C1->CR1 |= CR1_STOP_Set;
      /* Disable EVT IT In order to not have again a BTF IT */
      I2C1->CR2 &= (uint16_t) ~I2C_IT_EVT;
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If RXNE is set */
    if ((SR1Register & 0x0040) == 0x0040)
    {
      /* Read the data register */
      Buffer_Rx1[Rx_Idx1++] = I2C1->DR;
      /* Decrement the number of bytes to be read */
      NumbOfBytes1--;
      /* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
      if (NumbOfBytes1 == 1)
      {
        /* Clear ACK */
        I2C1->CR1 &= CR1_ACK_Reset;
        /* Program the STOP */
        I2C1->CR1 |= CR1_STOP_Set;
      }
      SR1Register = 0;
      SR2Register = 0;
    }
  }
}

/**
 * @brief  This function handles I2C Event interrupt request.
 * @param  None
 * @retval : None
 */
void __attribute__((used)) I2C2_EV_IRQHandler(void)
{
  __IO uint32_t SR1Register = 0;
  __IO uint32_t SR2Register = 0;

  /* Read the I2C SR1 and SR2 status registers */
  SR1Register = I2C2->SR1;
  SR2Register = I2C2->SR2;

  /* If SB = 1, I2C master sent a START on the bus: EV5) */
  if ((SR1Register & 0x0001) == 0x0001)
  {
    /* Send the slave address for transmssion or for reception (according to the configured value
     in the write master write routine */
    I2C2->DR = Address2;
    SR1Register = 0;
    SR2Register = 0;
  }
  /* If I2C is Master (MSL flag = 1) */

  if ((SR2Register & 0x0001) == 0x0001)
  {
    /* If ADDR = 1, EV6 */
    if ((SR1Register & 0x0002) == 0x0002)
    {
      /* Write the first data in case the Master is Transmitter */
      if (I2CDirection2 == I2C_DIRECTION_TX)
      {
        /* Initialize the Transmit counter */
        Tx_Idx2 = 0;
        /* Write the first data in the data register */
        I2C2->DR = Buffer_Tx2[Tx_Idx2++];
        /* Decrement the number of bytes to be written */
        NumbOfBytes2--;
        /* If no further data to be sent, disable the I2C BUF IT
         in order to not have a TxE  interrupt */
        if (NumbOfBytes2 == 0)
        {
          I2C2->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      /* Master Receiver */
      else
      {
        /* Initialize Receive counter */
        Rx_Idx2 = 0;
        /* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
        /* EV6_1: used for single byte reception. The ACK disable and the STOP
         Programming should be done just after ADDR is cleared. */
        if (NumbOfBytes2 == 1)
        {
          /* Clear ACK */
          I2C2->CR1 &= CR1_ACK_Reset;
          /* Program the STOP */
          I2C2->CR1 |= CR1_STOP_Set;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* Master transmits the remaing data: from data2 until the last one.  */
    /* If TXE is set */
    if ((SR1Register & 0x0084) == 0x0080)
    {
      /* If there is still data to write */
      if (NumbOfBytes2 != 0)
      {
        /* Write the data in DR register */
        I2C2->DR = Buffer_Tx2[Tx_Idx2++];
        /* Decrment the number of data to be written */
        NumbOfBytes2--;
        /* If  no data remains to write, disable the BUF IT in order
         to not have again a TxE interrupt. */
        if (NumbOfBytes2 == 0)
        {
          /* Disable the BUF IT */
          I2C2->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If BTF and TXE are set (EV8_2), program the STOP */
    if ((SR1Register & 0x0084) == 0x0084)
    {
      /* Program the STOP */
      I2C2->CR1 |= CR1_STOP_Set;
      /* Disable EVT IT In order to not have again a BTF IT */
      I2C2->CR2 &= (uint16_t) ~I2C_IT_EVT;
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If RXNE is set */
    if ((SR1Register & 0x0040) == 0x0040)
    {
      /* Read the data register */
      Buffer_Rx2[Rx_Idx2++] = I2C2->DR;
      /* Decrement the number of bytes to be read */
      NumbOfBytes2--;
      /* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
      if (NumbOfBytes2 == 1)
      {
        /* Clear ACK */
        I2C2->CR1 &= CR1_ACK_Reset;
        /* Program the STOP */
        I2C2->CR1 |= CR1_STOP_Set;
      }
      SR1Register = 0;
      SR2Register = 0;
    }
  }
}

void __attribute__((used)) I2C1_ER_IRQHandler(void)
{
  i2cErrorInterruptHandler(I2C1);
}

void __attribute__((used)) I2C2_ER_IRQHandler(void)
{
  i2cErrorInterruptHandler(I2C2);
}

/**
 * @brief  This function handles I2C Error interrupt request.
 * @param  None
 * @retval : None
 */
void i2cErrorInterruptHandler(I2C_TypeDef* I2Cx)
{

  __IO uint32_t SR1Register = 0;

  /* Read the I2C status register */
  SR1Register = I2Cx->SR1;
  /* If AF = 1 */
  if ((SR1Register & 0x0400) == 0x0400)
  {
    I2Cx->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If ARLO = 1 */
  if ((SR1Register & 0x0200) == 0x0200)
  {
    I2Cx->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If BERR = 1 */
  if ((SR1Register & 0x0100) == 0x0100)
  {
    I2Cx->SR1 &= 0xFEFF;
    SR1Register = 0;
  }
  /* If OVR = 1 */
  if ((SR1Register & 0x0800) == 0x0800)
  {
    I2Cx->SR1 &= 0xF7FF;
    SR1Register = 0;
  }
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
