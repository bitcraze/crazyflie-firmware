/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 */

// Standard includes.
#include <string.h>
// Scheduler include files.
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "stm32fxxx.h"
// Application includes.
#include "i2c_drv.h"
#include "config.h"
#include "nvicconf.h"

//DEBUG
#include "usec_time.h"

#define I2C_TYPE                    I2C3
#define I2C_TYPE_EV_IRQn            I2C3_EV_IRQn
#define I2C_TYPE_ER_IRQn            I2C3_ER_IRQn
#define I2C_RCC                     RCC_APB1Periph_I2C3
#define I2C_GPIO_RCC_SCL            RCC_AHB1Periph_GPIOA
#define I2C_GPIO_PORT_SCL           GPIOA
#define I2C_GPIO_PIN_SCL            GPIO_Pin_8
#define I2C_GPIO_SOURCE_SCL         GPIO_PinSource8
#define I2C_GPIO_RCC_SDA            RCC_AHB1Periph_GPIOC
#define I2C_GPIO_PORT_SDA           GPIOC
#define I2C_GPIO_PIN_SDA            GPIO_Pin_9
#define I2C_GPIO_SOURCE_SDA         GPIO_PinSource9
#define I2C_GPIO_AF                 GPIO_AF_I2C3

// Misc constants.
#define I2C_NO_BLOCK				    0
#define I2C_QUEUE_LENGTH			  5
#define I2C_CLOCK_SPEED         400000
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2

// The I2C send message
volatile I2cMessage txMessage;
static uint32_t nbrOfretries = 0;

// Queue of messages that are waiting transmission.
static xQueueHandle xMessagesForTx;
// Flag to indicate the state of the I2C ISR state machine.
static bool isBusFree;
// DMA configuration structure used during transfer setup.
static DMA_InitTypeDef DMA_InitStructureShare;

uint32_t eventDebug[1024][2];
uint32_t eventDebugUnknown[1024];
uint32_t eventPos = 0;
uint32_t eventPosUnknown = 0;
uint32_t stops = 0;
uint32_t eventDMA = 0;

static void i2cDmaSetup(void);


#if 1
static void i2cStartTransfer(void)
{
  if (txMessage.direction == i2cRead /*&& txMessage.messageLength > 1*/)
  {
    DMA_InitStructureShare.DMA_BufferSize = txMessage.messageLength;
    DMA_InitStructureShare.DMA_Memory0BaseAddr = txMessage.buffer;
    DMA_Init(DMA1_Stream2, &DMA_InitStructureShare);
    DMA_Cmd(DMA1_Stream2, ENABLE);
  }

  I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, DISABLE);
  I2C_ITConfig(I2C_TYPE, I2C_IT_EVT, ENABLE);
  I2C_TYPE->CR1 = (I2C_CR1_START | I2C_CR1_PE);
}
#else
static void i2cStartTransfer(void)
{
  if (txMessage.direction == i2cRead /*&& txMessage.messageLength > 1*/)
  {
    /* Generate start with ack enable.
     * For some reason setting CR1 reg in sequence with
     * I2C_AcknowledgeConfig(I2C_TYPE, ENABLE) and after
     * I2C_GenerateSTART(I2C_TYPE, ENABLE) sometimes creates an
     * instant start->stop condition (3.9us long) which I found out with an I2C
     * analyzer. This fast start->stop is only possible to generate if both
     * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
     * at once with I2C_TYPE->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
     * problem is gone. Go figure...
     */
    I2C_TYPE->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE);
    /* Wait until SB flag is set */
    while (I2C_GetFlagStatus(I2C_TYPE, I2C_FLAG_SB) == RESET);
    /* Send slave address */
    I2C_Send7bitAddress(I2C_TYPE, txMessage.slaveAddress << 1, I2C_Direction_Receiver);
    /* Wait until ADDR flag is set */
    while (I2C_GetFlagStatus(I2C_TYPE, I2C_FLAG_ADDR) == RESET);
    /* SETUP DMA */
    DMA_InitStructureShare.DMA_BufferSize = txMessage.messageLength;
    DMA_InitStructureShare.DMA_Memory0BaseAddr = txMessage.buffer;
    DMA_Init(DMA1_Stream2, &DMA_InitStructureShare);
    DMA_Cmd(DMA1_Stream2, ENABLE);
    /* Enable Last DMA bit */
    I2C_DMALastTransferCmd(I2C_TYPE, ENABLE); // No repetitive start
    /* Start transfer */
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
    I2C_DMACmd(I2C_TYPE, ENABLE); // Enable before ADDR clear
    /* Clear ADDR */
    I2C_GetLastEvent(I2C_TYPE);
  }
}
#endif

//-----------------------------------------------------------
void i2cMessageTransfer(I2cMessage* message, portTickType xBlockTime)
{
  // Is the I2C interrupt in the middle of transmitting a message?
  if (isBusFree == true )
  {
    // No message is currently being sent or queued to be sent.  We
    // can start the ISR sending this message immediately.
    memcpy((char*)&txMessage, (char*)message, sizeof(I2cMessage));
    isBusFree = pdFALSE;

    i2cStartTransfer();
  }
  else
  {
    signed portBASE_TYPE xReturn;
    xReturn = xQueueSend(xMessagesForTx, (char*)message, xBlockTime);
    // We may have blocked while trying to queue the message.  If this
    // was the case then the interrupt would have been enabled and we may
    // now find that the I2C interrupt routine is no longer sending a
    // message.
    if ((isBusFree == true ) && (xReturn == pdPASS))
    {
      // Get the next message in the queue (this should be the
      // message we just posted) and start off the transmission
      // again.
      xQueueReceive(xMessagesForTx, (char*)&txMessage, I2C_NO_BLOCK);
      isBusFree = pdFALSE;
      i2cStartTransfer();
    }
  }
}


void i2cInit(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIOA clock
  RCC_AHB1PeriphClockCmd(I2C_GPIO_RCC_SDA, ENABLE);
  RCC_AHB1PeriphClockCmd(I2C_GPIO_RCC_SCL, ENABLE);
  // Enable I2C_TYPE clock
  RCC_APB1PeriphClockCmd(I2C_RCC, ENABLE);

  // Configure I2C_TYPE pins: SCL and SDA
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Pin = I2C_GPIO_PIN_SCL; // SCL
  GPIO_Init(I2C_GPIO_PORT_SCL, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  I2C_GPIO_PIN_SDA; // SDA
  GPIO_Init(I2C_GPIO_PORT_SDA, &GPIO_InitStructure);

  i2cdevUnlockBus(I2C_GPIO_PORT_SCL, I2C_GPIO_PORT_SDA, I2C_GPIO_PIN_SCL, I2C_GPIO_PIN_SDA);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = I2C_GPIO_PIN_SCL; // SCL
  GPIO_Init(I2C_GPIO_PORT_SCL, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  I2C_GPIO_PIN_SDA; // SDA
  GPIO_Init(I2C_GPIO_PORT_SDA, &GPIO_InitStructure);

  //Map gpios to alternate functions
  GPIO_PinAFConfig(I2C_GPIO_PORT_SCL, I2C_GPIO_SOURCE_SCL, I2C_GPIO_AF);
  GPIO_PinAFConfig(I2C_GPIO_PORT_SDA, I2C_GPIO_SOURCE_SDA, I2C_GPIO_AF);


  // I2C_TYPE configuration
  I2C_DeInit(I2C_TYPE);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK_SPEED;
  I2C_Init(I2C_TYPE, &I2C_InitStructure);

  // Enable I2C_TYPE error interrupts
  I2C_ITConfig(I2C_TYPE, I2C_IT_ERR, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = I2C_TYPE_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = I2C_TYPE_ER_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  i2cDmaSetup();

  initUsecTimer();

  // Create the queues used to hold Rx and Tx characters.
  xMessagesForTx = xQueueCreate(I2C_QUEUE_LENGTH, sizeof(I2cMessage));
  isBusFree = true;
}

static void i2cDmaSetup(void)
{

  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // I2C3 RX DMA Channel Config
  DMA_InitStructureShare.DMA_Channel = DMA_Channel_3;
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&I2C3->DR;
  DMA_InitStructureShare.DMA_Memory0BaseAddr = 0;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructureShare.DMA_BufferSize = 0;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructureShare.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShare.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void i2cQueueInternalAddress(uint16_t internalAddress)
{

}

void i2cCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      uint8_t  direction,
                      xQueueHandle queue,
                      uint32_t length,
                      uint8_t  *buffer)
{
  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = false;
  message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = buffer;
  message->clientQueue = queue;
  message->nbrOfRetries = I2C_MAX_RETRIES;
  message->ackDisableBeforeSR = false;
}

void i2cCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             uint8_t  direction,
                             xQueueHandle queue,
                             uint32_t length,
                             uint8_t  *buffer)
{
  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = IsInternal16;
  message->internalAddress = intAddress;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = buffer;
  message->clientQueue = queue;
  message->nbrOfRetries = I2C_MAX_RETRIES;
  message->ackDisableBeforeSR = false;
}

static bool i2cTryNextMessage(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  bool nextMessage = false;

  if (xQueueReceiveFromISR(xMessagesForTx, (void*)&txMessage,
      &xHigherPriorityTaskWoken) == pdTRUE)
  {
    i2cStartTransfer();
    nextMessage = true;
  }
  else
  {
   // No more messages were found to be waiting for
   // transaction so the bus is free.
   isBusFree = true;
//   I2C_GenerateSTOP(I2C_TYPE, ENABLE);                     // program the Stop
//   I2C_ITConfig(I2C_TYPE, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
  }

  return nextMessage;
}

static void i2cNotifyClient(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (txMessage.clientQueue != 0)
  {
    xQueueSendFromISR(txMessage.clientQueue, (void*)&txMessage,
                      &xHigherPriorityTaskWoken);
  }
}

void __attribute__((used)) I2C3_EV_IRQHandler(void)
{
  i2cEventIsrHandler();
}

static inline void i2cdevRoughLoopDelay(uint32_t us) __attribute__((optimize("O2")));
static inline void i2cdevRoughLoopDelay(uint32_t us)
{
  volatile uint32_t delay = 0;
  for(delay = 0; delay < 17 * us; ++delay) { };
}

uint16_t SR1, SR2;

void i2cEventIsrHandler(void)
{
  /* Holds the current transmission state. */
  static uint32_t messageIndex = 0;
  volatile uint32_t event;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;


  SR1 = I2C_TYPE->SR1;                                                 // read the status register here

  eventDebug[eventPos][0] = usecTimestamp();
  eventDebug[eventPos][1] = SR1;
  if (++eventPos == 1024)
  {
    eventPos = 0;
  }

  // Start bit event
  if (SR1 & I2C_SR1_SB)
  {
    messageIndex = 0;

    if(txMessage.direction == i2cWrite ||
       txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      I2C_Send7bitAddress(I2C_TYPE, txMessage.slaveAddress << 1, I2C_Direction_Transmitter);
    }
    else
    {
      I2C_AcknowledgeConfig(I2C_TYPE, ENABLE);
      I2C_Send7bitAddress(I2C_TYPE, txMessage.slaveAddress << 1, I2C_Direction_Receiver);
    }
  }
  // Address event with transmit empty
  else if (SR1 & I2C_SR1_ADDR)
  {
    if(txMessage.direction == i2cWrite ||
       txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      SR2 = I2C_TYPE->SR2;                               // clear ADDR

      if (txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
      {
        if (txMessage.isInternal16bit)
        {
          I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0xFF00) >> 8);
          I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0x00FF));
        }
        else
        {
          I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0x00FF));
        }
        txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
      }
      I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, ENABLE);        // allow us to have an EV7
    }
    else // Reading
    {
      if(txMessage.messageLength == 1)
      {
        I2C_AcknowledgeConfig(I2C_TYPE, DISABLE);
      }
      else
      {
        I2C_DMALastTransferCmd(I2C_TYPE, ENABLE); // No repetitive start
      }
      // Disable I2C interrupts
      I2C_ITConfig(I2C_TYPE, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
      // Enable the Transfer Complete interrupt
      DMA_ITConfig(DMA1_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
      I2C_DMACmd(I2C_TYPE, ENABLE); // Enable before ADDR clear

      __DMB();
      SR2 = I2C_TYPE->SR2;                               // clear ADDR
#if 0
      if(txMessage.messageLength == 1)
      {
        I2C_AcknowledgeConfig(I2C_TYPE, DISABLE);
        __DMB();
        SR2 = I2C_TYPE->SR2;                               // clear ADDR after ACK is turned off
        I2C_GenerateSTOP(I2C_TYPE, ENABLE);
        I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, ENABLE);        // allow us to have an EV7
      }
      else
      {
        I2C_ITConfig(I2C_TYPE, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
        // Enable the Transfer Complete interrupt
        DMA_ITConfig(DMA1_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
        I2C_DMALastTransferCmd(I2C_TYPE, ENABLE); // No repetitive start
        I2C_DMACmd(I2C_TYPE, ENABLE); // Enable before ADDR clear

        SR2 = I2C_TYPE->SR2;                                  // clear the ADDR here
      }
#endif
    }
  }
  // Byte transfer finished - EV7_2, EV7_3 or EV8_2
  else if (SR1 & I2C_SR1_BTF)
  {
    SR2 = I2C_TYPE->SR2;
    if (SR2 & I2C_SR2_TRA) // In write mode?
    {
      if (txMessage.direction == i2cRead) // internal address read
      {
        //I2C_TYPE->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
      }
      else
      {
        i2cNotifyClient();
        // Are there any other messages to transact?
        if (!i2cTryNextMessage())
        {
          I2C_GenerateSTOP(I2C_TYPE, ENABLE);                     // program the Stop
          I2C_ITConfig(I2C_TYPE, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
        }
      }
    }
    else // Reading
    {
      txMessage.buffer[messageIndex++] = I2C_ReceiveData(I2C_TYPE);
      if(messageIndex == txMessage.messageLength)
      {
        i2cNotifyClient();
        // Are there any other messages to transact?
        i2cTryNextMessage();
      }
    }
    while (I2C_TYPE->CR1 & 0x0100) { ; }
  }
  // Byte received - EV7
  else if (SR1 & I2C_SR1_RXNE)
  {
    txMessage.buffer[messageIndex++] = I2C_ReceiveData(I2C_TYPE);
    if(messageIndex == txMessage.messageLength)
    {
      I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, DISABLE);                // disable RXE to get BTF
    }
  }
  // Byte transmitted EV8 / EV8_1
  else if (SR1 & I2C_SR1_TXE)
  {
    if (txMessage.direction == i2cRead)
    {
      I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, DISABLE);
      /* Switch to read */
      I2C_TYPE->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
    }
    else
    {
      I2C_SendData(I2C_TYPE, txMessage.buffer[messageIndex++]);
      if(messageIndex == txMessage.messageLength)
      {
        I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush and get BTF
      }
    }
//    if(txMessage.internalAddress == I2C_NO_INTERNAL_ADDRESS)
//    {
//      I2C_SendData(I2C_TYPE, txMessage.buffer[messageIndex++]);
//      if(messageIndex == txMessage.messageLength)
//      {
//        I2C_ITConfig(I2C_TYPE, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush and get BTF
//      }
//    }
//    else
//    {
//      if (txMessage.isInternal16bit)
//      {
//        // FIXME: How to hande 16 bit reg
//        I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0xFF00) >> 8);
//      }
//      else
//      {
//        I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0x00FF));
//        txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
//        if (txMessage.direction == i2cRead)
//        {
//          /* Switch to read */
//          I2C_TYPE->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
//        }
//      }
  }
}

void __attribute__((used)) I2C1_ER_IRQHandler(void)
{
  i2cErrorIsrHandler();
}

void __attribute__((used)) I2C3_ER_IRQHandler(void)
{
  i2cErrorIsrHandler();
}

void __attribute__((used)) DMA1_Stream2_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2)) // Tranasfer complete
  {
    DMA_ClearITPendingBit(DMA1_Stream2, DMA_FLAG_TCIF2);
    DMA_Cmd(DMA1_Stream2, DISABLE);
    I2C_DMACmd(I2C_TYPE, DISABLE);
    I2C_GenerateSTOP(I2C_TYPE, ENABLE);
    while (I2C_TYPE->SR1 & I2C_CR1_STOP)
      ;
    I2C_DMALastTransferCmd(I2C_TYPE, DISABLE);
    I2C_AcknowledgeConfig(I2C_TYPE, DISABLE);

    i2cNotifyClient();
    // Are there any other messages to transact?
    i2cTryNextMessage();
  }
  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TEIF2)) // Transfer error
  {
    DMA_ClearITPendingBit(DMA1_Stream2, DMA_FLAG_TEIF2);
    //TODO: Implement error handling
  }
}

void i2cErrorIsrHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (I2C_GetFlagStatus(I2C_TYPE, I2C_FLAG_AF))
  {
    if(txMessage.nbrOfRetries-- > 0)
    {
      I2C_GenerateSTART(I2C_TYPE, ENABLE);
    }
    else
    {
      I2C_GenerateSTOP(I2C_TYPE, ENABLE);
      txMessage.status = i2cNack;
      i2cNotifyClient();
      i2cTryNextMessage();
    }
    I2C_ClearFlag(I2C_TYPE, I2C_FLAG_AF);
  }
  if (I2C_GetFlagStatus(I2C_TYPE, I2C_FLAG_BERR))
  {
      I2C_ClearFlag(I2C_TYPE, I2C_FLAG_BERR);
  }
  if (I2C_GetFlagStatus(I2C_TYPE, I2C_FLAG_OVR))
  {
      I2C_ClearFlag(I2C_TYPE, I2C_FLAG_OVR);
  }
  if (I2C_GetFlagStatus(I2C_TYPE, I2C_FLAG_ARLO))
  {
      I2C_ClearFlag(I2C_TYPE,I2C_FLAG_ARLO);
  }
}
