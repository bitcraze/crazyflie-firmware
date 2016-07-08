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

#define I2C_TYPE                    I2C3
#define I2C_TYPE_EV_IRQn            I2C3_EV_IRQn
#define I2C_TYPE_ER_IRQn            I2C3_ER_IRQn
#define I2C_RCC                     RCC_APB1Periph_I2C3
#define I2C_GPIO_RCC_SDA            RCC_AHB1Periph_GPIOA
#define I2C_GPIO_PORT_SDA           GPIOA
#define I2C_GPIO_PIN_SDA            GPIO_Pin_8
#define I2C_GPIO_SOURCE_SDA         GPIO_PinSource8
#define I2C_GPIO_RCC_SCL            RCC_AHB1Periph_GPIOC
#define I2C_GPIO_PORT_SCL           GPIOC
#define I2C_GPIO_PIN_SCL            GPIO_Pin_9
#define I2C_GPIO_SOURCE_SCL         GPIO_PinSource9
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

uint32_t eventDebug[21];
uint32_t eventPos = 0;
uint32_t stops = 0;

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
    I2C_ITConfig(I2C_TYPE, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
    I2C_GenerateSTART(I2C_TYPE, ENABLE);
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
      I2C_GenerateSTART(I2C_TYPE, ENABLE);
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Pin = I2C_GPIO_PIN_SCL; // SCL
  GPIO_Init(I2C_GPIO_PORT_SCL, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  I2C_GPIO_PIN_SDA; // SDA
  GPIO_Init(I2C_GPIO_PORT_SDA, &GPIO_InitStructure);

  //Map gpios to alternate functions
  GPIO_PinAFConfig(I2C_GPIO_PORT_SCL, I2C_GPIO_SOURCE_SCL, I2C_GPIO_AF);
  GPIO_PinAFConfig(I2C_GPIO_PORT_SDA, I2C_GPIO_SOURCE_SDA, I2C_GPIO_AF);

 // i2cdevUnlockBus(I2C_GPIO_PORT_SCL, I2C_GPIO_PORT_SDA, I2C_GPIO_PIN_SCL, I2C_GPIO_PIN_SDA);

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

  // Create the queues used to hold Rx and Tx characters.
  xMessagesForTx = xQueueCreate(I2C_QUEUE_LENGTH, sizeof(I2cMessage));
  isBusFree = true;
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
}

static void i2cTryNextMessage(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (xQueueReceiveFromISR(xMessagesForTx, (void*)&txMessage,
      &xHigherPriorityTaskWoken) == pdTRUE)
  {
    I2C_GenerateSTART(I2C_TYPE, ENABLE);
  }
  else
  {
   // No more messages were found to be waiting for
   // transaction so the bus is free.
   isBusFree = true;
  }
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

void i2cEventIsrHandler(void)
{
  /* Holds the current transmission state. */
  static uint32_t messageIndex = 0;
  volatile uint32_t event;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  event = I2C_GetLastEvent(I2C_TYPE);
  switch (event)
  {
    case I2C_EVENT_MASTER_MODE_SELECT:                 // EV5
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
      messageIndex = 0;
      break;

    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
      // Send the first data
      if(txMessage.internalAddress == I2C_NO_INTERNAL_ADDRESS)
      {
        I2C_SendData(I2C_TYPE, txMessage.buffer[messageIndex++]);
      }
      else
      {
        if (txMessage.isInternal16bit)
        {
          I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0xFF00) >> 8);
        }
        else
        {
          I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0x00FF));
        }
      }
      break;

    /* Test on I2C_TYPE EV8 and clear it */
    case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */
      if(txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
      {
        if (txMessage.isInternal16bit)
        {
          // Send second byte if 16 bit internal address
          I2C_SendData(I2C_TYPE, (txMessage.internalAddress & 0x00FF));
        }
        txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
      }
      else
      {
        if(messageIndex < (txMessage.messageLength) &&
           txMessage.direction == i2cWrite)
        {
          // Transmit I2C_TYPE data
          I2C_SendData(I2C_TYPE, txMessage.buffer[messageIndex++]);
        }
        else
        {
          I2C_GenerateSTOP(I2C_TYPE, ENABLE);
        }
      }
      break;

    case I2C_EVENT_MASTER_BYTE_TRANSMITTED: // With BTF EV8-2
      if (txMessage.direction == i2cRead) // internal address read
      {
        I2C_GenerateSTART(I2C_TYPE, ENABLE);
      }
      else
      {
        i2cNotifyClient();
        // Are there any other messages to transact?
        i2cTryNextMessage();
      }
      break;


    case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
      if(txMessage.messageLength == 1)
      {
       I2C_AcknowledgeConfig(I2C_TYPE, DISABLE);
       I2C_GenerateSTOP(I2C_TYPE, ENABLE);
      }
      break;

    case I2C_EVENT_MASTER_BYTE_RECEIVED:  // Test on I2C_TYPE EV7 and clear it
      txMessage.buffer[messageIndex++] = I2C_ReceiveData(I2C_TYPE);
      // Disable ACK and send I2C_TYPE STOP condition before receiving the last data
      if(messageIndex == (txMessage.messageLength - 1))
      {
        I2C_AcknowledgeConfig(I2C_TYPE, DISABLE);
        I2C_GenerateSTOP(I2C_TYPE, ENABLE);
      }
      else if(messageIndex == txMessage.messageLength)
      {
        i2cNotifyClient();
        // Are there any other messages to transact?
        i2cTryNextMessage();
      }
      break;

    case I2C_EVENT_SLAVE_STOP_DETECTED:
      stops++;
      break;
    default:
      eventDebug[eventPos++] = event;
      if (eventPos == 20)
      {
        eventPos = 0;
      }
      break;
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
