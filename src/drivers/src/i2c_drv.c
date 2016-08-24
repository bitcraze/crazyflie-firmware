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

// Definitions of sensors I2C bus
#define I2C_SENSORS                         I2C3
#define I2C_SENSORS_EV_IRQn                 I2C3_EV_IRQn
#define I2C_SENSORS_ER_IRQn                 I2C3_ER_IRQn
#define I2C_SENSORS_RCC                     RCC_APB1Periph_I2C3
#define I2C_SENSORS_GPIO_RCC_SCL            RCC_AHB1Periph_GPIOA
#define I2C_SENSORS_GPIO_PORT_SCL           GPIOA
#define I2C_SENSORS_GPIO_PIN_SCL            GPIO_Pin_8
#define I2C_SENSORS_GPIO_SOURCE_SCL         GPIO_PinSource8
#define I2C_SENSORS_GPIO_RCC_SDA            RCC_AHB1Periph_GPIOC
#define I2C_SENSORS_GPIO_PORT_SDA           GPIOC
#define I2C_SENSORS_GPIO_PIN_SDA            GPIO_Pin_9
#define I2C_SENSORS_GPIO_SOURCE_SDA         GPIO_PinSource9
#define I2C_SENSORS_GPIO_AF                 GPIO_AF_I2C3
#define I2C_SENSORS_QUEUE_LENGTH            5
#define I2C_SENSORS_CLOCK_SPEED             400000
// DMA definitions
#define I2C_SENSORS_DMA_RCC                 RCC_AHB1Periph_DMA1
#define I2C_SENSORS_DMA_CHANNEL             DMA_Channel_3
#define I2C_SENSORS_DMA_RX_STREAM           DMA1_Stream2
#define I2C_SENSORS_DMA_RX_IRQn             DMA1_Stream2_IRQn
#define I2C_SENSORS_DMA_RX_TC_FLAG          DMA_FLAG_TCIF2
#define I2C_SENSORS_DMA_RX_TE_FLAG          DMA_FLAG_TEIF2
#define I2C_SENSORS_DR                      ((uint32_t)&I2C3->DR)

// Definition of eeprom and deck I2C buss
#define I2C_DECK                            I2C1
#define I2C_DECK_EV_IRQn                    I2C1_EV_IRQn
#define I2C_DECK_ER_IRQn                    I2C1_ER_IRQn
#define I2C_DECK_RCC                        RCC_APB1Periph_I2C1
#define I2C_DECK_GPIO_RCC_SCL               RCC_AHB1Periph_GPIOB
#define I2C_DECK_GPIO_PORT_SCL              GPIOB
#define I2C_DECK_GPIO_PIN_SCL               GPIO_Pin_6
#define I2C_DECK_GPIO_SOURCE_SCL            GPIO_PinSource6
#define I2C_DECK_GPIO_RCC_SDA               RCC_AHB1Periph_GPIOB
#define I2C_DECK_GPIO_PORT_SDA              GPIOB
#define I2C_DECK_GPIO_PIN_SDA               GPIO_Pin_7
#define I2C_DECK_GPIO_SOURCE_SDA            GPIO_PinSource7
#define I2C_DECK_GPIO_AF                    GPIO_AF_I2C1
#define I2C_DECK_QUEUE_LENGTH               5
#define I2C_DECK_CLOCK_SPEED                400000
// DMA definitions
#define I2C_DECK_DMA_RCC                    RCC_AHB1Periph_DMA1
#define I2C_DECK_DMA_CHANNEL                DMA_Channel_1
#define I2C_DECK_DMA_RX_STREAM              DMA1_Stream0
#define I2C_DECK_DMA_RX_IRQn                DMA1_Stream0_IRQn
#define I2C_DECK_DMA_RX_TC_FLAG             DMA_FLAG_TCIF0
#define I2C_DECK_DMA_RX_TE_FLAG             DMA_FLAG_TEIF0
#define I2C_DECK_DR                         ((uint32_t)&I2C1->DR)

// Misc constants.
#define I2C_NO_BLOCK				    0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2

// Delay is approx 0.06us per loop @168Mhz
#define I2CDEV_LOOPS_PER_US  17
#define I2CDEV_LOOPS_PER_MS  (16789) // measured

// Defines to unlock bus
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


// Debug variables
uint32_t eventDebug[1024][2];
uint32_t eventDebugUnknown[1024];
uint32_t eventPos = 0;
uint32_t eventPosUnknown = 0;
uint32_t stops = 0;
uint32_t eventDMA = 0;

/* Private functions */
void i2cInitBus(I2cDrv* i2c);
static void i2cDmaSetupBus(I2cDrv* i2c);
static void i2cStartTransfer(I2cDrv *i2c);
static inline void i2cdrvRoughLoopDelay(uint32_t us) __attribute__((optimize("O2")));
/**
 * Unlocks the i2c bus if needed.
 * @param portSCL  Port of the SCL pin
 * @param portSDA  Port of the SDA pin
 * @param pinSCL   SCL Pin
 * @param pinSDA   SDA Pin
 */
static void i2cdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA);
static void i2cEventIsrHandler(I2cDrv* i2c);
static void i2cErrorIsrHandler(I2cDrv* i2c);
static void i2cDmaIsrHandler(I2cDrv* i2c);

static const I2cDef sensorBusDef =
{
  .i2cPort            = I2C3,
  .i2cPerif           = RCC_APB1Periph_I2C3,
  .i2cEVIRQn          = I2C3_EV_IRQn,
  .i2cERIRQn          = I2C3_ER_IRQn,
  .gpioSCLPerif       = RCC_AHB1Periph_GPIOA,
  .gpioSCLPort        = GPIOA,
  .gpioSCLPin         = GPIO_Pin_8,
  .gpioSCLPinSource   = GPIO_PinSource8,
  .gpioSDAPerif       = RCC_AHB1Periph_GPIOC,
  .gpioSDAPort        = GPIOC,
  .gpioSDAPin         = GPIO_Pin_9,
  .gpioSDAPinSource   = GPIO_PinSource9,
  .gpioAF             = GPIO_AF_I2C3,
  .dmaPerif           = RCC_AHB1Periph_DMA1,
  .dmaChannel         = DMA_Channel_3,
  .dmaRxStream        = DMA1_Stream2,
  .dmaRxIRQ           = DMA1_Stream2_IRQn,
  .dmaRxTCFlag        = DMA_FLAG_TCIF2,
  .dmaRxTEFlag        = DMA_FLAG_TEIF2,
};

I2cDrv sensorsBus =
{
  .def                = &sensorBusDef,
  .xMessagesForTx     = 0,
  .nbrOfretries       = 0,
  .isBusFree          = 0,
};

static const I2cDef deckBusDef =
{
  .i2cPort            = I2C1,
  .i2cPerif           = RCC_APB1Periph_I2C1,
  .i2cEVIRQn          = I2C1_EV_IRQn,
  .i2cERIRQn          = I2C1_ER_IRQn,
  .gpioSCLPerif       = RCC_AHB1Periph_GPIOB,
  .gpioSCLPort        = GPIOB,
  .gpioSCLPin         = GPIO_Pin_6,
  .gpioSCLPinSource   = GPIO_PinSource6,
  .gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
  .gpioSDAPort        = GPIOB,
  .gpioSDAPin         = GPIO_Pin_7,
  .gpioSDAPinSource   = GPIO_PinSource7,
  .gpioAF             = GPIO_AF_I2C1,
  .dmaPerif           = RCC_AHB1Periph_DMA1,
  .dmaChannel         = DMA_Channel_1,
  .dmaRxStream        = DMA1_Stream0,
  .dmaRxIRQ           = DMA1_Stream0_IRQn,
  .dmaRxTCFlag        = DMA_FLAG_TCIF0,
  .dmaRxTEFlag        = DMA_FLAG_TEIF0,
};

I2cDrv deckBus =
{
  .def                = &deckBusDef,
  .xMessagesForTx     = 0,
  .nbrOfretries       = 0,
  .isBusFree          = 0,
};


static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
  volatile uint32_t delay = 0;
  for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}

static void i2cStartTransfer(I2cDrv *i2c)
{
  if (i2c->txMessage.direction == i2cRead /*&& txMessage.messageLength > 1*/)
  {
    i2c->DMAStruct.DMA_BufferSize = i2c->txMessage.messageLength;
    i2c->DMAStruct.DMA_Memory0BaseAddr = (uint32_t)i2c->txMessage.buffer;
    DMA_Init(i2c->def->dmaRxStream, &i2c->DMAStruct);
    DMA_Cmd(i2c->def->dmaRxStream, ENABLE);
  }

  I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
  I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT, ENABLE);
  i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
}

#if 0
static void i2cStartTransfer(void)
{
  if (txMessage.direction == i2cRead /*&& txMessage.messageLength > 1*/)
  {
    /* Generate start with ack enable.
     * For some reason setting CR1 reg in sequence with
     * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
     * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
     * instant start->stop condition (3.9us long) which I found out with an I2C
     * analyzer. This fast start->stop is only possible to generate if both
     * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
     * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
     * problem is gone. Go figure...
     */
    I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE);
    /* Wait until SB flag is set */
    while (I2C_GetFlagStatus(I2C_SENSORS, I2C_FLAG_SB) == RESET);
    /* Send slave address */
    I2C_Send7bitAddress(I2C_SENSORS, txMessage.slaveAddress << 1, I2C_Direction_Receiver);
    /* Wait until ADDR flag is set */
    while (I2C_GetFlagStatus(I2C_SENSORS, I2C_FLAG_ADDR) == RESET);
    /* SETUP DMA */
    sensorsBus.DMAStruct->DMA_BufferSize = txMessage.messageLength;
    sensorsBus.DMAStruct->DMA_Memory0BaseAddr = txMessage.buffer;
    DMA_Init(DMA1_Stream2, &DMA_InitStructureShare);
    DMA_Cmd(DMA1_Stream2, ENABLE);
    /* Enable Last DMA bit */
    I2C_DMALastTransferCmd(I2C_SENSORS, ENABLE); // No repetitive start
    /* Start transfer */
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
    I2C_DMACmd(I2C_SENSORS, ENABLE); // Enable before ADDR clear
    /* Clear ADDR */
    I2C_GetLastEvent(I2C_SENSORS);
  }
}
#endif

//-----------------------------------------------------------
void i2cMessageTransfer(I2cDrv* i2c, I2cMessage* message, portTickType xBlockTime)
{
  // Is the I2C interrupt in the middle of transmitting a message?
  if (i2c->isBusFree == true )
  {
    // No message is currently being sent or queued to be sent.  We
    // can start the ISR sending this message immediately.
    memcpy((char*)&i2c->txMessage, (char*)message, sizeof(I2cMessage));
    i2c->isBusFree = false;

    i2cStartTransfer(i2c);
  }
  else
  {
    signed portBASE_TYPE xReturn;
    xReturn = xQueueSend(i2c->xMessagesForTx, (char*)message, xBlockTime);
    // We may have blocked while trying to queue the message.  If this
    // was the case then the interrupt would have been enabled and we may
    // now find that the I2C interrupt routine is no longer sending a
    // message.
    if ((i2c->isBusFree == true ) && (xReturn == pdPASS))
    {
      // Get the next message in the queue (this should be the
      // message we just posted) and start off the transmission
      // again.
      xQueueReceive(i2c->xMessagesForTx, (char*)&i2c->txMessage, I2C_NO_BLOCK);
      i2c->isBusFree = false;
      i2cStartTransfer(i2c);
    }
  }
}

void i2cInit(I2cDrv* i2c)
{
  i2cInitBus(i2c);

  initUsecTimer();
}

void i2cInitBus(I2cDrv* i2c)
{
  I2C_InitTypeDef  I2C_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIOA clock
  RCC_AHB1PeriphClockCmd(i2c->def->gpioSDAPerif, ENABLE);
  RCC_AHB1PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);
  // Enable I2C_SENSORS clock
  RCC_APB1PeriphClockCmd(i2c->def->i2cPerif, ENABLE);

  // Configure I2C_SENSORS pins to unlock bus.
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
  GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
  GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

  i2cdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);

  // Configure I2C_SENSORS pins for AF.
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
  GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
  GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

  //Map gpios to alternate functions
  GPIO_PinAFConfig(i2c->def->gpioSCLPort, i2c->def->gpioSCLPinSource, i2c->def->gpioAF);
  GPIO_PinAFConfig(i2c->def->gpioSDAPort, i2c->def->gpioSDAPinSource, i2c->def->gpioAF);

  // I2C_SENSORS configuration
  I2C_DeInit(i2c->def->i2cPort);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SENSORS_CLOCK_SPEED;
  I2C_Init(i2c->def->i2cPort, &I2C_InitStructure);

  // Enable I2C_SENSORS error interrupts
  I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cEVIRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cERIRQn;
  NVIC_Init(&NVIC_InitStructure);

  i2cDmaSetupBus(i2c);

  // Create the queues used to hold Rx and Tx characters.
  i2c->xMessagesForTx = xQueueCreate(I2C_SENSORS_QUEUE_LENGTH, sizeof(I2cMessage));
  i2c->isBusFree = true;
}

static void i2cDmaSetupBus(I2cDrv* i2c)
{

  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(i2c->def->dmaPerif, ENABLE);

  // RX DMA Channel Config
  i2c->DMAStruct.DMA_Channel = i2c->def->dmaChannel;
  i2c->DMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
  i2c->DMAStruct.DMA_Memory0BaseAddr = 0;
  i2c->DMAStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  i2c->DMAStruct.DMA_BufferSize = 0;
  i2c->DMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  i2c->DMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  i2c->DMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  i2c->DMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  i2c->DMAStruct.DMA_Mode = DMA_Mode_Normal;
  i2c->DMAStruct.DMA_Priority = DMA_Priority_High;
  i2c->DMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  i2c->DMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  i2c->DMAStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  i2c->DMAStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  NVIC_InitStructure.NVIC_IRQChannel = i2c->def->dmaRxIRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void i2cdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
  GPIO_SetBits(portSDA, pinSDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(portSCL, pinSCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(portSCL, pinSCL);
    i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(portSCL, pinSCL);
    i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(portSCL, pinSCL);
  i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(portSDA, pinSDA);
  i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(portSDA, pinSDA);
  i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(portSDA, pinSDA);
  GPIO_SetBits(portSCL, pinSCL);
  GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, 10 * I2CDEV_LOOPS_PER_MS);
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

I2cDrv* i2cdrvGetSensorsBus()
{
  return &sensorsBus;
}

I2cDrv* i2cdrvGetDeckBus()
{
  return &deckBus;
}

static bool i2cTryNextMessage(I2cDrv* i2c)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  bool nextMessage = false;

  if (xQueueReceiveFromISR(i2c->xMessagesForTx, (void*)&i2c->txMessage,
      &xHigherPriorityTaskWoken) == pdTRUE)
  {
    i2cStartTransfer(i2c);
    nextMessage = true;
  }
  else
  {
   // No more messages were found to be waiting for
   // transaction so the bus is free.
    i2c->isBusFree = true;
//   I2C_GenerateSTOP(I2C_SENSORS, ENABLE);                     // program the Stop
//   I2C_ITConfig(I2C_SENSORS, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
  }

  return nextMessage;
}

static void i2cNotifyClient(I2cDrv* i2c)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (i2c->txMessage.clientQueue != 0)
  {
    xQueueSendFromISR(i2c->txMessage.clientQueue, (void*)&i2c->txMessage,
                      &xHigherPriorityTaskWoken);
  }
}

static void i2cEventIsrHandler(I2cDrv* i2c)
{
  volatile uint32_t event;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint16_t SR1;
  uint16_t SR2;

  // read the status register first
  SR1 = i2c->def->i2cPort->SR1;

  // Debug code
  eventDebug[eventPos][0] = usecTimestamp();
  eventDebug[eventPos][1] = SR1;
  if (++eventPos == 1024)
  {
    eventPos = 0;
  }

  // Start bit event
  if (SR1 & I2C_SR1_SB)
  {
    i2c->messageIndex = 0;

    if(i2c->txMessage.direction == i2cWrite ||
       i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1, I2C_Direction_Transmitter);
    }
    else
    {
      I2C_AcknowledgeConfig(i2c->def->i2cPort, ENABLE);
      I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1, I2C_Direction_Receiver);
    }
  }
  // Address event with transmit empty
  else if (SR1 & I2C_SR1_ADDR)
  {
    if(i2c->txMessage.direction == i2cWrite ||
       i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      SR2 = i2c->def->i2cPort->SR2;                               // clear ADDR

      if (i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
      {
        if (i2c->txMessage.isInternal16bit)
        {
          I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0xFF00) >> 8);
          I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
        }
        else
        {
          I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
        }
        i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
      }
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, ENABLE);        // allow us to have an EV7
    }
    else // Reading
    {
      if(i2c->txMessage.messageLength == 1)
      {
        I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
      }
      else
      {
        I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); // No repetitive start
      }
      // Disable I2C interrupts
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
      // Enable the Transfer Complete interrupt
      DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, ENABLE);
      I2C_DMACmd(i2c->def->i2cPort, ENABLE); // Enable before ADDR clear

      __DMB();
      SR2 = i2c->def->i2cPort->SR2;                               // clear ADDR
#if 0
      if(i2c->txMessage.messageLength == 1)
      {
        I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
        __DMB();
        SR2 = i2c->def->i2cPort->SR2;                               // clear ADDR after ACK is turned off
        I2C_GenerateSTOP(i2c->def->i2cPort, ENABLE);
        I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, ENABLE);        // allow us to have an EV7
      }
      else
      {
        I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
        // Enable the Transfer Complete interrupt
        DMA_ITConfig(DMA1_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
        I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); // No repetitive start
        I2C_DMACmd(i2c->def->i2cPort, ENABLE); // Enable before ADDR clear

        SR2 = i2c->def->i2cPort->SR2;                                  // clear the ADDR here
      }
#endif
    }
  }
  // Byte transfer finished - EV7_2, EV7_3 or EV8_2
  else if (SR1 & I2C_SR1_BTF)
  {
    SR2 = i2c->def->i2cPort->SR2;
    if (SR2 & I2C_SR2_TRA) // In write mode?
    {
      if (i2c->txMessage.direction == i2cRead) // internal address read
      {
        //i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
      }
      else
      {
        i2cNotifyClient(i2c);
        // Are there any other messages to transact?
        if (!i2cTryNextMessage(i2c))
        {
          I2C_GenerateSTOP(i2c->def->i2cPort, ENABLE);                     // program the Stop
          I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
        }
      }
    }
    else // Reading
    {
      i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
      if(i2c->messageIndex == i2c->txMessage.messageLength)
      {
        i2cNotifyClient(i2c);
        // Are there any other messages to transact?
        i2cTryNextMessage(i2c);
      }
    }
    while (i2c->def->i2cPort->CR1 & 0x0100) { ; }
  }
  // Byte received - EV7
  else if (SR1 & I2C_SR1_RXNE)
  {
    i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
    if(i2c->messageIndex == i2c->txMessage.messageLength)
    {
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);                // disable RXE to get BTF
    }
  }
  // Byte transmitted EV8 / EV8_1
  else if (SR1 & I2C_SR1_TXE)
  {
    if (i2c->txMessage.direction == i2cRead)
    {
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
      /* Switch to read */
      i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
    }
    else
    {
      I2C_SendData(i2c->def->i2cPort, i2c->txMessage.buffer[i2c->messageIndex++]);
      if(i2c->messageIndex == i2c->txMessage.messageLength)
      {
        I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush and get BTF
      }
    }
//    if(i2c->txMessage.internalAddress == I2C_NO_INTERNAL_ADDRESS)
//    {
//      I2C_SendData(i2c->def->i2cPort, i2c->txMessage.buffer[i2c->messageIndex++]);
//      if(i2c->messageIndex == i2c->txMessage.messageLength)
//      {
//        I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush and get BTF
//      }
//    }
//    else
//    {
//      if (i2c->txMessage.isInternal16bit)
//      {
//        // FIXME: How to hande 16 bit reg
//        I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0xFF00) >> 8);
//      }
//      else
//      {
//        I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
//        i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
//        if (i2c->txMessage.direction == i2cRead)
//        {
//          /* Switch to read */
//          i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
//        }
//      }
  }
}

static void i2cErrorIsrHandler(I2cDrv* i2c)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_AF))
  {
    if(i2c->txMessage.nbrOfRetries-- > 0)
    {
      I2C_GenerateSTART(i2c->def->i2cPort, ENABLE);
    }
    else
    {
      I2C_GenerateSTOP(i2c->def->i2cPort, ENABLE);
      i2c->txMessage.status = i2cNack;
      i2cNotifyClient(i2c);
      i2cTryNextMessage(i2c);
    }
    I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_AF);
  }
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_BERR))
  {
      I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_BERR);
  }
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_OVR))
  {
      I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_OVR);
  }
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_ARLO))
  {
      I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_ARLO);
  }
}

static void i2cDmaIsrHandler(I2cDrv* i2c)
{
  if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag)) // Tranasfer complete
  {
    DMA_ClearITPendingBit(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag);
    DMA_Cmd(i2c->def->dmaRxStream, DISABLE);
    I2C_DMACmd(i2c->def->i2cPort, DISABLE);
    I2C_GenerateSTOP(i2c->def->i2cPort, ENABLE);
    while (i2c->def->i2cPort->SR1 & I2C_CR1_STOP)
      ;
    I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
    I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);

    i2cNotifyClient(i2c);
    // Are there any other messages to transact?
    i2cTryNextMessage(i2c);
  }
  if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTEFlag)) // Transfer error
  {
    DMA_ClearITPendingBit(i2c->def->dmaRxStream, i2c->def->dmaRxTEFlag);
    //TODO: Implement error handling
  }
}


void __attribute__((used)) I2C1_ER_IRQHandler(void)
{
  i2cErrorIsrHandler(&deckBus);
}

void __attribute__((used)) I2C1_EV_IRQHandler(void)
{
  i2cEventIsrHandler(&deckBus);
}

void __attribute__((used)) DMA1_Stream0_IRQHandler(void)
{
  i2cDmaIsrHandler(&deckBus);
}

void __attribute__((used)) I2C3_ER_IRQHandler(void)
{
  i2cErrorIsrHandler(&sensorsBus);
}

void __attribute__((used)) I2C3_EV_IRQHandler(void)
{
  i2cEventIsrHandler(&sensorsBus);
}

void __attribute__((used)) DMA1_Stream2_IRQHandler(void)
{
  i2cDmaIsrHandler(&sensorsBus);
}

