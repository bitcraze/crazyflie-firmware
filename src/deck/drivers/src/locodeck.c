/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * locodeck.c: Dwm1000 deck driver.
 */

#define DEBUG_MODULE "DWM"

#include <stdint.h>
#include <string.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "nvicconf.h"

#include "locodeck.h"
#include "lpsTwrTag.h"

#define CS_PIN DECK_GPIO_IO1

#define RX_TIMEOUT 1000

#define ANTENNA_OFFSET 154.6   // In meter

// The anchor position can be set using parameters
// As an option you can set a static position in this file and set
// anchorPositionOk to enable sending the anchor rangings to the Kalman filter

static lpsAlgoOptions_t algoOptions = {
  .tagAddress = 0xbccf000000000008,
  .anchorAddress = {
    0xbccf000000000001,
    0xbccf000000000002,
    0xbccf000000000003,
    0xbccf000000000004,
    0xbccf000000000005,
    0xbccf000000000006
  },
  .antennaDelay = (ANTENNA_OFFSET*499.2e6*128)/299792458.0, // In radio tick
  .rangingFailedThreshold = 6,

  .anchorPositionOk = false
};

static uwbAlgorithm_t *algorithm = &uwbTwrTagAlgorithm;

static bool isInit = false;
static xSemaphoreHandle spiSemaphore;
static SemaphoreHandle_t irqSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static uint32_t timeout;

static void txCallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketSent);
}

static void rxCallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(dwDevice_t * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveTimeout);
}

// static void rxfailedcallback(dwDevice_t *dev) {
//   timeout = algorithm->onEvent(dev, eventReceiveFailed);
// }


static void uwbTask(void* parameters)
{
  systemWaitStart();

  algorithm->init(dwm, &algoOptions);

  while(1) {
    if (xSemaphoreTake(irqSemaphore, timeout/portTICK_PERIOD_MS)) {
      do{
          dwHandleInterrupt(dwm);
      } while(digitalRead(DECK_GPIO_RX1) != 0);
    } else {
      timeout = algorithm->onEvent(dwm, eventTimeout);
    }
  }
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];

/************ Low level ops for libdw **********/
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  xSemaphoreTake(spiSemaphore, portMAX_DELAY);

  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);

  xSemaphoreGive(spiSemaphore);
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  xSemaphoreTake(spiSemaphore, portMAX_DELAY);

  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);

  xSemaphoreGive(spiSemaphore);
}

void __attribute__((used)) EXTI11_Callback(void)
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  EXTI_ClearITPendingBit(EXTI_Line11);

  //To unlock RadioTask
  xSemaphoreGiveFromISR(irqSemaphore, &xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken)
    portYIELD();
}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiConfigureSlow();
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiConfigureFast();
  }
}

static void delayms(dwDevice_t* dev, unsigned int delay)
{
  vTaskDelay(M2T(delay));
}

static dwOps_t dwOps = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
};

/*********** Deck driver initialization ***************/

static void dwm1000Init(DeckInfo *info)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  spiBegin();

  // Init IRQ input
  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource11);

  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init reset output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Init CS pin
  pinMode(CS_PIN, OUTPUT);

  // Reset the DW1000 chip
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, 0);
  vTaskDelay(M2T(10));
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, 1);
  vTaskDelay(M2T(10));

  // Semaphore that protect the SPI communication
  spiSemaphore = xSemaphoreCreateMutex();

  // Initialize the driver
  dwInit(dwm, &dwOps);       // Init libdw

  int result = dwConfigure(dwm);
  if (result != 0) {
    isInit = false;
    DEBUG_PRINT("Failed to configure DW1000!\r\n");
    return;
  }

  dwEnableAllLeds(dwm);

  dwTime_t delay = {.full = 0};
  dwSetAntenaDelay(dwm, delay);

  dwAttachSentHandler(dwm, txCallback);
  dwAttachReceivedHandler(dwm, rxCallback);
  dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);
  dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
  dwSetChannel(dwm, CHANNEL_2);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwSetReceiveWaitTimeout(dwm, RX_TIMEOUT);

  dwCommitConfiguration(dwm);

  // Enable interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_LOW_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  vSemaphoreCreateBinary(irqSemaphore);

  xTaskCreate(uwbTask, "lps", 3*configMINIMAL_STACK_SIZE, NULL,
                    5/*priority*/, NULL);

  isInit = true;
}

static bool dwm1000Test()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM1000\n");
  }

  return isInit;
}

static const DeckDriver dwm1000_deck = {
  .vid = 0xBC,
  .pid = 0x06,
  .name = "bcDWM1000",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = dwm1000Init,
  .test = dwm1000Test,
};

DECK_DRIVER(dwm1000_deck);

LOG_GROUP_START(ranging)
LOG_ADD(LOG_FLOAT, distance1, &algoOptions.distance[0])
LOG_ADD(LOG_FLOAT, distance2, &algoOptions.distance[1])
LOG_ADD(LOG_FLOAT, distance3, &algoOptions.distance[2])
LOG_ADD(LOG_FLOAT, distance4, &algoOptions.distance[3])
LOG_ADD(LOG_FLOAT, distance5, &algoOptions.distance[4])
LOG_ADD(LOG_FLOAT, distance6, &algoOptions.distance[5])
LOG_ADD(LOG_FLOAT, distance7, &algoOptions.distance[6])
LOG_ADD(LOG_FLOAT, distance8, &algoOptions.distance[7])
LOG_ADD(LOG_FLOAT, pressure1, &algoOptions.pressures[0])
LOG_ADD(LOG_FLOAT, pressure2, &algoOptions.pressures[1])
LOG_ADD(LOG_FLOAT, pressure3, &algoOptions.pressures[2])
LOG_ADD(LOG_FLOAT, pressure4, &algoOptions.pressures[3])
LOG_ADD(LOG_FLOAT, pressure5, &algoOptions.pressures[4])
LOG_ADD(LOG_FLOAT, pressure6, &algoOptions.pressures[5])
LOG_ADD(LOG_FLOAT, pressure7, &algoOptions.pressures[6])
LOG_ADD(LOG_FLOAT, pressure8, &algoOptions.pressures[7])
LOG_ADD(LOG_UINT16, state, &algoOptions.rangingState)
LOG_GROUP_STOP(ranging)

PARAM_GROUP_START(anchorpos)
PARAM_ADD(PARAM_FLOAT, anchor0x, &algoOptions.anchorPosition[0].x)
PARAM_ADD(PARAM_FLOAT, anchor0y, &algoOptions.anchorPosition[0].y)
PARAM_ADD(PARAM_FLOAT, anchor0z, &algoOptions.anchorPosition[0].z)
PARAM_ADD(PARAM_FLOAT, anchor1x, &algoOptions.anchorPosition[1].x)
PARAM_ADD(PARAM_FLOAT, anchor1y, &algoOptions.anchorPosition[1].y)
PARAM_ADD(PARAM_FLOAT, anchor1z, &algoOptions.anchorPosition[1].z)
PARAM_ADD(PARAM_FLOAT, anchor2x, &algoOptions.anchorPosition[2].x)
PARAM_ADD(PARAM_FLOAT, anchor2y, &algoOptions.anchorPosition[2].y)
PARAM_ADD(PARAM_FLOAT, anchor2z, &algoOptions.anchorPosition[2].z)
PARAM_ADD(PARAM_FLOAT, anchor3x, &algoOptions.anchorPosition[3].x)
PARAM_ADD(PARAM_FLOAT, anchor3y, &algoOptions.anchorPosition[3].y)
PARAM_ADD(PARAM_FLOAT, anchor3z, &algoOptions.anchorPosition[3].z)
PARAM_ADD(PARAM_FLOAT, anchor4x, &algoOptions.anchorPosition[4].x)
PARAM_ADD(PARAM_FLOAT, anchor4y, &algoOptions.anchorPosition[4].y)
PARAM_ADD(PARAM_FLOAT, anchor4z, &algoOptions.anchorPosition[4].z)
PARAM_ADD(PARAM_FLOAT, anchor5x, &algoOptions.anchorPosition[5].x)
PARAM_ADD(PARAM_FLOAT, anchor5y, &algoOptions.anchorPosition[5].y)
PARAM_ADD(PARAM_FLOAT, anchor5z, &algoOptions.anchorPosition[5].z)
PARAM_ADD(PARAM_UINT8, enable, &algoOptions.anchorPositionOk)
PARAM_GROUP_STOP(anchorpos)
