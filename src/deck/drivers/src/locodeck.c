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

#define CS_PIN DECK_GPIO_IO1

#define RX_TIMEOUT 1000

extern uwbAlgorithm_t uwbTwrTagAlgorithm;
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

  algorithm->init(dwm);

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
