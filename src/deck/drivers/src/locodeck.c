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
#include "queue.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "nvicconf.h"
#include "estimator.h"

#include "configblock.h"

#include "locodeck.h"
#include "lpsTdma.h"

#include "lpsTdoaTag.h"
#include "lpsTwrTag.h"


#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_3) instead of default (RX1, TX1), leaving UART1 free for use
#ifdef LOCODECK_USE_ALT_PINS
    #define GPIO_PIN_IRQ 	GPIO_Pin_5
	#define GPIO_PIN_RESET 	GPIO_Pin_4
	#define GPIO_PORT		GPIOB
	#define EXTI_PortSource EXTI_PortSourceGPIOB
	#define EXTI_PinSource 	EXTI_PinSource5
	#define EXTI_LineN 		EXTI_Line5
	#define EXTI_IRQChannel EXTI9_5_IRQn
#else
    #define GPIO_PIN_IRQ 	GPIO_Pin_11
	#define GPIO_PIN_RESET 	GPIO_Pin_10
	#define GPIO_PORT		GPIOC
	#define EXTI_PortSource EXTI_PortSourceGPIOC
	#define EXTI_PinSource 	EXTI_PinSource11
	#define EXTI_LineN 		EXTI_Line11
	#define EXTI_IRQChannel EXTI15_10_IRQn
#endif


#define DEFAULT_RX_TIMEOUT 10000


#define ANTENNA_OFFSET 154.6   // In meter

// The anchor position can be set using parameters
// As an option you can set a static position in this file and set
// combinedAnchorPositionOk to enable sending the anchor rangings to the Kalman filter

static lpsAlgoOptions_t algoOptions = {
  .tagAddress = 0xbccf000000000008,
  .anchorAddress = {
    0xbccf000000000000,
    0xbccf000000000001,
    0xbccf000000000002,
    0xbccf000000000003,
    0xbccf000000000004,
    0xbccf000000000005,
#if LOCODECK_NR_OF_ANCHORS > 6
    0xbccf000000000006,
#endif
#if LOCODECK_NR_OF_ANCHORS > 7
    0xbccf000000000007,
#endif
  },
  .antennaDelay = (ANTENNA_OFFSET*499.2e6*128)/299792458.0, // In radio tick
  .rangingFailedThreshold = 6,

  .combinedAnchorPositionOk = false,

#ifdef LPS_TDMA_ENABLE
  .useTdma = true,
  .tdmaSlot = TDMA_SLOT,
#endif

  // .rangingMode is the wanted algorithm, available as a parameter
#if LPS_TDOA_ENABLE
  .rangingMode = lpsMode_TDoA,
#elif defined(LPS_TWR_ENABLE)
  .rangingMode = lpsMode_TWR,
#else
  .rangingMode = lpsMode_auto,
#endif
  // .currentRangingMode is the currently running algorithm, available as a log
  // -1 is an impossible mode which forces initialization of the requested mode
  // at startup
  .currentRangingMode = -1,

  // To set a static anchor position from startup, uncomment and modify the
  // following code:
//   .anchorPosition = {
//     {timestamp: 1, x: 0.99, y: 1.49, z: 1.80},
//     {timestamp: 1, x: 0.99, y: 3.29, z: 1.80},
//     {timestamp: 1, x: 4.67, y: 2.54, z: 1.80},
//     {timestamp: 1, x: 0.59, y: 2.27, z: 0.20},
//     {timestamp: 1, x: 4.70, y: 3.38, z: 0.20},
//     {timestamp: 1, x: 4.70, y: 1.14, z: 0.20},
//   },
//
//   .combinedAnchorPositionOk = true,
};

struct {
  uwbAlgorithm_t *algorithm;
  char *name;
} algorithmsList[LPS_NUMBER_OF_ALGORITHM+1] = {
  [lpsMode_TWR] = {.algorithm = &uwbTwrTagAlgorithm, .name="TWR"},
  [lpsMode_TDoA] = {.algorithm = &uwbTdoaTagAlgorithm, .name="TDoA"},
};

point_t* locodeckGetAnchorPosition(uint8_t anchor)
{
  return &algoOptions.anchorPosition[anchor];
}

#if LPS_TDOA_ENABLE
static uwbAlgorithm_t *algorithm = &uwbTdoaTagAlgorithm;
#else
static uwbAlgorithm_t *algorithm = &uwbTwrTagAlgorithm;
#endif

static bool isInit = false;
static SemaphoreHandle_t irqSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static QueueHandle_t lppShortQueue;

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

static void updateTagTdmaSlot(lpsAlgoOptions_t * options)
{
  if (options->tdmaSlot < 0) {
    uint64_t radioAddress = configblockGetRadioAddress();
    int nslot = 1;
    for (int i=0; i<TDMA_NSLOTS_BITS; i++) {
      nslot *= 2;
    }
    options->tdmaSlot = radioAddress % nslot;
  }
  options->tagAddress += options->tdmaSlot;
}

static void uwbTask(void* parameters)
{
  lppShortQueue = xQueueCreate(10, sizeof(lpsLppShortPacket_t));

  algoOptions.currentRangingMode = lpsMode_auto;

  systemWaitStart();

  updateTagTdmaSlot(&algoOptions);

  while(1) {
    // Change and init algorithm uppon request
    // The first time this loop enters, currentRangingMode is set to auto which forces
    // the initialization of the set algorithm
    if (algoOptions.rangingMode == lpsMode_auto) { // Auto switch
      if (algoOptions.rangingModeDetected == false) {
        if (algoOptions.currentRangingMode == lpsMode_auto) {
          // Initialize the algorithm, set time for next switch
          algoOptions.nextSwitchTick = xTaskGetTickCount() + LPS_AUTO_MODE_SWITCH_PERIOD;

          // Defaults to TDoA algorithm
          algoOptions.currentRangingMode = lpsMode_TDoA;
          algorithm = algorithmsList[algoOptions.currentRangingMode].algorithm;
          algorithm->init(dwm, &algoOptions);
          timeout = algorithm->onEvent(dwm, eventTimeout);
        } else if (xTaskGetTickCount() > algoOptions.nextSwitchTick) {
          // Test if we have detected anchors
          if (algoOptions.autoStarted && algorithm->isRangingOk()) {
            algoOptions.rangingModeDetected = true;
            DEBUG_PRINT("Automatic mode: detected %s\n", algorithmsList[algoOptions.currentRangingMode].name);
          } else {
            // We have started the auto mode by initializing the next modes
            algoOptions.autoStarted = true;

            // Setting up next switching time
            algoOptions.nextSwitchTick = xTaskGetTickCount() + LPS_AUTO_MODE_SWITCH_PERIOD;

            // Switch to next algorithm!
            if ((algoOptions.currentRangingMode+1) > LPS_NUMBER_OF_ALGORITHM) {
              algoOptions.currentRangingMode = 1;
            } else {
              algoOptions.currentRangingMode++;
            }

            algorithm = algorithmsList[algoOptions.currentRangingMode].algorithm;
            algorithm->init(dwm, &algoOptions);
            timeout = algorithm->onEvent(dwm, eventTimeout);
          }
        }
      }
    } else if (algoOptions.currentRangingMode != algoOptions.rangingMode) {  // Set modes
      // Reset auto mode
      algoOptions.rangingModeDetected = false;
      algoOptions.autoStarted = false;

      if (algoOptions.rangingMode < 1 || algoOptions.rangingMode > LPS_NUMBER_OF_ALGORITHM) {
        DEBUG_PRINT("Trying to select wrong LPS algorithm, defaulting to TDoA!\n");
        algoOptions.currentRangingMode = algoOptions.rangingMode;
        algorithm = algorithmsList[lpsMode_TDoA].algorithm;
      } else {
        algoOptions.currentRangingMode = algoOptions.rangingMode;
        algorithm = algorithmsList[algoOptions.currentRangingMode].algorithm;
        DEBUG_PRINT("Switching mode to %s\n", algorithmsList[algoOptions.currentRangingMode].name);
      }

      algorithm->init(dwm, &algoOptions);
      timeout = algorithm->onEvent(dwm, eventTimeout);
    }

    if (xSemaphoreTake(irqSemaphore, timeout/portTICK_PERIOD_MS)) {
      do{
          dwHandleInterrupt(dwm);
      } while(digitalRead(GPIO_PIN_IRQ) != 0);
    } else {
      timeout = algorithm->onEvent(dwm, eventTimeout);
    }
  }
}

static lpsLppShortPacket_t lppShortPacket;

bool lpsSendLppShort(uint8_t destId, void* data, size_t length)
{
  bool result = false;

  if (isInit)
  {
    lppShortPacket.dest = destId;
    lppShortPacket.length = length;
    memcpy(lppShortPacket.data, data, length);
    result = xQueueSend(lppShortQueue, &lppShortPacket,0) == pdPASS;
  }

  return result;
}

bool lpsGetLppShort(lpsLppShortPacket_t* shortPacket)
{
  return xQueueReceive(lppShortQueue, shortPacket, 0) == pdPASS;
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

/************ Low level ops for libdw **********/
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

#if LOCODECK_USE_ALT_PINS
	void __attribute__((used)) EXTI5_Callback(void)
#else
	void __attribute__((used)) EXTI11_Callback(void)
#endif
	{
	  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

	  NVIC_ClearPendingIRQ(EXTI_IRQChannel);
	  EXTI_ClearITPendingBit(EXTI_LineN);

	  //To unlock RadioTask
	  xSemaphoreGiveFromISR(irqSemaphore, &xHigherPriorityTaskWoken);

	  if(xHigherPriorityTaskWoken)
		portYIELD();
	}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiSpeed = SPI_BAUDRATE_21MHZ;
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
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_IRQ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init reset output
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RESET;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

  // Init CS pin
  pinMode(CS_PIN, OUTPUT);

  // Reset the DW1000 chip
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));

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
  dwUseSmartPower(dwm, true);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwSetReceiveWaitTimeout(dwm, DEFAULT_RX_TIMEOUT);

  dwCommitConfiguration(dwm);

  // Enable interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_VERY_HIGH_PRI;
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
  .requiredEstimator = kalmanEstimator,
  .requiredLowInterferenceRadioMode = true,

  .init = dwm1000Init,
  .test = dwm1000Test,
};

DECK_DRIVER(dwm1000_deck);

LOG_GROUP_START(ranging)
#if (LOCODECK_NR_OF_ANCHORS > 0)
LOG_ADD(LOG_FLOAT, distance0, &algoOptions.distance[0])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 1)
LOG_ADD(LOG_FLOAT, distance1, &algoOptions.distance[1])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 2)
LOG_ADD(LOG_FLOAT, distance2, &algoOptions.distance[2])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 3)
LOG_ADD(LOG_FLOAT, distance3, &algoOptions.distance[3])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 4)
LOG_ADD(LOG_FLOAT, distance4, &algoOptions.distance[4])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 5)
LOG_ADD(LOG_FLOAT, distance5, &algoOptions.distance[5])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 6)
LOG_ADD(LOG_FLOAT, distance6, &algoOptions.distance[6])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 7)
LOG_ADD(LOG_FLOAT, distance7, &algoOptions.distance[7])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 0)
LOG_ADD(LOG_FLOAT, pressure0, &algoOptions.pressures[0])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 1)
LOG_ADD(LOG_FLOAT, pressure1, &algoOptions.pressures[1])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 2)
LOG_ADD(LOG_FLOAT, pressure2, &algoOptions.pressures[2])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 3)
LOG_ADD(LOG_FLOAT, pressure3, &algoOptions.pressures[3])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 4)
LOG_ADD(LOG_FLOAT, pressure4, &algoOptions.pressures[4])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 5)
LOG_ADD(LOG_FLOAT, pressure5, &algoOptions.pressures[5])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 6)
LOG_ADD(LOG_FLOAT, pressure6, &algoOptions.pressures[6])
#endif
#if (LOCODECK_NR_OF_ANCHORS > 7)
LOG_ADD(LOG_FLOAT, pressure7, &algoOptions.pressures[7])
#endif
LOG_ADD(LOG_UINT16, state, &algoOptions.rangingState)
LOG_GROUP_STOP(ranging)

LOG_GROUP_START(loco)
LOG_ADD(LOG_UINT8, mode, &algoOptions.currentRangingMode)
LOG_GROUP_STOP(loco)

PARAM_GROUP_START(loco)
PARAM_ADD(PARAM_UINT8, mode, &algoOptions.rangingMode)
PARAM_GROUP_STOP(loco)


PARAM_GROUP_START(anchorpos)
#if (LOCODECK_NR_OF_ANCHORS > 0)
PARAM_ADD(PARAM_FLOAT, anchor0x, &algoOptions.anchorPosition[0].x)
PARAM_ADD(PARAM_FLOAT, anchor0y, &algoOptions.anchorPosition[0].y)
PARAM_ADD(PARAM_FLOAT, anchor0z, &algoOptions.anchorPosition[0].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 1)
PARAM_ADD(PARAM_FLOAT, anchor1x, &algoOptions.anchorPosition[1].x)
PARAM_ADD(PARAM_FLOAT, anchor1y, &algoOptions.anchorPosition[1].y)
PARAM_ADD(PARAM_FLOAT, anchor1z, &algoOptions.anchorPosition[1].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 2)
PARAM_ADD(PARAM_FLOAT, anchor2x, &algoOptions.anchorPosition[2].x)
PARAM_ADD(PARAM_FLOAT, anchor2y, &algoOptions.anchorPosition[2].y)
PARAM_ADD(PARAM_FLOAT, anchor2z, &algoOptions.anchorPosition[2].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 3)
PARAM_ADD(PARAM_FLOAT, anchor3x, &algoOptions.anchorPosition[3].x)
PARAM_ADD(PARAM_FLOAT, anchor3y, &algoOptions.anchorPosition[3].y)
PARAM_ADD(PARAM_FLOAT, anchor3z, &algoOptions.anchorPosition[3].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 4)
PARAM_ADD(PARAM_FLOAT, anchor4x, &algoOptions.anchorPosition[4].x)
PARAM_ADD(PARAM_FLOAT, anchor4y, &algoOptions.anchorPosition[4].y)
PARAM_ADD(PARAM_FLOAT, anchor4z, &algoOptions.anchorPosition[4].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 5)
PARAM_ADD(PARAM_FLOAT, anchor5x, &algoOptions.anchorPosition[5].x)
PARAM_ADD(PARAM_FLOAT, anchor5y, &algoOptions.anchorPosition[5].y)
PARAM_ADD(PARAM_FLOAT, anchor5z, &algoOptions.anchorPosition[5].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 6)
PARAM_ADD(PARAM_FLOAT, anchor6x, &algoOptions.anchorPosition[6].x)
PARAM_ADD(PARAM_FLOAT, anchor6y, &algoOptions.anchorPosition[6].y)
PARAM_ADD(PARAM_FLOAT, anchor6z, &algoOptions.anchorPosition[6].z)
#endif
#if (LOCODECK_NR_OF_ANCHORS > 7)
PARAM_ADD(PARAM_FLOAT, anchor7x, &algoOptions.anchorPosition[7].x)
PARAM_ADD(PARAM_FLOAT, anchor7y, &algoOptions.anchorPosition[7].y)
PARAM_ADD(PARAM_FLOAT, anchor7z, &algoOptions.anchorPosition[7].z)
#endif
PARAM_ADD(PARAM_UINT8, enable, &algoOptions.combinedAnchorPositionOk)
PARAM_GROUP_STOP(anchorpos)

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &isInit)
PARAM_GROUP_STOP(deck)

// Loco Posisioning Protocol (LPP) handling

void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data, const int length)
{
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    if (srcId < LOCODECK_NR_OF_ANCHORS) {
      struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
      algoOptions.anchorPosition[srcId].timestamp = xTaskGetTickCount();
      algoOptions.anchorPosition[srcId].x = newpos->x;
      algoOptions.anchorPosition[srcId].y = newpos->y;
      algoOptions.anchorPosition[srcId].z = newpos->z;
    }
  }
}
