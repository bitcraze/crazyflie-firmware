/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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

#include "autoconf.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "nvicconf.h"
#include "estimator.h"
#include "statsCnt.h"
#include "mem.h"

#include "locodeck.h"

#include "lpsTdoa2Tag.h"
#include "lpsTdoa3Tag.h"
#include "lpsTwrTag.h"


#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_3) instead of default (RX1, TX1), leaving UART1 free for use
#ifdef CONFIG_DECK_LOCODECK_USE_ALT_PINS
  #define GPIO_PIN_IRQ    DECK_GPIO_IO2

  #ifndef CONFIG_LOCODECK_ALT_PIN_RESET
  #define GPIO_PIN_RESET  DECK_GPIO_IO3
  #else
  #define GPIO_PIN_RESET  DECK_GPIO_IO4
  #endif

  #define EXTI_PortSource EXTI_PortSourceGPIOB
  #define EXTI_PinSource  EXTI_PinSource5
  #define EXTI_LineN      EXTI_Line5
#else
  #define GPIO_PIN_IRQ    DECK_GPIO_RX1
  #define GPIO_PIN_RESET  DECK_GPIO_TX1
  #define EXTI_PortSource EXTI_PortSourceGPIOC
  #define EXTI_PinSource  EXTI_PinSource11
  #define EXTI_LineN      EXTI_Line11
#endif


#define DEFAULT_RX_TIMEOUT 10000

// The anchor position can be set using parameters
// As an option you can set a static position in this file and set
// combinedAnchorPositionOk to enable sending the anchor rangings to the Kalman filter

static lpsAlgoOptions_t algoOptions = {
  // .userRequestedMode is the wanted algorithm, available as a parameter
#if defined(CONFIG_DECK_LOCO_ALGORITHM_TDOA2)
  .userRequestedMode = lpsMode_TDoA2,
#elif defined(CONFIG_DECK_LOCO_ALGORITHM_TDOA3)
  .userRequestedMode = lpsMode_TDoA3,
#elif defined(CONFIG_DECK_LOCO_ALGORITHM_TWR)
  .userRequestedMode = lpsMode_TWR,
#else
  .userRequestedMode = lpsMode_auto,
#endif
  // .currentRangingMode is the currently running algorithm, available as a log
  // lpsMode_auto is an impossible mode which forces initialization of the requested mode
  // at startup
  .currentRangingMode = lpsMode_auto,
  .modeAutoSearchActive = true,
  .modeAutoSearchDoInitialize = true,
};

struct {
  uwbAlgorithm_t *algorithm;
  char *name;
} algorithmsList[LPS_NUMBER_OF_ALGORITHMS + 1] = {
  [lpsMode_TWR] = {.algorithm = &uwbTwrTagAlgorithm, .name="TWR"},
  [lpsMode_TDoA2] = {.algorithm = &uwbTdoa2TagAlgorithm, .name="TDoA2"},
  [lpsMode_TDoA3] = {.algorithm = &uwbTdoa3TagAlgorithm, .name="TDoA3"},
};

#if defined(CONFIG_DECK_LOCO_ALGORITHM_TDOA2)
static uwbAlgorithm_t *algorithm = &uwbTdoa2TagAlgorithm;
#elif defined(CONFIG_DECK_LOCO_ALGORITHM_TDOA3)
static uwbAlgorithm_t *algorithm = &uwbTdoa3TagAlgorithm;
#else
static uwbAlgorithm_t *algorithm = &uwbTwrTagAlgorithm;
#endif

static bool isInit = false;
static TaskHandle_t uwbTaskHandle = 0;
static SemaphoreHandle_t algoSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static QueueHandle_t lppShortQueue;

static uint32_t timeout;

static STATS_CNT_RATE_DEFINE(spiWriteCount, 1000);
static STATS_CNT_RATE_DEFINE(spiReadCount, 1000);

// Memory read/write handling
#define MEM_LOCO_INFO             0x0000
#define MEM_LOCO_ANCHOR_BASE      0x1000
#define MEM_LOCO_ANCHOR_PAGE_SIZE 0x0100
#define MEM_LOCO_PAGE_LEN         (3 * sizeof(float) + 1)

#define MEM_ANCHOR_ID_LIST_LENGTH 256

#define MEM_LOCO2_ID_LIST          0x0000
#define MEM_LOCO2_ACTIVE_LIST      0x1000
#define MEM_LOCO2_ANCHOR_BASE      0x2000
#define MEM_LOCO2_ANCHOR_PAGE_SIZE 0x0100
#define MEM_LOCO2_PAGE_LEN         (3 * sizeof(float) + 1)

static uint32_t handleMemGetSize(void) { return MEM_LOCO_ANCHOR_BASE + MEM_LOCO_ANCHOR_PAGE_SIZE * 256; }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* dest);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_LOCO2,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = 0, // Write is not supported
};
static void buildAnchorMemList(const uint32_t memAddr, const uint8_t readLen, uint8_t* dest, const uint32_t pageBase_address, const uint8_t anchorCount, const uint8_t unsortedAnchorList[]);

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

static void rxFailedCallback(dwDevice_t * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveFailed);
}

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* dest) {
  bool result = false;

  static uint8_t unsortedAnchorList[MEM_ANCHOR_ID_LIST_LENGTH];

  if (memAddr >= MEM_LOCO2_ID_LIST && memAddr < MEM_LOCO2_ACTIVE_LIST) {
    uint8_t anchorCount = locoDeckGetAnchorIdList(unsortedAnchorList, MEM_ANCHOR_ID_LIST_LENGTH);
    buildAnchorMemList(memAddr, readLen, dest, MEM_LOCO2_ID_LIST, anchorCount, unsortedAnchorList);
    result = true;
  } else if (memAddr >= MEM_LOCO2_ACTIVE_LIST && memAddr < MEM_LOCO2_ANCHOR_BASE) {
    uint8_t anchorCount = locoDeckGetActiveAnchorIdList(unsortedAnchorList, MEM_ANCHOR_ID_LIST_LENGTH);
    buildAnchorMemList(memAddr, readLen, dest, MEM_LOCO2_ACTIVE_LIST, anchorCount, unsortedAnchorList);
    result = true;
  } else {
    if (memAddr >= MEM_LOCO2_ANCHOR_BASE) {
      uint32_t pageAddress = memAddr - MEM_LOCO2_ANCHOR_BASE;
      if ((pageAddress % MEM_LOCO2_ANCHOR_PAGE_SIZE) == 0 && MEM_LOCO2_PAGE_LEN == readLen) {
        uint32_t anchorId = pageAddress / MEM_LOCO2_ANCHOR_PAGE_SIZE;

        point_t position;
        memset(&position, 0, sizeof(position));
        locoDeckGetAnchorPosition(anchorId, &position);

        float* destAsFloat = (float*)dest;
        destAsFloat[0] = position.x;
        destAsFloat[1] = position.y;
        destAsFloat[2] = position.z;

        bool hasBeenSet = (position.timestamp != 0);
        dest[sizeof(float) * 3] = hasBeenSet;

        result = true;
      }
    }
  }

  return result;
}

static void buildAnchorMemList(const uint32_t memAddr, const uint8_t readLen, uint8_t* dest, const uint32_t pageBase_address, const uint8_t anchorCount, const uint8_t unsortedAnchorList[]) {
  for (int i = 0; i < readLen; i++) {
    int address = memAddr + i;
    int addressInPage = address - pageBase_address;
    uint8_t val = 0;

    if (addressInPage == 0) {
      val = anchorCount;
    } else {
      int anchorIndex = addressInPage - 1;
      if (anchorIndex < anchorCount) {
        val = unsortedAnchorList[anchorIndex];
      }
    }

    dest[i] = val;
  }
}

// This function is called from the memory sub system that runs in a different
// task, protect it from concurrent calls from this task
bool locoDeckGetAnchorPosition(const uint8_t anchorId, point_t* position)
{
  if (!isInit) {
    return false;
  }

  xSemaphoreTake(algoSemaphore, portMAX_DELAY);
  bool result = algorithm->getAnchorPosition(anchorId, position);
  xSemaphoreGive(algoSemaphore);
  return result;
}

// This function is called from the memory sub system that runs in a different
// task, protect it from concurrent calls from this task
uint8_t locoDeckGetAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  if (!isInit) {
    return 0;
  }

  xSemaphoreTake(algoSemaphore, portMAX_DELAY);
  uint8_t result = algorithm->getAnchorIdList(unorderedAnchorList, maxListSize);
  xSemaphoreGive(algoSemaphore);
  return result;
}

// This function is called from the memory sub system that runs in a different
// task, protect it from concurrent calls from this task
uint8_t locoDeckGetActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  if (!isInit) {
    return 0;
  }

  xSemaphoreTake(algoSemaphore, portMAX_DELAY);
  uint8_t result = algorithm->getActiveAnchorIdList(unorderedAnchorList, maxListSize);
  xSemaphoreGive(algoSemaphore);
  return result;
}

static bool switchToMode(const lpsMode_t newMode) {
  bool result = false;

  if (lpsMode_auto != newMode && newMode <= LPS_NUMBER_OF_ALGORITHMS) {
    algoOptions.currentRangingMode = newMode;
    algorithm = algorithmsList[algoOptions.currentRangingMode].algorithm;

    algorithm->init(dwm);
    timeout = algorithm->onEvent(dwm, eventTimeout);

    result = true;
  }

  return result;
}

static void autoModeSearchTryMode(const lpsMode_t newMode, const uint32_t now) {
  // Set up next time to check
  algoOptions.nextSwitchTick = now + LPS_AUTO_MODE_SWITCH_PERIOD;
  switchToMode(newMode);
}

static lpsMode_t autoModeSearchGetNextMode() {
  lpsMode_t newMode = algoOptions.currentRangingMode + 1;
  if (newMode > LPS_NUMBER_OF_ALGORITHMS) {
    newMode = lpsMode_TWR;
  }

  return newMode;
}

static void processAutoModeSwitching() {
  uint32_t now = xTaskGetTickCount();

  if (algoOptions.modeAutoSearchActive) {
    if (algoOptions.modeAutoSearchDoInitialize) {
      autoModeSearchTryMode(lpsMode_TDoA2, now);
      algoOptions.modeAutoSearchDoInitialize = false;
    } else {
      if (now > algoOptions.nextSwitchTick) {
        if (algorithm->isRangingOk()) {
          // We have found an algorithm, stop searching and lock to it.
          algoOptions.modeAutoSearchActive = false;
          DEBUG_PRINT("Automatic mode: detected %s\n", algorithmsList[algoOptions.currentRangingMode].name);
        } else {
          lpsMode_t newMode = autoModeSearchGetNextMode();
          autoModeSearchTryMode(newMode, now);
        }
      }
    }
  }
}

static void resetAutoSearchMode() {
  algoOptions.modeAutoSearchActive = true;
  algoOptions.modeAutoSearchDoInitialize = true;
}

static void handleModeSwitch() {
  if (algoOptions.userRequestedMode == lpsMode_auto) {
    processAutoModeSwitching();
  } else {
    resetAutoSearchMode();
    if (algoOptions.userRequestedMode != algoOptions.currentRangingMode) {
      if (switchToMode(algoOptions.userRequestedMode)) {
        DEBUG_PRINT("Switching to mode %s\n", algorithmsList[algoOptions.currentRangingMode].name);
      }
    }
  }
}

static void uwbTask(void* parameters) {
  lppShortQueue = xQueueCreate(10, sizeof(lpsLppShortPacket_t));

  algoOptions.currentRangingMode = lpsMode_auto;

  systemWaitStart();

  while(1) {
    xSemaphoreTake(algoSemaphore, portMAX_DELAY);
    handleModeSwitch();
    xSemaphoreGive(algoSemaphore);

    if (ulTaskNotifyTake(pdTRUE, timeout / portTICK_PERIOD_MS) > 0) {
      do{
        xSemaphoreTake(algoSemaphore, portMAX_DELAY);
        dwHandleInterrupt(dwm);
        xSemaphoreGive(algoSemaphore);
      } while(digitalRead(GPIO_PIN_IRQ) != 0);
    } else {
      xSemaphoreTake(algoSemaphore, portMAX_DELAY);
      timeout = algorithm->onEvent(dwm, eventTimeout);
      xSemaphoreGive(algoSemaphore);
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
  STATS_CNT_RATE_EVENT(&spiWriteCount);
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
  STATS_CNT_RATE_EVENT(&spiReadCount);
}

#if CONFIG_DECK_LOCODECK_USE_ALT_PINS
  void __attribute__((used)) EXTI5_Callback(void)
#else
  void __attribute__((used)) EXTI11_Callback(void)
#endif
  {
    portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

    // Unlock interrupt handling task
    vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken) {
      portYIELD();
    }
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

  spiBegin();

  // Set up interrupt
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init pins
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);

  // Reset the DW1000 chip
  digitalWrite(GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(GPIO_PIN_RESET, 1);
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
  dwAttachReceiveFailedHandler(dwm, rxFailedCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);


  #ifdef CONFIG_DECK_LOCO_LONGER_RANGE
  dwEnableMode(dwm, MODE_SHORTDATA_MID_ACCURACY);
  #else
  dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
  #endif

  dwSetChannel(dwm, CHANNEL_2);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  #ifdef CONFIG_DECK_LOCO_FULL_TX_POWER
  dwUseSmartPower(dwm, false);
  dwSetTxPower(dwm, 0x1F1F1F1Ful);
  #else
  dwUseSmartPower(dwm, true);
  #endif

  dwSetReceiveWaitTimeout(dwm, DEFAULT_RX_TIMEOUT);

  dwCommitConfiguration(dwm);

  memoryRegisterHandler(&memDef);

  algoSemaphore= xSemaphoreCreateMutex();

  xTaskCreate(uwbTask, LPS_DECK_TASK_NAME, LPS_DECK_STACKSIZE, NULL,
                    LPS_DECK_TASK_PRI, &uwbTaskHandle);

  isInit = true;
}

uint16_t locoDeckGetRangingState() {
  return algoOptions.rangingState;
}

void locoDeckSetRangingState(const uint16_t newState) {
  algoOptions.rangingState = newState;
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
  .name = "bcLoco",

#ifdef CONFIG_DECK_LOCODECK_USE_ALT_PINS
  #ifndef CONFIG_LOCODECK_ALT_PIN_RESET
  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_3,
  #else
  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_4,
  #endif
#else
   // (PC10/PC11 is UART1 TX/RX)
  .usedGpio = DECK_USING_IO_1 | DECK_USING_PC10 | DECK_USING_PC11,
#endif
  .usedPeriph = DECK_USING_SPI,
  .requiredEstimator = StateEstimatorTypeKalman,
  #ifdef LOCODECK_NO_LOW_INTERFERENCE
  .requiredLowInterferenceRadioMode = false,
  #else
  .requiredLowInterferenceRadioMode = true,
  #endif


  .init = dwm1000Init,
  .test = dwm1000Test,
};

DECK_DRIVER(dwm1000_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Deprecated (removed after August 2023). Use the "deck.bcLoco" parameter instead.
 *
 * Nonzero if [Loco positioning deck](%https://store.bitcraze.io/products/loco-positioning-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &isInit)

/**
 * @brief Nonzero if [Loco positioning deck](%https://store.bitcraze.io/products/loco-positioning-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &isInit)

PARAM_GROUP_STOP(deck)

LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, state, &algoOptions.rangingState)
LOG_GROUP_STOP(ranging)

/**
 * Log group for basic information about the Loco Positioning System
 */
LOG_GROUP_START(loco)

/**
 * @brief The current mode of the Loco Positioning system
 *
 * | Value | Mode   | \n
 * | -     | -      | \n
 * |   1   | TWR    | \n
 * |   2   | TDoA 2 | \n
 * |   3   | TDoA 3 | \n
 */
LOG_ADD_CORE(LOG_UINT8, mode, &algoOptions.currentRangingMode)

STATS_CNT_RATE_LOG_ADD(spiWr, &spiWriteCount)
STATS_CNT_RATE_LOG_ADD(spiRe, &spiReadCount)
LOG_GROUP_STOP(loco)

/**
 * The Loco Positioning System implements three different positioning modes:
 * Two Way Ranging (TWR), Time Difference of Arrival 2 (TDoA 2) and Time Difference of Arrival 3 (TDoA 3)
 *
 * ### TWR mode
 *
 * In this mode, the tag pings the anchors in sequence, this allows it to
 * measure the distance between the tag and the anchors. Using this information
 * a theoretical minimum of 4 Anchors is required to calculate the 3D position
 * of a Tag, but a more realistic number is 6 to add redundancy and accuracy.
 * This mode is the most accurate mode and also works when the tag or quad
 * leaves the space delimited by the anchors. The tag is actively communicating
 * with the anchors in a time slotted fashion and in this mode only one tag or
 * quad can be positioned with a maximum of 8 anchors.
 *
 * ### TDoA 2 mode
 *
 * In TDoA 2 mode, the anchor system is continuously sending synchronization
 * packets. A tag listening to these packets can calculate the relative
 * distance to two anchors by measuring the time difference of arrival of the
 * packets. From the TDoA information it is possible to calculate the 3D
 * position in space. In this mode the tag is only passively listening, so new
 * tags do not add any load to the system which makes it possible to position
 * any number of tags or quads simultaneously. This makes it a perfect
 * mode for swarming.
 *
 * Compared to TWR, TDoA 2 is more restrictive when it comes to the space where
 * positioning works, ideally the tag should be within, or very close to,
 * the space delimited by the anchor system. This means that TDoA 2 works best
 * with 8 anchors placed in the corners of the flying space. In this space the
 * accuracy and precision is comparable to TWR.

 * In this mode the anchor system is time slotted and synchronized and the
 * number of anchors is limited to 8.
 *
 * ### TDoA 3 mode
 *
 * The TDoA 3 mode has many similarities with TDoA 2 and supports any number
 * of tags or quads. The main difference is that the time slotted scheme of
 * TDoA 2 has been replaced by a randomized transmission schedule which makes
 * it possible to add more anchors. By adding more anchors the system can be
 * scaled to larger spaces or span multiple rooms without line of sight between
 * all anchors. It also makes it more robust and can handle loss or addition of
 * anchors dynamically. The estimated position in this mode might be slightly
 * more noisy compared to TDoA 2.
 */
PARAM_GROUP_START(loco)

/**
 * @brief The Loco positioning mode to use (default: 0)
 *
 * | Value | Mode   |\n
 * | -     | -      |\n
 * |   0   | Auto   |\n
 * |   1   | TWR    |\n
 * |   2   | TDoA 2 |\n
 * |   3   | TDoA 3 |\n
 */
PARAM_ADD_CORE(PARAM_UINT8, mode, &algoOptions.userRequestedMode)

PARAM_GROUP_STOP(loco)
