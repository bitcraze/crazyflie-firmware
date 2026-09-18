/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2026, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/* flowdeck_v3.c: Flow deck V3 driver
 *
 * Flow and range measurements are received from the RP2350 on the deck over UART.
 * The deck is powered through the deck controller, which also gives access to the
 * RP2350 flash for firmware upgrades (deck memory bcFlow3:rp2350).
 */

#define DEBUG_MODULE "FlowDeckV3"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "debug.h"
#include "param.h"
#include "log.h"
#include "system.h"
#include "uart1.h"
#include "range.h"

#include "cf_math.h"

#include "flowdeck_v3.h"
#include "stabilizer_types.h"

#include "deckctrl_gpio.h"
#include "deckctrl_spi.h"


#define OULIER_LIMIT 100
#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]


static uint8_t resolution = 0x4c;
static flowMeasurement_t flowData;

static uint8_t outlierCount = 0;
static float stdFlow = 2.0f;

static bool isInit = false;
static flowdeckV3UartFrame_t rxFrame;

// Raw values of the last received frame, for bring-up and testing
static uint8_t flowMotionLog;
static int16_t flowDeltaXLog;
static int16_t flowDeltaYLog;
static uint16_t flowShutterLog;
static uint16_t flowRangeLog;
static uint16_t flowFrameCountLog;

// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

// Turn on adaptive standard deviation for the kalman filter
static bool useAdaptiveStd = false;

// Set standard deviation flow
// (will not work if useAdaptiveStd is on)
static float flowStdFixed = 2.0f;

// Range sensor measurement noise model
static const float expPointA = 1.0f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 1.3f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;


static void flowdeckV3ReadByte(uint8_t *byte) {
  while (uart1bytesAvailable() < 1) {
    vTaskDelay(M2T(1));
  }
  uart1Getchar((char *) byte);
}

// Used inside a frame, where a missing byte means the deck stopped sending
// mid-frame. Returns false on timeout, so we can go back to looking for a header.
static bool flowdeckV3ReadByteTimeout(uint8_t *byte, const uint32_t timeoutMs) {
  const TickType_t start = xTaskGetTickCount();

  while (uart1bytesAvailable() < 1) {
    if ((xTaskGetTickCount() - start) > M2T(timeoutMs)) {
      return false;
    }
    vTaskDelay(M2T(1));
  }
  uart1Getchar((char *) byte);

  return true;
}

// The deck sends bring-up messages as text, print them on the console
static void flowdeckV3ReadText(void) {
  uint8_t length;
  if (!flowdeckV3ReadByteTimeout(&length, FLOWDECK_V3_UART_FRAME_TIMEOUT_MS)) {
    return;
  }

  char text[FLOWDECK_V3_UART_TEXT_MAX_LENGTH + 1];
  uint32_t index = 0;

  for (uint8_t i = 0; i < length; i++) {
    uint8_t byte;
    if (!flowdeckV3ReadByteTimeout(&byte, FLOWDECK_V3_UART_FRAME_TIMEOUT_MS)) {
      break;
    }
    if (index < FLOWDECK_V3_UART_TEXT_MAX_LENGTH) {
      text[index++] = (char)byte;
    }
  }
  text[index] = 0;

  if (index > 0) {
    DEBUG_PRINT("%s\n", text);
  }
}

// Returns true when a measurement frame was read, false when the deck sent something else
static bool flowdeckV3ReadData(flowdeckV3UartFrame_t *frame) {
  uint8_t *raw = (uint8_t *)frame;
  uint16_t header = 0;

  // the deck sends a sync header before the start of each frame.
  // Wait for it.
  while (header != FLOWDECK_V3_UART_SYNC_HEADER && header != FLOWDECK_V3_UART_TEXT_HEADER) {
    uint8_t byte;
    flowdeckV3ReadByte(&byte);
    header = (header << 8) | byte;
  }

  if (header == FLOWDECK_V3_UART_TEXT_HEADER) {
    flowdeckV3ReadText();
    return false;
  }

  while (uart1bytesAvailable() < sizeof(*frame)) {
    vTaskDelay(M2T(1));
  }

  for (uint32_t i = 0; i < sizeof(*frame); i++) {
    uart1Getchar((char *)&raw[i]);
  }

  return true;
}

static void flowdeckV3Task(void *param) {
  (void)param;

  uart1Init(FLOWDECK_V3_UART_BAUDRATE);
  systemWaitStart();

  ASSERT(uart1QueueMaxLength() >= sizeof(flowdeckV3UartFrame_t));

  uint32_t frameCount = 0;
  while (1) {
    if (!flowdeckV3ReadData(&rxFrame)) {
      continue;
    }

    // Flow -------------------------------------------------------
    // Flip motion information to comply with sensor mounting
    // (might need to be changed if mounted differently)
    int16_t accpx = (int16_t) -((int32_t) rxFrame.deltaY + INT16_MIN);
    int16_t accpy = (int16_t) -((int32_t) rxFrame.deltaX + INT16_MIN);

    // Logged before the outlier removal, so that the raw sensor output is visible
    flowMotionLog = (uint8_t)rxFrame.motion;
    flowDeltaXLog = accpx;
    flowDeltaYLog = accpy;
    flowShutterLog = rxFrame.shutter;
    flowRangeLog = rxFrame.rangeMm;
    flowFrameCountLog++;

    // Outlier removal
    if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {
       if (useAdaptiveStd) {
        // The standard deviation is fitted by measurements flying over low and high texture
        //   and looking at the shutter time
        float shutter_f = (float)rxFrame.shutter;
        stdFlow=0.0007984f *shutter_f + 0.4335f;

        // The formula with the amount of features instead
        /*float squal_f = (float)currentMotion.squal;
        stdFlow =  -0.01257f * squal_f + 4.406f; */
        if (stdFlow < 0.1f) stdFlow=0.1f;
      } else {
        stdFlow = flowStdFixed;
      }
    
      flowData.stdDevX = stdFlow * 0.1f;
      flowData.stdDevY = stdFlow * 0.1f;
      flowData.dt = 1.0f / 126.0f;
      frameCount++;

      flowData.dpixelx = (float) accpx;
      flowData.dpixely = (float) accpy;
      
      // Push measurements into the estimator if flow is not disabled
      // and the PMW flow sensor indicates motion detection
      if (!useFlowDisabled && rxFrame.motion & 0x80) {
        estimatorEnqueueFlow(&flowData);
      }
    } else {
      outlierCount++;
    }

    // Z-range -------------------------------------------------------
    rangeSet(rangeDown, rxFrame.rangeMm / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (rxFrame.rangeMm < RANGE_OUTLIER_LIMIT) {
      float distance = (float) rxFrame.rangeMm * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }

  }
}


// Deck controller GPIO mapping
#define GPIO_RP_RUN   DECKCTRL_GPIO_PIN_0   // PA0 - RP2350 RUN, low holds the RP2350 in reset
#define GPIO_PWR_EN   DECKCTRL_GPIO_PIN_12  // PC15 - Enables the 3V0 and 1V8 regulators

#define POWER_UP_DELAY_MS 10
#define RESET_DELAY_MS    10

// SPI flash (W25Q32RV)
#define FLASH_SIZE          (4 * 1024 * 1024)
#define FLASH_PAGE_SIZE     256
#define FLASH_BLOCK_SIZE    0x10000

#define FLASH_CMD_WRITE_ENABLE  0x06
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_READ          0x03
#define FLASH_CMD_PAGE_PROGRAM  0x02
#define FLASH_CMD_BLOCK_ERASE   0xD8
#define FLASH_CMD_JEDEC_ID      0x9F
#define FLASH_CMD_RELEASE_PD    0xAB
#define FLASH_CMD_ENABLE_RESET  0x66
#define FLASH_CMD_RESET         0x99

#define FLASH_STATUS_BUSY       0x01

// Command byte and 24-bit address
#define FLASH_CMD_HEADER_SIZE   4

#define FLASH_PROGRAM_TIMEOUT_MS  50
#define FLASH_ERASE_TIMEOUT_MS    3000
#define FLASH_ERASE_POLL_MS       10

static DeckInfo* flowDeckInfo = NULL;
static bool isInFlashMode = false;

// Firmware upgrade state, writes are expected to be sequential starting at address 0
static uint32_t newFwSize = 0;
static uint32_t nextAddress = 0;
static uint32_t erasedUntil = 0;
static bool isImageComplete = false;
// Page program command followed by the data of the page being filled
static uint8_t programBuffer[FLASH_CMD_HEADER_SIZE + FLASH_PAGE_SIZE];
static uint16_t pageFill = 0;


static void resetUpgradeState(void) {
  nextAddress = 0;
  erasedUntil = 0;
  pageFill = 0;
  isImageComplete = false;
}

static void setCommandAddress(uint8_t* command, const uint32_t address) {
  command[1] = (address >> 16) & 0xFF;
  command[2] = (address >> 8) & 0xFF;
  command[3] = address & 0xFF;
}

static bool flashCommand(const uint8_t command) {
  return deckctrl_spi_transfer(flowDeckInfo, &command, 1, NULL, 0, false);
}

static bool flashWaitReady(const uint32_t timeoutMs, const uint32_t pollMs) {
  const TickType_t start = xTaskGetTickCount();
  const uint8_t command = FLASH_CMD_READ_STATUS;
  uint8_t status;

  while (true) {
    if (!deckctrl_spi_transfer(flowDeckInfo, &command, 1, &status, 1, false)) {
      return false;
    }

    if ((status & FLASH_STATUS_BUSY) == 0) {
      return true;
    }

    if ((xTaskGetTickCount() - start) > M2T(timeoutMs)) {
      DEBUG_PRINT("Flash busy timeout\n");
      return false;
    }

    vTaskDelay(M2T(pollMs));
  }
}

static bool flashEraseBlock(const uint32_t address) {
  uint8_t command[FLASH_CMD_HEADER_SIZE] = {FLASH_CMD_BLOCK_ERASE};
  setCommandAddress(command, address);

  return flashCommand(FLASH_CMD_WRITE_ENABLE) &&
    deckctrl_spi_transfer(flowDeckInfo, command, sizeof(command), NULL, 0, false) &&
    flashWaitReady(FLASH_ERASE_TIMEOUT_MS, FLASH_ERASE_POLL_MS);
}

// Program the buffered page, erasing blocks on the way when needed
static bool flashProgramBufferedPage(void) {
  if (pageFill == 0) {
    return true;
  }

  const uint32_t pageAddress = nextAddress - pageFill;
  const uint16_t length = pageFill;
  pageFill = 0;

  while (erasedUntil < pageAddress + length) {
    if (!flashEraseBlock(erasedUntil)) {
      DEBUG_PRINT("Failed to erase block at 0x%X\n", (unsigned int)erasedUntil);
      return false;
    }
    erasedUntil += FLASH_BLOCK_SIZE;
  }

  programBuffer[0] = FLASH_CMD_PAGE_PROGRAM;
  setCommandAddress(programBuffer, pageAddress);

  const bool result = flashCommand(FLASH_CMD_WRITE_ENABLE) &&
    deckctrl_spi_transfer(flowDeckInfo, programBuffer, FLASH_CMD_HEADER_SIZE + length, NULL, 0, false) &&
    flashWaitReady(FLASH_PROGRAM_TIMEOUT_MS, 1);

  if (!result) {
    DEBUG_PRINT("Failed to program page at 0x%X\n", (unsigned int)pageAddress);
  }

  return result;
}

// The RP2350 may have left the flash in continuous read mode or in deep power-down
static bool flashWakeUp(void) {
  // Clocking out 0xFF with CS asserted ends continuous read mode
  static const uint8_t modeBitReset[] = {0xFF, 0xFF};
  if (!deckctrl_spi_transfer(flowDeckInfo, modeBitReset, sizeof(modeBitReset), NULL, 0, false)) {
    return false;
  }

  if (!flashCommand(FLASH_CMD_RELEASE_PD)) {
    return false;
  }
  vTaskDelay(M2T(1));

  if (!flashCommand(FLASH_CMD_ENABLE_RESET) || !flashCommand(FLASH_CMD_RESET)) {
    return false;
  }
  vTaskDelay(M2T(1));

  const uint8_t command = FLASH_CMD_JEDEC_ID;
  uint8_t id[3];
  if (!deckctrl_spi_transfer(flowDeckInfo, &command, 1, id, sizeof(id), false)) {
    return false;
  }

  DEBUG_PRINT("Flash JEDEC ID: %X %X %X\n", (unsigned int)id[0], (unsigned int)id[1], (unsigned int)id[2]);

  // A missing or silent flash reads as all zeros or all ones
  return id[0] != 0x00 && id[0] != 0xFF;
}

static bool rp2350Restart(void) {
  bool result = deckctrl_gpio_write(flowDeckInfo, GPIO_RP_RUN, LOW);
  vTaskDelay(M2T(RESET_DELAY_MS));
  result = result && deckctrl_gpio_write(flowDeckInfo, GPIO_RP_RUN, HIGH);
  return result;
}

static void enterFlashMode(void) {
  resetUpgradeState();

  // The RP2350 releases the flash pins while in reset
  if (!deckctrl_gpio_write(flowDeckInfo, GPIO_RP_RUN, LOW)) {
    DEBUG_PRINT("Failed to reset RP2350\n");
    return;
  }
  vTaskDelay(M2T(RESET_DELAY_MS));

  if (!deckctrl_spi_enable(flowDeckInfo, DECKCTRL_SPI_MODE_0, DECKCTRL_SPI_DIV_2)) {
    DEBUG_PRINT("Failed to enable SPI bridge\n");
    rp2350Restart();
    return;
  }

  if (!flashWakeUp()) {
    DEBUG_PRINT("No response from flash\n");
    if (deckctrl_spi_disable(flowDeckInfo)) {
      rp2350Restart();
    }
    return;
  }

  isInFlashMode = true;
  DEBUG_PRINT("RP2350 in reset, flash accessible\n");
}

static void exitFlashMode(void) {
  // Write the last page if the image size was not known
  flashProgramBufferedPage();

  // The SPI pins must be released before the RP2350 leaves reset, otherwise both drive the bus
  if (!deckctrl_spi_disable(flowDeckInfo)) {
    DEBUG_PRINT("Failed to disable SPI bridge, keeping RP2350 in reset\n");
    return;
  }
  isInFlashMode = false;

  deckctrl_gpio_write(flowDeckInfo, GPIO_RP_RUN, HIGH);
  DEBUG_PRINT("RP2350 started\n");
}

static bool flowWriteFlash(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer, const DeckMemDef_t* memDef) {
  if (!isInFlashMode) {
    return false;
  }

  uint32_t address = memAddr;
  uint32_t length = writeLen;
  const uint8_t* data = buffer;

  if (address == 0 && isImageComplete) {
    resetUpgradeState();
  }

  // The client resends a write if the reply is late, for instance during an erase. Skip data we already have.
  if (address < nextAddress) {
    const uint32_t alreadyWritten = nextAddress - address;
    if (alreadyWritten >= length) {
      return true;
    }
    address += alreadyWritten;
    data += alreadyWritten;
    length -= alreadyWritten;
  }

  if (address != nextAddress) {
    DEBUG_PRINT("Non sequential write at 0x%X, expected 0x%X\n", (unsigned int)address, (unsigned int)nextAddress);
    return false;
  }

  if (address + length > FLASH_SIZE) {
    return false;
  }

  while (length > 0) {
    uint32_t chunk = FLASH_PAGE_SIZE - pageFill;
    if (chunk > length) {
      chunk = length;
    }

    memcpy(&programBuffer[FLASH_CMD_HEADER_SIZE + pageFill], data, chunk);
    pageFill += chunk;
    nextAddress += chunk;
    data += chunk;
    length -= chunk;

    if (pageFill == FLASH_PAGE_SIZE) {
      if (!flashProgramBufferedPage()) {
        return false;
      }
    }
  }

  if (newFwSize > 0 && nextAddress >= newFwSize) {
    if (!flashProgramBufferedPage()) {
      return false;
    }
    isImageComplete = true;
    DEBUG_PRINT("Wrote %d bytes to flash\n", (unsigned int)nextAddress);
  }

  return true;
}

static bool flowReadFlash(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  if (!isInFlashMode) {
    return false;
  }

  if (memAddr + readLen > FLASH_SIZE) {
    return false;
  }

  uint8_t command[FLASH_CMD_HEADER_SIZE] = {FLASH_CMD_READ};
  setCommandAddress(command, memAddr);

  return deckctrl_spi_transfer(flowDeckInfo, command, sizeof(command), buffer, readLen, false);
}

static uint8_t flowPropertiesQuery(void) {
  uint8_t result = 0;

  if (isInit) {
    result |= DECK_MEMORY_MASK_STARTED;
  }

  if (isInFlashMode) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void flowResetToBootloader(void) {
  if (!isInit) {
    return;
  }

  enterFlashMode();
}

static void flowResetToFw(void) {
  if (!isInit) {
    return;
  }

  if (isInFlashMode) {
    exitFlashMode();
  } else {
    rp2350Restart();
  }
}

static const DeckMemDef_t memoryDef = {
  .write = flowWriteFlash,
  .read = flowReadFlash,
  .properties = flowPropertiesQuery,
  .supportsUpgrade = true,
  .newFwSizeP = &newFwSize,
  .id = "rp2350",
  .commandResetToBootloader = flowResetToBootloader,
  .commandResetToFw = flowResetToFw,
};

static void flowdeck3Init(DeckInfo *info) {
  if (isInit) {
    return;
  }

  flowDeckInfo = info;

  // A pin switched to output starts low: power on with the RP2350 held in reset, then let it boot
  bool powered = deckctrl_gpio_set_direction(info, GPIO_RP_RUN, OUTPUT) &&
    deckctrl_gpio_set_direction(info, GPIO_PWR_EN, OUTPUT) &&
    deckctrl_gpio_write(info, GPIO_PWR_EN, HIGH);

  vTaskDelay(M2T(POWER_UP_DELAY_MS));

  powered = powered && deckctrl_gpio_write(info, GPIO_RP_RUN, HIGH);

  if (!powered) {
    DEBUG_PRINT("Failed to power the deck\n");
    return;
  }
  
  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  xTaskCreate(flowdeckV3Task, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL,
              FLOW_TASK_PRI, NULL);

  isInit = true;
}

static bool flowdeck3Test(void) {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing the flow V3 deck\n");
    return false;
  }

  return uart1Test();
}

static const DeckDriver flowdeck3_deck = {
  .vid = 0xBC,
  .pid = 0x16,
  .name = "bcFlow3",

  // IO_3 pulls the flash CS low through a resistor (RP2350 BOOTSEL), it must not be driven by others
  .usedGpio = DECK_USING_IO_3,
  .usedPeriph = DECK_USING_UART1,
  .requiredEstimator = StateEstimatorTypeKalman,

  .memoryDef = &memoryDef,

  .init = flowdeck3Init,
  .test = flowdeck3Test,
};

DECK_DRIVER(flowdeck3_deck);

/**
 * Raw data from the Flow v3 deck, as received over the UART
 */
LOG_GROUP_START(flow3)
/**
 * @brief Motion register of the flow sensor, bit 7 is set when motion was detected
 */
LOG_ADD(LOG_UINT8, motion, &flowMotionLog)
/**
 * @brief Flow movement in x, in pixels since the last frame
 */
LOG_ADD(LOG_INT16, deltaX, &flowDeltaXLog)
/**
 * @brief Flow movement in y, in pixels since the last frame
 */
LOG_ADD(LOG_INT16, deltaY, &flowDeltaYLog)
/**
 * @brief Shutter value of the flow sensor
 */
LOG_ADD(LOG_UINT16, shutter, &flowShutterLog)
/**
 * @brief Distance measured by the range sensor [mm]
 */
LOG_ADD(LOG_UINT16, range, &flowRangeLog)
/**
 * @brief Number of frames received from the deck
 */
LOG_ADD(LOG_UINT16, frames, &flowFrameCountLog)
LOG_GROUP_STOP(flow3)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if the Flow deck V3 template driver is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow3, &isInit)

PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(flow)
PARAM_ADD(PARAM_UINT8, resolution, &resolution)
PARAM_GROUP_STOP(flow)
