/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * bccam_deck.c - Deck driver for the WiFi camera deck
 */

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#include "deck.h"
#include "deckctrl_gpio.h"
#include "log.h"
#include "param.h"
#include "bccam_uart_service.h"

#define DEBUG_MODULE "BCCAM"
#include "debug.h"

// Deck controller logical GPIO mapping (STM32 deck-ctrl MCU)
#define GPIO_UART_SEL DECKCTRL_GPIO_PIN_11 // PA0 - select expansion UART1
#define GPIO_PWR_EN   DECKCTRL_GPIO_PIN_10 // PC15 - power enable

static bool isInit = false;
static uint32_t newFwSize = 0;

static uint8_t driverStateLog;
static uint8_t linkStateLog;
static uint8_t txCreditLog;
static uint8_t rxSlotsLog;
static uint16_t negotiatedPayloadLog;
static uint16_t rxFramesLog;
static uint16_t txFramesLog;
static uint16_t rxBytesLog;
static uint16_t txBytesLog;
static uint16_t rxCrcErrorsLog;
static uint16_t linkFaultsLog;
static uint16_t txFlushesLog;
static int32_t lastLinkErrorLog;

// ---------------------------------------------------------------------------
// Deck memory subsystem callbacks (firmware update)
// ---------------------------------------------------------------------------

static void bcCamRefreshLogsFromService(void) {
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  driverStateLog = (uint8_t)status.service_state;
  linkStateLog = status.link_state;
  txCreditLog = status.tx_credit;
  rxSlotsLog = status.rx_slots;
  negotiatedPayloadLog = status.negotiated_payload;
  rxFramesLog = status.rx_frames;
  txFramesLog = status.tx_frames;
  rxBytesLog = status.rx_bytes;
  txBytesLog = status.tx_bytes;
  rxCrcErrorsLog = status.rx_crc_errors;
  linkFaultsLog = status.link_faults;
  txFlushesLog = status.tx_flushes;
  lastLinkErrorLog = status.last_error;
}

static uint8_t bcCamLogDriverState(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return (uint8_t)status.service_state;
}

static uint8_t bcCamLogLinkState(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.link_state;
}

static uint8_t bcCamLogTxCredit(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.tx_credit;
}

static uint8_t bcCamLogRxSlots(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.rx_slots;
}

static uint16_t bcCamLogPayload(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.negotiated_payload;
}

static uint16_t bcCamLogRxFrames(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.rx_frames;
}

static uint16_t bcCamLogTxFrames(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.tx_frames;
}

static uint16_t bcCamLogRxBytes(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.rx_bytes;
}

static uint16_t bcCamLogTxBytes(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.tx_bytes;
}

static uint16_t bcCamLogCrcErrors(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.rx_crc_errors;
}

static uint16_t bcCamLogFaults(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.link_faults;
}

static uint16_t bcCamLogTxFlushes(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.tx_flushes;
}

static int32_t bcCamLogLastErr(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.last_error;
}

static uint8_t bcCamLogControlDone(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.control_probe_done;
}

static uint8_t bcCamLogSchemaCount(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  bccam_uart_service_status_t status;
  bccam_uart_service_get_status(&status);
  return status.control_schema_count;
}

static logByFunction_t driverStateLogDef = {
  .acquireUInt8 = bcCamLogDriverState,
};
static logByFunction_t linkStateLogDef = {
  .acquireUInt8 = bcCamLogLinkState,
};
static logByFunction_t txCreditLogDef = {
  .acquireUInt8 = bcCamLogTxCredit,
};
static logByFunction_t rxSlotsLogDef = {
  .acquireUInt8 = bcCamLogRxSlots,
};
static logByFunction_t negotiatedPayloadLogDef = {
  .acquireUInt16 = bcCamLogPayload,
};
static logByFunction_t rxFramesLogDef = {
  .acquireUInt16 = bcCamLogRxFrames,
};
static logByFunction_t txFramesLogDef = {
  .acquireUInt16 = bcCamLogTxFrames,
};
static logByFunction_t rxBytesLogDef = {
  .acquireUInt16 = bcCamLogRxBytes,
};
static logByFunction_t txBytesLogDef = {
  .acquireUInt16 = bcCamLogTxBytes,
};
static logByFunction_t rxCrcErrorsLogDef = {
  .acquireUInt16 = bcCamLogCrcErrors,
};
static logByFunction_t linkFaultsLogDef = {
  .acquireUInt16 = bcCamLogFaults,
};
static logByFunction_t txFlushesLogDef = {
  .acquireUInt16 = bcCamLogTxFlushes,
};
static logByFunction_t lastLinkErrorLogDef = {
  .acquireInt32 = bcCamLogLastErr,
};
static logByFunction_t controlDoneLogDef = {
  .acquireUInt8 = bcCamLogControlDone,
};
static logByFunction_t schemaCountLogDef = {
  .acquireUInt8 = bcCamLogSchemaCount,
};

static bool bcCamWriteFlash(const uint32_t memAddr, const uint8_t writeLen,
                            const uint8_t *buffer, const DeckMemDef_t *memDef) {
  (void)memDef;
  return bccam_uart_service_write_flash(memAddr, writeLen, buffer, newFwSize);
}

static bool bcCamReadFlash(const uint32_t memAddr, const uint8_t readLen,
                           uint8_t *buffer) {
  return bccam_uart_service_read_flash(memAddr, readLen, buffer);
}

static uint8_t bcCamPropertiesQuery(void) {
  bcCamRefreshLogsFromService();

  const bccam_uart_service_state_t state =
    (bccam_uart_service_state_t)driverStateLog;
  uint8_t result = DECK_MEMORY_MASK_SUPPORTS_HOT_RESTART;

  if (isInit) {
    result |= DECK_MEMORY_MASK_STARTED;
  }

  if (bccam_uart_service_state_has_active_bootloader(state)) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void bcCamResetToBootloader(void) {
  bccam_uart_service_request_bootloader();
}

static void bcCamResetToFw(void) {
  bccam_uart_service_request_firmware();
}

static const DeckMemDef_t bcCamMemoryDef = {
  .write = bcCamWriteFlash,
  .read = bcCamReadFlash,
  .properties = bcCamPropertiesQuery,
  .supportsUpgrade = true,
  .newFwSizeP = &newFwSize,
  .id = "qcc",
  .commandResetToBootloader = bcCamResetToBootloader,
  .commandResetToFw = bcCamResetToFw,
};

// ---------------------------------------------------------------------------
// Deck driver init / test
// ---------------------------------------------------------------------------

static bool gpioSetup(DeckInfo *info, DeckCtrlGPIOPin pin, uint32_t direction, uint32_t value) {
  if (!deckctrl_gpio_set_direction(info, pin, direction)) {
    return false;
  }
  if (direction == OUTPUT) {
    return deckctrl_gpio_write(info, pin, value);
  }
  return true;
}

static void bcCamDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  if (info == NULL || info->boardRevision == NULL ||
      strcmp(info->boardRevision, "G") != 0) {
    DEBUG_PRINT("Incompatible bcCam deck revision: expected G, got %s\n",
                (info != NULL && info->boardRevision != NULL) ?
                  info->boardRevision : "<missing>");
    return;
  }

  // Power on the deck
  if (!gpioSetup(info, GPIO_PWR_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable deck power\n");
    return;
  }

  vTaskDelay(M2T(50));

  // Select expansion UART1 for the STM32/QCC protocol link.
  if (!gpioSetup(info, GPIO_UART_SEL, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable UART1\n");
    return;
  }

  bccam_uart_service_init(info);

  isInit = true;
  DEBUG_PRINT("bcCam deck initialized\n");
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_deck_test_reset(void) {
  isInit = false;
}
#endif

static bool bcCamDeckTest(void) {
  if (!isInit) {
    return false;
  }

  bcCamRefreshLogsFromService();

  DEBUG_PRINT("bcCam deck test passed\n");
  return true;
}

static const DeckDriver bccam_deck = {
  .vid = 0xBC,
  .pid = 0x15,
  .name = "bcCam",

  .usedPeriph = DECK_USING_UART1,

  .memoryDef = &bcCamMemoryDef,

  .init = bcCamDeckInit,
  .test = bcCamDeckTest,
};

DECK_DRIVER(bccam_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if bcCam deck is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcCam, &isInit)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCamLink, &bccam_uart_service_link_state_log)

PARAM_GROUP_STOP(deck)

LOG_GROUP_START(bcCam)
LOG_ADD_BY_FUNCTION(LOG_UINT8, driverState, &driverStateLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT8, linkState, &linkStateLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, payload, &negotiatedPayloadLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, rxFrames, &rxFramesLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, txFrames, &txFramesLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, rxBytes, &rxBytesLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, txBytes, &txBytesLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, crcErrors, &rxCrcErrorsLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, faults, &linkFaultsLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT8, txCredit, &txCreditLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT8, rxSlots, &rxSlotsLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT16, txFlushes, &txFlushesLogDef)
LOG_ADD_BY_FUNCTION(LOG_INT32, lastErr, &lastLinkErrorLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT8, controlDone, &controlDoneLogDef)
LOG_ADD_BY_FUNCTION(LOG_UINT8, schemaCount, &schemaCountLogDef)
LOG_GROUP_STOP(bcCam)
