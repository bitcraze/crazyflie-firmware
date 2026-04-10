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
 * bccam_deck.c - Deck driver for the bcCam deck (QCC744)
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "param.h"
#include "log.h"
#include "i2cdev.h"
#include "deckctrl_gpio.h"
#include "uart1.h"
#include "console.h"
#include "bccam_deck.h"

#define DEBUG_MODULE "BCCAM"
#include "debug.h"

#define BCCAM_UART_BAUDRATE       2000000
#define BCCAM_ISP_BAUDRATE        500000
#define BCCAM_TASK_STACKSIZE  configMINIMAL_STACK_SIZE
#define BCCAM_TASK_PRI        tskIDLE_PRIORITY

// Deck controller GPIO pin mapping (STM32 deck-ctrl MCU)
#define GPIO_UART1_EN   DECKCTRL_GPIO_PIN_0   // PA0 - UART1 enable
#define GPIO_UART2_EN   DECKCTRL_GPIO_PIN_1   // PA1 - UART2 enable
#define GPIO_QCC_EN     DECKCTRL_GPIO_PIN_2   // PA2 - QCC744 chip enable
#define GPIO_QCC_BOOT   DECKCTRL_GPIO_PIN_7   // PA7 - QCC744 boot select
#define GPIO_PWR_EN     DECKCTRL_GPIO_PIN_12  // PC15 - Power enable

// QCC744 UART ISP protocol constants
#define ISP_HANDSHAKE_BYTE      0x55
#define ISP_HANDSHAKE_COUNT     32
#define ISP_HANDSHAKE_RESP_LEN  16   // "Boot2 ISP Ready\0"
#define ISP_HANDSHAKE_TIMEOUT   M2T(8000)
#define ISP_CMD_TIMEOUT         M2T(5000)

// ISP command IDs
#define ISP_CMD_GET_BOOTINFO    0x10
#define ISP_CMD_CHANGE_RATE     0x20
#define ISP_CMD_RESET           0x21
#define ISP_CMD_CLK_SET         0x22
#define ISP_CMD_FLASH_ERASE     0x30
#define ISP_CMD_FLASH_WRITE     0x31
#define ISP_CMD_FLASH_READ      0x32
#define ISP_CMD_FLASH_SET_PARA  0x3B
#define ISP_CMD_FLASH_CHIPERASE 0x3C
#define ISP_CMD_FLASH_WRITE_CHECK 0x3A
#define ISP_CMD_ISP_MODE        0xA0

// ISP response markers
#define ISP_ACK_OK_L            0x4F  // 'O'
#define ISP_ACK_OK_H            0x4B  // 'K'

static bool isInit = false;
static bool isInBootloader = false;
static bool isInFirmware = true;
static uint32_t newFwSize = 0;
static bool flashErased = false;
static DeckInfo *deckInfoG;
// static TaskHandle_t consoleTaskHandle = NULL;

// ---------------------------------------------------------------------------
// UART ISP protocol helpers
// ---------------------------------------------------------------------------

static bool ispRecvBytes(uint8_t *buf, uint32_t len, uint32_t timeoutTicks) {
  for (uint32_t i = 0; i < len; i++) {
    if (!uart1GetDataWithTimeout(&buf[i], timeoutTicks)) {
      return false;
    }
  }
  return true;
}

static void ispDrainRx(uint32_t timeoutMs) {
  uint8_t dummy;
  TickType_t end = xTaskGetTickCount() + M2T(timeoutMs);
  while (xTaskGetTickCount() < end) {
    if (!uart1GetDataWithTimeout(&dummy, M2T(5))) {
      break;
    }
  }
}

static bool ispSendCmd(uint8_t cmdId, const uint8_t *payload, uint16_t payloadLen) {
  // Packet: [CMD_ID(1)] [CHECKSUM(1)] [LEN_L(1)] [LEN_H(1)] [PAYLOAD...]
  uint8_t header[4];
  header[0] = cmdId;

  // Compute checksum over payload
  uint8_t checksum = 0;
  for (uint16_t i = 0; i < payloadLen; i++) {
    checksum += payload[i];
  }
  header[1] = payloadLen > 0 ? checksum : 0;
  header[2] = (uint8_t)(payloadLen & 0xFF);
  header[3] = (uint8_t)((payloadLen >> 8) & 0xFF);

  uart1SendData(4, header);
  if (payloadLen > 0 && payload != NULL) {
    uart1SendData(payloadLen, (uint8_t *)payload);
  }

  return true;
}

static bool ispWaitAck(void) {
  uint8_t resp[2];
  if (!ispRecvBytes(resp, 2, ISP_CMD_TIMEOUT)) {
    DEBUG_PRINT("ISP: No ACK received\n");
    return false;
  }

  if (resp[0] == ISP_ACK_OK_L && resp[1] == ISP_ACK_OK_H) {
    return true;
  }

  // Error response: read 2 more bytes for error code
  uint8_t errBytes[2];
  ispRecvBytes(errBytes, 2, M2T(100));
  uint16_t errCode = (uint16_t)errBytes[0] | ((uint16_t)errBytes[1] << 8);
  DEBUG_PRINT("ISP: NACK error 0x%04X\n", errCode);
  return false;
}

static bool ispFlashErase(uint32_t startAddr, uint32_t endAddr) {
  uint8_t payload[8];
  memcpy(&payload[0], &startAddr, 4);
  memcpy(&payload[4], &endAddr, 4);

  ispSendCmd(ISP_CMD_FLASH_ERASE, payload, 8);
  return ispWaitAck();
}

static bool ispFlashWrite(uint32_t addr, const uint8_t *data, uint16_t len) {
  // Payload: [address(4)] [data(len)]
  uint8_t payload[4 + 256];  // Max write chunk from memory subsystem is 256
  if (len > 256) {
    return false;
  }
  memcpy(&payload[0], &addr, 4);
  memcpy(&payload[4], data, len);

  ispSendCmd(ISP_CMD_FLASH_WRITE, payload, 4 + len);
  return ispWaitAck();
}

static bool ispFlashRead(uint32_t addr, uint8_t *data, uint16_t len) {
  // Payload: [address(4)] [length(4)]
  uint8_t payload[8];
  uint32_t readLen = len;
  memcpy(&payload[0], &addr, 4);
  memcpy(&payload[4], &readLen, 4);

  ispSendCmd(ISP_CMD_FLASH_READ, payload, 8);
  if (!ispWaitAck()) {
    return false;
  }

  // Response after OK: [data_length(2)] [data(...)]
  uint8_t lenBytes[2];
  if (!ispRecvBytes(lenBytes, 2, ISP_CMD_TIMEOUT)) {
    return false;
  }

  uint16_t respLen = (uint16_t)lenBytes[0] | ((uint16_t)lenBytes[1] << 8);
  if (respLen != len) {
    DEBUG_PRINT("ISP: Read length mismatch (%u vs %u)\n", respLen, len);
    return false;
  }

  return ispRecvBytes(data, len, ISP_CMD_TIMEOUT);
}

// ---------------------------------------------------------------------------
// Deck memory subsystem callbacks (firmware update)
// ---------------------------------------------------------------------------

static bool bcCamWriteFlash(const uint32_t memAddr, const uint8_t writeLen,
                            const uint8_t *buffer, const DeckMemDef_t *memDef) {
  if (!isInBootloader) {
    return false;
  }

  // Auto-erase: on first write, erase the region that will be written
  if (!flashErased && newFwSize > 0) {
    uint32_t eraseEnd = memAddr + newFwSize;
    DEBUG_PRINT("ISP: Erasing 0x%08lX - 0x%08lX\n",
                (unsigned long)memAddr, (unsigned long)eraseEnd);
    if (!ispFlashErase(memAddr, eraseEnd)) {
      DEBUG_PRINT("ISP: Flash erase failed\n");
      return false;
    }
    flashErased = true;
  }

  return ispFlashWrite(memAddr, buffer, writeLen);
}

static bool bcCamReadFlash(const uint32_t memAddr, const uint8_t readLen,
                           uint8_t *buffer) {
  if (!isInBootloader) {
    return false;
  }

  return ispFlashRead(memAddr, buffer, readLen);
}

static uint8_t bcCamPropertiesQuery(void) {
  uint8_t result = DECK_MEMORY_MASK_STARTED | DECK_MEMORY_MASK_SUPPORTS_HOT_RESTART;

  if (isInBootloader) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void bcCamBootloaderTask(void *arg) {
  DEBUG_PRINT("ISP: Bootloader entry starting\n");

  uint8_t handshake[ISP_HANDSHAKE_COUNT];
  memset(handshake, ISP_HANDSHAKE_BYTE, ISP_HANDSHAKE_COUNT);

  ispDrainRx(50);

  // Disable QCC744
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, LOW);
  vTaskDelay(M2T(50));

  // Switch UART to ROM bootloader baudrate (500000)
  uart1Init(BCCAM_ISP_BAUDRATE);

  // Re-enable QCC744 — it will enter ROM bootloader
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, HIGH);

  // Flood 0x55 and look for "OK" response from ROM bootloader
  bool gotOk = false;
  TickType_t deadline = xTaskGetTickCount() + M2T(5000);

  while (xTaskGetTickCount() < deadline) {
    uart1SendData(ISP_HANDSHAKE_COUNT, handshake);

    uint8_t c;
    if (uart1GetDataWithTimeout(&c, M2T(1))) {
      if (c == ISP_ACK_OK_L) {  // 'O'
        uint8_t c2;
        if (uart1GetDataWithTimeout(&c2, M2T(100))) {
          if (c2 == ISP_ACK_OK_H) {  // 'K'
            gotOk = true;
            break;
          }
        }
      }
    }
  }

  if (!gotOk) {
    DEBUG_PRINT("ISP: Handshake timeout (no OK from ROM)\n");
    uart1Init(BCCAM_UART_BAUDRATE);
    goto done;
  }

  ispDrainRx(50);
  DEBUG_PRINT("ISP: ROM handshake OK\n");

  // Get boot info (required before changing baud rate)
  ispSendCmd(ISP_CMD_GET_BOOTINFO, NULL, 0);
  if (!ispWaitAck()) {
    DEBUG_PRINT("ISP: GET_BOOTINFO failed\n");
    uart1Init(BCCAM_UART_BAUDRATE);
    goto done;
  }
  // Drain bootinfo response data
  ispDrainRx(100);
  DEBUG_PRINT("ISP: Got boot info\n");

  // Change baud rate to 2Mbaud
  // Payload: [uart_clk(4)] [baudrate(4)]
  uint8_t ratePayload[8];
  uint32_t uartClk = 1;  // PLL clock source
  uint32_t newBaud = BCCAM_UART_BAUDRATE;
  memcpy(&ratePayload[0], &uartClk, 4);
  memcpy(&ratePayload[4], &newBaud, 4);
  ispSendCmd(ISP_CMD_CLK_SET, ratePayload, 8);
  if (!ispWaitAck()) {
    DEBUG_PRINT("ISP: CLK_SET failed\n");
    uart1Init(BCCAM_UART_BAUDRATE);
    goto done;
  }

  // Switch our UART to 2Mbaud
  vTaskDelay(M2T(20));
  uart1Init(BCCAM_UART_BAUDRATE);
  vTaskDelay(M2T(20));
  ispDrainRx(50);

  DEBUG_PRINT("ISP: Switched to %d baud\n", BCCAM_UART_BAUDRATE);

  isInBootloader = true;
  DEBUG_PRINT("ISP: ROM bootloader ready\n");

done:
  return;
}

static volatile bool bootloaderRequested = false;

static void bcCamResetToBootloader(void) {
  DEBUG_PRINT("=== BOOTLOADER CALLBACK CALLED ===\n");
  isInFirmware = false;
  isInBootloader = false;
  flashErased = false;
  bootloaderRequested = true;
}

static void bcCamFwTask(void *arg) {
  // Disable QCC744
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, LOW);
  vTaskDelay(M2T(10));

  // Set boot select LOW for normal boot
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_BOOT, LOW);
  vTaskDelay(M2T(10));

  // Switch back to firmware baudrate
  uart1Init(BCCAM_UART_BAUDRATE);

  // Re-enable QCC744 - it will boot firmware
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, HIGH);
  vTaskDelay(M2T(100));

  ispDrainRx(50);

  isInFirmware = true;
  DEBUG_PRINT("QCC744 in firmware mode\n");
}

static volatile bool fwResetRequested = false;

static void bcCamResetToFw(void) {
  isInBootloader = false;
  isInFirmware = false;
  flashErased = false;
  fwResetRequested = true;
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
// Console forwarding task
// ---------------------------------------------------------------------------

static void bcCamWorkerTask(void *arg) {
  systemWaitStart();

  DEBUG_PRINT("Worker task started\n");

  while (1) {
    if (bootloaderRequested) {
      DEBUG_PRINT("Bootloader requested!\n");
      bootloaderRequested = false;
      bcCamBootloaderTask(NULL);
    } else if (fwResetRequested) {
      fwResetRequested = false;
      bcCamFwTask(NULL);
    } else {
      // Drain UART RX to prevent overflow, but check flags frequently
      uint8_t c;
      uart1GetDataWithTimeout(&c, M2T(1));
    }
  }
}

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

  deckInfoG = info;

  // Power on the deck
  if (!gpioSetup(info, GPIO_PWR_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable deck power\n");
    return;
  }

  vTaskDelay(M2T(50));

  // Boot into normal mode (not bootloader)
  if (!gpioSetup(info, GPIO_QCC_BOOT, OUTPUT, LOW)) {
    DEBUG_PRINT("Failed to set boot select\n");
    return;
  }

  // Enable QCC744
  if (!gpioSetup(info, GPIO_QCC_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable QCC744\n");
    return;
  }

  vTaskDelay(M2T(100));

  // Enable UART1 pass-through on deck controller
  if (!gpioSetup(info, GPIO_UART1_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable UART1\n");
    return;
  }

  // Initialize UART1 for QCC744 communication
  uart1Init(BCCAM_UART_BAUDRATE);

  xTaskCreate(bcCamWorkerTask, "bcCam", configMINIMAL_STACK_SIZE * 4,
              NULL, tskIDLE_PRIORITY, NULL);

  isInit = true;
  DEBUG_PRINT("bcCam deck initialized\n");
}

static bool bcCamDeckTest(void) {
  if (!isInit) {
    return false;
  }

  DEBUG_PRINT("bcCam deck test passed\n");
  return true;
}

static const DeckDriver bccam_deck = {
  .vid = 0xBC,
  .pid = 0x21,
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

PARAM_GROUP_STOP(deck)
