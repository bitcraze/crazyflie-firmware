/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 */

/* Deck driver for AI deck which supports bootloading of GAP8/ESP32 as well as CPX
 * communication.
 */

#define DEBUG_MODULE "AIDECK"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "aideck.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "uart2.h"
#include "debug.h"
#include "deck.h"
#include "esp_deck_flasher.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "queue.h"
#include "stm32fxxx.h"
#include "system.h"
#include "buf2buf.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "cpx_uart_transport.h"
#include "cpx.h"

#include "aideck.h"

static bool isInit = false;

#define WIFI_SET_SSID_CMD         0x10
#define WIFI_SET_KEY_CMD          0x11

#define WIFI_CONNECT_CMD          0x20
#define WIFI_CONNECT_AS_AP        0x01
#define WIFI_CONNECT_AS_STA       0x00
#define WIFI_CONNECT_AS_LENGTH    2

#define WIFI_AP_CONNECTED_CMD     0x31
#define WIFI_CLIENT_CONNECTED_CMD 0x32

#define CPX_ENABLE_CRTP_BRIDGE    0x10

#define GAP8_MAX_MEM_WRITE_TIMEOUT_MS 5000
#define GAP8_MAX_MEM_VERIFY_TIMEOUT_MS 5000

typedef struct {
  uint8_t cmd;
  uint32_t startAddress;
  uint32_t writeSize;
} __attribute__((packed)) GAP8BlCmdPacket_t;

typedef struct {
  uint8_t cmd;
} __attribute__((packed)) ESP32SysPacket_t;

#define GAP8_BL_CMD_START_WRITE (0x02)
#define GAP8_BL_CMD_MD5         (0x04)

#define ESP32_SYS_CMD_RESET_GAP8 (0x10)

static EventGroupHandle_t bootloaderSync;
#define CPX_WAIT_FOR_BOOTLOADER_REPLY (1<<0)

typedef enum {
  ESP_MODE_NORMAL,
  ESP_MODE_PREPARE_FOR_BOOTLOADER,
  ESP_MODE_BOOTLOADER,
} EspMode_t;

EspMode_t espMode = ESP_MODE_NORMAL;
const uint32_t espUartReadMaxWait = M2T(100);

void cpxBootloaderMessage(const CPXPacket_t * packet) {
  xEventGroupSetBits(bootloaderSync, CPX_WAIT_FOR_BOOTLOADER_REPLY);
}

static CPXPacket_t txPacket;

#define FLASH_BUFFER_SIZE 64
static uint8_t flashBuffer[FLASH_BUFFER_SIZE];
static Buf2bufContext_t gap8BufContext;

static void sendFlashInit(const uint32_t fwSize) {
  GAP8BlCmdPacket_t* gap8BlPacket = (GAP8BlCmdPacket_t*)txPacket.data;

  gap8BlPacket->cmd = GAP8_BL_CMD_START_WRITE;
  gap8BlPacket->startAddress = 0x40000;
  gap8BlPacket->writeSize = fwSize;
  txPacket.dataLength = sizeof(GAP8BlCmdPacket_t);
  bool writeOk = cpxSendPacketBlockingTimeout(&txPacket, M2T(GAP8_MAX_MEM_WRITE_TIMEOUT_MS));
  ASSERT(writeOk);
}

static void sendFlashBuffer(const uint32_t size) {
  memcpy(&txPacket.data, flashBuffer, size);
  txPacket.dataLength = size;
  bool writeOk = cpxSendPacketBlockingTimeout(&txPacket, M2T(GAP8_MAX_MEM_WRITE_TIMEOUT_MS));
  ASSERT(writeOk);
}

static void sendFlashMd5Request(const uint32_t fwSize) {
  GAP8BlCmdPacket_t* gap8BlPacket = (GAP8BlCmdPacket_t*)txPacket.data;
  gap8BlPacket->cmd = GAP8_BL_CMD_MD5;
  gap8BlPacket->startAddress = 0x40000;
  gap8BlPacket->writeSize = fwSize;
  txPacket.dataLength = sizeof(GAP8BlCmdPacket_t);
  bool writeOk = cpxSendPacketBlockingTimeout(&txPacket, M2T(GAP8_MAX_MEM_WRITE_TIMEOUT_MS));
  ASSERT(writeOk);
}

static void waitForCpxResponse() {
  EventBits_t bits = xEventGroupWaitBits(bootloaderSync,
                      CPX_WAIT_FOR_BOOTLOADER_REPLY,
                      pdTRUE,  // Clear bits before returning
                      pdFALSE, // Wait for any bit
                      M2T(GAP8_MAX_MEM_VERIFY_TIMEOUT_MS));
  bool flashWritten = (bits & CPX_WAIT_FOR_BOOTLOADER_REPLY);
  ASSERT(flashWritten);
}

static bool gap8DeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t *buffer, const DeckMemDef_t* memDef) {
  cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_BOOTLOADER, &txPacket.route);

  const uint32_t fwSize = *(memDef->newFwSizeP);

  // The GAP8 can only flash data in multiples of 4 bytes,
  // buffering will guard against this and also speed things up.
  // The full binary that will be flashed is multiple of 4.

  const bool isFirstBuf = (memAddr == 0);
  if (isFirstBuf) {
    sendFlashInit(fwSize);
    buf2bufInit(&gap8BufContext, flashBuffer, FLASH_BUFFER_SIZE);
  }

  buf2bufAddInBuf(&gap8BufContext, buffer, writeLen);
  ASSERT(buf2bufBytesAdded(&gap8BufContext) <= fwSize);
  while(buf2bufConsumeInBuf(&gap8BufContext)) {
    sendFlashBuffer(FLASH_BUFFER_SIZE);
  }
  buf2bufReleaseInBuf(&gap8BufContext);

  const bool isLastBuf = (buf2bufBytesConsumed(&gap8BufContext) == fwSize);
  if (isLastBuf) {
    uint32_t size = buf2bufReleaseOutBuf(&gap8BufContext);
    if (size > 0) {
      sendFlashBuffer(size);
    }
    ASSERT(buf2bufBytesAdded(&gap8BufContext) == buf2bufBytesConsumed(&gap8BufContext));

    // Request the MD5 checksum of the flashed data. This is only done
    // for synchronizing and making sure everything has been written,
    // we do not care about the results.
    sendFlashMd5Request(fwSize);
    waitForCpxResponse();
  }

  return true;
}


static bool isGap8InBootloaderMode = false;

static void resetGap8ToBootloader() {
  cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_SYSTEM, &txPacket.route);

  ESP32SysPacket_t* esp32SysPacket = (ESP32SysPacket_t*)txPacket.data;

  esp32SysPacket->cmd = ESP32_SYS_CMD_RESET_GAP8;
  txPacket.dataLength = sizeof(ESP32SysPacket_t);

  cpxSendPacketBlocking(&txPacket);
  // This should be handled on RX on CPX instead
  vTaskDelay(100);
  isGap8InBootloaderMode = true;
}

static uint8_t gap8DeckFlasherPropertiesQuery()
{
  uint8_t result = 0;

  if (isInit) {
    result |= DECK_MEMORY_MASK_STARTED;
  }

  if (isGap8InBootloaderMode) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void resetEspToBootloader() {
  espMode = ESP_MODE_PREPARE_FOR_BOOTLOADER;

  // Free up the UART and re-initialize it to the correct
  // baud rate.
  cpxUARTTransportDeinit();
  uart2Init(115200);

  // Set ESP in reset
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);

  // Signal to go to bootloader mode after reset
  pinMode(DECK_GPIO_IO1, OUTPUT);
  digitalWrite(DECK_GPIO_IO1, LOW);
  vTaskDelay(M2T(10));

  // Release reset
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  // Release pin
  vTaskDelay(M2T(100));
  pinMode(DECK_GPIO_IO1, INPUT);

  espMode = ESP_MODE_BOOTLOADER;
}

uint8_t espDeckFlasherPropertiesQuery()
{
  uint8_t result = 0;

  if (isInit)
  {
    result |= DECK_MEMORY_MASK_STARTED;
  }

  if (ESP_MODE_BOOTLOADER == espMode)
  {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

#ifndef CONFIG_DECK_AI_WIFI_NO_SETUP
  static CPXPacket_t cpxTx;
  static void setupWiFi() {
  #ifdef CONFIG_DECK_AI_WIFI_SETUP_STA
    DEBUG_PRINT("AI-deck will connect to WiFi\n");
  #endif

  #ifdef CONFIG_DECK_AI_WIFI_SETUP_AP
    DEBUG_PRINT("AI-deck will become access point\n");
  #endif

    cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_WIFI_CTRL, &cpxTx.route);

    cpxTx.data[0] = WIFI_SET_SSID_CMD; // Set SSID
    memcpy(&cpxTx.data[1], CONFIG_DECK_AI_SSID, sizeof(CONFIG_DECK_AI_SSID));
    cpxTx.dataLength = sizeof(CONFIG_DECK_AI_SSID);
    cpxSendPacketBlocking(&cpxTx);

    cpxTx.data[0] = WIFI_SET_KEY_CMD; // Set SSID
    memcpy(&cpxTx.data[1], CONFIG_DECK_AI_PASSWORD, sizeof(CONFIG_DECK_AI_PASSWORD));
    cpxTx.dataLength = sizeof(CONFIG_DECK_AI_PASSWORD);
    cpxSendPacketBlocking(&cpxTx);

    cpxTx.data[0] = WIFI_CONNECT_CMD; // Connect wifi
  #ifdef CONFIG_DECK_AI_WIFI_SETUP_STA
    cpxTx.data[1] = WIFI_CONNECT_AS_STA;
  #endif
  #ifdef CONFIG_DECK_AI_WIFI_SETUP_AP
    cpxTx.data[1] = WIFI_CONNECT_AS_AP;
  #endif
    cpxTx.dataLength = WIFI_CONNECT_AS_LENGTH;
    cpxSendPacketBlocking(&cpxTx);
  }
  #endif


static void aideckInit(DeckInfo *info)
{
  if (isInit)
    return;

  pinMode(DECK_GPIO_IO2, OUTPUT);
  pinMode(DECK_GPIO_IO3, OUTPUT);

  bootloaderSync = xEventGroupCreate();

  // Pull reset for GAP8/ESP32
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);

  cpxUARTTransportInit();
  cpxInternalRouterInit();
  cpxExternalRouterInit();
  cpxInit();

  // Release reset for GAP8/ESP32
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

#ifdef CONFIG_DECK_AI_WIFI_NO_SETUP
  DEBUG_PRINT("Not setting up WiFi\n");
#else
  setupWiFi();
#endif

  isInit = true;
}

static bool aideckTest()
{

  return true;
}

static const DeckMemDef_t espMemoryDef = {
    .write = espDeckFlasherWrite,
    .read = 0,
    .properties = espDeckFlasherPropertiesQuery,
    .supportsUpgrade = true,
    .id = "esp",
    .newFwSizeP = &espDeckFlasherNewBinarySize,

    .commandResetToBootloader = resetEspToBootloader,
};

static uint32_t gap8NewFlashSize;
static const DeckMemDef_t gap8MemoryDef = {
    .write = gap8DeckFlasherWrite,
    .read = 0,
    .properties = gap8DeckFlasherPropertiesQuery,
    .supportsUpgrade = true,
    .id = "gap8",
    .newFwSizeP = &gap8NewFlashSize,

    .commandResetToBootloader = resetGap8ToBootloader,
};

static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_4,
    .usedPeriph = DECK_USING_UART2,

    .memoryDef = &espMemoryDef,
    .memoryDefSecondary = &gap8MemoryDef,

    .init = aideckInit,
    .test = aideckTest,
};


/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [AI deck](%https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
