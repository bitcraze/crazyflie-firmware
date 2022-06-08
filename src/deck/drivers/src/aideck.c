/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * aideck.c - Deck driver for the AIdeck
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
#include "FreeRTOS.h"
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

#include "aideck.h"
#include "aideck-router.h"

static bool isInit = false;
static uint8_t byte;

#define ESP_TX_QUEUE_LENGTH 4
#define ESP_RX_QUEUE_LENGTH 4

static xQueueHandle espTxQueue;
static xQueueHandle espRxQueue;

// Length of start + payloadLength
#define UART_HEADER_LENGTH 2
#define UART_CRC_LENGTH 1
#define UART_META_LENGTH (UART_HEADER_LENGTH + UART_CRC_LENGTH)

typedef struct {
  CPXTarget_t destination : 3;
  CPXTarget_t source : 3;
  bool lastPacket : 1;
  bool reserved : 1;
  CPXFunction_t function : 8;
} __attribute__((packed)) CPXRoutingPacked_t;

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

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[AIDECK_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct {
    uint8_t start;
    uint8_t payloadLength; // Excluding start and crc
    union {
        uartTransportPayload_t routablePayload;
        uint8_t payload[AIDECK_UART_TRANSPORT_MTU];
    };

    uint8_t crcPlaceHolder; // Not actual position. CRC is added after the last byte of payload
} __attribute__((packed)) uart_transport_packet_t;

// Used when sending/receiving data on the UART
static uart_transport_packet_t espTxp;
static CPXPacket_t cpxTxp;
static uart_transport_packet_t espRxp;

static EventGroupHandle_t evGroup;
#define ESP_CTS_EVENT (1 << 0)
#define ESP_CTR_EVENT (1 << 1)
#define ESP_TXQ_EVENT (1 << 2)

static EventGroupHandle_t bootloaderSync;
#define CPX_WAIT_FOR_BOOTLOADER_REPLY (1<<0)

static void assemblePacket(const CPXPacket_t *packet, uart_transport_packet_t * txp);


typedef enum {
  ESP_MODE_NORMAL,
  ESP_MODE_PREPARE_FOR_BOOTLOADER,
  ESP_MODE_BOOTLOADER,
} EspMode_t;

EspMode_t espMode = ESP_MODE_NORMAL;
const uint32_t espUartReadMaxWait = M2T(100);

// This function gets data from the UART that is connected to the ESP.
// The default behavior is to wait until data arrives, but it can be terminated when the ESP is set to boot loader mode.
// In this case the function will be NOPed and block forever.
static void getDataFromEspUart(uint8_t *c) {
  bool readSuccess = false;
  while(!readSuccess) {
    if (ESP_MODE_NORMAL == espMode) {
      readSuccess = uart2GetDataWithTimeout(c, espUartReadMaxWait);
    } else {
      vTaskDelay(portMAX_DELAY);
    }
  }
}

// This function sends data to the UART that is connected to the ESP.
// It will be NOPed if the ESP is set in boot loader mode.
static void sendDataToEspUart(uint32_t size, uint8_t* data) {
  if (ESP_MODE_NORMAL == espMode) {
    uart2SendData(size, data);
  }
}

static uint8_t calcCrc(const uart_transport_packet_t* packet) {
  const uint8_t* start = (const uint8_t*) packet;
  const uint8_t* end = &packet->payload[packet->payloadLength];

  uint8_t crc = 0;
  for (const uint8_t* p = start; p < end; p++) {
    crc ^= *p;
  }

  return crc;
}

static void ESP_RX(void *param)
{
  systemWaitStart();

  while (1)
  {
    // Wait for start!
    do
    {
      getDataFromEspUart(&espRxp.start);
    } while (espRxp.start != 0xFF);

    getDataFromEspUart(&espRxp.payloadLength);

    if (espRxp.payloadLength == 0)
    {
      xEventGroupSetBits(evGroup, ESP_CTS_EVENT);
    }
    else
    {
      for (int i = 0; i < espRxp.payloadLength; i++)
      {
        getDataFromEspUart(&espRxp.payload[i]);
      }

      uint8_t crc;
      getDataFromEspUart(&crc);
      ASSERT(crc == calcCrc(&espRxp));

      xQueueSend(espRxQueue, &espRxp, portMAX_DELAY);
      xEventGroupSetBits(evGroup, ESP_CTR_EVENT);
    }
  }
}

static void ESP_TX(void *param)
{
  systemWaitStart();

  uint8_t ctr[] = {0xFF, 0x00};
  EventBits_t evBits = 0;

  // We need to hold off here to make sure that the RX task
  // has started up and is waiting for chars, otherwise we might send
  // CTR and miss CTS (which means that the ESP32 will stop sending CTS
  // too early and we cannot sync)
  vTaskDelay(100);

  // Sync with ESP32 so both are in CTS
  do
  {
    sendDataToEspUart(sizeof(ctr), (uint8_t *)&ctr);
    vTaskDelay(100);
    evBits = xEventGroupGetBits(evGroup);
  } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);

  while (1)
  {
    // If we have nothing to send then wait, either for something to be
    // queued or for a request to send CTR
    if (uxQueueMessagesWaiting(espTxQueue) == 0)
    {
      evBits = xEventGroupWaitBits(evGroup,
                                   ESP_CTR_EVENT | ESP_TXQ_EVENT,
                                   pdTRUE,  // Clear bits before returning
                                   pdFALSE, // Wait for any bit
                                   portMAX_DELAY);
      if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
      {
        sendDataToEspUart(sizeof(ctr), (uint8_t *)&ctr);
      }
    }

    if (uxQueueMessagesWaiting(espTxQueue) > 0)
    {
      // Dequeue and wait for either CTS or CTR
      xQueueReceive(espTxQueue, &cpxTxp, 0);
      espTxp.start = 0xFF;
      assemblePacket(&cpxTxp, &espTxp);
      do
      {
        evBits = xEventGroupWaitBits(evGroup,
                                     ESP_CTR_EVENT | ESP_CTS_EVENT,
                                     pdTRUE,  // Clear bits before returning
                                     pdFALSE, // Wait for any bit
                                     portMAX_DELAY);
        if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
        {
          sendDataToEspUart(sizeof(ctr), (uint8_t *)&ctr);
        }
      } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);
      sendDataToEspUart((uint32_t) espTxp.payloadLength + UART_META_LENGTH, (uint8_t *)&espTxp);
    }
  }
}

static void Gap8Task(void *param)
{
  systemWaitStart();
  vTaskDelay(M2T(1000));

  // Read out the byte the Gap8 sends and immediately send it to the console.
  while (1)
  {
    uart1GetDataWithTimeout(&byte, portMAX_DELAY);
    //consolePutchar(byte);
  }
}

static void assemblePacket(const CPXPacket_t *packet, uart_transport_packet_t * txp) {
  ASSERT((packet->route.destination >> 4) == 0);
  ASSERT((packet->route.source >> 4) == 0);
  ASSERT((packet->route.function >> 8) == 0);
  ASSERT(packet->dataLength <= AIDECK_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);

  txp->payloadLength = packet->dataLength + CPX_ROUTING_PACKED_SIZE;
  txp->routablePayload.route.destination = packet->route.destination;
  txp->routablePayload.route.source = packet->route.source;
  txp->routablePayload.route.lastPacket = packet->route.lastPacket;
  txp->routablePayload.route.function = packet->route.function;
  memcpy(txp->routablePayload.data, &packet->data, packet->dataLength);
  txp->payload[txp->payloadLength] = calcCrc(txp);
}

void cpxReceivePacketBlocking(CPXPacket_t *packet)
{
  static uart_transport_packet_t cpxRxp;

  xQueueReceive(espRxQueue, &cpxRxp, portMAX_DELAY);

  packet->dataLength = (uint32_t) cpxRxp.payloadLength - CPX_ROUTING_PACKED_SIZE;
  packet->route.destination = cpxRxp.routablePayload.route.destination;
  packet->route.source = cpxRxp.routablePayload.route.source;
  packet->route.function = cpxRxp.routablePayload.route.function;
  packet->route.lastPacket = cpxRxp.routablePayload.route.lastPacket;
  memcpy(&packet->data, cpxRxp.routablePayload.data, packet->dataLength);
}

void cpxSendPacketBlocking(const CPXPacket_t *packet)
{
  xQueueSend(espTxQueue, packet, portMAX_DELAY);
  xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
}

bool cpxSendPacket(const CPXPacket_t *packet, uint32_t timeout)
{
  bool packetWasSent = false;
  if (xQueueSend(espTxQueue, packet, timeout) == pdTRUE)
  {
    xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
    packetWasSent = true;
  }
  return packetWasSent;
}

void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route) {
    route->source = source;
    route->destination = destination;
    route->function = function;
    route->lastPacket = true;
}

void cpxBootloaderMessage(const CPXPacket_t * packet) {
  xEventGroupSetBits(bootloaderSync, CPX_WAIT_FOR_BOOTLOADER_REPLY);
}

static CPXPacket_t bootPacket;

#define FLASH_BUFFER_SIZE 64
static uint8_t flashBuffer[FLASH_BUFFER_SIZE];
static int flashBufferIndex = 0;

static bool gap8DeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t *buffer, const DeckMemDef_t* memDef) {

  cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_BOOTLOADER, &bootPacket.route);

  if (memAddr == 0) {
    GAP8BlCmdPacket_t* gap8BlPacket = (GAP8BlCmdPacket_t*)bootPacket.data;

    gap8BlPacket->cmd = GAP8_BL_CMD_START_WRITE;
    gap8BlPacket->startAddress = 0x40000;
    gap8BlPacket->writeSize = *(memDef->newFwSizeP);
    bootPacket.dataLength = sizeof(GAP8BlCmdPacket_t);
    cpxSendPacketBlocking(&bootPacket);
  }

  // The GAP8 can only flash data in multiples of 4 bytes,
  // buffering will guard against this and also speed things up.
  // The full binary that will be flashed is multiple of 4.

  uint32_t sizeLeftToBufferFull = sizeof(flashBuffer) - flashBufferIndex;
  uint32_t sizeAbleToBuffer = sizeLeftToBufferFull < writeLen ? sizeLeftToBufferFull : writeLen;
  uint32_t lastAddressToWrite = memAddr + sizeAbleToBuffer;

  memcpy(&flashBuffer[flashBufferIndex], buffer, sizeAbleToBuffer);
  flashBufferIndex += sizeAbleToBuffer;

  if (flashBufferIndex == sizeof(flashBuffer) || lastAddressToWrite == *(memDef->newFwSizeP)) {
    memcpy(&bootPacket.data, flashBuffer, flashBufferIndex);
    bootPacket.dataLength = flashBufferIndex;

    cpxSendPacketBlocking(&bootPacket);

    flashBufferIndex = 0;
    int sizeLeftToBuffer = writeLen - sizeLeftToBufferFull;
    if (sizeLeftToBuffer > 0) {
      memcpy(&flashBuffer[flashBufferIndex], &buffer[sizeLeftToBufferFull], sizeLeftToBuffer);
      flashBufferIndex += sizeLeftToBuffer;
    }
  }

  if (memAddr + writeLen == *(memDef->newFwSizeP)) {
    // Request the MD5 checksum of the flashed data. This is only done
    // for synchronizing and making sure everything has been written,
    // we do not care about the results.
    GAP8BlCmdPacket_t* gap8BlPacket = (GAP8BlCmdPacket_t*)bootPacket.data;
    gap8BlPacket->cmd = GAP8_BL_CMD_MD5;
    gap8BlPacket->startAddress = 0x40000;
    gap8BlPacket->writeSize = *(memDef->newFwSizeP);
    bootPacket.dataLength = sizeof(GAP8BlCmdPacket_t);
    cpxSendPacketBlocking(&bootPacket);
    
    xEventGroupWaitBits(bootloaderSync,
                        CPX_WAIT_FOR_BOOTLOADER_REPLY,
                        pdTRUE,  // Clear bits before returning
                        pdFALSE, // Wait for any bit
                        portMAX_DELAY);
}

  return true;
}


static bool isGap8InBootloaderMode = false;

static void resetGap8ToBootloader() {
  cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_SYSTEM, &bootPacket.route);

  ESP32SysPacket_t* esp32SysPacket = (ESP32SysPacket_t*)bootPacket.data;

  esp32SysPacket->cmd = ESP32_SYS_CMD_RESET_GAP8;
  bootPacket.dataLength = sizeof(ESP32SysPacket_t);

  cpxSendPacketBlocking(&bootPacket);
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

  // Give the UART some time to terminate any pending operations
  vTaskDelay(espUartReadMaxWait * 2);

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



static void aideckInit(DeckInfo *info)
{
  if (isInit)
    return;

  // Initialize task for the GAP8
  xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

  espTxQueue = xQueueCreate(ESP_TX_QUEUE_LENGTH, sizeof(CPXPacket_t));
  espRxQueue = xQueueCreate(ESP_RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

  evGroup = xEventGroupCreate();
  bootloaderSync = xEventGroupCreate();

  // Pull reset for GAP8/ESP32
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  //Initialize UARTs while GAP8/ESP32 is held in reset
  uart1Init(115200);
  uart2Init(115200);

  // Initialize task for the ESP while it's held in reset
  xTaskCreate(ESP_RX, AIDECK_ESP_RX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);
  xTaskCreate(ESP_TX, AIDECK_ESP_TX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

  // Release reset for GAP8/ESP32
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  aideckRouterInit();

  // Make sure a full CPX packet can be stored in the UART2 RX buffer
  ASSERT(UART2_RX_QUEUE_LENGTH > sizeof(uart_transport_packet_t));

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

    .usedGpio = DECK_USING_IO_4,
    .usedPeriph = DECK_USING_UART1,

    .memoryDef = &espMemoryDef,
    .memoryDefSecondary = &gap8MemoryDef,

    .init = aideckInit,
    .test = aideckTest,
};

LOG_GROUP_START(aideck)
LOG_ADD(LOG_UINT8, receivebyte, &byte)
LOG_GROUP_STOP(aideck)

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [AI deck](%https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
