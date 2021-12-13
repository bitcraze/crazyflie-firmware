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
#include "task.h"
#include "uart1.h"
#include "uart2.h"
#include "usec_time.h"

#include "aideck.h"

static bool isInit = false;
static uint8_t byte;

#define ESP_TX_QUEUE_LENGTH 4
#define ESP_RX_QUEUE_LENGTH 4

static xQueueHandle espTxQueue;
static xQueueHandle espFromESPRxQueue;
static xQueueHandle espFromGAPRxQueue;

static uart_transport_packet_t espTxp;
static uart_transport_packet_t espRxp;

static EventGroupHandle_t evGroup;

#define ESP_CTS_EVENT (1<<0)
#define ESP_CTR_EVENT (1<<1)
#define ESP_TXQ_EVENT (1<<2)

static void ESP_RX(void *param)
{
    systemWaitStart();

    while (1)
    {
      // Wait for start!
      do {
        uart2GetDataWithTimeout(&espRxp.start, (TickType_t)portMAX_DELAY);
      } while (espRxp.start != 0xFF);

      uart2GetDataWithTimeout(&espRxp.length, (TickType_t)portMAX_DELAY);

      //DEBUG_PRINT("Length is %u\n", espRxp.length);

      if (espRxp.length == 0) {
        //DEBUG_PRINT("Got CTS from ESP\n");
        xEventGroupSetBits(evGroup, ESP_CTS_EVENT);
      } else {
        // Here we should be able to call function that returns the correct amount
        // of bytes (instead of going though the queue for each byte)
        for (int i = 0; i < espRxp.length; i++) {
          uart2GetDataWithTimeout(&espRxp.data[i], (TickType_t)portMAX_DELAY);
        }

        // Queue the package...
        switch (ROUTE_TARGET( ((aideckRoutablePacket_t*) &espRxp)->route.src)) {
          case ESP32: xQueueSend(espFromESPRxQueue, &espRxp, portMAX_DELAY);break;
          case GAP8: xQueueSend(espFromGAPRxQueue, &espRxp, portMAX_DELAY);break;
          default: DEBUG_PRINT("UNKNOWN SOURCE\n");
        }
        // .. once we have the package queued, send CTR to ESP
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

    do {
      uart2SendData(sizeof(ctr), (uint8_t*) &ctr);
      vTaskDelay(100);
      evBits = xEventGroupGetBits(evGroup);
    } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT) ;

    while (1)
    {
      // If we have nothing to send then wait, either for something to be
      // queued or for a request to send CTR
      if (uxQueueMessagesWaiting(espTxQueue) == 0) {
        //DEBUG_PRINT("Waiting for CTR/TXQ\n");
        evBits = xEventGroupWaitBits(evGroup,
                                  ESP_CTR_EVENT | ESP_TXQ_EVENT,
                                  pdTRUE, // Clear bits before returning
                                  pdFALSE, // Wait for any bit
                                  portMAX_DELAY);
        if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT) {
          //DEBUG_PRINT("Sent CTR\n");
          uart2SendData(sizeof(ctr), (uint8_t*) &ctr);
        }
        if ((evBits & ESP_TXQ_EVENT) == ESP_TXQ_EVENT) {
          //DEBUG_PRINT("TXQ EVENT\n");
        }
      }

      if (uxQueueMessagesWaiting(espTxQueue) > 0) {
        // Dequeue and wait for either CTS or CTR
        xQueueReceive(espTxQueue, &espTxp, 0);
        do {
          //DEBUG_PRINT("Waiting for CTS/CTR\n");
          digitalWrite(DECK_GPIO_IO3, HIGH);
          evBits = xEventGroupWaitBits(evGroup,
                                  ESP_CTR_EVENT | ESP_CTS_EVENT,
                                  pdTRUE, // Clear bits before returning
                                  pdFALSE, // Wait for any bit
                                  portMAX_DELAY);
          if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT) {
            //DEBUG_PRINT("Sent CTR while waiting for CTS\n");
            uart2SendData(sizeof(ctr), (uint8_t*) &ctr);
          }       
        } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);
        //DEBUG_PRINT("Sending data!\n");
        digitalWrite(DECK_GPIO_IO3, LOW);
        espTxp.start = 0xFF;
        uart2SendData(espTxp.length + 2, (uint8_t*) &espTxp);
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
        uart1GetDataWithDefaultTimeout(&byte);
    }
}

// These are for your own stuff, where your own protocol can be implemented
void aideckSendBlocking(aideckRoutablePacket_t *packet) {
  xQueueSend(espTxQueue, packet, (TickType_t) portMAX_DELAY);
  xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
}

void aideckReceiveFromGAPBlocking(aideckRoutablePacket_t *packet) {
  xQueueReceive(espFromGAPRxQueue, packet, (TickType_t) portMAX_DELAY);
}

void aideckReceiveFromESPBlocking(aideckRoutablePacket_t *packet) {
  xQueueReceive(espFromESPRxQueue, packet, (TickType_t) portMAX_DELAY);
}

bool aideckSend(aideckRoutablePacket_t *packet, unsigned int timeoutInMS) {
  BaseType_t messagePosted = xQueueSend(espTxQueue, packet, pdMS_TO_TICKS(timeoutInMS));

  if (messagePosted == pdTRUE) {
    xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
    return true;
  } else {
    return false;
  }
}

static char hw[] = "Hello world from the Crazyflie!";

static aideckRoutablePacket_t espReceivePacket;
static aideckRoutablePacket_t hwPacket;
static void ESP_receive_task(void *param)
{
  while (1) {
    aideckReceiveFromESPBlocking( (aideckRoutablePacket_t*) &espReceivePacket);

    DEBUG_PRINT("Got message from ESP\n");

    switch(ROUTE_FUNCTION(espReceivePacket.route.dst)) {
      case WIFI_CTRL:
        if (espReceivePacket.data[0] == 0x21) {
          DEBUG_PRINT("WiFi connected to ip: %u.%u.%u.%u\n",
            espReceivePacket.data[1],
            espReceivePacket.data[2],
            espReceivePacket.data[3],
            espReceivePacket.data[4]);
        }
        if (espReceivePacket.data[0] == 0x22) {
          DEBUG_PRINT("WiFi client connected");
          hwPacket.length = 2 + sizeof(hw);
          hwPacket.route.dst = MAKE_ROUTE(ESP32, WIFI_DATA);
          hwPacket.route.src = MAKE_ROUTE(STM32, WIFI_DATA);
          memcpy(hwPacket.data, hw, sizeof(hw));
          aideckSendBlocking((aideckRoutablePacket_t*) &hwPacket);
        }
        break;
      case WIFI_DATA:
        DEBUG_PRINT("Got WiFi data!");
        DEBUG_PRINT("%s", espReceivePacket.data);
    }
  }
}

static const char ssid[] = {};
static const char passwd[] = {};
static aideckRoutablePacket_t wifiSetupPacket;

void sendWiFiSetup() {
  if (sizeof(ssid) > 0 && sizeof(passwd) > 0) {
    wifiSetupPacket.length = 2 + sizeof(ssid);
    wifiSetupPacket.route.dst = MAKE_ROUTE(ESP32, WIFI_CTRL);
    wifiSetupPacket.route.src = MAKE_ROUTE(STM32, WIFI_CTRL);
    wifiSetupPacket.data[0] = 0x10; // Set SSID
    memcpy(&wifiSetupPacket.data[1], ssid, sizeof(ssid));
    aideckSendBlocking((aideckRoutablePacket_t*) &wifiSetupPacket);

    wifiSetupPacket.length = 2 + sizeof(passwd);
    wifiSetupPacket.data[0] = 0x11; // Set passwd
    memcpy(&wifiSetupPacket.data[1], passwd, sizeof(passwd));
    aideckSendBlocking((aideckRoutablePacket_t*) &wifiSetupPacket);

    wifiSetupPacket.length = 3;
    wifiSetupPacket.data[0] = 0x20; // Connect wifi
    aideckSendBlocking((aideckRoutablePacket_t*) &wifiSetupPacket);
  }      
}

static void aideckInit(DeckInfo *info)
{

    if (isInit)
        return;
   
    // Initialize task for the GAP8
    xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

    espTxQueue = xQueueCreate(ESP_TX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));
    espFromESPRxQueue = xQueueCreate(ESP_RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));
    espFromGAPRxQueue = xQueueCreate(ESP_RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

    evGroup = xEventGroupCreate();

    xTaskCreate(ESP_receive_task, "ESP COM", AI_DECK_TASK_STACKSIZE*2, NULL,
                AI_DECK_TASK_PRI, NULL);
                
    pinMode(DECK_GPIO_IO3, OUTPUT);
    // Pull reset for GAP8/ESP32
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    //Initialize UARTs
    uart1Init(115200);
    uart2Init(115200);

    // Initialize task for the ESP
    xTaskCreate(ESP_RX, AIDECK_ESP_RX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);
    xTaskCreate(ESP_TX, AIDECK_ESP_TX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

    // Release reset for GAP8/ESP32
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    sendWiFiSetup();

  isInit = true;
}

static bool aideckTest()
{

  return true;
}

static const DeckMemDef_t memoryDef = {
    .write = espDeckFlasherWrite,
    .read = 0,
    .properties = espDeckFlasherPropertiesQuery,
    .supportsUpgrade = true,

    .requiredSize = ESP_BITSTREAM_SIZE,
    // .requiredHash = ESP_BITSTREAM_CRC,
};

static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedGpio = DECK_USING_IO_4 | DECK_USING_IO_3,
    .usedPeriph = DECK_USING_UART1,

    .memoryDef = &memoryDef,

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
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
