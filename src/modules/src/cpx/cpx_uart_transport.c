/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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

/* UART transport layer for CPX */

#define DEBUG_MODULE "CPX-UART-TRANSP"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "config.h"
#include "console.h"
#include "uart2.h"
#include "debug.h"
#include "deck.h"
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
#include "autoconf.h"

#include "cpx.h"
#include "cpx_uart_transport.h"

#define UART_TX_QUEUE_LENGTH 4
#define UART_RX_QUEUE_LENGTH 4

static xQueueHandle uartTxQueue;
static xQueueHandle uartRxQueue;

// Length of start + payloadLength
#define UART_HEADER_LENGTH 2
#define UART_CRC_LENGTH 1
#define UART_META_LENGTH (UART_HEADER_LENGTH + UART_CRC_LENGTH)

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[CPX_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct {
    uint8_t start;
    uint8_t payloadLength; // Excluding start and crc
    union {
        uartTransportPayload_t routablePayload;
        uint8_t payload[CPX_UART_TRANSPORT_MTU];
    };

    uint8_t crcPlaceHolder; // Not actual position. CRC is added after the last byte of payload
} __attribute__((packed)) uart_transport_packet_t;

// Used when sending/receiving data on the UART
static uart_transport_packet_t uartTxp;
static CPXPacket_t cpxTxp;
static uart_transport_packet_t uartRxp;

static EventGroupHandle_t evGroup;
/* Used to signal when ESP has said clear-to-send */
#define ESP_CTS_EVENT (1 << 0)
/* Used to signal when we should tell ESP it's clear-to-send */
#define ESP_CTR_EVENT (1 << 1)
/* Used to signal that there are packets in the outgoing TX queue */
#define ESP_TXQ_EVENT (1 << 2)
/* Used to signal that TX/RX tasks should shut down */
#define DEINIT_EVENT  (1 << 3)
/* Used to signal that RX task has shut down */
#define RX_DEINIT_EVENT (1 << 4)
/* Used to signal that TX task has shut down */
#define TX_DEINIT_EVENT (1 << 5)

/* Set when transport is de-initialized (i.e needs to release uart) */
static bool shutdownTransport = false;

static bool isInit = false;

static uint8_t calcCrc(const uart_transport_packet_t* packet) {
  const uint8_t* start = (const uint8_t*) packet;
  const uint8_t* end = &packet->payload[packet->payloadLength];

  uint8_t crc = 0;
  for (const uint8_t* p = start; p < end; p++) {
    crc ^= *p;
  }

  return crc;
}

static void assemblePacket(const CPXPacket_t *packet, uart_transport_packet_t * txp) {
  ASSERT((packet->route.destination >> 4) == 0);
  ASSERT((packet->route.source >> 4) == 0);
  ASSERT((packet->route.function >> 8) == 0);
  ASSERT(packet->dataLength <= CPX_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);

  txp->payloadLength = packet->dataLength + CPX_ROUTING_PACKED_SIZE;
  txp->routablePayload.route.destination = packet->route.destination;
  txp->routablePayload.route.source = packet->route.source;
  txp->routablePayload.route.lastPacket = packet->route.lastPacket;
  txp->routablePayload.route.function = packet->route.function;
  memcpy(txp->routablePayload.data, &packet->data, packet->dataLength);
  txp->payload[txp->payloadLength] = calcCrc(txp);
}

static void CPX_UART_RX(void *param)
{
  systemWaitStart();

  while (shutdownTransport == false)
  {
    // Wait for start!
    uartRxp.start = 0x00;
    do
    {
      uart2GetDataWithTimeout(1, &uartRxp.start, M2T(200));
    } while (uartRxp.start != 0xFF && shutdownTransport == false);

    if (uartRxp.start == 0xFF) {
      uart2GetData(1, &uartRxp.payloadLength);

      if (uartRxp.payloadLength == 0)
      {
        xEventGroupSetBits(evGroup, ESP_CTS_EVENT);
      }
      else
      {
        uart2GetData(uartRxp.payloadLength, (uint8_t*) &uartRxp.payload);

        uint8_t crc;
        uart2GetData(1, &crc);
        ASSERT(crc == calcCrc(&uartRxp));
        ASSERT(cpxCheckVersion(uartRxp.routablePayload.route.version));
        xQueueSend(uartRxQueue, &uartRxp, portMAX_DELAY);
        xEventGroupSetBits(evGroup, ESP_CTR_EVENT);
      }
    }
  }

  xEventGroupSetBits(evGroup, RX_DEINIT_EVENT);
  vTaskDelete(NULL);
}

static void CPX_UART_TX(void *param)
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
    uart2SendData(sizeof(ctr), (uint8_t *)&ctr);
    vTaskDelay(100);
    evBits = xEventGroupGetBits(evGroup);
  } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT && shutdownTransport == false);

  while (shutdownTransport == false)
  {
    // If we have nothing to send then wait, either for something to be
    // queued or for a request to send CTR
    if (uxQueueMessagesWaiting(uartTxQueue) == 0)
    {
      evBits = xEventGroupWaitBits(evGroup,
                                   ESP_CTR_EVENT | ESP_TXQ_EVENT | DEINIT_EVENT,
                                   pdTRUE,  // Clear bits before returning
                                   pdFALSE, // Wait for any bit
                                   portMAX_DELAY);
      if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
      {
        uart2SendData(sizeof(ctr), (uint8_t *)&ctr);
      }
    }

    if (uxQueueMessagesWaiting(uartTxQueue) > 0)
    {
      // Dequeue and wait for either CTS or CTR
      xQueueReceive(uartTxQueue, &cpxTxp, 0);
      uartTxp.start = 0xFF;
      assemblePacket(&cpxTxp, &uartTxp);
      do
      {
        evBits = xEventGroupWaitBits(evGroup,
                                     ESP_CTR_EVENT | ESP_CTS_EVENT,
                                     pdTRUE,  // Clear bits before returning
                                     pdFALSE, // Wait for any bit
                                     portMAX_DELAY);
        if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
        {
          uart2SendData(sizeof(ctr), (uint8_t *)&ctr);
        }
      } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);
      uart2SendData((uint32_t) uartTxp.payloadLength + UART_META_LENGTH, (uint8_t *)&uartTxp);
    }
  }

  xEventGroupSetBits(evGroup, TX_DEINIT_EVENT);
  vTaskDelete(NULL);
}

void cpxUARTTransportSend(const CPXRoutablePacket_t* packet) {
  ASSERT(isInit == true && shutdownTransport == false);
  ASSERT(packet);

  xQueueSend(uartTxQueue, packet, portMAX_DELAY);
  xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
}

void cpxUARTTransportReceive(CPXRoutablePacket_t* packet) {
  ASSERT(isInit == true && shutdownTransport == false);
  ASSERT(packet);

  static uart_transport_packet_t cpxRxp;

  xQueueReceive(uartRxQueue, &cpxRxp, portMAX_DELAY);

  packet->dataLength = (uint32_t) cpxRxp.payloadLength - CPX_ROUTING_PACKED_SIZE;
  packet->route.destination = cpxRxp.routablePayload.route.destination;
  packet->route.source = cpxRxp.routablePayload.route.source;
  packet->route.function = cpxRxp.routablePayload.route.function;
  packet->route.lastPacket = cpxRxp.routablePayload.route.lastPacket;
  memcpy(&packet->data, cpxRxp.routablePayload.data, packet->dataLength);

}

void cpxUARTTransportInit() {
  // There's no support for re-initializing the UART transport once it's
  // been de-initialized. This is not needed by the ESP bootloader use-case,
  // since the procedure will reset the Crazyflie after ESP has been bootloaded
  ASSERT(shutdownTransport==false);

  uartTxQueue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(CPXPacket_t));
  uartRxQueue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

  evGroup = xEventGroupCreate();

  uart2Init(CONFIG_CPX_UART2_BAUDRATE);

  // Initialize task for the ESP while it's held in reset
  xTaskCreate(CPX_UART_RX, AIDECK_ESP_RX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);
  xTaskCreate(CPX_UART_TX, AIDECK_ESP_TX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

  isInit = true;
}

void cpxUARTTransportDeinit() {
  shutdownTransport = true;
  // Send event to unlock TX
  xEventGroupSetBits(evGroup, DEINIT_EVENT);
  // Wait for RX/TX event shutdown
  xEventGroupWaitBits(evGroup,
                      TX_DEINIT_EVENT | RX_DEINIT_EVENT,
                      pdTRUE,  // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);
}
