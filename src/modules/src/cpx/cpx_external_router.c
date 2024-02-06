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

/* Routing CPX messages between external interfaces (like UART) or between
 * external interfaces and the Crazyflie firmware internal router.
 */

#define DEBUG_MODULE "CPX-EXT-ROUTER"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "debug.h"

#include "cpx_external_router.h"
#include "cpx_internal_router.h"
#include "cpx_uart_transport.h"

typedef struct {
  CPXRoutablePacket_t txp;
} RouteContext_t;

static RouteContext_t internal_task_context;
static CPXRoutablePacket_t internalRxBuf;

static RouteContext_t uart_task_context;
static CPXRoutablePacket_t uartRxBuf;

typedef void (*Receiver_t)(CPXRoutablePacket_t* packet);
typedef void (*Sender_t)(const CPXRoutablePacket_t* packet);

static const int START_UP_UART_ROUTER_RUNNING = (1<<0);
static const int START_UP_RADIO_ROUTER_RUNNING = (1<<1);
static const int START_UP_INTERNAL_ROUTER_RUNNING = (1<<2);

static EventGroupHandle_t startUpEventGroup;

static void splitAndSend(const CPXRoutablePacket_t* rxp, RouteContext_t* context, Sender_t sender, const uint16_t mtu) {
  CPXRoutablePacket_t* txp = &context->txp;

  txp->route = rxp->route;

  uint16_t remainingToSend = rxp->dataLength;
  const uint8_t* startOfDataToSend = rxp->data;
  while (remainingToSend > 0) {
    uint16_t toSend = remainingToSend;
    bool lastPacket = rxp->route.lastPacket;
    if (toSend > mtu) {
      toSend = mtu;
      lastPacket = false;
    }

    memcpy(txp->data, startOfDataToSend, toSend);
    txp->dataLength = toSend;
    txp->route.lastPacket = lastPacket;
    sender(txp);

    remainingToSend -= toSend;
    startOfDataToSend += toSend;
  }
}

static void route(Receiver_t receive, CPXRoutablePacket_t* rxp, RouteContext_t* context, const char* routerName) {
  while(1) {
    receive(rxp);
    // this should never fail, as it should be checked when the packet is received
    // however, double checking doesn't harm
    if (cpxCheckVersion(rxp->route.version)) {
      const CPXTarget_t source = rxp->route.source;
      const CPXTarget_t destination = rxp->route.destination;
      const uint16_t cpxDataLength = rxp->dataLength;

      switch (destination) {
        case CPX_T_WIFI_HOST:
        case CPX_T_ESP32:
        case CPX_T_GAP8:
          //DEBUG_PRINT("%s [0x%02X] -> UART2 [0x%02X] (%u)\n", routerName, source, destination, cpxDataLength);
          splitAndSend(rxp, context, cpxUARTTransportSend, CPX_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
          break;
        case CPX_T_STM32:
          //DEBUG_PRINT("%s [0x%02X] -> STM32 [0x%02X] (%u)\n", routerName, source, destination, cpxDataLength);
          splitAndSend(rxp, context, cpxInternalRouterRouteIn, CPX_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
          break;
        default:
          DEBUG_PRINT("Cannot route from %s [0x%02X] to [0x%02X](%u)\n", routerName, source, destination, cpxDataLength);
          break;
      }
    }
  }
}

static void router_from_uart(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_UART_ROUTER_RUNNING);
  route(cpxUARTTransportReceive, &uartRxBuf, &uart_task_context, "UART2");
}

static void router_from_internal(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_INTERNAL_ROUTER_RUNNING);
  route(cpxInternalRouterRouteOut, &internalRxBuf, &internal_task_context, "STM32");
}

void cpxExternalRouterInit() {
  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_UART_ROUTER_RUNNING | START_UP_INTERNAL_ROUTER_RUNNING);

  xTaskCreate(router_from_uart, CPX_RT_UART_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL, AI_DECK_TASK_PRI, NULL);
  xTaskCreate(router_from_internal, CPX_RT_INT_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL, AI_DECK_TASK_PRI, NULL);

  DEBUG_PRINT("Waiting for CPX External router initialization\n");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_UART_ROUTER_RUNNING | START_UP_INTERNAL_ROUTER_RUNNING,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  DEBUG_PRINT("CPX External router initialized, CPX_VERSION: %d\n", CPX_VERSION);
}
