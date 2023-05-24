/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * AI-deck GAP8
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

/* Routing messages inside the Crazyflie firmware to specific CPX functions */

#define DEBUG_MODULE "CPX-INT-ROUTER"

#include "FreeRTOS.h"
#include "config.h"
#include "debug.h"
#include "queue.h"

#include "crtp.h"
#include "cpx_internal_router.h"
#include "cpx.h"

#define QUEUE_LENGTH (2)

static xQueueHandle crtpQueue;
static xQueueHandle mixedQueue;

static xQueueHandle txq;

int cpxInternalRouterReceiveCRTP(CPXPacket_t * packet) {
  return xQueueReceive(crtpQueue, packet, M2T(100));
}

void cpxInternalRouterReceiveOthers(CPXPacket_t * packet) {
  xQueueReceive(mixedQueue, packet, (TickType_t)portMAX_DELAY);
}

void cpxSendPacketBlocking(const CPXPacket_t * packet) {
  if (cpxCheckVersion(packet->route.version)) {
    xQueueSend(txq, packet, portMAX_DELAY);
  }
}

bool cpxSendPacketBlockingTimeout(const CPXPacket_t * packet, const uint32_t timeout) {
  if (cpxCheckVersion(packet->route.version)) {
    return xQueueSend(txq, packet, timeout) == pdTRUE;
  } else {
    return pdTRUE;
  }
}

bool cpxSendPacket(const CPXPacket_t * packet, uint32_t timeout) {
  return true;
}

void cpxInternalRouterRouteIn(const CPXRoutablePacket_t* packet) {
  // this should never fail, as it should be checked when the packet is received
  // however, double checking doesn't harm
  if (cpxCheckVersion(packet->route.version)) {
    switch (packet->route.function) {
      case CPX_F_SYSTEM:
      case CPX_F_CONSOLE:
      case CPX_F_WIFI_CTRL:
      case CPX_F_BOOTLOADER:
      case CPX_F_APP:
      case CPX_F_TEST:
        xQueueSend(mixedQueue, packet, portMAX_DELAY);
        break;
      case CPX_F_CRTP:
        xQueueSend(crtpQueue, packet, portMAX_DELAY);
        break;
      default:
        DEBUG_PRINT("Message on function which is not handled (0x%X)\n", packet->route.function);
    }
  }
}

// Route from STM to external targets
void cpxInternalRouterRouteOut(CPXRoutablePacket_t* packet) {
  xQueueReceive(txq, packet, (TickType_t)portMAX_DELAY);
}

void cpxInternalRouterInit(void) {
  txq = xQueueCreate(QUEUE_LENGTH, sizeof(CPXPacket_t));
  crtpQueue = xQueueCreate(QUEUE_LENGTH, sizeof(CPXPacket_t));;
  mixedQueue = xQueueCreate(QUEUE_LENGTH, sizeof(CPXPacket_t));;
}
