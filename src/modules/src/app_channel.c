/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2020, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* app_channel.c: App realtime communication channel with the ground */

#include "app_channel.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#include "crtp.h"
#include "platformservice.h"

static SemaphoreHandle_t sendMutex;

static xQueueHandle  rxQueue;

static bool overflow;

static int sendDataPacket(void* data, size_t length, const bool doBlock);

// Deprecated (removed after August 2023)
void appchannelSendPacket(void* data, size_t length)
{
  appchannelSendDataPacketBlock(data, length);
}

int appchannelSendDataPacket(void* data, size_t length)
{
  return sendDataPacket(data, length, false);
}

void appchannelSendDataPacketBlock(void* data, size_t length)
{
  sendDataPacket(data, length, true);
}

// Deprecated (removed after August 2023)
size_t appchannelReceivePacket(void* buffer, size_t max_length, int timeout_ms) {
  return appchannelReceiveDataPacket(buffer, max_length, timeout_ms);
}

size_t appchannelReceiveDataPacket(void* buffer, size_t max_length, int timeout_ms) {
  static CRTPPacket packet;
  int tickToWait = 0;

  if (timeout_ms < 0) {
    tickToWait = portMAX_DELAY;
  } else {
    tickToWait = M2T(timeout_ms);
  }

  int result = xQueueReceive(rxQueue, &packet, tickToWait);

  if (result == pdTRUE) {
    int lenghtToCopy = (max_length < packet.size)?max_length:packet.size;
    memcpy(buffer, packet.data, lenghtToCopy);
    return lenghtToCopy;
  } else {
    return 0;
  }
}

bool appchannelHasOverflowOccurred()
{
  bool hasOverflowed = overflow;
  overflow = false;

  return hasOverflowed;
}

bool appchannelHasOverflowOccured() {
    return appchannelHasOverflowOccurred();
}

void appchannelInit()
{
  sendMutex = xSemaphoreCreateMutex();

  rxQueue = xQueueCreate(10, sizeof(CRTPPacket));

  overflow = false;
}

void appchannelIncomingPacket(CRTPPacket *p)
{
  int res = xQueueSend(rxQueue, p, 0);

  if (res != pdTRUE) {
    overflow = true;
  }
}

static int sendDataPacket(void* data, size_t length, const bool doBlock)
{
  static CRTPPacket packet;

  xSemaphoreTake(sendMutex, portMAX_DELAY);

  packet.size = (length > APPCHANNEL_MTU)?APPCHANNEL_MTU:length;
  memcpy(packet.data, data, packet.size);

  // CRTP channel and ports are set in platformservice
  int result = 0;
  if (doBlock)
  {
    result = platformserviceSendAppchannelPacketBlock(&packet);
  } else {
    result = platformserviceSendAppchannelPacket(&packet);
  }

  xSemaphoreGive(sendMutex);

  return result;
}
