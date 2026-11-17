/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * crtp_control.c - CRTP port 10 bridge to the Control router.
 *
 * The bridge keeps a fixed number of slots. Each slot holds one reassembled
 * request, is handed to the Control router with controlSubmit(), and receives
 * the response in place in its completion callback. One task owns all bridge
 * state except the SUBMITTED -> DONE transition, which the completion
 * callback makes from the provider's task. The bridge only reads the first
 * two bytes of a Control message.
 */

#include "crtp_control.h"

#include <errno.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "control.h"
#include "crc32.h"
#include "crtp.h"

#ifndef UNIT_TEST_MODE
#include "config.h"
#include "static_mem.h"
#endif

#ifndef CONFIG_CRTP_CONTROL_SLOTS
#define CONFIG_CRTP_CONTROL_SLOTS 4
#endif
#ifndef CONFIG_CRTP_CONTROL_MAX_MESSAGE
#define CONFIG_CRTP_CONTROL_MAX_MESSAGE 256
#endif

#define SLOTS CONFIG_CRTP_CONTROL_SLOTS
#define MAX_MESSAGE CONFIG_CRTP_CONTROL_MAX_MESSAGE

/** CRTP channels on port 10. */
#define CHANNEL_CATALOG 0u
#define CHANNEL_ENDPOINT 1u
#define CHANNEL_TUNNEL 2u

/** Channel 0 commands. */
#define CATALOG_GET_ITEM 0x00u
#define CATALOG_GET_INFO 0x01u
/** Channel 1 commands. */
#define ENDPOINT_GET_STATUS 0x00u
#define ENDPOINT_SET_NOTIFY 0x01u
#define ENDPOINT_RESET 0x02u

/** Tunnel fragment header: bit 7 marks the last fragment. */
#define FRAGMENT_LAST 0x80u
#define FRAGMENT_INDEX_MASK 0x7fu
/** Message bytes in one tunnel fragment. */
#define FRAGMENT_PAYLOAD (CRTP_MAX_DATA_SIZE - 1u)
/** Bytes before the Control message in each tunnel direction. */
#define UPLINK_HEADER 1u      // endpoint_id
#define DOWNLINK_HEADER 2u    // endpoint_id, status

#define RX_QUEUE_LENGTH 16

typedef enum {
  SLOT_FREE = 0,
  SLOT_RECEIVING,
  SLOT_SUBMITTED,
  SLOT_DONE,
} slotState_t;

typedef struct {
  slotState_t state;               ///< SUBMITTED -> DONE written by the callback.
  bool orphan;                     ///< Set by RESET: free on completion, send nothing.
  uint8_t endpointId;
  uint8_t kind;                    ///< Kept for failure bodies.
  uint8_t requestId;               ///< Kept for failure bodies and EBUSY.
  uint8_t nextIndex;               ///< Next expected fragment index while receiving.
  uint16_t length;                 ///< Request length, then response length.
  int status;                      ///< Completion errno, 0 on success.
  ControlRequest request;          ///< .response = buffer, .user = this slot.
  uint8_t buffer[MAX_MESSAGE];     ///< Request, then response in place.
} controlSlot_t;

static controlSlot_t slots[SLOTS];
/** The slot being reassembled, or NULL. At most one, as messages do not interleave. */
static controlSlot_t *receiving;

static bool catalogReady;
static uint8_t catalogCount;
static uint32_t catalogCrc;

static bool isInit;

#ifndef UNIT_TEST_MODE
static TaskHandle_t task;
static xQueueHandle rxQueue;
static uint32_t rxDropped;
STATIC_MEM_QUEUE_ALLOC(rxQueue, RX_QUEUE_LENGTH, sizeof(CRTPPacket));
STATIC_MEM_TASK_ALLOC(crtpControlTask, CRTP_CONTROL_TASK_STACKSIZE);
#endif

// ---- Hooks: the only FreeRTOS calls of the handlers ----

static void sendPacket(CRTPPacket *packet)
{
  (void)crtpSendPacketBlock(packet);
}

static void notifyTask(void)
{
#ifndef UNIT_TEST_MODE
  if (task != NULL) {
    xTaskNotifyGive(task);
  }
#endif
}

// ---- Slots ----

static slotState_t slotState(const controlSlot_t *slot)
{
  return __atomic_load_n(&slot->state, __ATOMIC_ACQUIRE);
}

static void slotSetState(controlSlot_t *slot, slotState_t state)
{
  __atomic_store_n(&slot->state, state, __ATOMIC_RELEASE);
}

static void slotFree(controlSlot_t *slot)
{
  slot->orphan = false;
  slotSetState(slot, SLOT_FREE);
}

static controlSlot_t *slotFindFree(void)
{
  for (int i = 0; i < SLOTS; i++) {
    if (slotState(&slots[i]) == SLOT_FREE) {
      return &slots[i];
    }
  }
  return NULL;
}

/** True when this client already has this request ID outstanding at this endpoint. */
static bool requestIdOutstanding(uint8_t endpointId, uint8_t requestId)
{
  for (int i = 0; i < SLOTS; i++) {
    const slotState_t state = slotState(&slots[i]);
    if ((state == SLOT_SUBMITTED || state == SLOT_DONE) && !slots[i].orphan &&
        slots[i].endpointId == endpointId && slots[i].requestId == requestId) {
      return true;
    }
  }
  return false;
}

/** Router completion. Provider's task, maybe inside controlSubmit(). */
static void onComplete(ControlRequest *request, int status, size_t length)
{
  controlSlot_t *slot = request->user;
  slot->status = status;
  slot->length = (uint16_t)length;
  slotSetState(slot, SLOT_DONE);
  notifyTask();
}

// ---- Downlink ----

static uint8_t statusByte(int status)
{
  return (status > 0 && status <= 0xff) ? (uint8_t)status : (uint8_t)EIO;
}

/** Send [endpointId, status, body] as tunnel fragments. Task only. */
static void sendMessage(uint8_t endpointId, uint8_t status,
                        const uint8_t *body, size_t bodyLength)
{
  const size_t total = DOWNLINK_HEADER + bodyLength;
  size_t offset = 0u;   // offset in the whole message
  uint8_t index = 0u;
  while (offset < total) {
    CRTPPacket packet;
    packet.header = CRTP_HEADER(CRTP_PORT_CONTROL, CHANNEL_TUNNEL);
    size_t chunk = total - offset;
    if (chunk > FRAGMENT_PAYLOAD) {
      chunk = FRAGMENT_PAYLOAD;
    }
    const bool last = offset + chunk == total;
    packet.data[0] = (uint8_t)(index | (last ? FRAGMENT_LAST : 0u));
    for (size_t i = 0u; i < chunk; i++) {
      const size_t at = offset + i;
      uint8_t byte;
      if (at == 0u) {
        byte = endpointId;
      } else if (at == 1u) {
        byte = status;
      } else {
        byte = body[at - DOWNLINK_HEADER];
      }
      packet.data[1u + i] = byte;
    }
    packet.size = (uint8_t)(1u + chunk);
    sendPacket(&packet);
    offset += chunk;
    index++;
  }
}

static void sendFailure(uint8_t endpointId, int status, uint8_t kind, uint8_t requestId)
{
  const uint8_t body[2] = {kind, requestId};
  sendMessage(endpointId, statusByte(status), body, sizeof(body));
}

/** Send or drop the response of a DONE slot, then free it. Task only. */
static void finishSlot(controlSlot_t *slot)
{
  if (!slot->orphan) {
    if (slot->status == 0) {
      sendMessage(slot->endpointId, 0u, slot->buffer, slot->length);
    } else {
      sendFailure(slot->endpointId, slot->status, slot->kind, slot->requestId);
    }
  }
  slotFree(slot);
}

static void finishSlots(void)
{
  for (int i = 0; i < SLOTS; i++) {
    if (slotState(&slots[i]) == SLOT_DONE) {
      finishSlot(&slots[i]);
    }
  }
}

// ---- Catalog ----

static bool catalogLoad(void)
{
  if (!controlEndpointsAreFrozen()) {
    return false;
  }
  if (!catalogReady) {
    crc32Context_t context;
    crc32ContextInit(&context);
    catalogCount = controlEndpointCount();
    for (uint8_t id = 0u; id < catalogCount; id++) {
      const ControlEndpoint *endpoint = controlEndpointById(id);
      crc32Update(&context, &endpoint->id, sizeof(endpoint->id));
      crc32Update(&context, endpoint->path, endpoint->pathLength);
    }
    catalogCrc = crc32Out(&context);
    catalogReady = true;
  }
  return true;
}

static void sendCommandError(CRTPPacket *packet, uint8_t error)
{
  packet->size = 2u;
  packet->data[1] = error;
  sendPacket(packet);
}

static void putU16(uint8_t *at, uint16_t value)
{
  at[0] = (uint8_t)value;
  at[1] = (uint8_t)(value >> 8);
}

static void handleCatalog(CRTPPacket *packet)
{
  if (packet->size == 0u) {
    return;
  }
  const uint8_t command = packet->data[0];
  if (command != CATALOG_GET_ITEM && command != CATALOG_GET_INFO) {
    sendCommandError(packet, ENOSYS);
    return;
  }
  if (command == CATALOG_GET_ITEM && packet->size < 2u) {
    sendCommandError(packet, EINVAL);
    return;
  }
  if (!catalogLoad()) {
    sendCommandError(packet, EAGAIN);
    return;
  }

  if (command == CATALOG_GET_INFO) {
    packet->data[1] = 0u;
    packet->data[2] = catalogCount;
    memcpy(&packet->data[3], &catalogCrc, sizeof(catalogCrc));
    packet->data[7] = (uint8_t)SLOTS;
    putU16(&packet->data[8], (uint16_t)MAX_MESSAGE);
    packet->size = 10u;
  } else {
    const uint8_t id = packet->data[1];
    const ControlEndpoint *endpoint = controlEndpointById(id);
    if (endpoint == NULL) {
      sendCommandError(packet, ENOENT);
      return;
    }
    packet->data[1] = 0u;
    packet->data[2] = id;
    memcpy(&packet->data[3], endpoint->path, endpoint->pathLength);
    packet->size = (uint8_t)(3u + endpoint->pathLength);
  }
  sendPacket(packet);
}

/** Drop the message being reassembled and orphan everything in flight. */
static void reset(void)
{
  if (receiving != NULL) {
    slotFree(receiving);
    receiving = NULL;
  }
  for (int i = 0; i < SLOTS; i++) {
    switch (slotState(&slots[i])) {
      case SLOT_SUBMITTED:
        slots[i].orphan = true;
        break;
      case SLOT_DONE:
        slotFree(&slots[i]);
        break;
      default:
        break;
    }
  }
  // A SUBMITTED slot may complete right after it is read here. That is safe:
  // the orphan flag is read by this task only, in finishSlot(), after this.
}

static void handleEndpointCommand(CRTPPacket *packet)
{
  if (packet->size == 0u) {
    return;
  }
  switch (packet->data[0]) {
    case ENDPOINT_GET_STATUS: {
      if (packet->size < 2u) {
        sendCommandError(packet, EINVAL);
        return;
      }
      if (!catalogLoad()) {
        sendCommandError(packet, EAGAIN);
        return;
      }
      const uint8_t id = packet->data[1];
      const ControlEndpoint *endpoint = controlEndpointById(id);
      if (endpoint == NULL) {
        sendCommandError(packet, ENOENT);
        return;
      }
      packet->data[1] = 0u;
      packet->data[2] = id;
      packet->data[3] = controlEndpointIsReady(endpoint) ? 1u : 0u;
      putU16(&packet->data[4], endpoint->maxRequest);
      putU16(&packet->data[6], endpoint->maxResponse);
      packet->size = 8u;
      sendPacket(packet);
      break;
    }
    case ENDPOINT_RESET:
      reset();
      packet->data[1] = 0u;
      packet->size = 2u;
      sendPacket(packet);
      break;
    case ENDPOINT_SET_NOTIFY:
    default:
      sendCommandError(packet, ENOSYS);
      break;
  }
}

// ---- Uplink ----

/** Answer the message being reassembled with an error and drop it. */
static void receivingFail(int status)
{
  controlSlot_t *slot = receiving;
  receiving = NULL;
  sendFailure(slot->endpointId, status, slot->kind, slot->requestId);
  slotFree(slot);
}

/** Check fragment 0 and take a slot. Answers and returns NULL on failure. */
static controlSlot_t *startMessage(const uint8_t *payload, size_t length)
{
  if (length < UPLINK_HEADER + 2u) {
    return NULL;   // No request ID to answer with.
  }
  const uint8_t endpointId = payload[0];
  const uint8_t kind = payload[1];
  const uint8_t requestId = payload[2];
  int error = 0;
  if (!catalogLoad()) {
    error = EAGAIN;
  } else if (endpointId >= catalogCount) {
    error = ENOENT;
  } else if ((kind & 0x80u) != 0u) {
    error = EINVAL;
  } else if (requestIdOutstanding(endpointId, requestId)) {
    error = EBUSY;
  }
  controlSlot_t *slot = NULL;
  if (error == 0) {
    slot = slotFindFree();
    if (slot == NULL) {
      error = ENOBUFS;
    }
  }
  if (error != 0) {
    sendFailure(endpointId, error, kind, requestId);
    return NULL;
  }

  slot->orphan = false;
  slot->endpointId = endpointId;
  slot->kind = kind;
  slot->requestId = requestId;
  slot->nextIndex = 0u;
  slot->length = 0u;
  slot->status = 0;
  slot->request.response = slot->buffer;
  slot->request.responseCapacity = sizeof(slot->buffer);
  slot->request.complete = onComplete;
  slot->request.user = slot;
  slotSetState(slot, SLOT_RECEIVING);
  return slot;
}

static void submit(controlSlot_t *slot)
{
  ControlEndpoint *endpoint = controlEndpointById(slot->endpointId);
  // SUBMITTED before the call, so a completion inside it finds a consistent slot.
  slotSetState(slot, SLOT_SUBMITTED);
  const int result = controlSubmit(endpoint, &slot->request, slot->buffer, slot->length);
  if (result != 0) {
    sendFailure(slot->endpointId, result, slot->kind, slot->requestId);
    slotFree(slot);
  }
}

static void handleTunnel(CRTPPacket *packet)
{
  if (packet->size == 0u) {
    return;
  }
  const uint8_t index = packet->data[0] & FRAGMENT_INDEX_MASK;
  const bool last = (packet->data[0] & FRAGMENT_LAST) != 0u;
  const uint8_t *payload = &packet->data[1];
  size_t length = packet->size - 1u;

  if (index == 0u) {
    if (receiving != NULL) {
      receivingFail(EBADMSG);
    }
    receiving = startMessage(payload, length);
    if (receiving == NULL) {
      return;
    }
    payload += UPLINK_HEADER;
    length -= UPLINK_HEADER;
  } else if (receiving == NULL) {
    return;   // Ignoring until the next index 0.
  } else if (index != receiving->nextIndex) {
    receivingFail(EBADMSG);
    return;
  }

  if (receiving->length + length > sizeof(receiving->buffer)) {
    receivingFail(EMSGSIZE);
    return;
  }
  memcpy(&receiving->buffer[receiving->length], payload, length);
  receiving->length = (uint16_t)(receiving->length + length);
  receiving->nextIndex = (uint8_t)(index + 1u);

  if (last) {
    controlSlot_t *slot = receiving;
    receiving = NULL;
    submit(slot);
  }
}

static void handlePacket(CRTPPacket *packet)
{
  switch (packet->channel) {
    case CHANNEL_CATALOG:
      handleCatalog(packet);
      break;
    case CHANNEL_ENDPOINT:
      handleEndpointCommand(packet);
      break;
    case CHANNEL_TUNNEL:
      handleTunnel(packet);
      break;
    default:
      break;
  }
}

// ---- Task ----

#ifndef UNIT_TEST_MODE
/** CRTP RX task: copy, never block. */
static void onPacket(CRTPPacket *packet)
{
  if (xQueueSend(rxQueue, packet, 0) == pdTRUE) {
    xTaskNotifyGive(task);
  } else {
    rxDropped++;
  }
}

static void crtpControlTask(void *arg)
{
  (void)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    CRTPPacket packet;
    while (xQueueReceive(rxQueue, &packet, 0) == pdTRUE) {
      handlePacket(&packet);
    }
    finishSlots();
  }
}
#endif

void crtpControlInit(void)
{
  if (isInit) {
    return;
  }
#ifndef UNIT_TEST_MODE
  rxQueue = STATIC_MEM_QUEUE_CREATE(rxQueue);
  task = STATIC_MEM_TASK_CREATE(crtpControlTask, crtpControlTask,
                                CRTP_CONTROL_TASK_NAME, NULL, CRTP_CONTROL_TASK_PRI);
  crtpRegisterPortCB(CRTP_PORT_CONTROL, onPacket);
#endif
  isInit = true;
}

bool crtpControlTest(void)
{
  return isInit;
}

#ifdef UNIT_TEST_MODE
void crtpControlHandlePacketForTest(CRTPPacket *packet)
{
  handlePacket(packet);
}

void crtpControlFinishSlotsForTest(void)
{
  finishSlots();
}

void crtpControlResetForTest(void)
{
  memset(slots, 0, sizeof(slots));
  receiving = NULL;
  catalogReady = false;
  catalogCount = 0u;
  catalogCrc = 0u;
  isInit = false;
}
#endif
