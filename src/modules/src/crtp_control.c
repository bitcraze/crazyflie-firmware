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
 *
 * Notifications: the bridge is a Control listener. For each endpoint whose
 * forwarding the client turned on with SET_NOTIFY, it copies the endpoint's
 * NOTIFY messages into a byte ring, from the provider's task. The bridge task
 * sends them on the tunnel after the responses that are ready. When the ring
 * is full, a message is dropped and counted, and the count is reported with a
 * NOTIFY_LOST event on channel 3.
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
#ifndef CONFIG_CRTP_CONTROL_NOTIFY_BUFFER
#define CONFIG_CRTP_CONTROL_NOTIFY_BUFFER 512
#endif

#define SLOTS CONFIG_CRTP_CONTROL_SLOTS
#define MAX_MESSAGE CONFIG_CRTP_CONTROL_MAX_MESSAGE
#define NOTIFY_BUFFER CONFIG_CRTP_CONTROL_NOTIFY_BUFFER

/** CRTP channels on port 10. */
#define CHANNEL_CATALOG 0u
#define CHANNEL_ENDPOINT 1u
#define CHANNEL_TUNNEL 2u
#define CHANNEL_EVENTS 3u

/** Channel 0 commands. */
#define CATALOG_GET_ITEM 0x00u
#define CATALOG_GET_INFO 0x01u
/** Channel 1 commands. */
#define ENDPOINT_GET_STATUS 0x00u
#define ENDPOINT_SET_NOTIFY 0x01u
#define ENDPOINT_RESET 0x02u
/** SET_NOTIFY endpoint ID for every endpoint. */
#define ENDPOINT_ALL 0xffu
/** Channel 3 events. 0x00 is reserved for a STATE event. */
#define EVENT_NOTIFY_LOST 0x01u

/** Tunnel fragment header: bit 7 marks the last fragment. */
#define FRAGMENT_LAST 0x80u
#define FRAGMENT_INDEX_MASK 0x7fu
/** Message bytes in one tunnel fragment. */
#define FRAGMENT_PAYLOAD (CRTP_MAX_DATA_SIZE - 1u)
/** Bytes before the Control message in each tunnel direction. */
#define UPLINK_HEADER 1u      // endpoint_id
#define DOWNLINK_HEADER 2u    // endpoint_id, status

#define RX_QUEUE_LENGTH 16

/** A ring record is [endpoint_id, length u16 LE] then the message. */
#define NOTIFY_RECORD_HEADER 3u
/** While a loss is not reported yet, the task wakes this often to retry. */
#define NOTIFY_LOST_RETRY_MS 10u

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

// ---- Notification state ----
// Written by the listener in the provider's task and by this task, always in a
// critical section.

#if NOTIFY_BUFFER > 0
/** One bit per endpoint ID: forwarding turned on by SET_NOTIFY. */
static uint32_t notifyEnabled[(CONTROL_ENDPOINT_MAX + 31u) / 32u];
/** Notifications dropped per endpoint since the last NOTIFY_LOST, saturating. */
static uint8_t notifyDropped[CONTROL_ENDPOINT_MAX];
/** True while at least one notifyDropped entry is not 0. */
static bool notifyLostPending;
static uint8_t notifyRing[NOTIFY_BUFFER];
static size_t notifyHead;   ///< Next byte to read.
static size_t notifyUsed;
/** A message taken out of the ring for sending. Task only. */
static uint8_t notifyOut[MAX_MESSAGE];
static ControlListener notifyListener;
#endif

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

/** Send without waiting. False when the CRTP queue is full. */
static bool sendPacketTry(CRTPPacket *packet)
{
  return crtpSendPacket(packet) == pdTRUE;
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

// ---- Notifications ----

#if NOTIFY_BUFFER > 0
static bool notifyIsEnabled(uint8_t endpointId)
{
  return (notifyEnabled[endpointId / 32u] & (1uL << (endpointId % 32u))) != 0u;
}

static void notifySetEnabled(uint8_t endpointId, bool enabled)
{
  const uint32_t bit = 1uL << (endpointId % 32u);
  if (enabled) {
    notifyEnabled[endpointId / 32u] |= bit;
  } else {
    notifyEnabled[endpointId / 32u] &= ~bit;
  }
}

static void ringWrite(const uint8_t *bytes, size_t length)
{
  size_t at = (notifyHead + notifyUsed) % NOTIFY_BUFFER;
  for (size_t i = 0u; i < length; i++) {
    notifyRing[at] = bytes[i];
    at = (at + 1u) % NOTIFY_BUFFER;
  }
  notifyUsed += length;
}

static void ringRead(uint8_t *bytes, size_t length)
{
  for (size_t i = 0u; i < length; i++) {
    bytes[i] = notifyRing[notifyHead];
    notifyHead = (notifyHead + 1u) % NOTIFY_BUFFER;
  }
  notifyUsed -= length;
}

/** Router listener. Provider's task: copy and never block. */
static void onNotify(ControlListener *listener, ControlEndpoint *endpoint,
                     const uint8_t *message, size_t length)
{
  (void)listener;
  const uint8_t id = endpoint->id;
  bool queued = false;
  taskENTER_CRITICAL();
  if (notifyIsEnabled(id)) {
    if (length <= MAX_MESSAGE &&
        NOTIFY_RECORD_HEADER + length <= NOTIFY_BUFFER - notifyUsed) {
      const uint8_t header[NOTIFY_RECORD_HEADER] = {
        id, (uint8_t)length, (uint8_t)(length >> 8),
      };
      ringWrite(header, sizeof(header));
      ringWrite(message, length);
    } else if (notifyDropped[id] < UINT8_MAX) {
      notifyDropped[id]++;
    }
    if (notifyDropped[id] > 0u) {
      notifyLostPending = true;
    }
    queued = true;
  }
  taskEXIT_CRITICAL();
  if (queued) {
    notifyTask();
  }
}

/** Take the oldest message out of the ring. Task only. */
static bool notifyTake(uint8_t *endpointId, size_t *length)
{
  bool taken = false;
  taskENTER_CRITICAL();
  if (notifyUsed > 0u) {
    uint8_t header[NOTIFY_RECORD_HEADER];
    ringRead(header, sizeof(header));
    *endpointId = header[0];
    *length = (size_t)header[1] | ((size_t)header[2] << 8);
    ringRead(notifyOut, *length);
    taken = true;
  }
  taskEXIT_CRITICAL();
  return taken;
}

/** Report losses on channel 3, without waiting for room in the CRTP queue. */
static void sendLostEvents(void)
{
  // Forwarding needs SET_NOTIFY, which loads the catalog, so only catalog IDs
  // can have drops.
  bool pending = false;
  for (uint16_t id = 0u; id < catalogCount; id++) {
    taskENTER_CRITICAL();
    const uint8_t dropped = notifyDropped[id];
    taskEXIT_CRITICAL();
    if (dropped == 0u) {
      continue;
    }
    CRTPPacket packet;
    packet.header = CRTP_HEADER(CRTP_PORT_CONTROL, CHANNEL_EVENTS);
    packet.data[0] = EVENT_NOTIFY_LOST;
    packet.data[1] = (uint8_t)id;
    packet.data[2] = dropped;
    packet.size = 3u;
    if (sendPacketTry(&packet)) {
      // Drops counted meanwhile stay for the next report.
      taskENTER_CRITICAL();
      notifyDropped[id] = (uint8_t)(notifyDropped[id] - dropped);
      pending = pending || notifyDropped[id] > 0u;
      taskEXIT_CRITICAL();
    } else {
      pending = true;
    }
  }
  taskENTER_CRITICAL();
  // A drop counted after its endpoint was checked set the flag again.
  if (!pending) {
    for (uint16_t id = 0u; id < catalogCount && !pending; id++) {
      pending = notifyDropped[id] > 0u;
    }
  }
  notifyLostPending = pending;
  taskEXIT_CRITICAL();
}

static void notifyClear(void)
{
  taskENTER_CRITICAL();
  memset(notifyEnabled, 0, sizeof(notifyEnabled));
  memset(notifyDropped, 0, sizeof(notifyDropped));
  notifyLostPending = false;
  notifyHead = 0u;
  notifyUsed = 0u;
  taskEXIT_CRITICAL();
}

static bool notifyLostIsPending(void)
{
  taskENTER_CRITICAL();
  const bool pending = notifyLostPending;
  taskEXIT_CRITICAL();
  return pending;
}

static void notifyRegisterListener(void)
{
  memset(&notifyListener, 0, sizeof(notifyListener));
  notifyListener.onNotify = onNotify;
  (void)controlListenerRegister(&notifyListener);
}
#endif

/**
 * Send the ready responses, then the forwarded notifications, checking the
 * slots again after each one, then the loss reports. Task only.
 */
static void sendDownlink(void)
{
  finishSlots();
#if NOTIFY_BUFFER > 0
  uint8_t endpointId;
  size_t length;
  while (notifyTake(&endpointId, &length)) {
    sendMessage(endpointId, 0u, notifyOut, length);
    finishSlots();
  }
  if (notifyLostIsPending()) {
    sendLostEvents();
  }
#endif
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
#if NOTIFY_BUFFER > 0
  notifyClear();
#endif
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
    case ENDPOINT_SET_NOTIFY: {
#if NOTIFY_BUFFER > 0
      if (packet->size < 3u || packet->data[2] > 1u) {
        sendCommandError(packet, EINVAL);
        return;
      }
      if (!catalogLoad()) {
        sendCommandError(packet, EAGAIN);
        return;
      }
      const uint8_t id = packet->data[1];
      const bool enabled = packet->data[2] == 1u;
      if (id != ENDPOINT_ALL && id >= catalogCount) {
        sendCommandError(packet, ENOENT);
        return;
      }
      taskENTER_CRITICAL();
      for (uint16_t i = 0u; i < catalogCount; i++) {
        if (id == ENDPOINT_ALL || i == id) {
          notifySetEnabled((uint8_t)i, enabled);
        }
      }
      taskEXIT_CRITICAL();
      packet->data[3] = packet->data[2];
      packet->data[2] = id;
      packet->data[1] = 0u;
      packet->size = 4u;
      sendPacket(packet);
#else
      sendCommandError(packet, ENOSYS);
#endif
      break;
    }
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
    TickType_t wait = portMAX_DELAY;
#if NOTIFY_BUFFER > 0
    if (notifyLostIsPending()) {
      wait = M2T(NOTIFY_LOST_RETRY_MS);
    }
#endif
    ulTaskNotifyTake(pdTRUE, wait);
    CRTPPacket packet;
    while (xQueueReceive(rxQueue, &packet, 0) == pdTRUE) {
      handlePacket(&packet);
    }
    sendDownlink();
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
#if NOTIFY_BUFFER > 0
  // Before the freeze in systemStart(), like the endpoints.
  notifyRegisterListener();
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

void crtpControlSendDownlinkForTest(void)
{
  sendDownlink();
}

void crtpControlResetForTest(void)
{
  memset(slots, 0, sizeof(slots));
  receiving = NULL;
  catalogReady = false;
  catalogCount = 0u;
  catalogCrc = 0u;
  isInit = false;
#if NOTIFY_BUFFER > 0
  notifyClear();
  // The router was reset too: register again, before the test freezes it.
  notifyRegisterListener();
#endif
}
#endif
