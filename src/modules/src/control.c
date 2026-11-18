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
 * control.c - Control router: endpoint registry and request routing.
 */

#include "control.h"

#include <errno.h>
#include <stdatomic.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

/** Bit 7 of the kind byte: set for target-to-initiator messages. */
#define CONTROL_KIND_TI_BIT 0x80u
/** First and last response kind (OK, VALUE, ERROR and reserved responses). */
#define CONTROL_KIND_RESPONSE_FIRST 0x80u
#define CONTROL_KIND_RESPONSE_LAST 0xbfu

/** Registered endpoints in ID order. */
static ControlEndpoint *endpointHead;
static ControlEndpoint *endpointTail;
/** Can reach 255, one past the highest assignable ID. */
static uint16_t endpointCount;
/**
 * True after registration has closed. The release store in
 * controlEndpointFreeze() publishes the list; readers load it with acquire
 * and then walk the immutable list without a lock.
 */
static atomic_bool endpointsFrozen;

/** True for a path of printable ASCII with ':' between non-empty segments. */
static bool validPath(const char *path, size_t *lengthOut)
{
  if (path == NULL || path[0] == '\0') {
    return false;
  }
  bool segmentStart = true;
  size_t length = 0u;
  for (const char *cursor = path; *cursor != '\0'; cursor++) {
    const char c = *cursor;
    if (c <= ' ' || c > '~') {
      return false;
    }
    if (c == ':') {
      if (segmentStart) {
        return false;
      }
      segmentStart = true;
    } else {
      segmentStart = false;
    }
    length++;
    if (length > CONTROL_ENDPOINT_PATH_MAX_LENGTH) {
      return false;
    }
  }
  if (segmentStart) {
    return false;
  }
  *lengthOut = length;
  return true;
}

int controlEndpointRegister(ControlEndpoint *endpoint)
{
  if (endpoint == NULL) {
    return EINVAL;
  }
  size_t length = 0u;
  if (!validPath(endpoint->path, &length) || endpoint->ops == NULL ||
      endpoint->ops->isReady == NULL || endpoint->ops->submit == NULL ||
      endpoint->maxRequest < 2u || endpoint->maxResponse < 2u) {
    return EINVAL;
  }
  // No lock: registration only happens from the startup task before the
  // freeze, so the list has a single writer.
  if (atomic_load_explicit(&endpointsFrozen, memory_order_relaxed)) {
    return EBUSY;
  }
  for (ControlEndpoint *entry = endpointHead; entry != NULL; entry = entry->next) {
    if (entry == endpoint || strcmp(entry->path, endpoint->path) == 0) {
      return EEXIST;
    }
  }
  if (endpointCount >= CONTROL_ENDPOINT_MAX) {
    return ENOSPC;
  }

  endpoint->next = NULL;
  endpoint->id = (uint8_t)endpointCount++;
  endpoint->pathLength = (uint8_t)length;
  endpoint->nextWireId = 0u;
  memset(endpoint->wireIdsInFlight, 0, sizeof(endpoint->wireIdsInFlight));
  if (endpointTail == NULL) {
    endpointHead = endpoint;
  } else {
    endpointTail->next = endpoint;
  }
  endpointTail = endpoint;
  return 0;
}

void controlEndpointFreeze(void)
{
  atomic_store_explicit(&endpointsFrozen, true, memory_order_release);
}

bool controlEndpointsAreFrozen(void)
{
  return atomic_load_explicit(&endpointsFrozen, memory_order_acquire);
}

uint8_t controlEndpointCount(void)
{
  return controlEndpointsAreFrozen() ? (uint8_t)endpointCount : 0u;
}

ControlEndpoint *controlEndpointById(uint8_t id)
{
  if (!controlEndpointsAreFrozen()) {
    return NULL;
  }
  for (ControlEndpoint *endpoint = endpointHead; endpoint != NULL; endpoint = endpoint->next) {
    if (endpoint->id == id) {
      return endpoint;
    }
  }
  return NULL;
}

bool controlEndpointIsReady(const ControlEndpoint *endpoint)
{
  return endpoint != NULL && endpoint->ops->isReady(endpoint);
}

/** Take the next free wire ID from nextWireId. False when all are in flight. */
static bool wireIdTake(ControlEndpoint *endpoint, uint8_t *wireId)
{
  bool found = false;
  taskENTER_CRITICAL();
  uint8_t candidate = endpoint->nextWireId;
  for (unsigned i = 0u; i < 256u; i++) {
    const uint32_t mask = 1uL << (candidate & 31u);
    uint32_t *word = &endpoint->wireIdsInFlight[candidate >> 5];
    if ((*word & mask) == 0u) {
      *word |= mask;
      endpoint->nextWireId = (uint8_t)(candidate + 1u);
      *wireId = candidate;
      found = true;
      break;
    }
    candidate++;
  }
  taskEXIT_CRITICAL();
  return found;
}

static void wireIdRelease(ControlEndpoint *endpoint, uint8_t wireId)
{
  taskENTER_CRITICAL();
  endpoint->wireIdsInFlight[wireId >> 5] &= ~(1uL << (wireId & 31u));
  taskEXIT_CRITICAL();
}

int controlSubmit(ControlEndpoint *endpoint, ControlRequest *request,
                  uint8_t *message, size_t length)
{
  if (endpoint == NULL || request == NULL || message == NULL ||
      request->complete == NULL || request->response == NULL ||
      length < 2u || (message[0] & CONTROL_KIND_TI_BIT) != 0u) {
    return EINVAL;
  }
  if (length > endpoint->maxRequest) {
    return EMSGSIZE;
  }
  if (!endpoint->ops->isReady(endpoint)) {
    return ENOTCONN;
  }
  uint8_t wireId;
  if (!wireIdTake(endpoint, &wireId)) {
    return ENOBUFS;
  }

  request->endpoint = endpoint;
  request->message = message;
  request->originatorId = message[1];
  request->wireId = wireId;
  message[1] = wireId;

  const int result = endpoint->ops->submit(endpoint, request, message, length);
  if (result != 0) {
    // Refused: the provider never completes it. Read the fields we need from
    // the request before anything else, as it belongs to the caller again.
    message[1] = request->originatorId;
    wireIdRelease(endpoint, wireId);
    return result;
  }
  // Accepted: the completion may already have run, so the request is not
  // touched again here.
  return 0;
}

void controlRequestComplete(ControlRequest *request, int status,
                            const uint8_t *response, size_t length)
{
  ControlEndpoint *endpoint = request->endpoint;
  const uint8_t wireId = request->wireId;

  if (status == 0) {
    if (response == NULL || length < 2u ||
        response[0] < CONTROL_KIND_RESPONSE_FIRST ||
        response[0] > CONTROL_KIND_RESPONSE_LAST ||
        response[1] != wireId) {
      status = EPROTO;
    } else if (length > request->responseCapacity) {
      status = EMSGSIZE;
    }
  }

  if (status == 0) {
    memmove(request->response, response, length);
    request->response[1] = request->originatorId;
  } else {
    request->message[1] = request->originatorId;
    length = 0u;
  }

  // Release first, so the originator may submit again from its completion
  // as soon as it hears back.
  wireIdRelease(endpoint, wireId);
  request->complete(request, status, length);
}

#ifdef UNIT_TEST_MODE
void controlResetForTest(void)
{
  endpointHead = NULL;
  endpointTail = NULL;
  endpointCount = 0u;
  atomic_store(&endpointsFrozen, false);
}
#endif
