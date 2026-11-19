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
 */

/**
 * Host tests for the API Service router, with fake providers.
 *
 * @file
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "apiservice.h"
#include "unity.h"

void vPortEnterCritical(void) {}
void vPortExitCritical(void) {}

// ---- Fake provider ----

typedef struct {
  bool ready;
  int submitResult;
  bool completeInSubmit;
  int inlineStatus;
  unsigned submits;
  ApiserviceRequest *last;
  uint8_t lastMessage[64];
  size_t lastLength;
} FakeProvider;

static FakeProvider fakes[2];

static bool fakeIsReady(const ApiserviceEndpoint *endpoint) {
  return ((FakeProvider *)endpoint->context)->ready;
}

static int fakeSubmit(const ApiserviceEndpoint *endpoint, ApiserviceRequest *request,
                      const uint8_t *message, size_t length) {
  FakeProvider *fake = endpoint->context;
  fake->submits++;
  fake->last = request;
  memcpy(fake->lastMessage, message, length);
  fake->lastLength = length;
  if (fake->submitResult != 0) {
    return fake->submitResult;
  }
  if (fake->completeInSubmit) {
    const uint8_t ok[] = {0x80, message[1]};
    apiserviceRequestComplete(request, fake->inlineStatus,
                           fake->inlineStatus == 0 ? ok : NULL,
                           fake->inlineStatus == 0 ? sizeof(ok) : 0u);
  }
  return 0;
}

static const ApiserviceEndpointOps fakeOps = {fakeIsReady, fakeSubmit};

static ApiserviceEndpoint endpoints[260];
static size_t nextEndpoint;

static ApiserviceEndpoint *newEndpoint(const char *path) {
  TEST_ASSERT_LESS_THAN(sizeof(endpoints) / sizeof(endpoints[0]), nextEndpoint);
  ApiserviceEndpoint *endpoint = &endpoints[nextEndpoint++];
  memset(endpoint, 0, sizeof(*endpoint));
  endpoint->path = path;
  endpoint->ops = &fakeOps;
  endpoint->context = &fakes[0];
  endpoint->maxRequest = 64u;
  endpoint->maxResponse = 64u;
  return endpoint;
}

// ---- Recording originator ----

typedef struct {
  unsigned calls;
  int status;
  size_t length;
} Completion;

static void recordComplete(ApiserviceRequest *request, int status, size_t length) {
  Completion *completion = request->user;
  completion->calls++;
  completion->status = status;
  completion->length = length;
}

typedef struct {
  ApiserviceRequest request;
  uint8_t buffer[64];
  Completion completion;
} Originator;

static void originatorInit(Originator *o, const uint8_t *message, size_t length) {
  memset(o, 0, sizeof(*o));
  memcpy(o->buffer, message, length);
  o->request.response = o->buffer;
  o->request.responseCapacity = sizeof(o->buffer);
  o->request.complete = recordComplete;
  o->request.user = &o->completion;
}

static const uint8_t getQ[] = {0x01, 7, 1, 'q'};

static ApiserviceEndpoint *singleReadyEndpoint(void) {
  ApiserviceEndpoint *endpoint = newEndpoint("deck:fake");
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(endpoint));
  apiserviceEndpointFreeze();
  fakes[0].ready = true;
  return endpoint;
}

void setUp(void) {
  apiserviceResetForTest();
  memset(fakes, 0, sizeof(fakes));
  nextEndpoint = 0u;
}

void tearDown(void) {}

// ---- Registration ----

void testRegisterRejectsBadEndpoints(void) {
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(NULL));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint(NULL)));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint(":deck")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("deck:")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("deck::cam")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("deck cam")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("deck:\x7f")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("deck:\xc3\xa9")));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(newEndpoint("a234567890123456789012345678")));
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(newEndpoint("a23456789012345678901234567")));

  ApiserviceEndpoint *endpoint = newEndpoint("deck:noops");
  endpoint->ops = NULL;
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(endpoint));
  static const ApiserviceEndpointOps noReady = {NULL, fakeSubmit};
  endpoint->ops = &noReady;
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(endpoint));
  static const ApiserviceEndpointOps noSubmit = {fakeIsReady, NULL};
  endpoint->ops = &noSubmit;
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(endpoint));

  endpoint = newEndpoint("deck:small");
  endpoint->maxRequest = 1u;
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(endpoint));
  endpoint->maxRequest = 2u;
  endpoint->maxResponse = 1u;
  TEST_ASSERT_EQUAL(EINVAL, apiserviceEndpointRegister(endpoint));
  endpoint->maxResponse = 2u;
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(endpoint));
}

void testRegisterRejectsDuplicates(void) {
  ApiserviceEndpoint *endpoint = newEndpoint("deck:bcCam");
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(endpoint));
  TEST_ASSERT_EQUAL(EEXIST, apiserviceEndpointRegister(endpoint));
  TEST_ASSERT_EQUAL(EEXIST, apiserviceEndpointRegister(newEndpoint("deck:bcCam")));
}

void testRegisterAccepts255Endpoints(void) {
  static char paths[256][8];
  for (unsigned i = 0; i < 255u; i++) {
    snprintf(paths[i], sizeof(paths[i]), "e:%u", i);
    TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(newEndpoint(paths[i])));
  }
  snprintf(paths[255], sizeof(paths[255]), "e:255");
  TEST_ASSERT_EQUAL(ENOSPC, apiserviceEndpointRegister(newEndpoint(paths[255])));
  apiserviceEndpointFreeze();
  TEST_ASSERT_EQUAL_UINT8(255u, apiserviceEndpointCount());
  TEST_ASSERT_EQUAL_STRING("e:254", apiserviceEndpointById(254)->path);
  TEST_ASSERT_NULL(apiserviceEndpointById(255));
}

void testRegisterAfterFreezeIsBusy(void) {
  apiserviceEndpointFreeze();
  TEST_ASSERT_EQUAL(EBUSY, apiserviceEndpointRegister(newEndpoint("deck:late")));
}

void testCatalogHiddenUntilFreezeThenInRegistrationOrder(void) {
  ApiserviceEndpoint *a = newEndpoint("deck:a");
  ApiserviceEndpoint *b = newEndpoint("deck:b");
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(a));
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(b));
  TEST_ASSERT_FALSE(apiserviceEndpointsAreFrozen());
  TEST_ASSERT_EQUAL_UINT8(0u, apiserviceEndpointCount());
  TEST_ASSERT_NULL(apiserviceEndpointById(0));

  apiserviceEndpointFreeze();
  TEST_ASSERT_TRUE(apiserviceEndpointsAreFrozen());
  TEST_ASSERT_EQUAL_UINT8(2u, apiserviceEndpointCount());
  TEST_ASSERT_EQUAL_PTR(a, apiserviceEndpointById(0));
  TEST_ASSERT_EQUAL_PTR(b, apiserviceEndpointById(1));
  TEST_ASSERT_NULL(apiserviceEndpointById(2));
  TEST_ASSERT_EQUAL_UINT8(6u, a->pathLength);
}

void testIsReadyAsksProvider(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  TEST_ASSERT_TRUE(apiserviceEndpointIsReady(endpoint));
  fakes[0].ready = false;
  TEST_ASSERT_FALSE(apiserviceEndpointIsReady(endpoint));
}

// ---- Submit checks ----

void testSubmitChecksInOrder(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));

  TEST_ASSERT_EQUAL(EINVAL, apiserviceSubmit(NULL, &o.request, o.buffer, 4));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceSubmit(endpoint, NULL, o.buffer, 4));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceSubmit(endpoint, &o.request, NULL, 4));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceSubmit(endpoint, &o.request, o.buffer, 1));
  o.buffer[0] = 0x81;
  // Kind wins over size and readiness.
  fakes[0].ready = false;
  TEST_ASSERT_EQUAL(EINVAL, apiserviceSubmit(endpoint, &o.request, o.buffer, 65));
  o.buffer[0] = 0x01;
  TEST_ASSERT_EQUAL(EMSGSIZE, apiserviceSubmit(endpoint, &o.request, o.buffer, 65));
  TEST_ASSERT_EQUAL(ENOTCONN, apiserviceSubmit(endpoint, &o.request, o.buffer, 64));
  TEST_ASSERT_EQUAL_UINT(0, fakes[0].submits);
  TEST_ASSERT_EQUAL_UINT(0, o.completion.calls);
}

void testProviderRefusalRestoresIdAndFreesWireId(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  fakes[0].submitResult = ENOBUFS;
  TEST_ASSERT_EQUAL(ENOBUFS, apiserviceSubmit(endpoint, &o.request, o.buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL_UINT8(7, o.buffer[1]);
  TEST_ASSERT_EQUAL_UINT(0, o.completion.calls);
  for (unsigned i = 0; i < 8; i++) {
    TEST_ASSERT_EQUAL_UINT32(0, endpoint->wireIdsInFlight[i]);
  }
}

void testSameOriginatorIdGetsDifferentWireIds(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  Originator a, b;
  originatorInit(&a, getQ, sizeof(getQ));
  originatorInit(&b, getQ, sizeof(getQ));
  b.buffer[1] = 7;

  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &a.request, a.buffer, sizeof(getQ)));
  const uint8_t wireA = fakes[0].lastMessage[1];
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &b.request, b.buffer, sizeof(getQ)));
  const uint8_t wireB = fakes[0].lastMessage[1];
  TEST_ASSERT_NOT_EQUAL(wireA, wireB);

  // Complete out of order, each with its own wire ID and a distinct value.
  const uint8_t valueB[] = {0x81, wireB, 0x10, 1, 22};
  apiserviceRequestComplete(&b.request, 0, valueB, sizeof(valueB));
  const uint8_t valueA[] = {0x81, wireA, 0x10, 1, 11};
  apiserviceRequestComplete(&a.request, 0, valueA, sizeof(valueA));

  TEST_ASSERT_EQUAL_UINT(1, a.completion.calls);
  TEST_ASSERT_EQUAL(0, a.completion.status);
  TEST_ASSERT_EQUAL_UINT(5, a.completion.length);
  const uint8_t expectA[] = {0x81, 7, 0x10, 1, 11};
  TEST_ASSERT_EQUAL_MEMORY(expectA, a.buffer, sizeof(expectA));
  const uint8_t expectB[] = {0x81, 7, 0x10, 1, 22};
  TEST_ASSERT_EQUAL_MEMORY(expectB, b.buffer, sizeof(expectB));
}

void testCompletionInsideSubmit(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  fakes[0].completeInSubmit = true;
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &o.request, o.buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL_UINT(1, o.completion.calls);
  TEST_ASSERT_EQUAL(0, o.completion.status);
  TEST_ASSERT_EQUAL_UINT(2, o.completion.length);
  TEST_ASSERT_EQUAL_UINT8(0x80, o.buffer[0]);
  TEST_ASSERT_EQUAL_UINT8(7, o.buffer[1]);
}

void testFailedCompletionInsideSubmit(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  fakes[0].completeInSubmit = true;
  fakes[0].inlineStatus = ECONNRESET;
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &o.request, o.buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL_UINT(1, o.completion.calls);
  TEST_ASSERT_EQUAL(ECONNRESET, o.completion.status);
  TEST_ASSERT_EQUAL_MEMORY(getQ, o.buffer, sizeof(getQ));
}

// ---- Response checks ----

static void submitAndComplete(Originator *o, int status, const uint8_t *response,
                              size_t length, bool useWireId) {
  ApiserviceEndpoint *endpoint = apiserviceEndpointById(0);
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &o->request, o->buffer, sizeof(getQ)));
  uint8_t copy[80];
  if (response != NULL) {
    memcpy(copy, response, length);
    if (useWireId && length >= 2u) {
      copy[1] = fakes[0].lastMessage[1];
    }
  }
  apiserviceRequestComplete(&o->request, status, response != NULL ? copy : NULL, length);
}

void testShortResponseIsProtocolError(void) {
  singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  const uint8_t shortResponse[] = {0x80};
  submitAndComplete(&o, 0, shortResponse, sizeof(shortResponse), false);
  TEST_ASSERT_EQUAL(EPROTO, o.completion.status);
  TEST_ASSERT_EQUAL_UINT(0, o.completion.length);
  TEST_ASSERT_EQUAL_MEMORY(getQ, o.buffer, sizeof(getQ));
}

void testWrongKindIsProtocolError(void) {
  singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  const uint8_t notify[] = {0xc0, 0, 1, 'q'};
  submitAndComplete(&o, 0, notify, sizeof(notify), true);
  TEST_ASSERT_EQUAL(EPROTO, o.completion.status);

  originatorInit(&o, getQ, sizeof(getQ));
  const uint8_t request[] = {0x01, 0, 1, 'q'};
  submitAndComplete(&o, 0, request, sizeof(request), true);
  TEST_ASSERT_EQUAL(EPROTO, o.completion.status);
}

void testWrongWireIdIsProtocolError(void) {
  singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  const uint8_t ok[] = {0x80, 200};
  submitAndComplete(&o, 0, ok, sizeof(ok), false);
  TEST_ASSERT_EQUAL(EPROTO, o.completion.status);
  TEST_ASSERT_EQUAL_UINT8(7, o.buffer[1]);
}

void testResponseTooLongForCapacity(void) {
  singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  o.request.responseCapacity = 4u;
  const uint8_t value[] = {0x81, 0, 0x10, 1, 5};
  submitAndComplete(&o, 0, value, sizeof(value), true);
  TEST_ASSERT_EQUAL(EMSGSIZE, o.completion.status);
  TEST_ASSERT_EQUAL_UINT(0, o.completion.length);
}

void testResponseWrittenOverMessageBuffer(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &o.request, o.buffer, sizeof(getQ)));
  // A provider answering from the message buffer itself.
  uint8_t *shared = o.buffer;
  const uint8_t wire = shared[1];
  shared[0] = 0x81;
  shared[2] = 0x10;
  shared[3] = 1;
  shared[4] = 42;
  TEST_ASSERT_EQUAL_UINT8(wire, shared[1]);
  apiserviceRequestComplete(&o.request, 0, shared, 5);
  const uint8_t expected[] = {0x81, 7, 0x10, 1, 42};
  TEST_ASSERT_EQUAL(0, o.completion.status);
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, sizeof(expected));
}

void testErrorResponseIsPassedThrough(void) {
  singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  const uint8_t error[] = {0x82, 0, 0x03, 2, 'n', 'o'};
  submitAndComplete(&o, 0, error, sizeof(error), true);
  TEST_ASSERT_EQUAL(0, o.completion.status);
  TEST_ASSERT_EQUAL_UINT(sizeof(error), o.completion.length);
  const uint8_t expected[] = {0x82, 7, 0x03, 2, 'n', 'o'};
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, sizeof(expected));
}

void testFailedCompletionRestoresId(void) {
  singleReadyEndpoint();
  Originator o;
  originatorInit(&o, getQ, sizeof(getQ));
  submitAndComplete(&o, ETIMEDOUT, NULL, 0, false);
  TEST_ASSERT_EQUAL_UINT(1, o.completion.calls);
  TEST_ASSERT_EQUAL(ETIMEDOUT, o.completion.status);
  TEST_ASSERT_EQUAL_UINT(0, o.completion.length);
  TEST_ASSERT_EQUAL_MEMORY(getQ, o.buffer, sizeof(getQ));
}

// ---- Wire IDs ----

void testWireIdsAreReleased(void) {
  singleReadyEndpoint();
  for (unsigned i = 0; i < 300u; i++) {
    Originator o;
    originatorInit(&o, getQ, sizeof(getQ));
    const uint8_t ok[] = {0x80, 0};
    submitAndComplete(&o, 0, ok, sizeof(ok), true);
    TEST_ASSERT_EQUAL(0, o.completion.status);
  }
}

void testAllWireIdsInFlightIsNoBuffers(void) {
  ApiserviceEndpoint *endpoint = singleReadyEndpoint();
  static Originator many[257];
  for (unsigned i = 0; i < 256u; i++) {
    originatorInit(&many[i], getQ, sizeof(getQ));
    TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &many[i].request, many[i].buffer, sizeof(getQ)));
  }
  originatorInit(&many[256], getQ, sizeof(getQ));
  TEST_ASSERT_EQUAL(ENOBUFS, apiserviceSubmit(endpoint, &many[256].request, many[256].buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL_UINT8(7, many[256].buffer[1]);

  // Free one in the middle: the next submit gets exactly that ID.
  const uint8_t freed = many[100].request.wireId;
  const uint8_t ok[] = {0x80, freed};
  apiserviceRequestComplete(&many[100].request, 0, ok, sizeof(ok));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(endpoint, &many[256].request, many[256].buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL_UINT8(freed, many[256].request.wireId);
}

void testWireIdsArePerEndpoint(void) {
  ApiserviceEndpoint *a = newEndpoint("deck:a");
  ApiserviceEndpoint *b = newEndpoint("deck:b");
  b->context = &fakes[1];
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(a));
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(b));
  apiserviceEndpointFreeze();
  fakes[0].ready = true;
  fakes[1].ready = true;
  Originator oa, ob;
  originatorInit(&oa, getQ, sizeof(getQ));
  originatorInit(&ob, getQ, sizeof(getQ));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(a, &oa.request, oa.buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(b, &ob.request, ob.buffer, sizeof(getQ)));
  TEST_ASSERT_EQUAL_UINT8(fakes[0].lastMessage[1], fakes[1].lastMessage[1]);
}

// ---- Listeners and NOTIFY ----

typedef struct {
  ApiserviceListener listener;
  unsigned calls;
  ApiserviceEndpoint *endpoint;
  uint8_t message[64];
  size_t length;
} RecordingListener;

static void recordNotify(ApiserviceListener *listener, ApiserviceEndpoint *endpoint,
                         const uint8_t *message, size_t length) {
  RecordingListener *r = (RecordingListener *)listener;
  r->calls++;
  r->endpoint = endpoint;
  memcpy(r->message, message, length);
  r->length = length;
}

static const uint8_t notifyQ[] = {0xc0, 1, 'q', 0x10, 1, 42};

void testListenerRegisterRejectsBadListeners(void) {
  RecordingListener r = {.listener = {.onNotify = NULL}};
  TEST_ASSERT_EQUAL(EINVAL, apiserviceListenerRegister(NULL));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceListenerRegister(&r.listener));
  r.listener.onNotify = recordNotify;
  TEST_ASSERT_EQUAL(0, apiserviceListenerRegister(&r.listener));
  TEST_ASSERT_EQUAL(EEXIST, apiserviceListenerRegister(&r.listener));
  apiserviceEndpointFreeze();
  RecordingListener late = {.listener = {.onNotify = recordNotify}};
  TEST_ASSERT_EQUAL(EBUSY, apiserviceListenerRegister(&late.listener));
}

void testNotifyReachesMatchingListeners(void) {
  ApiserviceEndpoint *a = newEndpoint("cf:stm32");
  ApiserviceEndpoint *b = newEndpoint("deck:bcCam");
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(a));
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(b));
  RecordingListener all = {.listener = {.onNotify = recordNotify, .endpoint = NULL}};
  RecordingListener onlyB = {.listener = {.onNotify = recordNotify, .endpoint = b}};
  TEST_ASSERT_EQUAL(0, apiserviceListenerRegister(&all.listener));
  TEST_ASSERT_EQUAL(0, apiserviceListenerRegister(&onlyB.listener));
  apiserviceEndpointFreeze();

  apiserviceEndpointNotify(a, notifyQ, sizeof(notifyQ));
  TEST_ASSERT_EQUAL_UINT(1, all.calls);
  TEST_ASSERT_EQUAL_UINT(0, onlyB.calls);
  TEST_ASSERT_EQUAL_PTR(a, all.endpoint);

  apiserviceEndpointNotify(b, notifyQ, sizeof(notifyQ));
  TEST_ASSERT_EQUAL_UINT(2, all.calls);
  TEST_ASSERT_EQUAL_UINT(1, onlyB.calls);
  TEST_ASSERT_EQUAL_PTR(b, onlyB.endpoint);
  TEST_ASSERT_EQUAL_UINT(sizeof(notifyQ), onlyB.length);
  TEST_ASSERT_EQUAL_MEMORY(notifyQ, onlyB.message, sizeof(notifyQ));
  TEST_ASSERT_EQUAL_UINT32(0, apiserviceNotifyDroppedCount());
}

void testNotifyDropsBadMessagesAndBeforeFreeze(void) {
  ApiserviceEndpoint *a = newEndpoint("deck:bcCam");
  TEST_ASSERT_EQUAL(0, apiserviceEndpointRegister(a));
  RecordingListener r = {.listener = {.onNotify = recordNotify}};
  TEST_ASSERT_EQUAL(0, apiserviceListenerRegister(&r.listener));

  apiserviceEndpointNotify(a, notifyQ, sizeof(notifyQ));   // before the freeze
  apiserviceEndpointFreeze();
  const uint8_t response[] = {0x80, 3};
  apiserviceEndpointNotify(a, response, sizeof(response)); // a response kind
  apiserviceEndpointNotify(a, notifyQ, 0);                 // empty
  apiserviceEndpointNotify(a, NULL, 3);
  apiserviceEndpointNotify(NULL, notifyQ, sizeof(notifyQ));
  TEST_ASSERT_EQUAL_UINT(0, r.calls);
  TEST_ASSERT_EQUAL_UINT32(5, apiserviceNotifyDroppedCount());

  // Unknown unsolicited kinds are passed on: the listeners decide.
  const uint8_t future[] = {0xc1};
  apiserviceEndpointNotify(a, future, sizeof(future));
  TEST_ASSERT_EQUAL_UINT(1, r.calls);
}
