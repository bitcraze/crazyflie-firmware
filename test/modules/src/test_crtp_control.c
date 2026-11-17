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
 * Host tests for the CRTP Control bridge, with fake providers registered in
 * the real Control router. Captured CRTP packets are compared byte for byte.
 *
 * @file
 */

#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include "control.h"
#include "crc32.h"
#include "crtp.h" // @NO_MODULE
#include "crtp_control.h"
#include "unity.h"

#define SLOTS CONFIG_CRTP_CONTROL_SLOTS
#define MAX_MESSAGE CONFIG_CRTP_CONTROL_MAX_MESSAGE

void vPortEnterCritical(void) {}
void vPortExitCritical(void) {}

// ---- CRTP fixture ----

static CRTPPacket sent[64];
static size_t sentCount;

int crtpSendPacketBlock(CRTPPacket *packet) {
  TEST_ASSERT_LESS_THAN(sizeof(sent) / sizeof(sent[0]), sentCount);
  sent[sentCount++] = *packet;
  return 1;
}

static void deliver(uint8_t channel, const uint8_t *data, size_t length) {
  CRTPPacket packet;
  memset(&packet, 0, sizeof(packet));
  packet.header = CRTP_HEADER(CRTP_PORT_CONTROL, channel);
  memcpy(packet.data, data, length);
  packet.size = (uint8_t)length;
  crtpControlHandlePacketForTest(&packet);
}

#define DELIVER(channel, ...) do { \
    const uint8_t bytes_[] = {__VA_ARGS__}; \
    deliver((channel), bytes_, sizeof(bytes_)); \
  } while (0)

static void expectPacket(size_t index, uint8_t channel, const uint8_t *data, size_t length) {
  TEST_ASSERT_LESS_THAN(sentCount, index);
  TEST_ASSERT_EQUAL_UINT8(CRTP_PORT_CONTROL, sent[index].port);
  TEST_ASSERT_EQUAL_UINT8(channel, sent[index].channel);
  TEST_ASSERT_EQUAL_UINT8(length, sent[index].size);
  TEST_ASSERT_EQUAL_MEMORY(data, sent[index].data, length);
}

#define EXPECT_PACKET(index, channel, ...) do { \
    const uint8_t bytes_[] = {__VA_ARGS__}; \
    expectPacket((index), (channel), bytes_, sizeof(bytes_)); \
  } while (0)

// ---- Fake providers ----

typedef struct {
  bool ready;
  int submitResult;
  bool completeInSubmit;
  unsigned submits;
  ControlRequest *last;
  uint8_t lastMessage[MAX_MESSAGE];
  size_t lastLength;
} FakeProvider;

static FakeProvider fakes[2];
static ControlEndpoint endpoints[2];

static bool fakeIsReady(const ControlEndpoint *endpoint) {
  return ((FakeProvider *)endpoint->context)->ready;
}

static int fakeSubmit(const ControlEndpoint *endpoint, ControlRequest *request,
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
    controlRequestComplete(request, 0, ok, sizeof(ok));
  }
  return 0;
}

static const ControlEndpointOps fakeOps = {fakeIsReady, fakeSubmit};

/** Register cf:stm32 (0) and deck:bcCam (1), both ready, then freeze. */
static void registerAndFreeze(void) {
  const char *paths[] = {"cf:stm32", "deck:bcCam"};
  for (int i = 0; i < 2; i++) {
    endpoints[i].path = paths[i];
    endpoints[i].ops = &fakeOps;
    endpoints[i].context = &fakes[i];
    endpoints[i].maxRequest = 64u;
    endpoints[i].maxResponse = 64u;
    TEST_ASSERT_EQUAL(0, controlEndpointRegister(&endpoints[i]));
    fakes[i].ready = true;
  }
  controlEndpointFreeze();
}

/** Complete the last request seen by fake 1 with a response using its wire ID. */
static void completeLast(int fakeIndex, int status, const uint8_t *response, size_t length) {
  uint8_t copy[400];
  if (response != NULL) {
    memcpy(copy, response, length);
    copy[1] = fakes[fakeIndex].lastMessage[1];
  }
  controlRequestComplete(fakes[fakeIndex].last, status, response ? copy : NULL, length);
}

void setUp(void) {
  controlResetForTest();
  crtpControlResetForTest();
  memset(fakes, 0, sizeof(fakes));
  memset(endpoints, 0, sizeof(endpoints));
  memset(sent, 0, sizeof(sent));
  sentCount = 0u;
}

void tearDown(void) {}

// ---- Catalog ----

void testEverythingButResetIsAgainBeforeFreeze(void) {
  DELIVER(0, 0x01);
  DELIVER(0, 0x00, 0);
  DELIVER(1, 0x00, 0);
  DELIVER(2, 0x80, 0, 0x01, 7, 1, 'q');
  DELIVER(1, 0x02);
  TEST_ASSERT_EQUAL_UINT(5, sentCount);
  EXPECT_PACKET(0, 0, 0x01, EAGAIN);
  EXPECT_PACKET(1, 0, 0x00, EAGAIN);
  EXPECT_PACKET(2, 1, 0x00, EAGAIN);
  EXPECT_PACKET(3, 2, 0x80, 0, EAGAIN, 0x01, 7);
  EXPECT_PACKET(4, 1, 0x02, 0);
}

void testCatalogAfterFreeze(void) {
  registerAndFreeze();
  DELIVER(0, 0x01);
  crc32Context_t context;
  crc32ContextInit(&context);
  const uint8_t entries[] = {0, 'c', 'f', ':', 's', 't', 'm', '3', '2',
                             1, 'd', 'e', 'c', 'k', ':', 'b', 'c', 'C', 'a', 'm'};
  crc32Update(&context, entries, sizeof(entries));
  const uint32_t crc = crc32Out(&context);
  EXPECT_PACKET(0, 0, 0x01, 0, 2, (uint8_t)crc, (uint8_t)(crc >> 8), (uint8_t)(crc >> 16),
                (uint8_t)(crc >> 24), SLOTS, (uint8_t)MAX_MESSAGE, (uint8_t)(MAX_MESSAGE >> 8));

  DELIVER(0, 0x00, 1);
  EXPECT_PACKET(1, 0, 0x00, 0, 1, 'd', 'e', 'c', 'k', ':', 'b', 'c', 'C', 'a', 'm');
  DELIVER(0, 0x00, 2);
  EXPECT_PACKET(2, 0, 0x00, ENOENT);
  DELIVER(0, 0x00);
  EXPECT_PACKET(3, 0, 0x00, EINVAL);
  DELIVER(0, 0x07);
  EXPECT_PACKET(4, 0, 0x07, ENOSYS);
  deliver(0, NULL, 0);
  TEST_ASSERT_EQUAL_UINT(5, sentCount);
}

void testEmptyCatalog(void) {
  controlEndpointFreeze();
  DELIVER(0, 0x01);
  EXPECT_PACKET(0, 0, 0x01, 0, 0, 0, 0, 0, 0, SLOTS, (uint8_t)MAX_MESSAGE, (uint8_t)(MAX_MESSAGE >> 8));
}

// ---- Endpoint commands ----

void testGetStatus(void) {
  registerAndFreeze();
  endpoints[1].maxResponse = 300u;   // Not a real link: shows both limits.
  DELIVER(1, 0x00, 1);
  EXPECT_PACKET(0, 1, 0x00, 0, 1, 1, 64, 0, 0x2c, 0x01);
  fakes[1].ready = false;
  DELIVER(1, 0x00, 1);
  EXPECT_PACKET(1, 1, 0x00, 0, 1, 0, 64, 0, 0x2c, 0x01);
  DELIVER(1, 0x00, 9);
  EXPECT_PACKET(2, 1, 0x00, ENOENT);
  DELIVER(1, 0x01, 1, 1);
  EXPECT_PACKET(3, 1, 0x01, ENOSYS);
  DELIVER(1, 0x00);
  EXPECT_PACKET(4, 1, 0x00, EINVAL);
  DELIVER(1, 0x09);
  EXPECT_PACKET(5, 1, 0x09, ENOSYS);
}

// ---- Tunnel ----

void testOnePacketGetRoundTrip(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  TEST_ASSERT_EQUAL_UINT(1, fakes[1].submits);
  TEST_ASSERT_EQUAL_UINT(4, fakes[1].lastLength);
  // The provider sees the router's wire ID, not the client's.
  TEST_ASSERT_EQUAL_UINT8(fakes[1].last->wireId, fakes[1].lastMessage[1]);
  TEST_ASSERT_EQUAL_UINT8(0x01, fakes[1].lastMessage[0]);
  TEST_ASSERT_EQUAL_UINT8('q', fakes[1].lastMessage[3]);
  TEST_ASSERT_EQUAL_UINT(0, sentCount);

  const uint8_t value[] = {0x81, 0, 0x10, 1, 80};
  completeLast(1, 0, value, sizeof(value));
  TEST_ASSERT_EQUAL_UINT(0, sentCount);
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(0, 2, 0x80, 1, 0, 0x81, 7, 0x10, 1, 80);
  TEST_ASSERT_EQUAL_UINT(1, sentCount);
}

void testClientIdRestoredWhenWireIdDiffers(void) {
  registerAndFreeze();
  // Use up wire ID 0 so the next request gets another one.
  DELIVER(2, 0x80, 1, 0x01, 3, 1, 'q');
  const uint8_t ok[] = {0x80, 0};
  completeLast(1, 0, ok, sizeof(ok));
  crtpControlFinishSlotsForTest();
  DELIVER(2, 0x80, 1, 0x01, 3, 1, 'q');
  TEST_ASSERT_NOT_EQUAL(3, fakes[1].lastMessage[1]);
  completeLast(1, 0, ok, sizeof(ok));
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(1, 2, 0x80, 1, 0, 0x80, 3);
}

void testTwoPacketSetReassembles(void) {
  registerAndFreeze();
  uint8_t message[46] = {0x02, 0x08, 0x09, 'w', 'i', 'f', 'i', '.', 's', 's', 'i', 'd', 0x20, 0x20};
  for (int i = 0; i < 32; i++) {
    message[14 + i] = (uint8_t)('A' + i);
  }
  uint8_t first[30] = {0x00, 1};
  memcpy(&first[2], message, 28);
  deliver(2, first, sizeof(first));
  TEST_ASSERT_EQUAL_UINT(0, fakes[1].submits);
  uint8_t second[19] = {0x81};
  memcpy(&second[1], &message[28], 18);
  deliver(2, second, sizeof(second));
  TEST_ASSERT_EQUAL_UINT(1, fakes[1].submits);
  TEST_ASSERT_EQUAL_UINT(46, fakes[1].lastLength);
  message[1] = fakes[1].lastMessage[1];
  TEST_ASSERT_EQUAL_MEMORY(message, fakes[1].lastMessage, 46);

  const uint8_t ok[] = {0x80, 0};
  completeLast(1, 0, ok, sizeof(ok));
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(0, 2, 0x80, 1, 0, 0x80, 0x08);
}

void testLongResponseIsThreeFragments(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  uint8_t value[60] = {0x81, 0, 0x21, 56};
  for (int i = 4; i < 60; i++) {
    value[i] = (uint8_t)i;
  }
  completeLast(1, 0, value, sizeof(value));
  crtpControlFinishSlotsForTest();
  TEST_ASSERT_EQUAL_UINT(3, sentCount);
  TEST_ASSERT_EQUAL_UINT8(0x00, sent[0].data[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, sent[1].data[0]);
  TEST_ASSERT_EQUAL_UINT8(0x82, sent[2].data[0]);
  TEST_ASSERT_EQUAL_UINT8(30, sent[0].size);
  TEST_ASSERT_EQUAL_UINT8(30, sent[1].size);
  TEST_ASSERT_EQUAL_UINT8(5, sent[2].size);

  uint8_t whole[62];
  memcpy(whole, &sent[0].data[1], 29);
  memcpy(&whole[29], &sent[1].data[1], 29);
  memcpy(&whole[58], &sent[2].data[1], 4);
  value[1] = 7;
  TEST_ASSERT_EQUAL_UINT8(1, whole[0]);
  TEST_ASSERT_EQUAL_UINT8(0, whole[1]);
  TEST_ASSERT_EQUAL_MEMORY(value, &whole[2], 60);
}

void testErrorWithTheLongestDetailIsForwardedUnchanged(void) {
  // A Camera Deck ERROR with a 60-byte detail, the most that fits in its 64-byte
  // frame. The router and the bridge pass the detail through without reading it,
  // multi-byte UTF-8 included.
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x02, 9, 1, 'q', 0x10, 1, 101);
  const char detail[] = "r\xc3\xa9solution 101 > max 100: stop the stream before setting it";
  TEST_ASSERT_EQUAL_UINT(60, sizeof(detail) - 1);
  uint8_t error[64] = {0x82, 0, 0x05, 60};
  memcpy(&error[4], detail, 60);
  completeLast(1, 0, error, sizeof(error));
  crtpControlFinishSlotsForTest();

  // [ep, status, 64-byte body] is 66 bytes: 29 + 29 + 8.
  TEST_ASSERT_EQUAL_UINT(3, sentCount);
  TEST_ASSERT_EQUAL_UINT8(0x82, sent[2].data[0]);
  TEST_ASSERT_EQUAL_UINT8(9, sent[2].size);
  uint8_t whole[66];
  memcpy(whole, &sent[0].data[1], 29);
  memcpy(&whole[29], &sent[1].data[1], 29);
  memcpy(&whole[58], &sent[2].data[1], 8);
  error[1] = 9;
  TEST_ASSERT_EQUAL_UINT8(1, whole[0]);
  TEST_ASSERT_EQUAL_UINT8(0, whole[1]);
  TEST_ASSERT_EQUAL_MEMORY(error, &whole[2], 64);
}

void testFragmentGapIsBadMessage(void) {
  registerAndFreeze();
  DELIVER(2, 0x00, 1, 0x02, 5, 1, 'q');
  DELIVER(2, 0x82, 'x');
  EXPECT_PACKET(0, 2, 0x80, 1, EBADMSG, 0x02, 5);
  // Ignored until the next index 0.
  DELIVER(2, 0x81, 'x');
  TEST_ASSERT_EQUAL_UINT(1, sentCount);
  TEST_ASSERT_EQUAL_UINT(0, fakes[1].submits);
}

void testNewMessageMidMessageIsBadMessageThenWorks(void) {
  registerAndFreeze();
  DELIVER(2, 0x00, 1, 0x02, 5, 1, 'q');
  DELIVER(2, 0x80, 1, 0x01, 6, 1, 'q');
  EXPECT_PACKET(0, 2, 0x80, 1, EBADMSG, 0x02, 5);
  TEST_ASSERT_EQUAL_UINT(1, fakes[1].submits);
  TEST_ASSERT_EQUAL_UINT8(0x01, fakes[1].lastMessage[0]);
  const uint8_t value[] = {0x81, 0, 0x10, 1, 1};
  completeLast(1, 0, value, sizeof(value));
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(1, 2, 0x80, 1, 0, 0x81, 6, 0x10, 1, 1);
}

void testOversizeRequestIsMessageSize(void) {
  registerAndFreeze();
  uint8_t packet[30];
  memset(packet, 'a', sizeof(packet));
  packet[0] = 0x00;
  packet[1] = 1;
  packet[2] = 0x02;
  packet[3] = 9;
  deliver(2, packet, sizeof(packet));      // 28 message bytes
  size_t total = 28u;
  uint8_t index = 1u;
  while (total + 29u <= MAX_MESSAGE) {
    packet[0] = index++;
    deliver(2, packet, sizeof(packet));
    total += 29u;
  }
  TEST_ASSERT_EQUAL_UINT(0, sentCount);
  packet[0] = index++;
  deliver(2, packet, sizeof(packet));
  EXPECT_PACKET(0, 2, 0x80, 1, EMSGSIZE, 0x02, 9);
  // The rest is ignored.
  packet[0] = (uint8_t)(0x80 | index);
  deliver(2, packet, sizeof(packet));
  TEST_ASSERT_EQUAL_UINT(1, sentCount);
  TEST_ASSERT_EQUAL_UINT(0, fakes[1].submits);
}

void testRequestOverEndpointLimitIsMessageSize(void) {
  registerAndFreeze();
  uint8_t packet[30];
  memset(packet, 'a', sizeof(packet));
  packet[0] = 0x00;
  packet[1] = 1;
  packet[2] = 0x02;
  packet[3] = 9;
  deliver(2, packet, sizeof(packet));      // 28
  packet[0] = 0x01;
  deliver(2, packet, sizeof(packet));      // 57
  packet[0] = 0x82;
  deliver(2, packet, 9);                   // 65 > 64
  EXPECT_PACKET(0, 2, 0x80, 1, EMSGSIZE, 0x02, 9);
  TEST_ASSERT_EQUAL_UINT(0, fakes[1].submits);
}

void testOversizeResponseIsMessageSize(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  uint8_t value[MAX_MESSAGE + 1] = {0x81, 0};
  completeLast(1, 0, value, sizeof(value));
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(0, 2, 0x80, 1, EMSGSIZE, 0x01, 7);
}

void testShortResponseIsProtocolError(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  const uint8_t shortResponse[] = {0x80};
  controlRequestComplete(fakes[1].last, 0, shortResponse, sizeof(shortResponse));
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(0, 2, 0x80, 1, EPROTO, 0x01, 7);
}

void testAllSlotsBusyIsNoBuffers(void) {
  registerAndFreeze();
  for (uint8_t i = 0; i < SLOTS; i++) {
    DELIVER(2, 0x80, 1, 0x01, i, 1, 'q');
  }
  TEST_ASSERT_EQUAL_UINT(SLOTS, fakes[1].submits);
  DELIVER(2, 0x80, 0, 0x01, 99, 1, 'q');
  EXPECT_PACKET(0, 2, 0x80, 0, ENOBUFS, 0x01, 99);
}

void testDuplicateRequestIdIsBusy(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  EXPECT_PACKET(0, 2, 0x80, 1, EBUSY, 0x01, 7);
  // Same ID on another endpoint is fine.
  DELIVER(2, 0x80, 0, 0x01, 7, 1, 'q');
  TEST_ASSERT_EQUAL_UINT(1, fakes[0].submits);
  // A done but not yet sent request still holds its ID.
  const uint8_t ok[] = {0x80, 0};
  completeLast(1, 0, ok, sizeof(ok));
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  EXPECT_PACKET(1, 2, 0x80, 1, EBUSY, 0x01, 7);
}

void testKindWithBit7IsInvalid(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x81, 7, 1, 'q');
  EXPECT_PACKET(0, 2, 0x80, 1, EINVAL, 0x81, 7);
}

void testUnknownEndpointIsNoEntry(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 2, 0x01, 7, 1, 'q');
  EXPECT_PACKET(0, 2, 0x80, 2, ENOENT, 0x01, 7);
}

void testShortFirstFragmentIsDropped(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01);
  DELIVER(2, 0x80, 1);
  DELIVER(2, 0x80);
  deliver(2, NULL, 0);
  TEST_ASSERT_EQUAL_UINT(0, sentCount);
  TEST_ASSERT_EQUAL_UINT(0, fakes[1].submits);
}

void testNotReadyIsNotConnected(void) {
  registerAndFreeze();
  fakes[1].ready = false;
  DELIVER(2, 0x80, 1, 0x01, 7, 1, 'q');
  EXPECT_PACKET(0, 2, 0x80, 1, ENOTCONN, 0x01, 7);
  TEST_ASSERT_EQUAL_UINT(0, fakes[1].submits);
}

void testSubmitAndCallbackErrnosAreForwarded(void) {
  registerAndFreeze();
  fakes[1].submitResult = EIO;
  DELIVER(2, 0x80, 1, 0x02, 4, 1, 'q', 0x10, 1, 5);
  EXPECT_PACKET(0, 2, 0x80, 1, EIO, 0x02, 4);
  fakes[1].submitResult = 0;
  DELIVER(2, 0x80, 1, 0x02, 4, 1, 'q', 0x10, 1, 5);
  completeLast(1, ECONNRESET, NULL, 0);
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(1, 2, 0x80, 1, ECONNRESET, 0x02, 4);
  // Slot freed: the same ID can be used again.
  DELIVER(2, 0x80, 1, 0x02, 4, 1, 'q', 0x10, 1, 5);
  TEST_ASSERT_EQUAL_UINT(3, fakes[1].submits);
}

void testCompletionInsideSubmit(void) {
  registerAndFreeze();
  fakes[1].completeInSubmit = true;
  DELIVER(2, 0x80, 1, 0x02, 4, 1, 'q', 0x10, 1, 5);
  crtpControlFinishSlotsForTest();
  EXPECT_PACKET(0, 2, 0x80, 1, 0, 0x80, 4);
  TEST_ASSERT_EQUAL_UINT(1, sentCount);
}

void testResetOrphansInFlightRequests(void) {
  registerAndFreeze();
  DELIVER(2, 0x80, 1, 0x01, 1, 1, 'q');
  ControlRequest *first = fakes[1].last;
  const uint8_t firstWire = fakes[1].lastMessage[1];
  DELIVER(2, 0x80, 1, 0x01, 2, 1, 'q');
  // The second is done but not sent yet.
  const uint8_t ok[] = {0x80, 0};
  completeLast(1, 0, ok, sizeof(ok));
  // A third is being reassembled.
  DELIVER(2, 0x00, 1, 0x02, 3, 1, 'q');

  DELIVER(1, 0x02);
  EXPECT_PACKET(0, 1, 0x02, 0);

  // The first completes after the RESET: nothing is sent, the slot is freed.
  const uint8_t late[] = {0x80, firstWire};
  controlRequestComplete(first, 0, late, sizeof(late));
  crtpControlFinishSlotsForTest();
  TEST_ASSERT_EQUAL_UINT(1, sentCount);
  // The rest of the reassembled message is ignored.
  DELIVER(2, 0x81, 0x10, 1, 1);
  TEST_ASSERT_EQUAL_UINT(1, sentCount);

  // All slots are free, and the old IDs can be used again.
  for (uint8_t i = 0; i < SLOTS; i++) {
    DELIVER(2, 0x80, 1, 0x01, (uint8_t)(1 + i), 1, 'q');
  }
  TEST_ASSERT_EQUAL_UINT(1, sentCount);
  TEST_ASSERT_EQUAL_UINT(2 + SLOTS, fakes[1].submits);
}

void testResetLeavesSubmittedSlotBusyUntilCompletion(void) {
  registerAndFreeze();
  for (uint8_t i = 0; i < SLOTS; i++) {
    DELIVER(2, 0x80, 1, 0x01, i, 1, 'q');
  }
  DELIVER(1, 0x02);
  // The new client may reuse an orphaned slot's ID, but there is no free slot.
  DELIVER(2, 0x80, 1, 0x01, 0, 1, 'q');
  EXPECT_PACKET(1, 2, 0x80, 1, ENOBUFS, 0x01, 0);
}
