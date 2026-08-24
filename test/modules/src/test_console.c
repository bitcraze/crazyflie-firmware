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
 * Host tests for sourced Console catalog registration.
 *
 * @file
 */

#include <errno.h>
#include <string.h>

#include "FreeRTOS.h"
#include "console.h"
#include "crtp.h" // @NO_MODULE
#include "queue.h"
#include "unity.h"

// @MODULE "crc32.c"

/** Callback registered by Console initialization. */
static CrtpCallback consoleCallback;
/** Packets accepted by the host CRTP transmit fixture. */
static CRTPPacket sentPackets[32];
/** Number of packets accepted by the host CRTP transmit fixture. */
static size_t sentPacketCount;
/** Nesting depth of the simulated task critical section. */
static unsigned int criticalDepth;
/** Result returned by the non-blocking CRTP send stub. */
static int nonblockingSendResult;
/** Inject a disable attempt at the next sourced-packet enqueue. */
static bool injectDisableAtSend;
/** Defer the injected disable until the simulated critical section exits. */
static bool disableDeferred;
/** Source ID used by the injected disable request. */
static uint8_t injectedDisableSourceId;

/** Dispatch the disable request used by the race fixture. */
static void dispatchInjectedDisable(void) {
  CRTPPacket request = {
    .header = CRTP_HEADER(CRTP_PORT_CONSOLE, 2u),
    .size = 3u,
    .data = {0u, injectedDisableSourceId, 0u},
  };
  consoleCallback(&request);
}

/** Host stub for entering a FreeRTOS critical section. */
void vPortEnterCritical(void) {
  criticalDepth++;
}

/** Host stub for leaving a FreeRTOS critical section. */
void vPortExitCritical(void) {
  TEST_ASSERT_GREATER_THAN(0u, criticalDepth);
  criticalDepth--;
  if (criticalDepth == 0u && disableDeferred) {
    disableDeferred = false;
    dispatchInjectedDisable();
  }
}

/** Capture one CRTP packet in the host fixture. */
static int capturePacket(CRTPPacket *packet) {
  TEST_ASSERT_LESS_THAN(sizeof(sentPackets) / sizeof(sentPackets[0]), sentPacketCount);
  memcpy(&sentPackets[sentPacketCount++], packet, sizeof(CRTPPacket));
  return pdTRUE;
}

/** Accept a non-blocking CRTP packet in the host fixture. */
int crtpSendPacket(CRTPPacket *packet) {
  if (nonblockingSendResult != pdTRUE) {
    return nonblockingSendResult;
  }
  if (injectDisableAtSend) {
    injectDisableAtSend = false;
    if (criticalDepth == 0u) {
      dispatchInjectedDisable();
    } else {
      disableDeferred = true;
    }
  }
  return capturePacket(packet);
}

/** Accept a blocking CRTP packet in the host fixture. */
int crtpSendPacketBlock(CRTPPacket *packet) {
  return capturePacket(packet);
}

/** Capture CRTP callback registration for request/reply tests. */
void crtpRegisterPortCB(int port, CrtpCallback callback) {
  TEST_ASSERT_EQUAL(CRTP_PORT_CONSOLE, port);
  consoleCallback = callback;
}

/** Report one free CRTP transmit slot to the host fixture. */
int crtpGetFreeTxQueuePackets(void) {
  return 1;
}

/** Return dummy queue storage for Console initialization. */
QueueHandle_t xQueueGenericCreate(const UBaseType_t length,
                                  const UBaseType_t itemSize,
                                  const uint8_t queueType) {
  /** Stable dummy address used as the fixture queue handle. */
  static uint8_t queueStorage;
  (void)length;
  (void)itemSize;
  (void)queueType;
  return (QueueHandle_t)&queueStorage;
}

/** Accept a queued item in the host fixture. */
BaseType_t xQueueGenericSend(QueueHandle_t queue,
                             const void * const item,
                             TickType_t ticksToWait,
                             const BaseType_t copyPosition) {
  (void)queue;
  (void)item;
  (void)ticksToWait;
  (void)copyPosition;
  return pdTRUE;
}

/** Make the fixture semaphore immediately available. */
BaseType_t xQueueSemaphoreTake(QueueHandle_t queue, TickType_t ticksToWait) {
  (void)queue;
  (void)ticksToWait;
  return pdTRUE;
}

/** Report no queued semaphore item to interrupt-context code. */
BaseType_t xQueueReceiveFromISR(
  QueueHandle_t queue,
  void * const buffer,
  BaseType_t * const higherPriorityTaskWoken) {
  (void)queue;
  (void)buffer;
  (void)higherPriorityTaskWoken;
  return pdFALSE;
}

/** Accept an interrupt-context semaphore release. */
BaseType_t xQueueGiveFromISR(
  QueueHandle_t queue,
  BaseType_t * const higherPriorityTaskWoken) {
  (void)queue;
  (void)higherPriorityTaskWoken;
  return pdTRUE;
}

/** Reset Console and initialize the callback-capture fixture. */
void setUp(void) {
  consoleResetForTest();
  consoleCallback = NULL;
  sentPacketCount = 0u;
  criticalDepth = 0u;
  nonblockingSendResult = pdTRUE;
  injectDisableAtSend = false;
  disableDeferred = false;
  injectedDisableSourceId = 0u;
  consoleInit();
  TEST_ASSERT_NOT_NULL(consoleCallback);
}

/** Unity per-test teardown; the fixture owns no resources. */
void tearDown(void) {
}

/** Dispatch one Console CRTP request through the registered callback. */
static void dispatchRequest(uint8_t channel, uint8_t size,
                            uint8_t command, uint8_t argument,
                            uint8_t value) {
  CRTPPacket request = {
    .header = CRTP_HEADER(CRTP_PORT_CONSOLE, channel),
    .size = size,
    .data = {command, argument, value},
  };
  consoleCallback(&request);
}

/** Forget packets captured before the next request assertion. */
static void clearSentPackets(void) {
  sentPacketCount = 0u;
}

/** Assert the common two-byte command error response. */
static void assertCommandError(uint8_t command, uint8_t error) {
  TEST_ASSERT_EQUAL_UINT32(1u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(2u, sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8(command, sentPackets[0].data[0]);
  TEST_ASSERT_EQUAL_UINT8(error, sentPackets[0].data[1]);
}

/** Catalog requests before freeze report that the service is not ready. */
void testCatalogRequestsBeforeFreezeReturnEagain(void) {
  CRTPPacket request = {
    .header = CRTP_HEADER(CRTP_PORT_CONSOLE, 3u),
    .size = 1u,
    .data = {1u},
  };

  consoleCallback(&request);

  TEST_ASSERT_EQUAL_UINT32(1u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(2u, sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8(1u, sentPackets[0].data[0]);
  TEST_ASSERT_EQUAL_UINT8(EAGAIN, sentPackets[0].data[1]);

  clearSentPackets();
  dispatchRequest(3u, 2u, 0u, 0u, 0u);
  assertCommandError(0u, EAGAIN);
}

/** Non-empty requests receive forward-compatible command errors. */
void testRequestsBeforeFreezeReportCommandErrors(void) {
  dispatchRequest(3u, 0u, 0u, 0u, 0u);
  TEST_ASSERT_EQUAL_UINT32(0u, sentPacketCount);

  dispatchRequest(3u, 1u, 0x7fu, 0u, 0u);
  assertCommandError(0x7fu, ENOSYS);
  clearSentPackets();

  dispatchRequest(3u, 2u, 1u, 0u, 0u);
  assertCommandError(1u, EINVAL);
  clearSentPackets();

  dispatchRequest(2u, 0u, 0u, 0u, 0u);
  TEST_ASSERT_EQUAL_UINT32(0u, sentPacketCount);

  dispatchRequest(2u, 1u, 0x7fu, 0u, 0u);
  assertCommandError(0x7fu, ENOSYS);
  clearSentPackets();

  dispatchRequest(2u, 2u, 0u, 0u, 0u);
  assertCommandError(0u, EINVAL);
  clearSentPackets();

  dispatchRequest(2u, 3u, 0u, 0u, 2u);
  assertCommandError(0u, EINVAL);
  clearSentPackets();

  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:bcCam"));
  dispatchRequest(2u, 3u, 0u, 0u, 1u);
  TEST_ASSERT_EQUAL_UINT8(4u, sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8(0u, sentPackets[0].data[0]);
  TEST_ASSERT_EQUAL_UINT8(0u, sentPackets[0].data[1]);
  TEST_ASSERT_EQUAL_UINT8(1u, sentPackets[0].data[2]);
  TEST_ASSERT_EQUAL_UINT8(EAGAIN, sentPackets[0].data[3]);
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(0u));
}

/** Frozen catalog replies have stable framing, contents, and CRC. */
void testFrozenCatalogReturnsInfoItemsAndErrors(void) {
  static const uint8_t expectedInfo[] = {1u, 2u, 0x4bu, 0x5eu, 0x1fu, 0x8eu};
  static const uint8_t expectedItem[] = {0u, 0u, 'd', 'e', 'c', 'k', ':',
                                        'b', 'c', 'C', 'a', 'm'};
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:bcCam"));
  TEST_ASSERT_EQUAL_INT(1, consoleSourceRegister("cf:nRF51"));
  consoleSourceFreeze();
  consoleSourceFreeze();
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister("deck:late"));

  dispatchRequest(3u, 1u, 1u, 0u, 0u);
  TEST_ASSERT_EQUAL_UINT32(1u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(sizeof(expectedInfo), sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedInfo, sentPackets[0].data, sizeof(expectedInfo));
  clearSentPackets();

  dispatchRequest(3u, 2u, 0u, 0u, 0u);
  TEST_ASSERT_EQUAL_UINT8(sizeof(expectedItem), sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedItem, sentPackets[0].data, sizeof(expectedItem));
  clearSentPackets();

  dispatchRequest(3u, 2u, 0u, 2u, 0u);
  assertCommandError(0u, ENOENT);
}

/** A source-free build exposes a valid immutable empty catalog. */
void testFrozenEmptyCatalogIsUsable(void) {
  static const uint8_t expectedInfo[] = {1u, 0u, 0u, 0u, 0u, 0u};
  consoleSourceFreeze();

  dispatchRequest(3u, 1u, 1u, 0u, 0u);

  TEST_ASSERT_EQUAL_UINT8(sizeof(expectedInfo), sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedInfo, sentPackets[0].data, sizeof(expectedInfo));
}

/** A disable response cannot overtake a sourced packet that won the race. */
void testDisableResponseOrdersAfterWinningSourceSend(void) {
  static const uint8_t payload[] = {'o', 'k'};
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:bcCam"));
  consoleSourceFreeze();
  dispatchRequest(2u, 3u, 0u, 0u, 1u);
  clearSentPackets();

  injectedDisableSourceId = 0u;
  injectDisableAtSend = true;
  TEST_ASSERT_TRUE(consoleSourceSend(0u, payload, sizeof(payload)));

  TEST_ASSERT_EQUAL_UINT32(2u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(1u, sentPackets[0].channel);
  TEST_ASSERT_EQUAL_UINT8(0u, sentPackets[0].data[0]);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, &sentPackets[0].data[1], sizeof(payload));
  TEST_ASSERT_EQUAL_UINT8(2u, sentPackets[1].channel);
  TEST_ASSERT_EQUAL_UINT8(0u, sentPackets[1].data[3]);
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(0u));
  TEST_ASSERT_FALSE(consoleSourceSend(0u, payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_UINT32(2u, sentPacketCount);
}

/** Disable-all has the same response barrier as single-source disable. */
void testDisableAllResponseOrdersAfterWinningSourceSend(void) {
  static const uint8_t payload[] = {'o', 'k'};
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:bcCam"));
  TEST_ASSERT_EQUAL_INT(1, consoleSourceRegister("cf:nRF51"));
  consoleSourceFreeze();
  dispatchRequest(2u, 3u, 0u, 0xffu, 1u);
  clearSentPackets();

  injectedDisableSourceId = 0xffu;
  injectDisableAtSend = true;
  TEST_ASSERT_TRUE(consoleSourceSend(0u, payload, sizeof(payload)));

  TEST_ASSERT_EQUAL_UINT32(2u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(1u, sentPackets[0].channel);
  TEST_ASSERT_EQUAL_UINT8(2u, sentPackets[1].channel);
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(0u));
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(1u));
}

/** Runtime control is default-off, idempotent, and supports all sources. */
void testRuntimeControlIsIdempotent(void) {
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:bcCam"));
  TEST_ASSERT_EQUAL_INT(1, consoleSourceRegister("cf:nRF51"));
  consoleSourceFreeze();
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(0u));
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(1u));

  dispatchRequest(2u, 3u, 0u, 0u, 1u);
  dispatchRequest(2u, 3u, 0u, 0u, 1u);
  TEST_ASSERT_TRUE(consoleSourceIsEnabled(0u));
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(1u));

  dispatchRequest(2u, 3u, 0u, 0xffu, 1u);
  TEST_ASSERT_TRUE(consoleSourceIsEnabled(0u));
  TEST_ASSERT_TRUE(consoleSourceIsEnabled(1u));

  dispatchRequest(2u, 3u, 0u, 0xffu, 0u);
  dispatchRequest(2u, 3u, 0u, 0xffu, 0u);
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(0u));
  TEST_ASSERT_FALSE(consoleSourceIsEnabled(1u));

  clearSentPackets();
  dispatchRequest(2u, 3u, 0u, 2u, 1u);
  TEST_ASSERT_EQUAL_UINT8(4u, sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8(ENOENT, sentPackets[0].data[3]);
}

/** Enable-all and disable-all succeed for a frozen empty catalog. */
void testRuntimeControlAllSucceedsForEmptyCatalog(void) {
  consoleSourceFreeze();

  dispatchRequest(2u, 3u, 0u, 0xffu, 1u);
  dispatchRequest(2u, 3u, 0u, 0xffu, 0u);

  TEST_ASSERT_EQUAL_UINT32(2u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(0u, sentPackets[0].data[3]);
  TEST_ASSERT_EQUAL_UINT8(0u, sentPackets[1].data[3]);
}

/** Sourced sends preserve framing and report disabled or congested output. */
void testSourceSendFramesDataAndReportsBackpressure(void) {
  static const uint8_t payload[] = {'a', 'b', 'c'};
  static const uint8_t expected[] = {0u, 'a', 'b', 'c'};
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:bcCam"));
  consoleSourceFreeze();

  TEST_ASSERT_FALSE(consoleSourceSend(0u, payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_UINT32(0u, sentPacketCount);
  dispatchRequest(2u, 3u, 0u, 0u, 1u);
  clearSentPackets();

  TEST_ASSERT_TRUE(consoleSourceSend(0u, payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_UINT32(1u, sentPacketCount);
  TEST_ASSERT_EQUAL_UINT8(1u, sentPackets[0].channel);
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), sentPackets[0].size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, sentPackets[0].data, sizeof(expected));

  clearSentPackets();
  nonblockingSendResult = pdFALSE;
  TEST_ASSERT_FALSE(consoleSourceSend(0u, payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_UINT32(0u, sentPacketCount);
  TEST_ASSERT_FALSE(consoleSourceSend(1u, payload, sizeof(payload)));
  TEST_ASSERT_FALSE(consoleSourceSend(0u, NULL, sizeof(payload)));
  TEST_ASSERT_FALSE(consoleSourceSend(0u, payload, CRTP_MAX_DATA_SIZE));
}

/** Verify catalog ownership, duplicate lookup, and bounded capacity. */
void testSourceCatalogOwnsPathsAndFindsDuplicatesWhenFull(void) {
  char mutablePath[] = "deck:first";
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister(mutablePath));

  memcpy(mutablePath, "deck:other", sizeof(mutablePath));
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister("deck:first"));

  TEST_ASSERT_EQUAL_INT(1, consoleSourceRegister("deck:1"));
  TEST_ASSERT_EQUAL_INT(2, consoleSourceRegister("deck:2"));
  TEST_ASSERT_EQUAL_INT(3, consoleSourceRegister("deck:3"));
  TEST_ASSERT_EQUAL_INT(4, consoleSourceRegister("deck:4"));
  TEST_ASSERT_EQUAL_INT(5, consoleSourceRegister("deck:5"));
  TEST_ASSERT_EQUAL_INT(6, consoleSourceRegister("deck:6"));
  TEST_ASSERT_EQUAL_INT(7, consoleSourceRegister("deck:7"));

  TEST_ASSERT_EQUAL_INT(4, consoleSourceRegister("deck:4"));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister("deck:8"));
}

/** Source paths reject invalid structure, UTF-8, and encoded lengths. */
void testSourceCatalogRejectsInvalidPaths(void) {
  const char overlongUtf8[] = {(char)0xc0, (char)0x80, '\0'};
  const char truncatedUtf8[] = {(char)0xf0, '\0'};
  const char surrogateUtf8[] = {(char)0xed, (char)0xa0, (char)0x80, '\0'};
  char maximumPath[CRTP_MAX_DATA_SIZE - 1u];
  char oversizedPath[CRTP_MAX_DATA_SIZE];
  memset(maximumPath, 'a', sizeof(maximumPath));
  maximumPath[0] = 'd';
  maximumPath[1] = ':';
  maximumPath[sizeof(maximumPath) - 1u] = '\0';
  memset(oversizedPath, 'a', sizeof(oversizedPath));
  oversizedPath[0] = 'd';
  oversizedPath[1] = ':';
  oversizedPath[sizeof(oversizedPath) - 1u] = '\0';

  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(NULL));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(""));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(":deck"));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister("deck:"));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister("deck::camera"));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(overlongUtf8));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(truncatedUtf8));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(surrogateUtf8));
  TEST_ASSERT_EQUAL_INT(0, consoleSourceRegister(maximumPath));
  TEST_ASSERT_EQUAL_INT(-1, consoleSourceRegister(oversizedPath));
}
