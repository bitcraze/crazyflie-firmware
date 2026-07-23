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

#include <string.h>

#include "FreeRTOS.h"
#include "console.h"
#include "queue.h"
#include "unity.h"

// @MODULE "crc32.c"

/** Host stub for entering a FreeRTOS critical section. */
void vPortEnterCritical(void) {
}

/** Host stub for leaving a FreeRTOS critical section. */
void vPortExitCritical(void) {
}

/** Accept a non-blocking CRTP packet in the host fixture. */
int crtpSendPacket(void *packet) {
  (void)packet;
  return pdTRUE;
}

/** Accept a blocking CRTP packet in the host fixture. */
int crtpSendPacketBlock(void *packet) {
  (void)packet;
  return pdTRUE;
}

/** Ignore CRTP callback registration in registration-only tests. */
void crtpRegisterPortCB(int port, void (*callback)(void *)) {
  (void)port;
  (void)callback;
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

/** Unity per-test setup; the single catalog test starts from static defaults. */
void setUp(void) {
}

/** Unity per-test teardown; the fixture owns no resources. */
void tearDown(void) {
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
