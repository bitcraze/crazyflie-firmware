// @IGNORE_IF_NOT CONFIG_DECK_LIGHTHOUSE

// File under test lighthouse_core.c
#include "lighthouse_core.h"

#include "unity.h"
#include "mock_system.h"
#include "mock_pulse_processor.h"
#include "mock_pulse_processor_v1.h"
#include "mock_pulse_processor_v2.h"
#include "mock_lighthouse_transmit.h"
#include "mock_lighthouse_deck_flasher.h"
#include "mock_lighthouse_position_est.h"
#include "mock_lighthouse_calibration.h"
#include "mock_uart1.h"
#include "mock_statsCnt.h"
#include "mock_crtp_localization_service.h"
#include "mock_lighthouse_storage.h"
#include "mock_lighthouse_throttle.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

// Wire fixtures follow https://github.com/bitcraze/lighthouse-fpga#uart-protocol.
static const uint8_t syncFrame[12] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static const uint8_t pulseFrame[12] = {
  0x57, 0x34, 0x12, 0x56, 0x34, 0x01,
  0xab, 0xcd, 0x01, 0x98, 0xba, 0xdc,
};
static lighthouseUartFrame_t frame;
static lighthouseUartFrame_t queuedFrame;
static bool queueOccupied;
static UBaseType_t queueLength;
static uart1RxCallback_t registeredUart1RxCallback;
static size_t bytesFed;
static BaseType_t higherPriorityTaskWoken;

bool getUartFrameRaw(lighthouseUartFrame_t *frame);

QueueHandle_t xQueueGenericCreateStatic(const UBaseType_t length,
                                        const UBaseType_t itemSize,
                                        uint8_t *storage,
                                        StaticQueue_t *queueBuffer,
                                        const uint8_t queueType) {
  (void)storage;
  (void)queueType;
  TEST_ASSERT_EQUAL_UINT32(sizeof(lighthouseUartFrame_t), itemSize);
  TEST_ASSERT_EQUAL_UINT32(1, length);
  queueLength = length;
  queueOccupied = false;
  return (QueueHandle_t)queueBuffer;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void * const buffer,
                         TickType_t ticksToWait) {
  (void)queue;
  (void)ticksToWait;
  if (!queueOccupied) {
    return pdFALSE;
  }
  memcpy(buffer, &queuedFrame, sizeof(queuedFrame));
  queueOccupied = false;
  return pdTRUE;
}

BaseType_t xQueueGenericSendFromISR(QueueHandle_t queue,
                                    const void * const item,
                                    BaseType_t * const taskWoken,
                                    const BaseType_t copyPosition) {
  (void)queue;
  TEST_ASSERT_EQUAL_INT(queueSEND_TO_BACK, copyPosition);
  TEST_ASSERT_EQUAL_PTR(&higherPriorityTaskWoken, taskWoken);
  TEST_ASSERT_EQUAL_UINT32(1, queueLength);
  if (queueOccupied) {
    return pdFALSE;
  }
  memcpy(&queuedFrame, item, sizeof(queuedFrame));
  queueOccupied = true;
  *taskWoken = pdTRUE;
  return pdTRUE;
}

static void uart1SetRxCallbackStub(uart1RxCallback_t cb, int calls) {
  (void)calls;
  registeredUart1RxCallback = cb;
}

uint32_t xTaskGetTickCount(void) {return 0;}
void vTaskDelay(const uint32_t ticks) {(void)ticks;}

static void feedBytes(const uint8_t* bytes, size_t length) {
  for (size_t i = 0; i < length; i++) {
    registeredUart1RxCallback(bytes[i], &higherPriorityTaskWoken);
    bytesFed++;
  }
}

static void assertPulse(void) {
  TEST_ASSERT_TRUE(getUartFrameRaw(&frame));
  TEST_ASSERT_FALSE(frame.isSyncFrame);
  TEST_ASSERT_EQUAL_UINT8(3, frame.data.sensor);
  TEST_ASSERT_TRUE(frame.data.channelFound);
  // Protocol channels 1..16 are stored internally as 0..15.
  TEST_ASSERT_EQUAL_UINT8(10, frame.data.channel);
  TEST_ASSERT_TRUE(frame.data.slowBit);
  TEST_ASSERT_EQUAL_UINT32(0x1234, frame.data.width);
  TEST_ASSERT_EQUAL_UINT32(0x13456 * 4, frame.data.offset);
  TEST_ASSERT_EQUAL_UINT32(0x1cdab, frame.data.beamData);
  TEST_ASSERT_EQUAL_UINT32(0xdcba98, frame.data.timestamp);
}

static void synchronize(void) {
  feedBytes(syncFrame, sizeof(syncFrame));
  TEST_ASSERT_TRUE(getUartFrameRaw(&frame));
  TEST_ASSERT_TRUE(frame.isSyncFrame);
}

void setUp(void) {
  bytesFed = 0;
  higherPriorityTaskWoken = pdFALSE;
  memset(&frame, 0, sizeof(frame));
  registeredUart1RxCallback = NULL;
  lighthouseStorageInitializeSystemTypeFromStorage_Expect();
  lighthousePositionEstInit_Expect();
  uart1SetRxCallback_StubWithCallback(uart1SetRxCallbackStub);
  lighthouseCoreInit();
  TEST_ASSERT_NOT_NULL(registeredUart1RxCallback);
  synchronize();
  higherPriorityTaskWoken = pdFALSE;
}

void tearDown(void) {
  // Restore a known boundary even when a partial-frame assertion fails. The
  // ISR's static assembly state persists between Unity tests.
  const uint8_t zeros[12] = {0};
  size_t remainder = bytesFed % 12;
  if (remainder) {
    feedBytes(zeros, 12 - remainder);
  }
  queueOccupied = false;
  feedBytes(syncFrame, sizeof(syncFrame));
  queueOccupied = false;
  feedBytes(syncFrame, sizeof(syncFrame));
  queueOccupied = false;
}

void testThatEmptyQueueReturnsNoFrame(void) {
  TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
}

void testThatAllFieldsAreDecodedFromLittleEndianWireData(void) {
  feedBytes(pulseFrame, sizeof(pulseFrame));
  assertPulse();
  TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
}

void testThatFrameIsQueuedOnlyAfterTheTwelfthByte(void) {
  for (size_t i = 0; i < sizeof(pulseFrame) - 1; i++) {
    feedBytes(&pulseFrame[i], 1);
    TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
    TEST_ASSERT_EQUAL_INT(pdFALSE, higherPriorityTaskWoken);
  }
  feedBytes(&pulseFrame[11], 1);
  assertPulse();
  TEST_ASSERT_EQUAL_INT(pdTRUE, higherPriorityTaskWoken);
}

void testThatFramesCanBeSplitAtEveryByteBoundary(void) {
  for (size_t split = 1; split < sizeof(pulseFrame); split++) {
    feedBytes(pulseFrame, split);
    TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
    feedBytes(pulseFrame + split, sizeof(pulseFrame) - split);
    assertPulse();
  }
}

void testThatSyncFramesAreDeliveredAndDoNotContaminateFollowingPulse(void) {
  synchronize();
  synchronize();
  feedBytes(pulseFrame, sizeof(pulseFrame));
  assertPulse();
}

void testThatEveryHeaderBitCombinationIsDecodedIndependently(void) {
  uint8_t bytes[12] = {0};
  for (unsigned int header = 0; header <= 0xff; header++) {
    bytes[0] = header;
    feedBytes(bytes, sizeof(bytes));
    TEST_ASSERT_TRUE(getUartFrameRaw(&frame));
    TEST_ASSERT_FALSE(frame.isSyncFrame);
    TEST_ASSERT_EQUAL_UINT8(header % 4, frame.data.sensor);
    TEST_ASSERT_EQUAL_INT(header < 128, frame.data.channelFound);
    TEST_ASSERT_EQUAL_UINT8((header / 8) % 16, frame.data.channel);
    TEST_ASSERT_EQUAL_INT((header / 4) % 2, frame.data.slowBit);
    TEST_ASSERT_EQUAL_UINT32(0, frame.data.width);
    TEST_ASSERT_EQUAL_UINT32(0, frame.data.offset);
    TEST_ASSERT_EQUAL_UINT32(0, frame.data.beamData);
    TEST_ASSERT_EQUAL_UINT32(0, frame.data.timestamp);
  }
}

void testThatEachNonzeroPaddingBitRejectsTheFrame(void) {
  for (size_t paddingByte = 5; paddingByte <= 8; paddingByte += 3) {
    for (unsigned int bit = 1; bit < 8; bit++) {
      uint8_t bytes[12] = {0};
      bytes[paddingByte] = 1u << bit;
      feedBytes(bytes, sizeof(bytes));
      TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
      synchronize();
    }
  }
}

void testThatSeventeenthDataBitsAreAcceptedAndMaximumValuesDecode(void) {
  const uint8_t bytes[12] = {
    0x7f, 0xff, 0xff, 0xff, 0xff, 0x01,
    0xff, 0xff, 0x01, 0xff, 0xff, 0xff,
  };
  feedBytes(bytes, sizeof(bytes));
  TEST_ASSERT_TRUE(getUartFrameRaw(&frame));
  TEST_ASSERT_FALSE(frame.isSyncFrame);
  TEST_ASSERT_EQUAL_UINT32(0xffff, frame.data.width);
  TEST_ASSERT_EQUAL_UINT32(0x1ffff * 4, frame.data.offset);
  TEST_ASSERT_EQUAL_UINT32(0x1ffff, frame.data.beamData);
  TEST_ASSERT_EQUAL_UINT32(0xffffff, frame.data.timestamp);

  const uint8_t zeros[12] = {0};
  feedBytes(zeros, sizeof(zeros));
  TEST_ASSERT_TRUE(getUartFrameRaw(&frame));
  TEST_ASSERT_EQUAL_UINT32(0, frame.data.width);
  TEST_ASSERT_EQUAL_UINT32(0, frame.data.offset);
  TEST_ASSERT_EQUAL_UINT32(0, frame.data.beamData);
  TEST_ASSERT_EQUAL_UINT32(0, frame.data.timestamp);
}

void testThatQueueFullDropsNewFrameAndReceptionContinues(void) {
  const uint8_t zeros[12] = {0};
  feedBytes(pulseFrame, sizeof(pulseFrame));
  feedBytes(zeros, sizeof(zeros));
  assertPulse();
  TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
  feedBytes(pulseFrame, sizeof(pulseFrame));
  assertPulse();
}

void testThatInvalidPaddingRequiresSyncBeforeAcceptingMorePulses(void) {
  uint8_t corrupt[12] = {0};
  corrupt[5] = 0x02;
  feedBytes(corrupt, sizeof(corrupt));
  TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
  feedBytes(pulseFrame, sizeof(pulseFrame));
  TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
  synchronize();
  feedBytes(pulseFrame, sizeof(pulseFrame));
  assertPulse();
}

void testThatSyncRestoresAlignmentAtEveryPossibleByteOffset(void) {
  const uint8_t garbage[12] = {
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
  };
  for (size_t offset = 1; offset < sizeof(syncFrame); offset++) {
    feedBytes(garbage, offset);
    TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
    synchronize();
    feedBytes(pulseFrame, sizeof(pulseFrame));
    assertPulse();
  }
}
