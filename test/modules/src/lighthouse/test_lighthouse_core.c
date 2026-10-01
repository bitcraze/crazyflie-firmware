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

static void uart1SetSequence(const uint8_t* sequence, size_t length);
static lighthouseUartFrame_t frame;

extern pulseProcessor_t lighthouseCoreState;

// Functions under test
bool getUartFrameRaw(lighthouseUartFrame_t *frame);

// Minimal queue implementation used by getUartFrameRaw() in tests.
static lighthouseUartFrame_t frameQueue[8];
static int frameQueueReadPos = 0;
static int frameQueueWritePos = 0;
static uart1RxCallback_t registeredUart1RxCallback = NULL;

QueueHandle_t xQueueGenericCreateStatic(const UBaseType_t uxQueueLength,
                                        const UBaseType_t uxItemSize,
                                        uint8_t *pucQueueStorage,
                                        StaticQueue_t *pxQueueBuffer,
                                        const uint8_t ucQueueType) {
  (void)uxQueueLength;
  (void)uxItemSize;
  (void)pucQueueStorage;
  (void)pxQueueBuffer;
  (void)ucQueueType;

  return (QueueHandle_t)1;
}

BaseType_t xQueueReceive(QueueHandle_t xQueue,
                         void * const pvBuffer,
                         TickType_t xTicksToWait) {
  (void)xQueue;
  (void)xTicksToWait;

  if (frameQueueReadPos < frameQueueWritePos) {
    *((lighthouseUartFrame_t*)pvBuffer) = frameQueue[frameQueueReadPos++];
    return pdTRUE;
  }

  return pdFALSE;
}

BaseType_t xQueueGenericSendFromISR(QueueHandle_t xQueue,
                                    const void * const pvItemToQueue,
                                    BaseType_t * const pxHigherPriorityTaskWoken,
                                    const BaseType_t xCopyPosition) {
  (void)xQueue;
  (void)pxHigherPriorityTaskWoken;
  (void)xCopyPosition;

  if (frameQueueWritePos >= (int)(sizeof(frameQueue) / sizeof(frameQueue[0]))) {
    return pdFALSE;
  }

  frameQueue[frameQueueWritePos++] = *((const lighthouseUartFrame_t*)pvItemToQueue);
  return pdTRUE;
}

static void queueReset(void) {
  frameQueueReadPos = 0;
  frameQueueWritePos = 0;
}

static void uart1SetRxCallbackStub(uart1RxCallback_t cb, int cmock_num_calls) {
  (void)cmock_num_calls;
  registeredUart1RxCallback = cb;
}

// Dummy mocks timer
uint32_t xTaskGetTickCount() {return 0;}
void vTaskDelay(const uint32_t ignore) {(void)ignore;}

void setUp(void) {
  queueReset();
  registeredUart1RxCallback = NULL;

  memset(&frame, 0, sizeof(frame));

  lighthouseStorageInitializeSystemTypeFromStorage_Expect();
  lighthousePositionEstInit_Expect();
  uart1SetRxCallback_StubWithCallback(uart1SetRxCallbackStub);
  lighthouseCoreInit();

  TEST_ASSERT_NOT_NULL(registeredUart1RxCallback);
}

void tearDown(void) {
  // Empty
}


void testThatUartFrameIsDetected() {
  // Fixture
  unsigned char sequence[] = {0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_FALSE(frame.isSyncFrame);
}


void testThatUartSyncFramesAreQueued() {
  // Fixture
  unsigned char sequence[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  bool actual = getUartFrameRaw(&frame);
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_TRUE(frame.isSyncFrame);

  actual = getUartFrameRaw(&frame);
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_FALSE(frame.isSyncFrame);

  // Assert
  TEST_ASSERT_FALSE(getUartFrameRaw(&frame));
}


void testThatCorruptUartFramesAreDetectedWithOnesInFirstPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_FALSE(actual);
}


void testThatCorruptUartFramesAreDetectedWithOnesInSecondPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_FALSE(actual);
}


void testThatTimeStampIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1};
  uint32_t expected = 0x010203;
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.timestamp;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


void testThatWidthIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t expected = 0x0201;
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.width;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


void testThatOffsetIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0};

  // The offset is converted from a 6 MHz to 24 MHz clock when read
  uint32_t expected = 0x10203 * 4;
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  bool frameOk = getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.offset;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);

  // Verify the padding data was not affected
  TEST_ASSERT_TRUE(frameOk);
}


void testThatBeamDataIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0};
  uint32_t expected = 0x10203;
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  bool frameOk = getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.beamData;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);

  // Verify the padding data was not affected
  TEST_ASSERT_TRUE(frameOk);
}

void testThatSensorIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t expected = 0x3;
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.sensor;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);

  // Verify we did not get data in other fields
  TEST_ASSERT_TRUE(frame.data.channelFound);
}

void testThatLackOfChannelIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_FALSE(frame.data.channelFound);

  // Verify we did not get data in other fields
  TEST_ASSERT_EQUAL_UINT8(0, frame.data.channel);
  TEST_ASSERT_FALSE(frame.data.slowBit);
}


void testThatChannelIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0x78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(0x0f, frame.data.channel);

  // Verify we did not get data in other fields
  TEST_ASSERT_TRUE(frame.data.channelFound);
  TEST_ASSERT_FALSE(frame.data.slowBit);
}


void testThatSlowBitIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0x04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_TRUE(frame.data.slowBit);

  // Verify we did not get data in other fields
  TEST_ASSERT_TRUE(frame.data.channelFound);
  TEST_ASSERT_EQUAL_UINT8(0, frame.data.channel);
}

// Test support ----------------------------------------------------------------------------------------------------
static void uart1SetSequence(const uint8_t* sequence, size_t length) {
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  for (size_t i = 0; i < length; i++) {
    registeredUart1RxCallback(sequence[i], &higherPriorityTaskWoken);
  }
}
