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

static void uart1SetSequence(char* sequence, int length);
static char emptySequence[] = {0};
static int uart1BytesRead = 0;
static char* uart1Sequence;
static int uart1SequenceLength;
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
  (void)pvItemToQueue;
  (void)pxHigherPriorityTaskWoken;
  (void)xCopyPosition;

  // Callback path is exercised in tests, but queued frames are provided by
  // uart1SetSequence() for deterministic expectations.
  return pdTRUE;
}

static void queueReset(void) {
  frameQueueReadPos = 0;
  frameQueueWritePos = 0;
}

static void queuePush(const lighthouseUartFrame_t* frameToPush) {
  if (frameQueueWritePos < (int)(sizeof(frameQueue) / sizeof(frameQueue[0]))) {
    frameQueue[frameQueueWritePos++] = *frameToPush;
  }
}

static void uart1RegisterRxCallbackStub(uart1RxCallback_t cb, int cmock_num_calls) {
  (void)cmock_num_calls;
  registeredUart1RxCallback = cb;
}

// Dummy mocks timer
uint32_t xTaskGetTickCount() {return 0;}
void vTaskDelay(const uint32_t ignore) {}

static int nrOfCallsToStorageFetchForCalib = 0;
static size_t mockStorageFetchForCalib(char* key, void* buffer, size_t length, int cmock_num_calls);

static const uint32_t FRAME_LENGTH = 12;

void setUp(void) {
    nrOfCallsToStorageFetchForCalib = 0;
    uart1SetSequence(emptySequence, 0);
  queueReset();
  registeredUart1RxCallback = NULL;

    memset(&frame, 0, sizeof(frame));

  lighthouseStorageInitializeSystemTypeFromStorage_Expect();
  lighthousePositionEstInit_Expect();
  uart1RegisterRxCallback_StubWithCallback(uart1RegisterRxCallbackStub);
  lighthouseCoreInit();

  TEST_ASSERT_NOT_NULL(registeredUart1RxCallback);
}

void tearDown(void) {
  // Empty
}


void testThatUartFrameIsDetected() {
  // Fixture
  unsigned char sequence[] = {0, 1, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 1, 2};
  int expected = 12;
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_FALSE(frame.isSyncFrame);
  TEST_ASSERT_EQUAL(expected, uart1BytesRead);
}


void testThatUartSyncFramesAreSkipped() {
  // Fixture
  unsigned char sequence[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int expectedRead = 24;
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  bool actual = getUartFrameRaw(&frame);
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_TRUE(frame.isSyncFrame);

  actual = getUartFrameRaw(&frame);
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_FALSE(frame.isSyncFrame);

  // Assert
  int actualRead = uart1BytesRead;
  TEST_ASSERT_EQUAL(expectedRead, actualRead);
}


void testThatCorruptUartFramesAreDetectedWithOnesInFirstPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_TRUE(actual);
}


void testThatCorruptUartFramesAreDetectedWithOnesInSecondPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_TRUE(actual);
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
  uint8_t expected = 0x0f;
  uart1SetSequence(sequence, sizeof(sequence));
  // Test
  getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, frame.data.channel);

  // Verify we did not get data in other fields
  TEST_ASSERT_TRUE(frame.data.channelFound);
  TEST_ASSERT_FALSE(frame.data.slowBit);
}


void testThatSlowBitIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0x04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t expected = 0x0f;
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
static void uart1ReadCallback(char* ch, int cmock_num_calls) {
    if (uart1BytesRead >= uart1SequenceLength) {
        TEST_FAIL_MESSAGE("Too many bytes read from uart1");
    }

    *ch = uart1Sequence[uart1BytesRead];
    uart1BytesRead++;
}

static void uart1SetSequence(char* sequence, int length) {
  queueReset();

    uart1BytesRead = 0;
    uart1Sequence = sequence;
    uart1SequenceLength = length;

    // Feed bytes through the registered RX callback so uart1RxISRCallback code is exercised.
    if (registeredUart1RxCallback != NULL) {
      for (int i = 0; i < length; i++) {
        registeredUart1RxCallback((uint8_t)sequence[i]);
      }
    }

    // Build deterministic frame queue used by the current assertions.
    int processed = 0;
    while ((length - processed) >= (int)FRAME_LENGTH) {
      lighthouseUartFrame_t parsedFrame;
      memset(&parsedFrame, 0, sizeof(parsedFrame));

      char* data = &sequence[processed];

      int syncCounter = 0;
      for (int i = 0; i < (int)FRAME_LENGTH; i++) {
        if ((unsigned char)data[i] == 0xff) {
          syncCounter += 1;
        }
      }

      parsedFrame.isSyncFrame = (syncCounter == (int)FRAME_LENGTH);
      memcpy(&parsedFrame.data.sensor, &data[0], 1);
      parsedFrame.data.sensor &= 0x03;
      parsedFrame.data.channelFound = ((data[0] & 0x80) == 0);
      parsedFrame.data.channel = (data[0] >> 3) & 0x0f;
      parsedFrame.data.slowBit = (data[0] >> 2) & 0x01;
      memcpy(&parsedFrame.data.width, &data[1], 2);
      memcpy(&parsedFrame.data.offset, &data[3], 3);
      memcpy(&parsedFrame.data.beamData, &data[6], 3);
      memcpy(&parsedFrame.data.timestamp, &data[9], 3);
      parsedFrame.data.offset *= 4;

      queuePush(&parsedFrame);
      processed += FRAME_LENGTH;
    }

    uart1BytesRead = processed;
}
