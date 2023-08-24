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

static void uart1SetSequence(char* sequence, int length);
static emptySequence[] = {0};
static int uart1BytesRead = 0;
static char* uart1Sequence;
static int uart1SequenceLength;
static lighthouseUartFrame_t frame;

extern pulseProcessor_t lighthouseCoreState;

// Functions under test
void waitForUartSynchFrame();
bool getUartFrameRaw(lighthouseUartFrame_t *frame);
lighthouseBaseStationType_t identifyBaseStationType(const lighthouseUartFrame_t* frame, lighthouseBsIdentificationData_t* state);

// Dummy mocks timer
uint32_t xTaskGetTickCount() {return 0;}
void vTaskDelay(const uint32_t ignore) {}

static int nrOfCallsToStorageFetchForCalib = 0;
static size_t mockStorageFetchForCalib(char* key, void* buffer, size_t length, int cmock_num_calls);

static const uint32_t FRAME_LENGTH = 12;

void setUp(void) {
    nrOfCallsToStorageFetchForCalib = 0;
    uart1SetSequence(emptySequence, 0);

    memset(&frame, 0, sizeof(frame));
}

void tearDown(void) {
  // Empty
}


void testThatUartFrameIsDetected() {
  // Fixture
  unsigned char sequence[] = {0, 1, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 1, 2};
  int expected = 15;
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  waitForUartSynchFrame();

  // Assert
  int actual = uart1BytesRead;
  TEST_ASSERT_EQUAL(expected, actual);
}


void testThatUartSyncFramesAreSkipped() {
  // Fixture
  unsigned char sequence[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int expectedRead = 24;
  uart1SetSequence(sequence, sizeof(sequence));
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

  // Test
  do {
    bool actual = getUartFrameRaw(&frame);
    TEST_ASSERT_TRUE(actual);
  } while(frame.isSyncFrame);

  // Assert
  int actualRead = uart1BytesRead;
  TEST_ASSERT_EQUAL(expectedRead, actualRead);
}


void testThatCorruptUartFramesAreDetectedWithOnesInFirstPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

  // Test
  bool actual = getUartFrameRaw(&frame);

  // Assert
  TEST_ASSERT_FALSE(actual);
}


void testThatCorruptUartFramesAreDetectedWithOnesInSecondPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

  // Test
  bool frameOk = getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.offset;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);

  // Verify the padding data was not affected
  TEST_ASSERT_TRUE(frameOk)
}


void testThatBeamDataIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 3, 2, 1, 0, 0, 0};
  uint32_t expected = 0x10203;
  uart1SetSequence(sequence, sizeof(sequence));
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

  // Test
  bool frameOk = getUartFrameRaw(&frame);

  // Assert
  uint32_t actual = frame.data.beamData;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);

  // Verify the padding data was not affected
  TEST_ASSERT_TRUE(frameOk)
}

void testThatSensorIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t expected = 0x3;
  uart1SetSequence(sequence, sizeof(sequence));
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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
  uart1bytesAvailable_ExpectAndReturn(FRAME_LENGTH);

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

static bool uart1GetcharCallback(char* ch, int cmock_num_calls) {
    uart1ReadCallback(ch, cmock_num_calls);
    return true;
}

static void uart1SetSequence(char* sequence, int length) {
    uart1BytesRead = 0;
    uart1Sequence = sequence;
    uart1SequenceLength = length;

    uart1Getchar_StubWithCallback(uart1ReadCallback);
    uart1Getchar_StubWithCallback(uart1GetcharCallback);
}
