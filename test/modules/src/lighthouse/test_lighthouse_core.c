// File under test lighthouse_core.c
#include "lighthouse_core.h"

#include "unity.h"
#include "mock_system.h"
#include "mock_pulse_processor.h"
#include "mock_lighthouse_deck_flasher.h"
#include "mock_lighthouse_position_est.h"
#include "mock_uart1.h"
#include "mock_statsCnt.h"

#include <stdbool.h>

static void uart1SetSequence(char* sequence, int length);
static emptySequence[] = {0};
static int uart1BytesRead = 0;
static char* uart1Sequence;
static int uart1SequenceLength;
static lighthouseUartFrame_t frame;


void setUp(void) {
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

  // Test
  bool actual = getUartFrame(&frame);

  // Assert
  int actualRead = uart1BytesRead;
  TEST_ASSERT_EQUAL(expectedRead, actualRead);
  TEST_ASSERT_TRUE(actual);
}


void testThatCorruptUartFramesAreDetectedWithOnesInFirstPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  bool actual = getUartFrame(&frame);

  // Assert
  TEST_ASSERT_FALSE(actual);
}


void testThatCorruptUartFramesAreDetectedWithOnesInSecondPadding() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0};
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  bool actual = getUartFrame(&frame);

  // Assert
  TEST_ASSERT_FALSE(actual);
}


void testThatTimeStampIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1};
  uint32_t expected = 0x010203;
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  getUartFrame(&frame);

  // Assert
  uint32_t actual = frame.timestamp;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


void testThatWidthIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t expected = 0x0201;
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  getUartFrame(&frame);

  // Assert
  uint32_t actual = frame.width;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


void testThatSensorIsDecodedInUartFrame() {
  // Fixture
  unsigned char sequence[] = {2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t expected = 0x2;
  uart1SetSequence(sequence, sizeof(sequence));

  // Test
  getUartFrame(&frame);

  // Assert
  uint32_t actual = frame.sensor;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
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
    uart1BytesRead = 0;
    uart1Sequence = sequence;
    uart1SequenceLength = length;

    uart1Getchar_StubWithCallback(uart1ReadCallback);
}
