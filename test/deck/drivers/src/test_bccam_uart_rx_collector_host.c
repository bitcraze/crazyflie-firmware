#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_uart_frame.h"
#include "bccam_uart_rx_collector.h"
// @MODULE "bccam_uart_crc.c"

static bccam_uart_rx_collector_t collector;
static bccam_uart_rx_event_t events[4];
static uint8_t event_count;

static bool capture(void *context, const bccam_uart_rx_event_t *event) {
  (void)context;
  if (event_count >= 4 || event == NULL) {
    return false;
  }
  events[event_count++] = *event;
  return true;
}

static bool reject_raw_capture_fault(void *context,
                                     const bccam_uart_rx_event_t *event) {
  (void)context;
  if (event == NULL) {
    return false;
  }
  if (event->type == BCCAM_UART_RX_EVENT_RAW_FRAME) {
    return false;
  }
  return capture(NULL, event);
}

static void feed(const uint8_t *bytes, size_t length) {
  for (size_t i = 0; i < length; i++) {
    bccam_uart_rx_collector_feed_byte(&collector, bytes[i]);
  }
}

void setUp(void) {
  memset(events, 0, sizeof(events));
  event_count = 0;
  bccam_uart_rx_collector_init(&collector, capture, NULL);
}

void tearDown(void) {}

void testCollectorDeliversFirstCompletePacketWithoutDroppingItForSynchronization(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len;
  const uint8_t payload[] = { 1, 0x22, 1, 1, 64, 0 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0, payload, sizeof(payload),
                                    frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);
  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_RAW_FRAME, events[0].type);
  TEST_ASSERT_EQUAL_UINT16(frame_len, events[0].raw_frame.length);
  TEST_ASSERT_EQUAL_MEMORY(frame, events[0].raw_frame.bytes, frame_len);
}

void testCollectorScansPastGarbageAndPreservesPacketOrder(void) {
  uint8_t first[32], second[32];
  size_t first_len, second_len;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0, (const uint8_t *)"a", 1,
                                    first, sizeof(first), &first_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0, (const uint8_t *)"b", 1,
                                    second, sizeof(second), &second_len));
  const uint8_t garbage[] = { 0x00, 0xBC, 0x11, 0x42 };
  feed(garbage, sizeof(garbage));
  feed(first, first_len);
  feed(second, second_len);
  TEST_ASSERT_EQUAL_UINT8(2, event_count);
  TEST_ASSERT_EQUAL_MEMORY(first, events[0].raw_frame.bytes, first_len);
  TEST_ASSERT_EQUAL_MEMORY(second, events[1].raw_frame.bytes, second_len);
}

void testCollectorReportsPhysicalUartError(void) {
  bccam_uart_rx_collector_report_uart_error(&collector);
  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_UART_ERROR, events[0].fault);
}

void testCollectorRejectsLengthBeyondImplementationLimit(void) {
  const uint8_t header[] = { 0xBC, 0xCD, 1, 0, 1, 1 };
  feed(header, sizeof(header));
  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_BAD_LENGTH, events[0].fault);
}

void testRejectedQueueEventReportsOverflow(void) {
  bccam_uart_rx_collector_init(&collector, reject_raw_capture_fault, NULL);
  uint8_t frame[16];
  size_t frame_len;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0, NULL, 0,
                                    frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);
  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW, events[0].fault);
}
