#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_uart_frame.h"
#include "bccam_uart_rx_collector.h"
// @MODULE "bccam_uart_crc.c"

static bccam_uart_rx_collector_t collector;
static bccam_uart_rx_event_t events[4];
static uint8_t event_count;
static uint8_t pool[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
static bool pool_in_use;
static bool reject_next_raw_frame;
static bool reserve_fails;
static uint8_t release_count;
static uint8_t released_slot;

static bool capture_event(void *context, const bccam_uart_rx_event_t *event) {
  (void)context;
  if (event_count >= 4 || event == NULL) {
    return false;
  }
  events[event_count++] = *event;
  return true;
}

static bool reject_raw_frame_once(void *context, const bccam_uart_rx_event_t *event) {
  (void)context;
  if (event == NULL) {
    return false;
  }
  if (reject_next_raw_frame && event->type == BCCAM_UART_RX_EVENT_RAW_FRAME) {
    reject_next_raw_frame = false;
    return false;
  }
  return capture_event(context, event);
}

static bool reserve_slot(void *context, bccam_uart_rx_slot_t *slot) {
  (void)context;
  if (slot == NULL || reserve_fails || pool_in_use) {
    return false;
  }

  pool_in_use = true;
  slot->slot = 0;
  slot->bytes = pool;
  slot->capacity = sizeof(pool);
  return true;
}

static void release_slot(void *context, uint8_t slot) {
  (void)context;
  release_count++;
  released_slot = slot;
  if (slot == 0) {
    pool_in_use = false;
  }
}

static void feed(const uint8_t *bytes, size_t length) {
  for (size_t i = 0; i < length; i++) {
    bccam_uart_rx_collector_feed_byte(&collector, bytes[i]);
  }
}

void setUp(void) {
  memset(events, 0, sizeof(events));
  memset(pool, 0, sizeof(pool));
  event_count = 0;
  pool_in_use = false;
  reject_next_raw_frame = false;
  reserve_fails = false;
  release_count = 0;
  released_slot = BCCAM_UART_RX_SLOT_INVALID;
  bccam_uart_rx_collector_init(&collector,
                               capture_event,
                               reserve_slot,
                               release_slot,
                               NULL);
}

void tearDown(void) {
}

void testUnsynchronizedCollectorEmitsFirstValidFrame(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  const uint8_t payload[] = { 0x11, 0x22 };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, payload, sizeof(payload),
                                                frame, sizeof(frame), &frame_len));

  feed(frame, frame_len);

  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_RAW_FRAME, events[0].type);
  TEST_ASSERT_EQUAL_UINT8(0, events[0].raw_frame.slot);
  TEST_ASSERT_EQUAL_UINT16(frame_len, events[0].raw_frame.length);
  TEST_ASSERT_EQUAL_MEMORY(frame, pool, frame_len);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_COLLECTOR_SYNCHRONIZED,
                        bccam_uart_rx_collector_state(&collector));
}

void testSynchronizedCollectorQueuesCompleteRawFrames(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  const uint8_t payload[] = { 0x33 };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, payload, sizeof(payload),
                                                frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);
  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  release_slot(NULL, events[0].raw_frame.slot);
  feed(frame, frame_len);

  TEST_ASSERT_EQUAL_UINT8(2, event_count);
  for (uint8_t i = 0; i < event_count; i++) {
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_RAW_FRAME, events[i].type);
    TEST_ASSERT_EQUAL_UINT8(0, events[i].raw_frame.slot);
    TEST_ASSERT_EQUAL_UINT16(frame_len, events[i].raw_frame.length);
    TEST_ASSERT_EQUAL_MEMORY(frame, pool, frame_len);
  }
}

void testSynchronizedCollectorFaultsWhenNextByteIsNotMagic0(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, NULL, 0,
                                                frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);
  bccam_uart_rx_collector_feed_byte(&collector, 0x00);

  TEST_ASSERT_EQUAL_UINT8(2, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[1].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_SYNC_LOST, events[1].fault);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED,
                        bccam_uart_rx_collector_state(&collector));
}

void testCollectorReportsUartErrorAndReturnsUnsynchronized(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, NULL, 0,
                                                frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);
  release_slot(NULL, events[0].raw_frame.slot);
  bccam_uart_rx_collector_feed_byte(&collector, BCCAM_UART_MAGIC0);
  bccam_uart_rx_collector_report_uart_error(&collector);

  TEST_ASSERT_EQUAL_UINT8(2, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[1].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_UART_ERROR, events[1].fault);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED,
                        bccam_uart_rx_collector_state(&collector));
}

void testCollectorReportsQueueOverflowWhenRawFrameEventIsRejected(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  const uint8_t payload[] = { 0x44 };

  bccam_uart_rx_collector_init(&collector,
                               reject_raw_frame_once,
                               reserve_slot,
                               release_slot,
                               NULL);
  reject_next_raw_frame = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, payload, sizeof(payload),
                                                frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);

  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW, events[0].fault);
  TEST_ASSERT_FALSE(pool_in_use);
  TEST_ASSERT_EQUAL_UINT8(1, release_count);
  TEST_ASSERT_EQUAL_UINT8(0, released_slot);
}

void testCollectorReleasesSlotWhenRawFrameHasNoEmitter(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  const uint8_t payload[] = { 0x55 };

  bccam_uart_rx_collector_init(&collector,
                               NULL,
                               reserve_slot,
                               release_slot,
                               NULL);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, payload, sizeof(payload),
                                                frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);

  TEST_ASSERT_EQUAL_UINT8(0, event_count);
  TEST_ASSERT_FALSE(pool_in_use);
  TEST_ASSERT_EQUAL_UINT8(1, release_count);
  TEST_ASSERT_EQUAL_UINT8(0, released_slot);
}

void testCollectorReportsQueueOverflowWhenReserveFails(void) {
  reserve_fails = true;

  bccam_uart_rx_collector_feed_byte(&collector, BCCAM_UART_MAGIC0);

  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW, events[0].fault);
  TEST_ASSERT_FALSE(pool_in_use);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED,
                        bccam_uart_rx_collector_state(&collector));
}

void testUnsynchronizedSecondMagicMismatchReleasesSlot(void) {
  bccam_uart_rx_collector_feed_byte(&collector, BCCAM_UART_MAGIC0);
  TEST_ASSERT_TRUE(pool_in_use);

  bccam_uart_rx_collector_feed_byte(&collector, 0x00);

  TEST_ASSERT_FALSE(pool_in_use);
  TEST_ASSERT_EQUAL_UINT8(1, release_count);
  TEST_ASSERT_EQUAL_UINT8(0, released_slot);
  TEST_ASSERT_EQUAL_UINT8(0, event_count);
}

void testSynchronizedCollectorFaultsWhenSecondMagicByteMismatches(void) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(3, NULL, 0,
                                                frame, sizeof(frame), &frame_len));
  feed(frame, frame_len);
  release_slot(NULL, events[0].raw_frame.slot);
  bccam_uart_rx_collector_feed_byte(&collector, BCCAM_UART_MAGIC0);
  bccam_uart_rx_collector_feed_byte(&collector, BCCAM_UART_MAGIC0);

  TEST_ASSERT_EQUAL_UINT8(2, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[1].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_SYNC_LOST, events[1].fault);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED,
                        bccam_uart_rx_collector_state(&collector));
}

void testCollectorReportsBadLengthWithoutQueuingPacket(void) {
  const uint8_t bytes[] = {
    BCCAM_UART_MAGIC0,
    BCCAM_UART_MAGIC1,
    BCCAM_UART_FRAME_VERSION,
    3,
    0x01,
    0x02,
  };

  feed(bytes, sizeof(bytes));

  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_BAD_LENGTH, events[0].fault);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED,
                        bccam_uart_rx_collector_state(&collector));
}

void testBadLengthReleasesSlotAndReportsFault(void) {
  const uint8_t bytes[] = {
    BCCAM_UART_MAGIC0,
    BCCAM_UART_MAGIC1,
    BCCAM_UART_FRAME_VERSION,
    3,
    0x01,
    0x02,
  };

  feed(bytes, sizeof(bytes));

  TEST_ASSERT_FALSE(pool_in_use);
  TEST_ASSERT_EQUAL_UINT8(1, release_count);
  TEST_ASSERT_EQUAL_UINT8(1, event_count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_EVENT_FAULT, events[0].type);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_RX_FAULT_BAD_LENGTH, events[0].fault);
}
