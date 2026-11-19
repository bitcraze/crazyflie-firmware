#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "FreeRTOS.h" // @NO_MODULE
#include "syslink2_apiservice.h"
#include "syslink2_apiservice_codec.h"
#include "syslink2_apiservice_request_handling.h"
#include "syslink2_apiservice_request_queue.h"
#include "syslink2_apiservice_channel.h" // @NO_MODULE

void syslink2_apiservice_test_reset(void);

/* Fake channel. */
static bccam_uart_runtime_t runtime;
static bccam_uart_runtime_t *init_runtime;
static bool runtime_ready;
static unsigned opens;
static bool can_send;
static unsigned sends;
static syslink2_apiservice_frame_t sent;
static bool rx_full;
static syslink2_apiservice_frame_t rx;
static unsigned receives;

void syslink2_apiservice_channel_init(bccam_uart_runtime_t *r) { init_runtime = r; }
bool syslink2_apiservice_channel_runtime_ready(void) { return runtime_ready; }
void syslink2_apiservice_channel_open(void) { opens++; }
bool syslink2_apiservice_channel_can_send(void) { return can_send; }
syslink2_apiservice_send_result_t syslink2_apiservice_channel_send(
  const syslink2_apiservice_frame_t *frame) {
  sends++;
  sent = *frame;
  return SYSLINK2_APISERVICE_SEND_SENT;
}
bool syslink2_apiservice_channel_receive(syslink2_apiservice_frame_t *frame) {
  if (!rx_full) {
    return false;
  }
  receives++;
  rx_full = false;
  *frame = rx;
  return true;
}

static void deliver(const uint8_t *bytes, uint8_t len) {
  memset(&rx, 0, sizeof(rx));
  memcpy(rx.data, bytes, len);
  rx.len = len;
  rx_full = true;
}

/* Recording user. */
static unsigned completions;
static syslink2_apiservice_request_t completed_request;
static syslink2_apiservice_result_t completed_result;

static void on_complete(const syslink2_apiservice_request_t *request,
                        const syslink2_apiservice_result_t *result) {
  completions++;
  completed_request = *request;
  completed_result = *result;
}

static unsigned raw_completions;
static syslink2_apiservice_outcome_t raw_outcome;
static uint8_t raw_user_request_id;
static uint8_t raw_data[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
static uint8_t raw_len;

static void on_complete_raw(syslink2_apiservice_outcome_t outcome,
                            uint8_t user_request_id, const uint8_t *data,
                            uint8_t len) {
  raw_completions++;
  raw_outcome = outcome;
  raw_user_request_id = user_request_id;
  raw_len = len;
  if (data != NULL) {
    memcpy(raw_data, data, len);
  }
}

static syslink2_apiservice_request_t get_request(uint8_t user_id) {
  syslink2_apiservice_request_t request;
  memset(&request, 0, sizeof(request));
  request.operation = SYSLINK2_APISERVICE_KIND_GET;
  request.user_request_id = user_id;
  request.path_len = 1u;
  request.path[0] = 'q';
  return request;
}

void setUp(void) {
  syslink2_apiservice_test_reset();
  init_runtime = NULL;
  runtime_ready = false;
  opens = 0;
  can_send = true;
  sends = 0;
  rx_full = false;
  receives = 0;
  completions = 0;
  raw_completions = 0;
}

void tearDown(void) {}

void testSubmitBeforeInitIsNotInitialized(void) {
  const syslink2_apiservice_request_t request = get_request(1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED,
                    syslink2_apiservice_submit(&request, on_complete));
  const uint8_t raw[] = { 0x01, 1, 1, 'q' };
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED,
                    syslink2_apiservice_submit_raw(raw, sizeof(raw), on_complete_raw));
  /* Not initialized wins over invalid. */
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED,
                    syslink2_apiservice_submit(NULL, on_complete));
}

void testInitPassesRuntimeToChannel(void) {
  syslink2_apiservice_init(&runtime);
  TEST_ASSERT_EQUAL_PTR(&runtime, init_runtime);
  TEST_ASSERT_FALSE(syslink2_apiservice_is_ready());
}

void testInvalidAndQueueFull(void) {
  syslink2_apiservice_init(&runtime);
  syslink2_apiservice_request_t request = get_request(1);
  request.path_len = 0u;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit(&request, on_complete));
  request = get_request(1);
  for (unsigned i = 0; i < SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH; i++) {
    TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_ACCEPTED,
                      syslink2_apiservice_submit(&request, on_complete));
  }
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_QUEUE_FULL,
                    syslink2_apiservice_submit(&request, on_complete));
  /* Invalid wins over queue full. */
  request.path_len = 0u;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit(&request, on_complete));
}

void testSubmitWithoutCallbackOrRequestIsInvalid(void) {
  syslink2_apiservice_init(&runtime);
  const syslink2_apiservice_request_t request = get_request(1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit(&request, NULL));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit(NULL, on_complete));
}

void testSubmitRawDecodesAndChecks(void) {
  syslink2_apiservice_init(&runtime);
  const uint8_t undecodable[] = { 0x01, 1, 5, 'q' };
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit_raw(undecodable, sizeof(undecodable), on_complete_raw));
  const uint8_t empty_path[] = { 0x01, 1, 0 };
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit_raw(empty_path, sizeof(empty_path), on_complete_raw));
  const uint8_t good[] = { 0x01, 9, 1, 'q' };
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_INVALID,
                    syslink2_apiservice_submit_raw(good, sizeof(good), NULL));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_ACCEPTED,
                    syslink2_apiservice_submit_raw(good, sizeof(good), on_complete_raw));

  runtime_ready = true;
  syslink2_apiservice_step(0);
  syslink2_apiservice_step(1);
  const uint8_t answer[] = { 0x81, sent.data[1], 0x10, 1, 55 };
  deliver(answer, sizeof(answer));
  syslink2_apiservice_step(2);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  TEST_ASSERT_EQUAL_UINT(1, raw_completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, raw_outcome);
  TEST_ASSERT_EQUAL_UINT8(9, raw_user_request_id);
  const uint8_t expected[] = { 0x81, 9, 0x10, 1, 55 };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), raw_len);
  TEST_ASSERT_EQUAL_MEMORY(expected, raw_data, sizeof(expected));
}

void testReadyOpensChannelOnceThenSetsFlag(void) {
  syslink2_apiservice_init(&runtime);
  syslink2_apiservice_step(0);
  TEST_ASSERT_FALSE(syslink2_apiservice_is_ready());
  TEST_ASSERT_EQUAL_UINT(0, opens);
  runtime_ready = true;
  syslink2_apiservice_step(1);
  TEST_ASSERT_TRUE(syslink2_apiservice_is_ready());
  TEST_ASSERT_EQUAL_UINT(1, opens);
  syslink2_apiservice_step(2);
  TEST_ASSERT_EQUAL_UINT(1, opens);
  runtime_ready = false;
  syslink2_apiservice_step(3);
  TEST_ASSERT_FALSE(syslink2_apiservice_is_ready());
  runtime_ready = true;
  syslink2_apiservice_step(4);
  TEST_ASSERT_EQUAL_UINT(2, opens);
}

void testDoesNotReceiveWhileNotReady(void) {
  syslink2_apiservice_init(&runtime);
  const uint8_t frame[] = { 0x80, 1 };
  deliver(frame, sizeof(frame));
  syslink2_apiservice_step(0);
  TEST_ASSERT_EQUAL_UINT(0, receives);
}

void testFullRequestWithAnswer(void) {
  syslink2_apiservice_init(&runtime);
  runtime_ready = true;
  const syslink2_apiservice_request_t request = get_request(4);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_SUBMIT_ACCEPTED,
                    syslink2_apiservice_submit(&request, on_complete));
  syslink2_apiservice_step(0);
  syslink2_apiservice_step(1);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  const uint8_t expected[] = { 0x01, 1, 1, 'q' };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), sent.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, sent.data, sizeof(expected));

  const uint8_t answer[] = { 0x81, 1, 0x10, 1, 60 };
  deliver(answer, sizeof(answer));
  syslink2_apiservice_step(2);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL_UINT8(4, completed_request.user_request_id);
  TEST_ASSERT_EQUAL_UINT8(60, completed_result.value.data[0]);
}

void testUnsolicitedAndMalformedFramesAreDropped(void) {
  syslink2_apiservice_init(&runtime);
  runtime_ready = true;
  const syslink2_apiservice_request_t request = get_request(4);
  syslink2_apiservice_submit(&request, on_complete);
  syslink2_apiservice_step(0);
  syslink2_apiservice_step(1);

  const uint8_t notify[] = { 0xc0, 1, 0x80 };
  deliver(notify, sizeof(notify));
  syslink2_apiservice_step(2);
  const uint8_t malformed[] = { 0x80, 1, 0 };
  deliver(malformed, sizeof(malformed));
  syslink2_apiservice_step(3);
  TEST_ASSERT_EQUAL_UINT(2, receives);
  TEST_ASSERT_EQUAL_UINT(0, completions);

  syslink2_apiservice_step(M2T(SYSLINK2_APISERVICE_TIMEOUT_MS));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_TIMEOUT, completed_result.outcome);
}

static unsigned unsolicited_calls;
static uint8_t unsolicited_data[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
static uint8_t unsolicited_len;

static void on_unsolicited(const uint8_t *data, uint8_t len) {
  unsolicited_calls++;
  memcpy(unsolicited_data, data, len);
  unsolicited_len = len;
}

void testUnsolicitedFramesGoToTheirHandler(void) {
  unsolicited_calls = 0;
  syslink2_apiservice_init(&runtime);
  syslink2_apiservice_set_unsolicited_handler(on_unsolicited);
  runtime_ready = true;
  const syslink2_apiservice_request_t request = get_request(4);
  syslink2_apiservice_submit(&request, on_complete);
  syslink2_apiservice_step(0);
  syslink2_apiservice_step(1);

  /* A NOTIFY while a request is outstanding reaches the handler unchanged
   * and does not complete the request. */
  const uint8_t notify[] = { 0xc0, 1, 'q', 0x10, 1, 42 };
  deliver(notify, sizeof(notify));
  syslink2_apiservice_step(2);
  TEST_ASSERT_EQUAL_UINT(1, unsolicited_calls);
  TEST_ASSERT_EQUAL_UINT(sizeof(notify), unsolicited_len);
  TEST_ASSERT_EQUAL_MEMORY(notify, unsolicited_data, sizeof(notify));
  TEST_ASSERT_EQUAL_UINT(0, completions);

  /* A malformed frame does not. */
  const uint8_t malformed[] = { 0x80, 1, 0 };
  deliver(malformed, sizeof(malformed));
  syslink2_apiservice_step(3);
  TEST_ASSERT_EQUAL_UINT(1, unsolicited_calls);

  /* The request still gets its answer. */
  const uint8_t value[] = { 0x81, sent.data[1], 0x10, 1, 7 };
  deliver(value, sizeof(value));
  syslink2_apiservice_step(4);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, completed_result.outcome);
}

void testReadyLostAbortsOutstandingAndThenNotReady(void) {
  syslink2_apiservice_init(&runtime);
  runtime_ready = true;
  syslink2_apiservice_request_t request = get_request(1);
  syslink2_apiservice_submit(&request, on_complete);
  request.user_request_id = 2;
  syslink2_apiservice_submit(&request, on_complete);
  syslink2_apiservice_step(0);
  syslink2_apiservice_step(1);
  runtime_ready = false;
  syslink2_apiservice_step(2);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ABORTED, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(1, completed_request.user_request_id);
  syslink2_apiservice_step(3);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_NOT_READY, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(2, completed_request.user_request_id);
}
