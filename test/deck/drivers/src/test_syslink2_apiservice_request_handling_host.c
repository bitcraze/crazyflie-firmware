#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "FreeRTOS.h" // @NO_MODULE
#include "syslink2_apiservice_request_handling.h"
#include "syslink2_apiservice_codec.h"
#include "syslink2_apiservice_request_queue.h"
#include "syslink2_apiservice_channel.h" // @NO_MODULE

#define TIMEOUT M2T(SYSLINK2_APISERVICE_TIMEOUT_MS)

/* Fake channel. */
static bool can_send;
static syslink2_apiservice_send_result_t send_result;
static unsigned sends;
static syslink2_apiservice_frame_t sent;

bool syslink2_apiservice_channel_can_send(void) { return can_send; }
syslink2_apiservice_send_result_t syslink2_apiservice_channel_send(
  const syslink2_apiservice_frame_t *frame) {
  sends++;
  sent = *frame;
  return send_result;
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

/* Recording raw user. */
static unsigned raw_completions;
static syslink2_apiservice_outcome_t raw_outcome;
static uint8_t raw_user_request_id;
static bool raw_data_null;
static uint8_t raw_data[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
static uint8_t raw_len;

static void on_complete_raw(syslink2_apiservice_outcome_t outcome,
                            uint8_t user_request_id, const uint8_t *data,
                            uint8_t len) {
  raw_completions++;
  raw_outcome = outcome;
  raw_user_request_id = user_request_id;
  raw_data_null = data == NULL;
  raw_len = len;
  if (data != NULL) {
    memcpy(raw_data, data, len);
  }
}

static void put(uint8_t operation, uint8_t user_id, bool raw) {
  syslink2_apiservice_queue_entry_t entry;
  memset(&entry, 0, sizeof(entry));
  entry.request.operation = operation;
  entry.request.user_request_id = user_id;
  entry.request.path_len = 3u;
  memcpy(entry.request.path, "a.b", 3);
  entry.request.value.type = SYSLINK2_APISERVICE_TYPE_U8;
  entry.request.value.len = 1u;
  entry.request.value.data[0] = 42u;
  if (raw) {
    entry.on_complete_raw = on_complete_raw;
  } else {
    entry.on_complete = on_complete;
  }
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_put(&entry));
}

static void submit(uint8_t operation, uint8_t user_id) {
  put(operation, user_id, false);
}

static void submit_raw(uint8_t operation, uint8_t user_id) {
  put(operation, user_id, true);
}

static syslink2_apiservice_response_t response(uint8_t kind, uint8_t request_id) {
  syslink2_apiservice_response_t r;
  memset(&r, 0, sizeof(r));
  r.kind = kind;
  r.request_id = request_id;
  if (kind == SYSLINK2_APISERVICE_KIND_VALUE) {
    r.value_valid = true;
    r.value.type = SYSLINK2_APISERVICE_TYPE_U8;
    r.value.len = 1u;
    r.value.data[0] = 77u;
  }
  if (kind == SYSLINK2_APISERVICE_KIND_ERROR) {
    r.error_code = 3u;
  }
  return r;
}

static void step(bool ready, const syslink2_apiservice_response_t *r, uint32_t now) {
  syslink2_apiservice_request_handling_step(ready, r, now);
}

/* Takes and sends one request; returns its deck request ID. */
static uint8_t take_and_send(uint8_t operation, uint8_t user_id, uint32_t now) {
  submit(operation, user_id);
  step(true, NULL, now);
  TEST_ASSERT_EQUAL_UINT(0, sends);
  step(true, NULL, now);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  sends = 0;
  return sent.data[1];
}

void setUp(void) {
  can_send = true;
  send_result = SYSLINK2_APISERVICE_SEND_SENT;
  sends = 0;
  completions = 0;
  memset(&completed_request, 0, sizeof(completed_request));
  memset(&completed_result, 0, sizeof(completed_result));
  raw_completions = 0;
  raw_data_null = false;
  raw_len = 0xffu;
  memset(raw_data, 0, sizeof(raw_data));
  syslink2_apiservice_request_queue_init();
  syslink2_apiservice_request_handling_init();
}

void tearDown(void) {}

void testGetAnsweredWithValue(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 7, 0);
  TEST_ASSERT_EQUAL_UINT8(1, id);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_KIND_GET, sent.data[0]);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(77, completed_result.value.data[0]);
  TEST_ASSERT_EQUAL_UINT8(7, completed_request.user_request_id);
  TEST_ASSERT_EQUAL_MEMORY("a.b", completed_request.path, 3);
}

void testSetAnsweredWithOk(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_SET, 8, 0);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_KIND_SET, sent.data[0]);
  TEST_ASSERT_EQUAL_UINT8(42, sent.data[sent.len - 1]);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_OK, completed_result.outcome);
}

void testCmdWithoutArgumentAnsweredWithOk(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_CMD, 8, 0);
  const uint8_t expected[] = { 0x03, id, 3, 'a', '.', 'b' };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), sent.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, sent.data, sizeof(expected));
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_OK, completed_result.outcome);
}

void testCmdWithArgumentAnsweredWithValue(void) {
  syslink2_apiservice_queue_entry_t entry;
  memset(&entry, 0, sizeof(entry));
  entry.request.operation = SYSLINK2_APISERVICE_KIND_CMD;
  entry.request.user_request_id = 8u;
  entry.request.path_len = 1u;
  entry.request.path[0] = 'q';
  entry.request.has_argument = true;
  entry.request.value.type = SYSLINK2_APISERVICE_TYPE_U8;
  entry.request.value.len = 1u;
  entry.request.value.data[0] = 42u;
  entry.on_complete = on_complete;
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_put(&entry));
  step(true, NULL, 0);
  step(true, NULL, 0);
  const uint8_t expected[] = { 0x03, sent.data[1], 1, 'q', 0x10, 1, 42 };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), sent.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, sent.data, sizeof(expected));
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, sent.data[1]);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(77, completed_result.value.data[0]);
}

void testCmdAnsweredWithError(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_CMD, 8, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_ERROR, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ERROR, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(3, completed_result.error_code);
}

void testCmdWithInvalidValueIsProtocolError(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_CMD, 8, 0);
  syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  r.value_valid = false;
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);
}

void testErrorAnswerGivesErrorCode(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_SET, 8, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_ERROR, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ERROR, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(3, completed_result.error_code);
}

void testKindThatDoesNotFitIsProtocolError(void) {
  uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 1, 0);
  syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);

  id = take_and_send(SYSLINK2_APISERVICE_KIND_SET, 2, 2);
  r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  step(true, &r, 3);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT(2, completions);
}

void testInvalidValueIsProtocolError(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 1, 0);
  syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  r.value_valid = false;
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);
}

void testMismatchedRequestIdIsDroppedAndRequestTimesOut(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 1, 0);
  const syslink2_apiservice_response_t stale = response(SYSLINK2_APISERVICE_KIND_VALUE, (uint8_t)(id + 1u));
  step(true, &stale, 1);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, TIMEOUT - 1u);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_TIMEOUT, completed_result.outcome);
}

void testAnswerAtDeadlineCompletesRequest(void) {
  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 1, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  step(true, &r, TIMEOUT);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, completed_result.outcome);
}

void testTimeoutWhileWaitingToSend(void) {
  can_send = false;
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, 100);
  step(true, NULL, 100 + TIMEOUT - 1u);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, 100 + TIMEOUT);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_TIMEOUT, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT(0, sends);
}

void testTimeoutAcrossTickWrap(void) {
  const uint32_t start = 0xffffffffu - 10u;
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, start);
  step(true, NULL, start);
  step(true, NULL, start + 20u);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, start + TIMEOUT);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_TIMEOUT, completed_result.outcome);
}

void testBusyRetriesInLaterStep(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, 0);
  send_result = SYSLINK2_APISERVICE_SEND_BUSY;
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  send_result = SYSLINK2_APISERVICE_SEND_SENT;
  step(true, NULL, 2);
  TEST_ASSERT_EQUAL_UINT(2, sends);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, sent.data[1]);
  step(true, &r, 3);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, completed_result.outcome);
}

void testClosedSendAborts(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, 0);
  send_result = SYSLINK2_APISERVICE_SEND_CLOSED;
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ABORTED, completed_result.outcome);
}

void testRejectedSendFails(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, 0);
  send_result = SYSLINK2_APISERVICE_SEND_REJECTED;
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_SEND_FAILED, completed_result.outcome);
}

void testReadyLostAbortsInBothStates(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, 0);
  step(false, NULL, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ABORTED, completed_result.outcome);

  const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 2, 2);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  step(false, &r, 3);
  TEST_ASSERT_EQUAL_UINT(2, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ABORTED, completed_result.outcome);
}

void testRequestTakenWhileNotReadyCompletesNotReadyOnePerStep(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  submit(SYSLINK2_APISERVICE_KIND_GET, 2);
  step(false, NULL, 0);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_NOT_READY, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(1, completed_request.user_request_id);
  step(false, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(2, completions);
  TEST_ASSERT_EQUAL_UINT8(2, completed_request.user_request_id);
  TEST_ASSERT_EQUAL_UINT(0, sends);
}

void testStaleResponseInIdleIsDropped(void) {
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, 1);
  step(true, &r, 0);
  TEST_ASSERT_EQUAL_UINT(0, completions);
}

void testStaleResponseWhileWaitingToSendIsDropped(void) {
  can_send = false;
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  step(true, NULL, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, 1);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  can_send = true;
  step(true, NULL, 2);
  TEST_ASSERT_EQUAL_UINT(1, sends);
}

void testRequestIdWrapsSkippingZero(void) {
  for (unsigned i = 1; i <= 256; i++) {
    const uint8_t id = take_and_send(SYSLINK2_APISERVICE_KIND_GET, 0, i * 10u);
    TEST_ASSERT_NOT_EQUAL(0, id);
    TEST_ASSERT_EQUAL_UINT8(i <= 255 ? i : 1, id);
    const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
    step(true, &r, i * 10u + 1u);
  }
  TEST_ASSERT_EQUAL_UINT(256, completions);
}

void testNextRequestTakenOnlyAfterPreviousCompleted(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  submit(SYSLINK2_APISERVICE_KIND_GET, 2);
  step(true, NULL, 0);
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  const uint8_t first_id = sent.data[1];
  step(true, NULL, 2);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, first_id);
  step(true, &r, 3);
  TEST_ASSERT_EQUAL_UINT8(1, completed_request.user_request_id);
  step(true, NULL, 4);
  step(true, NULL, 5);
  TEST_ASSERT_EQUAL_UINT(2, sends);
  TEST_ASSERT_EQUAL_UINT8(first_id + 1u, sent.data[1]);
}

/* Raw requests */

/* Takes and sends one raw request; returns its deck request ID. */
static uint8_t take_and_send_raw(uint8_t operation, uint8_t user_id, uint32_t now) {
  submit_raw(operation, user_id);
  step(true, NULL, now);
  step(true, NULL, now);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  sends = 0;
  return sent.data[1];
}

void testRawGetAnsweredWithValueGivesRawResponse(void) {
  const uint8_t id = take_and_send_raw(SYSLINK2_APISERVICE_KIND_GET, 7, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_VALUE, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL_UINT(1, raw_completions);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_VALUE, raw_outcome);
  TEST_ASSERT_EQUAL_UINT8(7, raw_user_request_id);
  const uint8_t expected[] = { 0x81, 7, SYSLINK2_APISERVICE_TYPE_U8, 1, 77 };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), raw_len);
  TEST_ASSERT_EQUAL_MEMORY(expected, raw_data, sizeof(expected));
}

void testRawSetAnsweredWithOkGivesRawResponse(void) {
  const uint8_t id = take_and_send_raw(SYSLINK2_APISERVICE_KIND_SET, 8, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_OK, raw_outcome);
  const uint8_t expected[] = { 0x80, 8 };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), raw_len);
  TEST_ASSERT_EQUAL_MEMORY(expected, raw_data, sizeof(expected));
}

void testRawErrorAnswerKeepsDetail(void) {
  const uint8_t id = take_and_send_raw(SYSLINK2_APISERVICE_KIND_SET, 8, 0);
  syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_ERROR, id);
  r.detail_len = 2u;
  memcpy(r.detail, "no", 2);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_ERROR, raw_outcome);
  const uint8_t expected[] = { 0x82, 8, 3, 2, 'n', 'o' };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), raw_len);
  TEST_ASSERT_EQUAL_MEMORY(expected, raw_data, sizeof(expected));
}

void testRawTimeoutHasNoData(void) {
  take_and_send_raw(SYSLINK2_APISERVICE_KIND_GET, 5, 0);
  step(true, NULL, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT(1, raw_completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_TIMEOUT, raw_outcome);
  TEST_ASSERT_EQUAL_UINT8(5, raw_user_request_id);
  TEST_ASSERT_TRUE(raw_data_null);
  TEST_ASSERT_EQUAL_UINT8(0, raw_len);
}

void testRawProtocolErrorHasNoData(void) {
  const uint8_t id = take_and_send_raw(SYSLINK2_APISERVICE_KIND_GET, 5, 0);
  const syslink2_apiservice_response_t r = response(SYSLINK2_APISERVICE_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR, raw_outcome);
  TEST_ASSERT_TRUE(raw_data_null);
  TEST_ASSERT_EQUAL_UINT8(0, raw_len);
}

void testRawNotReadyHasNoData(void) {
  submit_raw(SYSLINK2_APISERVICE_KIND_GET, 6);
  step(false, NULL, 0);
  TEST_ASSERT_EQUAL_UINT(1, raw_completions);
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_OUTCOME_NOT_READY, raw_outcome);
  TEST_ASSERT_EQUAL_UINT8(6, raw_user_request_id);
  TEST_ASSERT_TRUE(raw_data_null);
}

void testRawAndStructuredRequestsGetTheirOwnCallback(void) {
  submit(SYSLINK2_APISERVICE_KIND_GET, 1);
  submit_raw(SYSLINK2_APISERVICE_KIND_GET, 2);
  step(false, NULL, 0);
  step(false, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL_UINT8(1, completed_request.user_request_id);
  TEST_ASSERT_EQUAL_UINT(1, raw_completions);
  TEST_ASSERT_EQUAL_UINT8(2, raw_user_request_id);
}
