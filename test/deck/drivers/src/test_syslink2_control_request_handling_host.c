#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "FreeRTOS.h" // @NO_MODULE
#include "syslink2_control_request_handling.h"
#include "syslink2_control_codec.h"
#include "syslink2_control_request_queue.h"
#include "syslink2_control_channel.h" // @NO_MODULE

#define TIMEOUT M2T(SYSLINK2_CONTROL_TIMEOUT_MS)

/* Fake channel. */
static bool can_send;
static syslink2_control_send_result_t send_result;
static unsigned sends;
static syslink2_control_frame_t sent;

bool syslink2_control_channel_can_send(void) { return can_send; }
syslink2_control_send_result_t syslink2_control_channel_send(
  const syslink2_control_frame_t *frame) {
  sends++;
  sent = *frame;
  return send_result;
}

/* Recording user. */
static unsigned completions;
static syslink2_control_request_t completed_request;
static syslink2_control_result_t completed_result;

static void on_complete(const syslink2_control_request_t *request,
                        const syslink2_control_result_t *result) {
  completions++;
  completed_request = *request;
  completed_result = *result;
}

static const syslink2_control_user_t user = { .on_complete = on_complete };

static void submit(uint8_t operation, uint8_t user_id) {
  syslink2_control_request_t request;
  memset(&request, 0, sizeof(request));
  request.operation = operation;
  request.user_request_id = user_id;
  request.path_len = 3u;
  memcpy(request.path, "a.b", 3);
  request.value.type = SYSLINK2_CONTROL_TYPE_U8;
  request.value.len = 1u;
  request.value.data[0] = 42u;
  request.user = &user;
  TEST_ASSERT_TRUE(syslink2_control_request_queue_put(&request));
}

static syslink2_control_response_t response(uint8_t kind, uint8_t request_id) {
  syslink2_control_response_t r;
  memset(&r, 0, sizeof(r));
  r.kind = kind;
  r.request_id = request_id;
  if (kind == SYSLINK2_CONTROL_KIND_VALUE) {
    r.value_valid = true;
    r.value.type = SYSLINK2_CONTROL_TYPE_U8;
    r.value.len = 1u;
    r.value.data[0] = 77u;
  }
  if (kind == SYSLINK2_CONTROL_KIND_ERROR) {
    r.error_code = 3u;
  }
  return r;
}

static void step(bool ready, const syslink2_control_response_t *r, uint32_t now) {
  syslink2_control_request_handling_step(ready, r, now);
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
  send_result = SYSLINK2_CONTROL_SEND_SENT;
  sends = 0;
  completions = 0;
  memset(&completed_request, 0, sizeof(completed_request));
  memset(&completed_result, 0, sizeof(completed_result));
  syslink2_control_request_queue_init();
  syslink2_control_request_handling_init();
}

void tearDown(void) {}

void testGetAnsweredWithValue(void) {
  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 7, 0);
  TEST_ASSERT_EQUAL_UINT8(1, id);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_CONTROL_KIND_GET, sent.data[0]);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_VALUE, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(77, completed_result.value.data[0]);
  TEST_ASSERT_EQUAL_UINT8(7, completed_request.user_request_id);
  TEST_ASSERT_EQUAL_MEMORY("a.b", completed_request.path, 3);
}

void testSetAnsweredWithOk(void) {
  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_SET, 8, 0);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_CONTROL_KIND_SET, sent.data[0]);
  TEST_ASSERT_EQUAL_UINT8(42, sent.data[sent.len - 1]);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_OK, completed_result.outcome);
}

void testErrorAnswerGivesErrorCode(void) {
  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_SET, 8, 0);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_ERROR, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_ERROR, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(3, completed_result.error_code);
}

void testKindThatDoesNotFitIsProtocolError(void) {
  uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 1, 0);
  syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_OK, id);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);

  id = take_and_send(SYSLINK2_CONTROL_KIND_SET, 2, 2);
  r = response(SYSLINK2_CONTROL_KIND_VALUE, id);
  step(true, &r, 3);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT(2, completions);
}

void testInvalidValueIsProtocolError(void) {
  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 1, 0);
  syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, id);
  r.value_valid = false;
  step(true, &r, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_PROTOCOL_ERROR, completed_result.outcome);
}

void testMismatchedRequestIdIsDroppedAndRequestTimesOut(void) {
  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 1, 0);
  const syslink2_control_response_t stale = response(SYSLINK2_CONTROL_KIND_VALUE, (uint8_t)(id + 1u));
  step(true, &stale, 1);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, TIMEOUT - 1u);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_TIMEOUT, completed_result.outcome);
}

void testAnswerAtDeadlineCompletesRequest(void) {
  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 1, 0);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, id);
  step(true, &r, TIMEOUT);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_VALUE, completed_result.outcome);
}

void testTimeoutWhileWaitingToSend(void) {
  can_send = false;
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, 100);
  step(true, NULL, 100 + TIMEOUT - 1u);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, 100 + TIMEOUT);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_TIMEOUT, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT(0, sends);
}

void testTimeoutAcrossTickWrap(void) {
  const uint32_t start = 0xffffffffu - 10u;
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, start);
  step(true, NULL, start);
  step(true, NULL, start + 20u);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  step(true, NULL, start + TIMEOUT);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_TIMEOUT, completed_result.outcome);
}

void testBusyRetriesInLaterStep(void) {
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, 0);
  send_result = SYSLINK2_CONTROL_SEND_BUSY;
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  send_result = SYSLINK2_CONTROL_SEND_SENT;
  step(true, NULL, 2);
  TEST_ASSERT_EQUAL_UINT(2, sends);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, sent.data[1]);
  step(true, &r, 3);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_VALUE, completed_result.outcome);
}

void testClosedSendAborts(void) {
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, 0);
  send_result = SYSLINK2_CONTROL_SEND_CLOSED;
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_ABORTED, completed_result.outcome);
}

void testRejectedSendFails(void) {
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, 0);
  send_result = SYSLINK2_CONTROL_SEND_REJECTED;
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_SEND_FAILED, completed_result.outcome);
}

void testReadyLostAbortsInBothStates(void) {
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, 0);
  step(false, NULL, 1);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_ABORTED, completed_result.outcome);

  const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 2, 2);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, id);
  step(false, &r, 3);
  TEST_ASSERT_EQUAL_UINT(2, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_ABORTED, completed_result.outcome);
}

void testRequestTakenWhileNotReadyCompletesNotReadyOnePerStep(void) {
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  submit(SYSLINK2_CONTROL_KIND_GET, 2);
  step(false, NULL, 0);
  TEST_ASSERT_EQUAL_UINT(1, completions);
  TEST_ASSERT_EQUAL(SYSLINK2_CONTROL_OUTCOME_NOT_READY, completed_result.outcome);
  TEST_ASSERT_EQUAL_UINT8(1, completed_request.user_request_id);
  step(false, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(2, completions);
  TEST_ASSERT_EQUAL_UINT8(2, completed_request.user_request_id);
  TEST_ASSERT_EQUAL_UINT(0, sends);
}

void testStaleResponseInIdleIsDropped(void) {
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, 1);
  step(true, &r, 0);
  TEST_ASSERT_EQUAL_UINT(0, completions);
}

void testStaleResponseWhileWaitingToSendIsDropped(void) {
  can_send = false;
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  step(true, NULL, 0);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, 1);
  step(true, &r, 1);
  TEST_ASSERT_EQUAL_UINT(0, completions);
  can_send = true;
  step(true, NULL, 2);
  TEST_ASSERT_EQUAL_UINT(1, sends);
}

void testRequestIdWrapsSkippingZero(void) {
  for (unsigned i = 1; i <= 256; i++) {
    const uint8_t id = take_and_send(SYSLINK2_CONTROL_KIND_GET, 0, i * 10u);
    TEST_ASSERT_NOT_EQUAL(0, id);
    TEST_ASSERT_EQUAL_UINT8(i <= 255 ? i : 1, id);
    const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, id);
    step(true, &r, i * 10u + 1u);
  }
  TEST_ASSERT_EQUAL_UINT(256, completions);
}

void testNextRequestTakenOnlyAfterPreviousCompleted(void) {
  submit(SYSLINK2_CONTROL_KIND_GET, 1);
  submit(SYSLINK2_CONTROL_KIND_GET, 2);
  step(true, NULL, 0);
  step(true, NULL, 1);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  const uint8_t first_id = sent.data[1];
  step(true, NULL, 2);
  TEST_ASSERT_EQUAL_UINT(1, sends);
  const syslink2_control_response_t r = response(SYSLINK2_CONTROL_KIND_VALUE, first_id);
  step(true, &r, 3);
  TEST_ASSERT_EQUAL_UINT8(1, completed_request.user_request_id);
  step(true, NULL, 4);
  step(true, NULL, 5);
  TEST_ASSERT_EQUAL_UINT(2, sends);
  TEST_ASSERT_EQUAL_UINT8(first_id + 1u, sent.data[1]);
}
