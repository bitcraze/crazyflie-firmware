#include "syslink2_control_request_handling.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "syslink2_control_channel.h"
#include "syslink2_control_codec.h"
#include "syslink2_control_request_queue.h"

typedef enum {
  STATE_IDLE = 0,
  STATE_WAITING_TO_SEND,
  STATE_WAITING_RESPONSE,
} state_t;

static state_t state;
static syslink2_control_request_t outstanding;
static uint8_t next_request_id;
static uint8_t request_id;
static uint32_t deadline;

void syslink2_control_request_handling_init(void) {
  state = STATE_IDLE;
  memset(&outstanding, 0, sizeof(outstanding));
  next_request_id = 1u;
  request_id = 0u;
  deadline = 0u;
}

static uint8_t assign_request_id(void) {
  const uint8_t id = next_request_id;
  next_request_id = next_request_id == 255u ? 1u :
    (uint8_t)(next_request_id + 1u);
  return id;
}

static bool timer_expired(uint32_t now_ticks) {
  return (int32_t)(now_ticks - deadline) >= 0;
}

static void complete(const syslink2_control_request_t *request,
                     syslink2_control_outcome_t outcome,
                     const syslink2_control_response_t *response) {
  syslink2_control_result_t result;
  memset(&result, 0, sizeof(result));
  result.outcome = outcome;
  if (response != NULL) {
    if (outcome == SYSLINK2_CONTROL_OUTCOME_VALUE) {
      result.value = response->value;
    } else if (outcome == SYSLINK2_CONTROL_OUTCOME_ERROR) {
      result.error_code = response->error_code;
    }
  }
  request->user->on_complete(request, &result);
}

static void complete_outstanding(syslink2_control_outcome_t outcome,
                                 const syslink2_control_response_t *response) {
  state = STATE_IDLE;
  complete(&outstanding, outcome, response);
}

static void step_idle(bool ready, uint32_t now_ticks) {
  syslink2_control_request_t request;
  if (!syslink2_control_request_queue_take(&request)) {
    return;
  }
  if (!ready) {
    complete(&request, SYSLINK2_CONTROL_OUTCOME_NOT_READY, NULL);
    return;
  }
  outstanding = request;
  request_id = assign_request_id();
  deadline = now_ticks + M2T(SYSLINK2_CONTROL_TIMEOUT_MS);
  state = STATE_WAITING_TO_SEND;
}

static void step_waiting_to_send(bool ready, uint32_t now_ticks) {
  if (!ready) {
    complete_outstanding(SYSLINK2_CONTROL_OUTCOME_ABORTED, NULL);
    return;
  }
  if (timer_expired(now_ticks)) {
    complete_outstanding(SYSLINK2_CONTROL_OUTCOME_TIMEOUT, NULL);
    return;
  }
  if (!syslink2_control_channel_can_send()) {
    return;
  }
  const syslink2_control_frame_t frame =
    outstanding.operation == SYSLINK2_CONTROL_KIND_SET ?
      syslink2_control_codec_encode_set(request_id, outstanding.path,
                                        outstanding.path_len,
                                        &outstanding.value) :
      syslink2_control_codec_encode_get(request_id, outstanding.path,
                                        outstanding.path_len);
  switch (syslink2_control_channel_send(&frame)) {
    case SYSLINK2_CONTROL_SEND_SENT:
      state = STATE_WAITING_RESPONSE;
      break;
    case SYSLINK2_CONTROL_SEND_BUSY:
      break;
    case SYSLINK2_CONTROL_SEND_CLOSED:
      complete_outstanding(SYSLINK2_CONTROL_OUTCOME_ABORTED, NULL);
      break;
    case SYSLINK2_CONTROL_SEND_REJECTED:
    default:
      complete_outstanding(SYSLINK2_CONTROL_OUTCOME_SEND_FAILED, NULL);
      break;
  }
}

/** True when the kind of the answer fits the operation of the request. */
static bool answer_fits(const syslink2_control_response_t *response) {
  if (response->kind == SYSLINK2_CONTROL_KIND_ERROR) {
    return true;
  }
  if (outstanding.operation == SYSLINK2_CONTROL_KIND_GET) {
    return response->kind == SYSLINK2_CONTROL_KIND_VALUE &&
           response->value_valid;
  }
  return response->kind == SYSLINK2_CONTROL_KIND_OK;
}

static syslink2_control_outcome_t outcome_for(
  const syslink2_control_response_t *response) {
  switch (response->kind) {
    case SYSLINK2_CONTROL_KIND_OK:
      return SYSLINK2_CONTROL_OUTCOME_OK;
    case SYSLINK2_CONTROL_KIND_VALUE:
      return SYSLINK2_CONTROL_OUTCOME_VALUE;
    default:
      return SYSLINK2_CONTROL_OUTCOME_ERROR;
  }
}

static void step_waiting_response(bool ready,
                                  const syslink2_control_response_t *response,
                                  uint32_t now_ticks) {
  if (!ready) {
    /* A response in this step is stale: the session has ended. */
    complete_outstanding(SYSLINK2_CONTROL_OUTCOME_ABORTED, NULL);
    return;
  }
  if (response != NULL && response->request_id == request_id) {
    if (answer_fits(response)) {
      complete_outstanding(outcome_for(response), response);
    } else {
      complete_outstanding(SYSLINK2_CONTROL_OUTCOME_PROTOCOL_ERROR, NULL);
    }
    return;
  }
  /* No response, or a stale one with another request_id. */
  if (timer_expired(now_ticks)) {
    complete_outstanding(SYSLINK2_CONTROL_OUTCOME_TIMEOUT, NULL);
  }
}

void syslink2_control_request_handling_step(
  bool ready, const syslink2_control_response_t *response, uint32_t now_ticks) {
  switch (state) {
    case STATE_IDLE:
      /* A response is stale here and dropped. */
      step_idle(ready, now_ticks);
      break;
    case STATE_WAITING_TO_SEND:
      /* A response is stale here and dropped. */
      step_waiting_to_send(ready, now_ticks);
      break;
    case STATE_WAITING_RESPONSE:
      step_waiting_response(ready, response, now_ticks);
      break;
    default:
      state = STATE_IDLE;
      break;
  }
}
