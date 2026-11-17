/**
 * Camera Deck Control endpoint: adapts the link Control Service
 * (syslink2_control_service_*) to the Control router's ControlEndpointOps.
 *
 * The router gives every request in flight on this endpoint a unique wire ID
 * in byte 1. The link service reports a raw completion with only that ID,
 * through a callback without context, so a small table maps the wire ID back
 * to the router's request. The raw response already is a Control response
 * with the wire ID in byte 1, so it goes to the router as is.
 */
#include "bccam_control.h"

#include <errno.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "syslink2_control_request_queue.h"
#include "syslink2_control_service.h"
#include "syslink2_control_types.h"

/** At most queue + outstanding requests live in the link service. */
#define BCCAM_CONTROL_PENDING (SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH + 1u)

typedef struct {
  ControlRequest *request;
  uint8_t wire_id;
} pending_t;

static pending_t pending[BCCAM_CONTROL_PENDING];

static bool is_ready(const ControlEndpoint *endpoint);
static int submit(const ControlEndpoint *endpoint, ControlRequest *request,
                  const uint8_t *message, size_t length);

static const ControlEndpointOps ops = {
  .isReady = is_ready,
  .submit = submit,
};

static ControlEndpoint endpoint = {
  .path = BCCAM_CONTROL_ENDPOINT_PATH,
  .ops = &ops,
  .maxRequest = SYSLINK2_CONTROL_FRAME_MAX_LEN,
  .maxResponse = SYSLINK2_CONTROL_FRAME_MAX_LEN,
};

static bool pending_add(ControlRequest *request, uint8_t wire_id) {
  bool added = false;
  taskENTER_CRITICAL();
  for (size_t i = 0; i < BCCAM_CONTROL_PENDING; i++) {
    if (pending[i].request == NULL) {
      pending[i].request = request;
      pending[i].wire_id = wire_id;
      added = true;
      break;
    }
  }
  taskEXIT_CRITICAL();
  return added;
}

static ControlRequest *pending_remove(uint8_t wire_id) {
  ControlRequest *request = NULL;
  taskENTER_CRITICAL();
  for (size_t i = 0; i < BCCAM_CONTROL_PENDING; i++) {
    if (pending[i].request != NULL && pending[i].wire_id == wire_id) {
      request = pending[i].request;
      pending[i].request = NULL;
      break;
    }
  }
  taskEXIT_CRITICAL();
  return request;
}

/** Runs in the Control Service task. */
static void on_complete_raw(syslink2_control_outcome_t outcome,
                            uint8_t user_request_id, const uint8_t *data,
                            uint8_t len) {
  ControlRequest *request = pending_remove(user_request_id);
  if (request == NULL) {
    return;
  }
  int status;
  switch (outcome) {
    case SYSLINK2_CONTROL_OUTCOME_OK:
    case SYSLINK2_CONTROL_OUTCOME_VALUE:
    case SYSLINK2_CONTROL_OUTCOME_ERROR:
      status = 0;
      break;
    case SYSLINK2_CONTROL_OUTCOME_NOT_READY:
      status = ENOTCONN;
      break;
    case SYSLINK2_CONTROL_OUTCOME_ABORTED:
      status = ECONNRESET;
      break;
    case SYSLINK2_CONTROL_OUTCOME_TIMEOUT:
      status = ETIMEDOUT;
      break;
    case SYSLINK2_CONTROL_OUTCOME_PROTOCOL_ERROR:
      status = EPROTO;
      break;
    case SYSLINK2_CONTROL_OUTCOME_SEND_FAILED:
    default:
      status = EIO;
      break;
  }
  controlRequestComplete(request, status, status == 0 ? data : NULL,
                         status == 0 ? len : 0u);
}

static bool is_ready(const ControlEndpoint *ep) {
  (void)ep;
  return syslink2_control_service_is_ready();
}

static int submit(const ControlEndpoint *ep, ControlRequest *request,
                  const uint8_t *message, size_t length) {
  (void)ep;
  if (length > SYSLINK2_CONTROL_FRAME_MAX_LEN) {
    return EMSGSIZE;
  }
  // Before submitting: the Control task may complete the request before
  // submit_raw() returns.
  if (!pending_add(request, message[1])) {
    return ENOBUFS;
  }
  switch (syslink2_control_service_submit_raw(message, (uint8_t)length,
                                              on_complete_raw)) {
    case SYSLINK2_CONTROL_SUBMIT_ACCEPTED:
      return 0;
    case SYSLINK2_CONTROL_SUBMIT_QUEUE_FULL:
      pending_remove(message[1]);
      return ENOBUFS;
    case SYSLINK2_CONTROL_SUBMIT_INVALID:
      // Also CMD, and string or bytes values, outside the link service's
      // current scope.
      pending_remove(message[1]);
      return EINVAL;
    case SYSLINK2_CONTROL_SUBMIT_NOT_INITIALIZED:
    default:
      pending_remove(message[1]);
      return ENOTCONN;
  }
}

int bccam_control_init(void) {
  return controlEndpointRegister(&endpoint);
}

ControlEndpoint *bccam_control_endpoint(void) {
  return &endpoint;
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_control_test_reset(void) {
  for (size_t i = 0; i < BCCAM_CONTROL_PENDING; i++) {
    pending[i].request = NULL;
  }
}
#endif
