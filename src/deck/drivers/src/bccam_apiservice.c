/**
 * Camera Deck API Service endpoint: adapts the link API Service
 * (syslink2_apiservice_*) to the API Service router's ApiserviceEndpointOps.
 *
 * The router gives every request in flight on this endpoint a unique wire ID
 * in byte 1. The link service reports a raw completion with only that ID,
 * through a callback without context, so a small table maps the wire ID back
 * to the router's request. The raw response already is an API Service response
 * with the wire ID in byte 1, so it goes to the router as is.
 */
#include "bccam_apiservice.h"

#include <errno.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "syslink2_apiservice_request_queue.h"
#include "syslink2_apiservice.h"
#include "syslink2_apiservice_types.h"

/** At most queue + outstanding requests live in the link service. */
#define BCCAM_APISERVICE_PENDING (SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH + 1u)

typedef struct {
  ApiserviceRequest *request;
  uint8_t wire_id;
} pending_t;

static pending_t pending[BCCAM_APISERVICE_PENDING];

static bool is_ready(const ApiserviceEndpoint *endpoint);
static int submit(const ApiserviceEndpoint *endpoint, ApiserviceRequest *request,
                  const uint8_t *message, size_t length);

static const ApiserviceEndpointOps ops = {
  .isReady = is_ready,
  .submit = submit,
};

static ApiserviceEndpoint endpoint = {
  .path = BCCAM_APISERVICE_ENDPOINT_PATH,
  .ops = &ops,
  .maxRequest = SYSLINK2_APISERVICE_FRAME_MAX_LEN,
  .maxResponse = SYSLINK2_APISERVICE_FRAME_MAX_LEN,
};

static bool pending_add(ApiserviceRequest *request, uint8_t wire_id) {
  bool added = false;
  taskENTER_CRITICAL();
  for (size_t i = 0; i < BCCAM_APISERVICE_PENDING; i++) {
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

static ApiserviceRequest *pending_remove(uint8_t wire_id) {
  ApiserviceRequest *request = NULL;
  taskENTER_CRITICAL();
  for (size_t i = 0; i < BCCAM_APISERVICE_PENDING; i++) {
    if (pending[i].request != NULL && pending[i].wire_id == wire_id) {
      request = pending[i].request;
      pending[i].request = NULL;
      break;
    }
  }
  taskEXIT_CRITICAL();
  return request;
}

/** Runs in the API Service task. */
static void on_complete_raw(syslink2_apiservice_outcome_t outcome,
                            uint8_t user_request_id, const uint8_t *data,
                            uint8_t len) {
  ApiserviceRequest *request = pending_remove(user_request_id);
  if (request == NULL) {
    return;
  }
  int status;
  switch (outcome) {
    case SYSLINK2_APISERVICE_OUTCOME_OK:
    case SYSLINK2_APISERVICE_OUTCOME_VALUE:
    case SYSLINK2_APISERVICE_OUTCOME_ERROR:
      status = 0;
      break;
    case SYSLINK2_APISERVICE_OUTCOME_NOT_READY:
      status = ENOTCONN;
      break;
    case SYSLINK2_APISERVICE_OUTCOME_ABORTED:
      status = ECONNRESET;
      break;
    case SYSLINK2_APISERVICE_OUTCOME_TIMEOUT:
      status = ETIMEDOUT;
      break;
    case SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR:
      status = EPROTO;
      break;
    case SYSLINK2_APISERVICE_OUTCOME_SEND_FAILED:
    default:
      status = EIO;
      break;
  }
  apiserviceRequestComplete(request, status, status == 0 ? data : NULL,
                         status == 0 ? len : 0u);
}

static bool is_ready(const ApiserviceEndpoint *ep) {
  (void)ep;
  return syslink2_apiservice_is_ready();
}

static int submit(const ApiserviceEndpoint *ep, ApiserviceRequest *request,
                  const uint8_t *message, size_t length) {
  (void)ep;
  if (length > SYSLINK2_APISERVICE_FRAME_MAX_LEN) {
    return EMSGSIZE;
  }
  // Before submitting: the API Service task may complete the request before
  // submit_raw() returns.
  if (!pending_add(request, message[1])) {
    return ENOBUFS;
  }
  switch (syslink2_apiservice_submit_raw(message, (uint8_t)length,
                                              on_complete_raw)) {
    case SYSLINK2_APISERVICE_SUBMIT_ACCEPTED:
      return 0;
    case SYSLINK2_APISERVICE_SUBMIT_QUEUE_FULL:
      pending_remove(message[1]);
      return ENOBUFS;
    case SYSLINK2_APISERVICE_SUBMIT_INVALID:
      // Also CMD, and string or bytes values, outside the link service's
      // current scope.
      pending_remove(message[1]);
      return EINVAL;
    case SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED:
    default:
      pending_remove(message[1]);
      return ENOTCONN;
  }
}

/** Runs in the API Service task. */
static void on_unsolicited(const uint8_t *data, uint8_t len) {
  apiserviceEndpointNotify(&endpoint, data, len);
}

int bccam_apiservice_init(void) {
  const int result = apiserviceEndpointRegister(&endpoint);
  if (result == 0) {
    syslink2_apiservice_set_unsolicited_handler(on_unsolicited);
  }
  return result;
}

ApiserviceEndpoint *bccam_apiservice_endpoint(void) {
  return &endpoint;
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_apiservice_test_reset(void) {
  for (size_t i = 0; i < BCCAM_APISERVICE_PENDING; i++) {
    pending[i].request = NULL;
  }
}
#endif
