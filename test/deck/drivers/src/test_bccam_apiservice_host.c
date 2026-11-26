/**
 * Host tests for the Camera Deck API Service endpoint: requests go through the
 * real API Service router, the adapter and the real link API Service, which
 * is stepped by hand over a fake channel, as in
 * test_syslink2_apiservice_host.c.
 */
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "FreeRTOS.h" // @NO_MODULE
#include "bccam_apiservice.h"
#include "apiservice.h"
#include "syslink2_apiservice.h"
#include "syslink2_apiservice_codec.h"
#include "syslink2_apiservice_request_handling.h"
#include "syslink2_apiservice_request_queue.h"
#include "syslink2_apiservice_channel.h" // @NO_MODULE

void syslink2_apiservice_test_reset(void);

void vPortEnterCritical(void) {}
void vPortExitCritical(void) {}

/* Fake channel. */
static bccam_uart_runtime_t runtime;
static bool runtime_ready;
static unsigned sends;
static syslink2_apiservice_frame_t sent;
static bool rx_full;
static syslink2_apiservice_frame_t rx;

void syslink2_apiservice_channel_init(bccam_uart_runtime_t *r) { (void)r; }
bool syslink2_apiservice_channel_runtime_ready(void) { return runtime_ready; }
void syslink2_apiservice_channel_open(void) {}
bool syslink2_apiservice_channel_can_send(void) { return true; }
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
  rx_full = false;
  *frame = rx;
  return true;
}

/* The deck answers the frame it was sent, with its deck request ID. */
static void deck_answer(const uint8_t *bytes, uint8_t len) {
  memset(&rx, 0, sizeof(rx));
  memcpy(rx.data, bytes, len);
  rx.data[1] = sent.data[1];
  rx.len = len;
  rx_full = true;
}

/* Recording originator. */
typedef struct {
  ApiserviceRequest request;
  uint8_t buffer[64];
  unsigned calls;
  int status;
  size_t length;
} originator_t;

static void record(ApiserviceRequest *request, int status, size_t length) {
  originator_t *o = request->user;
  o->calls++;
  o->status = status;
  o->length = length;
}

static void originator_init(originator_t *o, const uint8_t *message, size_t len) {
  memset(o, 0, sizeof(*o));
  memcpy(o->buffer, message, len);
  o->request.response = o->buffer;
  o->request.responseCapacity = sizeof(o->buffer);
  o->request.complete = record;
  o->request.user = o;
}

static ApiserviceEndpoint *ep;
static uint32_t now;

/* Recording listener. */
static unsigned notify_calls;
static ApiserviceEndpoint *notify_endpoint;
static uint8_t notify_data[64];
static size_t notify_len;

static void on_notify(ApiserviceListener *listener, ApiserviceEndpoint *endpoint,
                      const uint8_t *message, size_t length) {
  (void)listener;
  notify_calls++;
  notify_endpoint = endpoint;
  memcpy(notify_data, message, length);
  notify_len = length;
}

static ApiserviceListener listener = {.onNotify = on_notify};

static void step(void) {
  syslink2_apiservice_step(now++);
}

void setUp(void) {
  apiserviceResetForTest();
  syslink2_apiservice_test_reset();
  bccam_apiservice_test_reset();
  runtime_ready = false;
  sends = 0;
  rx_full = false;
  now = 0;
  syslink2_apiservice_init(&runtime);
  TEST_ASSERT_EQUAL(0, bccam_apiservice_init());
  notify_calls = 0;
  TEST_ASSERT_EQUAL(0, apiserviceListenerRegister(&listener));
  apiserviceEndpointFreeze();
  ep = apiserviceEndpointById(0);
  TEST_ASSERT_EQUAL_PTR(bccam_apiservice_endpoint(), ep);
}

void tearDown(void) {}

static const uint8_t get_q[] = {0x01, 7, 6, 's', 't', 'r', 'e', 'a', 'm'};

void testNotifyFromTheDeckReachesListeners(void) {
  runtime_ready = true;
  step();
  const uint8_t notify[] = {0xc0, 6, 's', 't', 'r', 'e', 'a', 'm', 0x10, 1, 42};
  memset(&rx, 0, sizeof(rx));
  memcpy(rx.data, notify, sizeof(notify));
  rx.len = sizeof(notify);
  rx_full = true;
  step();
  TEST_ASSERT_EQUAL_UINT(1, notify_calls);
  TEST_ASSERT_EQUAL_PTR(ep, notify_endpoint);
  TEST_ASSERT_EQUAL_UINT(sizeof(notify), notify_len);
  TEST_ASSERT_EQUAL_MEMORY(notify, notify_data, sizeof(notify));
}

void testRegistersDeckPathWithLinkLimits(void) {
  TEST_ASSERT_EQUAL_STRING("deck:bcCam", ep->path);
  TEST_ASSERT_EQUAL_UINT16(85, ep->maxRequest);
  TEST_ASSERT_EQUAL_UINT16(85, ep->maxResponse);
  // Registration is closed after the freeze.
  TEST_ASSERT_EQUAL(EBUSY, bccam_apiservice_init());
}

void testReadyFollowsLinkService(void) {
  TEST_ASSERT_FALSE(apiserviceEndpointIsReady(ep));
  originator_t o;
  originator_init(&o, get_q, sizeof(get_q));
  TEST_ASSERT_EQUAL(ENOTCONN, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(get_q)));
  runtime_ready = true;
  step();
  TEST_ASSERT_TRUE(apiserviceEndpointIsReady(ep));
}

void testGetThroughRouterAndAdapter(void) {
  runtime_ready = true;
  step();
  originator_t o;
  originator_init(&o, get_q, sizeof(get_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(get_q)));
  step();
  step();
  TEST_ASSERT_EQUAL_UINT(1, sends);
  TEST_ASSERT_EQUAL_UINT8(0x01, sent.data[0]);
  TEST_ASSERT_EQUAL_MEMORY(&get_q[2], &sent.data[2], sizeof(get_q) - 2);
  const uint8_t value[] = {0x81, 0, 0x10, 1, 80};
  deck_answer(value, sizeof(value));
  step();
  TEST_ASSERT_EQUAL_UINT(1, o.calls);
  TEST_ASSERT_EQUAL(0, o.status);
  const uint8_t expected[] = {0x81, 7, 0x10, 1, 80};
  TEST_ASSERT_EQUAL_UINT(sizeof(expected), o.length);
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, sizeof(expected));
}

void testSetThroughRouterAndAdapter(void) {
  runtime_ready = true;
  step();
  const uint8_t set_q[] = {0x02, 9, 1, 'q', 0x10, 1, 80};
  originator_t o;
  originator_init(&o, set_q, sizeof(set_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(set_q)));
  step();
  step();
  TEST_ASSERT_EQUAL_UINT8(0x02, sent.data[0]);
  const uint8_t ok[] = {0x80, 0};
  deck_answer(ok, sizeof(ok));
  step();
  TEST_ASSERT_EQUAL(0, o.status);
  const uint8_t expected[] = {0x80, 9};
  TEST_ASSERT_EQUAL_UINT(2, o.length);
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, 2);
}

void testCmdWithArgumentThroughRouterAndAdapter(void) {
  runtime_ready = true;
  step();
  const uint8_t cmd_q[] = {0x03, 9, 1, 'q', 0x10, 1, 80};
  originator_t o;
  originator_init(&o, cmd_q, sizeof(cmd_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(cmd_q)));
  step();
  step();
  TEST_ASSERT_EQUAL_UINT(1, sends);
  TEST_ASSERT_EQUAL_UINT(sizeof(cmd_q), sent.len);
  TEST_ASSERT_EQUAL_UINT8(0x03, sent.data[0]);
  TEST_ASSERT_EQUAL_MEMORY(&cmd_q[2], &sent.data[2], sizeof(cmd_q) - 2);
  const uint8_t value[] = {0x81, 0, 0x10, 1, 81};
  deck_answer(value, sizeof(value));
  step();
  TEST_ASSERT_EQUAL(0, o.status);
  const uint8_t expected[] = {0x81, 9, 0x10, 1, 81};
  TEST_ASSERT_EQUAL_UINT(sizeof(expected), o.length);
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, sizeof(expected));
}

void testCmdWithoutArgumentThroughRouterAndAdapter(void) {
  runtime_ready = true;
  step();
  const uint8_t cmd_q[] = {0x03, 9, 1, 'q'};
  originator_t o;
  originator_init(&o, cmd_q, sizeof(cmd_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(cmd_q)));
  step();
  step();
  TEST_ASSERT_EQUAL_UINT(sizeof(cmd_q), sent.len);
  const uint8_t ok[] = {0x80, 0};
  deck_answer(ok, sizeof(ok));
  step();
  TEST_ASSERT_EQUAL(0, o.status);
  const uint8_t expected[] = {0x80, 9};
  TEST_ASSERT_EQUAL_UINT(sizeof(expected), o.length);
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, sizeof(expected));
}

void testErrorResponseKeepsDetail(void) {
  runtime_ready = true;
  step();
  originator_t o;
  originator_init(&o, get_q, sizeof(get_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(get_q)));
  step();
  step();
  const uint8_t error[] = {0x82, 0, 0x03, 2, 'n', 'o'};
  deck_answer(error, sizeof(error));
  step();
  TEST_ASSERT_EQUAL(0, o.status);
  const uint8_t expected[] = {0x82, 7, 0x03, 2, 'n', 'o'};
  TEST_ASSERT_EQUAL_UINT(sizeof(expected), o.length);
  TEST_ASSERT_EQUAL_MEMORY(expected, o.buffer, sizeof(expected));
}

void testTimeoutIsTimedOut(void) {
  runtime_ready = true;
  step();
  originator_t o;
  originator_init(&o, get_q, sizeof(get_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(get_q)));
  step();
  step();
  now += M2T(SYSLINK2_APISERVICE_TIMEOUT_MS);
  step();
  TEST_ASSERT_EQUAL_UINT(1, o.calls);
  TEST_ASSERT_EQUAL(ETIMEDOUT, o.status);
  TEST_ASSERT_EQUAL_UINT(0, o.length);
  TEST_ASSERT_EQUAL_MEMORY(get_q, o.buffer, sizeof(get_q));
}

void testReadyLostIsConnectionResetThenNotConnected(void) {
  runtime_ready = true;
  step();
  originator_t a, b;
  originator_init(&a, get_q, sizeof(get_q));
  originator_init(&b, get_q, sizeof(get_q));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &a.request, a.buffer, sizeof(get_q)));
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &b.request, b.buffer, sizeof(get_q)));
  step();
  step();
  runtime_ready = false;
  step();
  TEST_ASSERT_EQUAL(ECONNRESET, a.status);
  step();
  TEST_ASSERT_EQUAL(ENOTCONN, b.status);
  TEST_ASSERT_EQUAL_UINT8(7, b.buffer[1]);
}

void testOutOfScopeRequestIsInvalid(void) {
  runtime_ready = true;
  step();
  const uint8_t unknown[] = {0x04, 1, 1, 'q'};
  originator_t o;
  originator_init(&o, unknown, sizeof(unknown));
  TEST_ASSERT_EQUAL(EINVAL, apiserviceSubmit(ep, &o.request, o.buffer, sizeof(unknown)));
  TEST_ASSERT_EQUAL_UINT8(1, o.buffer[1]);
  TEST_ASSERT_EQUAL_UINT(0, o.calls);
}

void testQueueFullIsNoBuffersAndFreesTable(void) {
  runtime_ready = true;
  step();
  static originator_t many[SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH + 1];
  for (unsigned i = 0; i < SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH; i++) {
    originator_init(&many[i], get_q, sizeof(get_q));
    TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &many[i].request, many[i].buffer, sizeof(get_q)));
  }
  originator_t extra;
  originator_init(&extra, get_q, sizeof(get_q));
  TEST_ASSERT_EQUAL(ENOBUFS, apiserviceSubmit(ep, &extra.request, extra.buffer, sizeof(get_q)));
  // One leaves the queue to be sent: there is room for one more.
  step();
  TEST_ASSERT_EQUAL(0, apiserviceSubmit(ep, &extra.request, extra.buffer, sizeof(get_q)));
  // Every request completes, each to its own originator.
  for (unsigned i = 0; i < SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH + 1; i++) {
    step();  // Take the next one, or send the one already taken.
    step();  // Send it, or wait for its answer.
    const uint8_t value[] = {0x81, 0, 0x10, 1, (uint8_t)i};
    deck_answer(value, sizeof(value));
    step();
  }
  for (unsigned i = 0; i < SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH; i++) {
    TEST_ASSERT_EQUAL_UINT(1, many[i].calls);
    TEST_ASSERT_EQUAL(0, many[i].status);
    TEST_ASSERT_EQUAL_UINT8(i, many[i].buffer[4]);
  }
  TEST_ASSERT_EQUAL_UINT(1, extra.calls);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH, extra.buffer[4]);
}
