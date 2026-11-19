#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "syslink2_apiservice_channel.h"
#include "bccam_uart_runtime.h" // @NO_MODULE

/* Fake runtime. */
static bccam_uart_runtime_t runtime;
static bccam_uart_runtime_t *seen_runtime;
static bool fake_ready;
static uint8_t fake_tx_credit;
static int fake_send_result;
static unsigned grants;
static unsigned sends;
static syslink2_apiservice_frame_t sent;
static bool fake_rx_full;
static syslink2_apiservice_frame_t fake_rx;

bool bccam_uart_runtime_apiservice_ready(bccam_uart_runtime_t *r) {
  seen_runtime = r;
  return fake_ready;
}
void bccam_uart_runtime_grant_apiservice_rx_credit(bccam_uart_runtime_t *r) {
  seen_runtime = r;
  grants++;
}
uint8_t bccam_uart_runtime_apiservice_tx_credit(const bccam_uart_runtime_t *r) {
  (void)r;
  return fake_tx_credit;
}
int bccam_uart_runtime_send_apiservice(bccam_uart_runtime_t *r,
                                    const syslink2_apiservice_frame_t *frame) {
  seen_runtime = r;
  sends++;
  sent = *frame;
  return fake_send_result;
}
bool bccam_uart_runtime_take_apiservice_rx(bccam_uart_runtime_t *r,
                                        syslink2_apiservice_frame_t *frame) {
  seen_runtime = r;
  if (!fake_rx_full) {
    return false;
  }
  *frame = fake_rx;
  fake_rx_full = false;
  return true;
}

void setUp(void) {
  seen_runtime = NULL;
  fake_ready = false;
  fake_tx_credit = 0u;
  fake_send_result = BCCAM_UART_OK;
  grants = 0u;
  sends = 0u;
  fake_rx_full = false;
  syslink2_apiservice_channel_init(&runtime);
}

void tearDown(void) {}

void testRuntimeReadyIsPassedOnWithStoredRuntime(void) {
  TEST_ASSERT_FALSE(syslink2_apiservice_channel_runtime_ready());
  fake_ready = true;
  TEST_ASSERT_TRUE(syslink2_apiservice_channel_runtime_ready());
  TEST_ASSERT_EQUAL_PTR(&runtime, seen_runtime);
}

void testOpenGrantsOneCredit(void) {
  syslink2_apiservice_channel_open();
  TEST_ASSERT_EQUAL_UINT(1, grants);
}

void testCanSendFollowsTransmitCredit(void) {
  TEST_ASSERT_FALSE(syslink2_apiservice_channel_can_send());
  fake_tx_credit = 1u;
  TEST_ASSERT_TRUE(syslink2_apiservice_channel_can_send());
}

void testSendResultsAreSorted(void) {
  syslink2_apiservice_frame_t frame = { .data = { 1, 2 }, .len = 2 };
  const struct { int runtime; syslink2_apiservice_send_result_t expected; } cases[] = {
    { BCCAM_UART_OK, SYSLINK2_APISERVICE_SEND_SENT },
    { BCCAM_UART_ERR_NO_CREDIT, SYSLINK2_APISERVICE_SEND_BUSY },
    { BCCAM_UART_ERR_TRANSACTION_BUSY, SYSLINK2_APISERVICE_SEND_BUSY },
    { BCCAM_UART_ERR_LINK_FAULT, SYSLINK2_APISERVICE_SEND_CLOSED },
    { BCCAM_UART_ERR_NOT_ACTIVE, SYSLINK2_APISERVICE_SEND_CLOSED },
    { BCCAM_UART_ERR_UNKNOWN_SERVICE, SYSLINK2_APISERVICE_SEND_CLOSED },
    { BCCAM_UART_ERR_PAYLOAD_TOO_LONG, SYSLINK2_APISERVICE_SEND_REJECTED },
    { BCCAM_UART_ERR_BAD_ARGUMENT, SYSLINK2_APISERVICE_SEND_REJECTED },
    { BCCAM_UART_ERR_BAD_CRC, SYSLINK2_APISERVICE_SEND_REJECTED },
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    fake_send_result = cases[i].runtime;
    TEST_ASSERT_EQUAL(cases[i].expected, syslink2_apiservice_channel_send(&frame));
  }
  TEST_ASSERT_EQUAL_UINT8(2, sent.len);
  TEST_ASSERT_EQUAL_PTR(&runtime, seen_runtime);
}

void testReceiveCopiesFrameAndGrantsCredit(void) {
  syslink2_apiservice_frame_t frame;
  TEST_ASSERT_FALSE(syslink2_apiservice_channel_receive(&frame));
  TEST_ASSERT_EQUAL_UINT(0, grants);

  fake_rx_full = true;
  fake_rx.len = 2;
  fake_rx.data[0] = 0x80;
  fake_rx.data[1] = 5;
  TEST_ASSERT_TRUE(syslink2_apiservice_channel_receive(&frame));
  TEST_ASSERT_EQUAL_UINT8(2, frame.len);
  TEST_ASSERT_EQUAL_UINT8(5, frame.data[1]);
  TEST_ASSERT_EQUAL_UINT(1, grants);
}
