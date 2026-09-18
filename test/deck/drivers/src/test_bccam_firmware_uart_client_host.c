#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_firmware_uart_client.h"
// @MODULE "bccam_deck_controller.c"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_uart_frame.c"
// @MODULE "bccam_uart_link.c"
// @MODULE "bccam_uart_runtime.c"

#define TEST_FIRMWARE_BAUDRATE 1000000
#define TEST_FIRMWARE_BOOT_WAIT_TICKS 2000

static bccam_deck_controller_t deck_controller;
static bccam_firmware_uart_client_t client;

void setUp(void) {
  bccam_deck_controller_init(&deck_controller, NULL);
  bccam_firmware_uart_client_init(&client, &deck_controller);
  bccam_firmware_uart_client_test_trace_reset();
}
void tearDown(void) {}

static const bccam_firmware_uart_client_test_trace_entry_t *trace(uint8_t i) {
  const bccam_firmware_uart_client_test_trace_entry_t *entry =
    bccam_firmware_uart_client_test_trace_entry(i);
  TEST_ASSERT_NOT_NULL(entry);
  return entry;
}

static uint8_t find_after(bccam_firmware_uart_client_test_event_t event,
                          uint8_t after) {
  for (uint8_t i = (uint8_t)(after + 1u);
       i < bccam_firmware_uart_client_test_trace_count(); i++) {
    if (trace(i)->event == event) {
      return i;
    }
  }
  TEST_FAIL_MESSAGE("trace event not found");
  return 0;
}

void testEnterFirmwarePreservesOneMbaudOrderingAndStartsEstablishment(void) {
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 10));
  uint8_t index = find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_BEGIN_BOOT,
                             (uint8_t)-1);
  TEST_ASSERT_EQUAL_INT(BCCAM_DECK_BOOT_FIRMWARE, trace(index)->boot_mode);
  index = find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SET_BAUDRATE, index);
  TEST_ASSERT_EQUAL_UINT32(TEST_FIRMWARE_BAUDRATE, trace(index)->value);
  index = find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_RELEASE_BOOT, index);
  TEST_ASSERT_EQUAL_UINT32(TEST_FIRMWARE_BOOT_WAIT_TICKS, trace(index)->value);
  index = find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_RUNTIME_START_ESTABLISHMENT,
                     index);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE,
                        bccam_uart_runtime_get_state(&client.runtime));
}

void testFirmwareAdapterFailureFailStopsWithoutRetry(void) {
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  bccam_firmware_uart_client_test_set_uart_send_result(false);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
                        bccam_firmware_uart_client_poll(&client, 0));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&client.runtime));
  const uint8_t send = find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                  (uint8_t)-1);
  TEST_ASSERT_EQUAL_UINT32(14, trace(send)->length);

  bccam_firmware_uart_client_test_set_uart_send_result(true);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 1));
  TEST_ASSERT_EQUAL_UINT8(send,
    find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND, (uint8_t)-1));
}

void testSuccessfulFirmwareAdapterResultCompletesOneRuntimeFlush(void) {
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  bccam_firmware_uart_client_test_set_uart_send_result(true);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 0));
  TEST_ASSERT_EQUAL_UINT16(1, client.runtime.tx_flushes);
  TEST_ASSERT_EQUAL_UINT16(14, client.runtime.tx_bytes);
  TEST_ASSERT_EQUAL_UINT16(0, client.runtime.link.counters.tx_failures);
}

void testPollWritesEstablishExactlyOnceWithoutTransparentRetry(void) {
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 0));
  const uint8_t send = find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                  (uint8_t)-1);
  TEST_ASSERT_EQUAL_UINT32(14, trace(send)->length);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH, trace(send)->bytes[6]);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 5000));
  TEST_ASSERT_EQUAL_UINT8(send,
    find_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND, (uint8_t)-1));
}

void testClientRxFaultBeforeEstablishmentLeavesLinkInactive(void) {
  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_UART_ERROR,
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_firmware_uart_client_on_rx_event(&client, &event));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE,
    bccam_uart_runtime_get_state(&client.runtime));
}

void testClientRxFaultWhileActiveEntersLinkFault(void) {
  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_UART_ERROR,
  };
  client.runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  client.runtime.link.active_link_version = 1;
  client.runtime.link.negotiated_payload = 64;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
    bccam_firmware_uart_client_on_rx_event(&client, &event));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
    bccam_uart_runtime_get_state(&client.runtime));
}

void testClientRejectsInvalidRxEvents(void) {
  bccam_uart_rx_event_t event = { .type = (bccam_uart_rx_event_type_t)255 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_firmware_uart_client_on_rx_event(&client, NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_firmware_uart_client_on_rx_event(&client, &event));
  event.type = BCCAM_UART_RX_EVENT_RAW_FRAME;
  event.raw_frame.length = BCCAM_UART_FRAME_MAX_ENCODED_SIZE + 1u;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_firmware_uart_client_on_rx_event(&client, &event));
}

void testSuspendInvalidatesSessionAndBinding(void) {
  client.runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  client.runtime.link.control_binding.valid = true;
  client.runtime.link.control_binding.descriptor.handle = 0x91;
  bccam_firmware_uart_client_suspend(&client);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, client.runtime.link.state);
  TEST_ASSERT_FALSE(client.runtime.link.control_binding.valid);
}
