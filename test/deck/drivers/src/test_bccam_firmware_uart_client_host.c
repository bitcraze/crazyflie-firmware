#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_firmware_uart_client.h"
#include "bccam_uart_link.h"
// @MODULE "bccam_deck_controller.c"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_uart_frame.c"
// @MODULE "bccam_uart_runtime.c"

#define TEST_FIRMWARE_BAUDRATE 2000000

static bccam_deck_controller_t deck_controller;
static bccam_firmware_uart_client_t client;

void setUp(void) {
  bccam_deck_controller_init(&deck_controller, NULL);
  bccam_firmware_uart_client_init(&client, &deck_controller);
  bccam_firmware_uart_client_test_trace_reset();
}

void tearDown(void) {
}

static const bccam_firmware_uart_client_test_trace_entry_t *
trace_entry(uint8_t index) {
  const bccam_firmware_uart_client_test_trace_entry_t *entry =
    bccam_firmware_uart_client_test_trace_entry(index);

  TEST_ASSERT_NOT_NULL(entry);
  return entry;
}

static uint8_t find_event_after(bccam_firmware_uart_client_test_event_t event,
                                uint8_t after_index) {
  const uint8_t count = bccam_firmware_uart_client_test_trace_count();

  for (uint8_t i = after_index + 1; i < count; i++) {
    const bccam_firmware_uart_client_test_trace_entry_t *entry =
      bccam_firmware_uart_client_test_trace_entry(i);
    if (entry != NULL && entry->event == event) {
      return i;
    }
  }

  TEST_FAIL_MESSAGE("event not found");
  return 0;
}

static void feed_bytes_to_endpoint(bccam_uart_link_endpoint_t *endpoint,
                                   const uint8_t *bytes,
                                   size_t length) {
  for (size_t i = 0; i < length; i++) {
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                          bccam_uart_link_receive_byte(endpoint, bytes[i]));
  }
}

static void flush_target_to_client(bccam_uart_link_endpoint_t *target) {
  uint8_t reply[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t reply_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(target,
                                                reply,
                                                sizeof(reply),
                                                &reply_len));
  TEST_ASSERT_TRUE(reply_len > 0);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_on_raw_frame(
                          &client,
                          reply,
                          (uint16_t)reply_len));
}

static void target_take_tx_frame(bccam_uart_link_endpoint_t *target,
                                 uint8_t *frame,
                                 size_t frame_capacity,
                                 size_t *frame_len) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(target,
                                                frame,
                                                frame_capacity,
                                                frame_len));
  TEST_ASSERT_TRUE(*frame_len > 0);
}

static void set_contract_id(uint8_t out[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
                            const char *contract_id) {
  TEST_ASSERT_TRUE(strlen(contract_id) <= BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memset(out, 0, BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memcpy(out, contract_id, strlen(contract_id));
}

void testEnterFirmwareSetsFirmwareBaudrateBeforeReleasingReset(void) {
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 10));

  uint8_t index = find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_BEGIN_BOOT,
                                   (uint8_t)-1);
  const bccam_firmware_uart_client_test_trace_entry_t *entry =
    trace_entry(index);
  TEST_ASSERT_EQUAL_INT(BCCAM_DECK_BOOT_FIRMWARE, entry->boot_mode);

  index = find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SET_BAUDRATE,
                           index);
  entry = trace_entry(index);
  TEST_ASSERT_EQUAL_UINT32(TEST_FIRMWARE_BAUDRATE, entry->value);

  index = find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_RELEASE_BOOT,
                           index);

  index = find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_RUNTIME_START_DISCOVERY,
                           index);
}

void testEnterFirmwareStartsDiscoveryAndQueuesDiscoverFrame(void) {
  bccam_uart_link_endpoint_t target;

  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_UNINITIALIZED,
                        bccam_uart_runtime_get_state(&client.runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 0));

  const bccam_firmware_uart_client_test_trace_entry_t *send =
    trace_entry(find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                 (uint8_t)-1));
  TEST_ASSERT_EQUAL_UINT32(14, send->length);

  feed_bytes_to_endpoint(&target, send->bytes, send->length);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, target.state);
}

void testPollFirmwareHandlesRxEventsAndRunsControlProbe(void) {
  bccam_uart_link_endpoint_t target;
  uint8_t service = 0;
  uint8_t request[32] = { 0 };
  uint16_t request_len = 0;
  const uint8_t expected_request[] = {
    0x01, 0x00, 19,
    'm', 'e', 't', 'a', '.', 's', 'c', 'h', 'e', 'm', 'a',
    '_', 'm', 'o', 'd', 'u', 'l', 'e', 's'
  };

  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 0));

  const uint8_t discover_index =
    find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND, (uint8_t)-1);
  const bccam_firmware_uart_client_test_trace_entry_t *discover =
    trace_entry(discover_index);
  feed_bytes_to_endpoint(&target, discover->bytes, discover->length);
  flush_target_to_client(&target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 1));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE,
                        bccam_uart_runtime_get_state(&client.runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_CONTROL_PROBE_WAITING_FOR_TARGET_TX_CREDIT,
                        bccam_firmware_uart_client_control_probe_phase(&client));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&target,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  flush_target_to_client(&target);
  const uint8_t before_request_poll =
    bccam_firmware_uart_client_test_trace_count() - 1u;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 2));

  const bccam_firmware_uart_client_test_trace_entry_t *request_frame =
    trace_entry(find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                 before_request_poll));
  feed_bytes_to_endpoint(&target, request_frame->bytes, request_frame->length);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(&target,
                                                &service,
                                                request,
                                                sizeof(request),
                                                &request_len));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(expected_request), request_len);
  TEST_ASSERT_EQUAL_MEMORY(expected_request, request, sizeof(expected_request));
}

void testFirmwareClientHandlesRawRxEventWithoutPollingUartBytes(void) {
  bccam_uart_link_endpoint_t target;
  uint8_t reply[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t reply_len = 0;

  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_pump_tx(&client));

  const bccam_firmware_uart_client_test_trace_entry_t *discover =
    trace_entry(find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                 (uint8_t)-1));
  feed_bytes_to_endpoint(&target, discover->bytes, discover->length);
  target_take_tx_frame(&target, reply, sizeof(reply), &reply_len);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_on_raw_frame(
                          &client,
                          reply,
                          (uint16_t)reply_len));
}

void testFirmwareClientRejectsNullRxEvent(void) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
                        bccam_firmware_uart_client_on_rx_event(&client,
                                                               NULL));
}

void testFirmwareClientFaultRxEventFaultsRuntimeLink(void) {
  bccam_uart_rx_event_t event;

  memset(&event, 0, sizeof(event));
  event.type = BCCAM_UART_RX_EVENT_FAULT;
  event.fault = BCCAM_UART_RX_FAULT_UART_ERROR;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
                        bccam_firmware_uart_client_on_rx_event(&client,
                                                               &event));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&client.runtime));
}

void testFirmwareClientRejectsUnknownRxEventType(void) {
  bccam_uart_rx_event_t event;

  memset(&event, 0, sizeof(event));
  event.type = (bccam_uart_rx_event_type_t)255;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
                        bccam_firmware_uart_client_on_rx_event(&client,
                                                               &event));
}

void testFirmwareClientRejectsOversizedRawRxEventWithoutMutatingRxCount(void) {
  uint8_t frame[1] = { 0 };
  const uint16_t rx_bytes_before = client.runtime.rx_bytes;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
                        bccam_firmware_uart_client_on_raw_frame(
                          &client,
                          frame,
                          BCCAM_UART_FRAME_MAX_ENCODED_SIZE + 1u));
  TEST_ASSERT_EQUAL_UINT16(rx_bytes_before, client.runtime.rx_bytes);
}

void testFirmwareStartupClassificationReportsIncompatibleWithoutResetPolicy(void) {
  bccam_uart_service_entry_t service = {
    .service_id = 3,
    .major = 1,
    .minor = 0,
  };
  uint8_t management[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t management_len = 0;
  size_t frame_len = 0;

  set_contract_id(service.contract_id, "bitcraze.video");

  TEST_ASSERT_TRUE(bccam_firmware_uart_client_enter(&client, 0));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_encode_discover_reply(1,
                                                              &service,
                                                              1,
                                                              management,
                                                              sizeof(management),
                                                              &management_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                                management,
                                                (uint16_t)management_len,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_on_raw_frame(
                          &client,
                          frame,
                          (uint16_t)frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_firmware_uart_client_poll(&client, 1));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
                        bccam_firmware_uart_client_startup_result(&client));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE,
                        bccam_firmware_uart_client_control_probe_phase(&client));
}
