#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_uart_link.h"
#include "bccam_uart_runtime.h"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_uart_frame.c"

typedef struct {
  uint8_t bytes[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t length;
  uint32_t calls;
} capture_t;

static void capture_send(void *context, const uint8_t *data, uint32_t length) {
  capture_t *capture = (capture_t *)context;

  TEST_ASSERT_TRUE(length <= sizeof(capture->bytes));
  memcpy(capture->bytes, data, length);
  capture->length = length;
  capture->calls++;
}

static void feed_bytes_to_endpoint(bccam_uart_link_endpoint_t *endpoint,
                                   const uint8_t *bytes,
                                   size_t length) {
  for (size_t i = 0; i < length; i++) {
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_receive_byte(endpoint, bytes[i]));
  }
}

static void feed_bytes_to_runtime(bccam_uart_runtime_t *runtime,
                                  const uint8_t *bytes,
                                  size_t length) {
  for (size_t i = 0; i < length; i++) {
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_receive_byte(runtime, bytes[i]));
  }
}

static void flush_runtime_to_target(bccam_uart_runtime_t *runtime,
                                    bccam_uart_link_endpoint_t *target) {
  capture_t capture = { 0 };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(runtime, capture_send, &capture));
  TEST_ASSERT_EQUAL_UINT32(1, capture.calls);
  feed_bytes_to_endpoint(target, capture.bytes, capture.length);
}

static void flush_target_to_runtime(bccam_uart_link_endpoint_t *target,
                                    bccam_uart_runtime_t *runtime) {
  uint8_t reply[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t reply_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(target, reply, sizeof(reply), &reply_len));
  TEST_ASSERT_TRUE(reply_len > 0);
  feed_bytes_to_runtime(runtime, reply, reply_len);
}

static void establish_runtime_link(bccam_uart_runtime_t *runtime,
                                   bccam_uart_link_endpoint_t *target) {
  bccam_uart_runtime_init(runtime);
  bccam_uart_link_init(target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(runtime));
  flush_runtime_to_target(runtime, target);
  flush_target_to_runtime(target, runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_service_is_discovered(runtime,
                                                            BCCAM_UART_SERVICE_CONTROL));
}

static void target_open_control_credit(bccam_uart_link_endpoint_t *target) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(target,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
}

static void target_expect_schema_modules_request(bccam_uart_link_endpoint_t *target) {
  uint8_t service = 0;
  uint8_t request[32];
  uint16_t request_len = 0;
  const uint8_t expected_request[] = {
    0x01, 0x00, 19,
    'm', 'e', 't', 'a', '.', 's', 'c', 'h', 'e', 'm', 'a',
    '_', 'm', 'o', 'd', 'u', 'l', 'e', 's'
  };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(target,
                                                &service,
                                                request,
                                                sizeof(request),
                                                &request_len));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(expected_request), request_len);
  TEST_ASSERT_EQUAL_MEMORY(expected_request, request, sizeof(expected_request));
}


static void set_contract_id(uint8_t out[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
                            const char *contract_id) {
  TEST_ASSERT_TRUE(strlen(contract_id) <= BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memset(out, 0, BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memcpy(out, contract_id, strlen(contract_id));
}

static void feed_discover_reply_with_services(
  bccam_uart_runtime_t *runtime,
  const bccam_uart_service_entry_t *services,
  uint8_t service_count) {
  uint8_t management[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t management_len = 0;
  size_t frame_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_encode_discover_reply(1,
                                                              services,
                                                              service_count,
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
  feed_bytes_to_runtime(runtime, frame, frame_len);
}

static void target_configure_services(
  bccam_uart_link_endpoint_t *target,
  const bccam_uart_service_entry_t *services,
  uint8_t service_count) {
  bccam_uart_link_init(target, BCCAM_UART_LINK_ROLE_TARGET);
  target->state = BCCAM_UART_LINK_ACTIVE;
  target->negotiated_payload = BCCAM_UART_NORMAL_MAX_PAYLOAD;
  target->service_count = service_count;
  memcpy(target->services,
         services,
         (size_t)service_count * sizeof(target->services[0]));
}

static void establish_runtime_link_with_services(
  bccam_uart_runtime_t *runtime,
  bccam_uart_link_endpoint_t *target,
  const bccam_uart_service_entry_t *services,
  uint8_t service_count) {
  bccam_uart_runtime_init(runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(runtime));
  feed_discover_reply_with_services(runtime, services, service_count);
  target_configure_services(target, services, service_count);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(runtime));
}

static void target_open_credit_for_service(bccam_uart_link_endpoint_t *target,
                                           uint8_t service_id) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(target, service_id, 1));
}

static void target_expect_schema_modules_request_on_service(
  bccam_uart_link_endpoint_t *target,
  uint8_t expected_service) {
  uint8_t service = 0;
  uint8_t request[32];
  uint16_t request_len = 0;
  const uint8_t expected_request[] = {
    0x01, 0x00, 19,
    'm', 'e', 't', 'a', '.', 's', 'c', 'h', 'e', 'm', 'a',
    '_', 'm', 'o', 'd', 'u', 'l', 'e', 's'
  };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(target,
                                                &service,
                                                request,
                                                sizeof(request),
                                                &request_len));
  TEST_ASSERT_EQUAL_UINT8(expected_service, service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(expected_request), request_len);
  TEST_ASSERT_EQUAL_MEMORY(expected_request, request, sizeof(expected_request));
}

static void target_reply_with_empty_schema_modules(bccam_uart_link_endpoint_t *target) {
  const uint8_t response[] = { 0x81, 0x00, 0x21, 0x01, 0x00 };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    response,
                                                    sizeof(response)));
}

static void target_reply_with_one_schema_module(bccam_uart_link_endpoint_t *target) {
  const uint8_t response[] = {
    0x81, 0x00, 0x21, 0x17,
    0x01,
    0x03, 'c', 'a', 'm',
    0x0f, 'b', 'i', 't', 'c', 'r', 'a', 'z', 'e', '.', 'c', 'a', 'm', 'e', 'r', 'a',
    0x01, 0x02
  };

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    response,
                                                    sizeof(response)));
}

void testFlushTxWithoutPendingFrameIsNoop(void) {
  bccam_uart_runtime_t runtime;
  capture_t capture = { 0 };

  bccam_uart_runtime_init(&runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime, capture_send, &capture));
  TEST_ASSERT_EQUAL_UINT32(0, capture.calls);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, runtime.last_error);
}

void testFirmwareBootStartsDiscoveryUntilActive(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  capture_t capture = { 0 };
  uint8_t reply[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t reply_len = 0;

  bccam_uart_runtime_init(&runtime);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime, capture_send, &capture));
  TEST_ASSERT_EQUAL_UINT32(1, capture.calls);
  TEST_ASSERT_TRUE(capture.length > 0);

  feed_bytes_to_endpoint(&target, capture.bytes, capture.length);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target, reply, sizeof(reply), &reply_len));
  feed_bytes_to_runtime(&runtime, reply, reply_len);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_service_is_discovered(&runtime,
                                                            BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_FALSE(bccam_uart_runtime_service_is_discovered(&runtime,
                                                             BCCAM_UART_SERVICE_CONSOLE));
}

void testDiscoveryRetryResendsPendingDiscover(void) {
  bccam_uart_runtime_t runtime;
  capture_t first_discover = { 0 };
  capture_t retry_discover = { 0 };

  bccam_uart_runtime_init(&runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime,
                                                    capture_send,
                                                    &first_discover));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_retry_discovery(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime,
                                                    capture_send,
                                                    &retry_discover));

  TEST_ASSERT_EQUAL_size_t(first_discover.length, retry_discover.length);
  TEST_ASSERT_EQUAL_MEMORY(first_discover.bytes,
                           retry_discover.bytes,
                           first_discover.length);
}

void testDiscoveryRetryIsCancelledWhenReplyCompletes(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  capture_t first_discover = { 0 };
  capture_t after_reply = { 0 };
  uint8_t reply[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t reply_len = 0;

  bccam_uart_runtime_init(&runtime);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime,
                                                    capture_send,
                                                    &first_discover));
  feed_bytes_to_endpoint(&target, first_discover.bytes, first_discover.length);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target, reply, sizeof(reply), &reply_len));
  TEST_ASSERT_TRUE(reply_len > 1);

  feed_bytes_to_runtime(&runtime, reply, reply_len - 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_UNINITIALIZED,
                        bccam_uart_runtime_get_state(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_retry_discovery(&runtime));
  feed_bytes_to_runtime(&runtime, &reply[reply_len - 1], 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime, capture_send, &after_reply));
  TEST_ASSERT_EQUAL_UINT32(0, after_reply.calls);
}

void testControlProbeOpensRxSlotAndWaitsForTargetCreditBeforeRequest(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  capture_t no_request = { 0 };

  establish_runtime_link(&runtime, &target);

  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&target,
                                                           BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime, capture_send, &no_request));
  TEST_ASSERT_EQUAL_UINT32(0, no_request.calls);
  TEST_ASSERT_FALSE(runtime.control_request_sent);

  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_request_sent);
  flush_runtime_to_target(&runtime, &target);

  target_expect_schema_modules_request(&target);
}

void testControlProbeIgnoresSchemaValueBeforeSchemaGetIsSent(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;

  establish_runtime_link(&runtime, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&target,
                                                           BCCAM_UART_SERVICE_CONTROL));

  target_reply_with_empty_schema_modules(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_WAITING,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testControlProbeCompletesAfterEmptySchemaModulesResponse(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);

  target_expect_schema_modules_request(&target);
  target_reply_with_empty_schema_modules(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_TRUE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_control_schema_module_count(&runtime));
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(&runtime));
}

void testControlProbeParsesSchemaModuleTable(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  const bccam_uart_control_schema_module_t *module = NULL;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);

  target_expect_schema_modules_request(&target);
  target_reply_with_one_schema_module(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_TRUE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_control_schema_module_count(&runtime));
  module = bccam_uart_runtime_control_schema_module(&runtime, 0);
  TEST_ASSERT_NOT_NULL(module);
  TEST_ASSERT_EQUAL_STRING("cam", module->namespace);
  TEST_ASSERT_EQUAL_STRING("bitcraze.camera", module->contract_id);
  TEST_ASSERT_EQUAL_UINT8(1, module->major);
  TEST_ASSERT_EQUAL_UINT8(2, module->minor);
}

void testMalformedControlResponseIncrementsCounterWithoutFaultingLink(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  const uint8_t malformed[] = { 0x81, 0x00 };

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    malformed,
                                                    sizeof(malformed)));
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_link_get_counters(&runtime.link)->link_faults);
}

void testControlProbeReturnsAfterRequestQueuesBeforeParsingPendingRx(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  const uint8_t malformed[] = { 0x81, 0x00 };

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    malformed,
                                                    sizeof(malformed)));
  flush_target_to_runtime(&target, &runtime);
  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);

  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&runtime.link,
                                                           BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_request_sent);
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));

  flush_runtime_to_target(&runtime, &target);
  target_expect_schema_modules_request(&target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_link_get_counters(&runtime.link)->link_faults);
}

void testControlProbeDoneDoesNotConsumeLaterControlTraffic(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  const uint8_t malformed[] = { 0x7E };
  uint8_t service = 0;
  uint8_t payload[sizeof(malformed)] = { 0 };
  uint16_t payload_len = 0;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  target_expect_schema_modules_request(&target);
  target_reply_with_empty_schema_modules(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_probe_done(&runtime));
  flush_runtime_to_target(&runtime, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    malformed,
                                                    sizeof(malformed)));
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_TRUE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(&runtime.link,
                                                &service,
                                                payload,
                                                sizeof(payload),
                                                &payload_len));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(malformed), payload_len);
  TEST_ASSERT_EQUAL_MEMORY(malformed, payload, sizeof(malformed));
}

void testControlProbeStateResetsOnRuntimeTransitions(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);
  runtime.control_rx_credit_opened = true;
  runtime.control_request_sent = true;
  runtime.control_probe_done = true;
  runtime.control_rx_release_pending = true;
  runtime.control_rx_malformed = 3;

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_FALSE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(runtime.control_probe_done);
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(0, runtime.control_rx_malformed);

  runtime.control_rx_credit_opened = true;
  runtime.control_request_sent = true;
  runtime.control_probe_done = true;
  runtime.control_rx_release_pending = true;
  runtime.control_rx_malformed = 3;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_FALSE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(runtime.control_probe_done);
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(0, runtime.control_rx_malformed);

  runtime.control_rx_credit_opened = true;
  runtime.control_request_sent = true;
  runtime.control_probe_done = true;
  runtime.control_rx_release_pending = true;
  runtime.control_rx_malformed = 3;

  bccam_uart_runtime_on_bootloader_enter(&runtime);
  TEST_ASSERT_FALSE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(runtime.control_probe_done);
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(0, runtime.control_rx_malformed);
}

void testControlBindingStartsUnboundAndClearsOnTransitions(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_control_service_id(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_CONTROL_PROBE_WAITING_FOR_DISCOVERY,
                        bccam_uart_runtime_control_probe_phase(&runtime));

  runtime.control_service_bound = true;
  runtime.control_service_id = 7;
  runtime.control_rx_credit_opened = true;
  runtime.control_request_sent = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_control_service_id(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_CONTROL_PROBE_WAITING_FOR_DISCOVERY,
                        bccam_uart_runtime_control_probe_phase(&runtime));

  runtime.control_service_bound = true;
  runtime.control_service_id = 7;
  bccam_uart_runtime_on_bootloader_enter(&runtime);
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_control_service_id(&runtime));
}

void testControlProbeUsesServiceIdAdvertisedByControlContract(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 7, .major = 1, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(7, bccam_uart_runtime_control_service_id(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&target, 7));

  target_open_credit_for_service(&target, 7);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);

  target_expect_schema_modules_request_on_service(&target, 7);

  const uint8_t response[] = { 0x81, 0x00, 0x21, 0x01, 0x00 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    7,
                                                    response,
                                                    sizeof(response)));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&target, 7));
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_probe_done(&runtime));

  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&target, 7));
}

void testControlProbeDoesNotTreatNumericServiceOneAsControl(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  capture_t capture = { 0 };
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 0, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.video");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE,
                        bccam_uart_runtime_control_probe_phase(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_flush_tx(&runtime, capture_send, &capture));
  TEST_ASSERT_EQUAL_UINT32(0, capture.calls);
}

void testControlBindingRebuildsAfterFirmwareBootWithDifferentHandle(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t first[] = {
    { .service_id = 7, .major = 1, .minor = 0 },
  };
  bccam_uart_service_entry_t second[] = {
    { .service_id = 3, .major = 1, .minor = 0 },
  };

  set_contract_id(first[0].contract_id, "bitcraze.control");
  set_contract_id(second[0].contract_id, "bitcraze.control");

  establish_runtime_link_with_services(&runtime, &target, first, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(7, bccam_uart_runtime_control_service_id(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  feed_discover_reply_with_services(&runtime, second, 1);
  target_configure_services(&target, second, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(3, bccam_uart_runtime_control_service_id(&runtime));
}

void testControlBindingClearsWhenServiceIdNoLongerAdvertisesControl(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 7, .major = 1, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, services, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(7, bccam_uart_runtime_control_service_id(&runtime));

  set_contract_id(runtime.link.services[0].contract_id, "bitcraze.video");
  runtime.control_probe_done = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_control_service_id(&runtime));
  TEST_ASSERT_FALSE(runtime.control_probe_done);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE,
                        bccam_uart_runtime_control_probe_phase(&runtime));
}

void testControlBindingClearsProbeStateWhenHandleChangesWithoutFirmwareBoot(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t first[] = {
    { .service_id = 7, .major = 1, .minor = 0 },
  };
  bccam_uart_service_entry_t second[] = {
    { .service_id = 3, .major = 1, .minor = 0 },
  };

  set_contract_id(first[0].contract_id, "bitcraze.control");
  set_contract_id(second[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, first, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(7, bccam_uart_runtime_control_service_id(&runtime));
  flush_runtime_to_target(&runtime, &target);

  runtime.control_rx_credit_opened = true;
  runtime.control_request_sent = true;
  runtime.control_probe_done = true;
  runtime.control_rx_release_pending = true;
  runtime.control_rx_malformed = 2;
  runtime.control_schema_module_count = 1;
  target_configure_services(&target, second, 1);
  runtime.link.service_count = 1;
  runtime.link.services[0] = second[0];

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(3, bccam_uart_runtime_control_service_id(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(runtime.control_probe_done);
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(0, runtime.control_rx_malformed);
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_control_schema_module_count(&runtime));
}

void testFirmwareStartupResultWaitsBeforeDiscoveryIsActive(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_WAITING,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testFirmwareStartupResultReportsIncompatibleWhenControlContractIsAbsent(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 1, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.video");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testFirmwareStartupResultReportsIncompatibleWhenControlVersionIsWrong(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 2, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testFirmwareStartupResultReportsIncompatibleForControlDevVersionZero(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 0, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
}

void testFirmwareStartupResultWaitsWhenCompatibleControlIsAdvertisedButProbeIsIncomplete(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 7, .major = 1, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_WAITING,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testFirmwareStartupResultReportsReadyAfterControlProbeCompletes(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 7, .major = 1, .minor = 0 },
  };
  const uint8_t response[] = { 0x81, 0x00, 0x21, 0x01, 0x00 };

  set_contract_id(services[0].contract_id, "bitcraze.control");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  target_open_credit_for_service(&target, 7);
  flush_target_to_runtime(&target, &runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    7,
                                                    response,
                                                    sizeof(response)));
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_READY,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testFirmwareStartupResultReportsAbnormalForLinkFault(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_FAULT;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL,
                        bccam_uart_runtime_firmware_startup_result(&runtime));
}

void testRuntimeAcceptsRawDiscoverReplyEventAndPumpsControlProbeTx(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_runtime_init(&runtime);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_firmware_boot(&runtime));
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        (uint16_t)frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE,
                        bccam_uart_runtime_get_state(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
}

void testRuntimeFaultEventFaultsLink(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
                        bccam_uart_runtime_on_rx_fault(&runtime,
                                                       BCCAM_UART_RX_FAULT_SYNC_LOST));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&runtime));
}

void testRuntimeFaultEventFaultsLinkBeforeDiscoveryCompletes(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_UNINITIALIZED,
                        bccam_uart_runtime_get_state(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
                        bccam_uart_runtime_on_rx_fault(&runtime,
                                                       BCCAM_UART_RX_FAULT_SYNC_LOST));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT, runtime.last_error);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&runtime.link)->link_faults);
}

void testRuntimeFaultBeforeDiscoveryFlushQuiescesQueuedTx(void) {
  bccam_uart_runtime_t runtime;
  capture_t capture = { 0 };

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_TRUE(runtime.link.tx_pending);
  TEST_ASSERT_TRUE(runtime.link.transaction_in_flight);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
                        bccam_uart_runtime_on_rx_fault(&runtime,
                                                       BCCAM_UART_RX_FAULT_SYNC_LOST));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT, runtime.last_error);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_pump_tx(&runtime,
                                                   capture_send,
                                                   &capture));
  TEST_ASSERT_EQUAL_UINT32(0, capture.calls);
  TEST_ASSERT_FALSE(runtime.link.tx_pending);
  TEST_ASSERT_FALSE(runtime.link.transaction_in_flight);
}

void testRuntimeRawFrameCountsRejectedBytesAndRecordsLastError(void) {
  bccam_uart_runtime_t runtime;
  const uint8_t frame[BCCAM_UART_FRAME_OVERHEAD - 1u] = { 0 };

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        sizeof(frame)));

  TEST_ASSERT_EQUAL_UINT16(sizeof(frame), runtime.rx_bytes);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH, runtime.last_error);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&runtime));
}

void testRuntimeRawFrameNullFrameDoesNotCountBytes(void) {
  bccam_uart_runtime_t runtime;

  bccam_uart_runtime_init(&runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
                        bccam_uart_runtime_on_raw_frame(&runtime, NULL, 12));

  TEST_ASSERT_EQUAL_UINT16(0, runtime.rx_bytes);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT, runtime.last_error);
}

void testRuntimePumpTxAliasesFlushTxAndUpdatesCounters(void) {
  bccam_uart_runtime_t runtime;
  capture_t capture = { 0 };

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_firmware_boot(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_pump_tx(&runtime,
                                                   capture_send,
                                                   &capture));

  TEST_ASSERT_EQUAL_UINT32(1, capture.calls);
  TEST_ASSERT_TRUE(capture.length > 0);
  TEST_ASSERT_EQUAL_UINT16(capture.length, runtime.tx_bytes);
  TEST_ASSERT_EQUAL_UINT16(1, runtime.tx_flushes);
}

void testRuntimeRawControlResponseCompletesProbeAndReleasesOnNextStep(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);
  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  flush_runtime_to_target(&runtime, &target);

  target_expect_schema_modules_request(&target);
  target_reply_with_empty_schema_modules(&target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        (uint16_t)frame_len));

  TEST_ASSERT_TRUE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));
}

void testRuntimeRawControlResponseBeforeRequestDoesNotCompleteProbe(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  flush_runtime_to_target(&runtime, &target);

  target_reply_with_empty_schema_modules(&target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        (uint16_t)frame_len));

  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &runtime.link,
                               BCCAM_UART_SERVICE_CONTROL));
}

void testRuntimeStaleControlResponseCannotCompleteProbeAfterTargetCredit(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  capture_t release_capture = { 0 };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  flush_runtime_to_target(&runtime, &target);

  target_reply_with_empty_schema_modules(&target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        (uint16_t)frame_len));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));

  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_control_tx_credit(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_pump_tx(&runtime,
                                                   capture_send,
                                                   &release_capture));
  TEST_ASSERT_EQUAL_UINT32(1, release_capture.calls);
  feed_bytes_to_endpoint(&target, release_capture.bytes, release_capture.length);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));

  flush_runtime_to_target(&runtime, &target);
  target_expect_schema_modules_request(&target);
}

void testRuntimeByteControlResponseBeforeRequestCannotCompleteAfterTargetCredit(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  establish_runtime_link(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_credit_opened);
  TEST_ASSERT_FALSE(runtime.control_request_sent);
  flush_runtime_to_target(&runtime, &target);

  target_reply_with_empty_schema_modules(&target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  feed_bytes_to_runtime(&runtime, frame, frame_len);
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_FALSE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_runtime_get_control_malformed_count(&runtime));

  target_open_control_credit(&target);
  flush_target_to_runtime(&target, &runtime);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_control_tx_credit(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_step_control_probe(&runtime));

  TEST_ASSERT_TRUE(runtime.control_request_sent);
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_probe_done(&runtime));
  TEST_ASSERT_TRUE(runtime.control_rx_release_pending);
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_runtime_get_control_malformed_count(&runtime));

  flush_runtime_to_target(&runtime, &target);
  target_expect_schema_modules_request(&target);
}

void testRuntimeRawFallbackStorageRefusalFaultsLink(void) {
  bccam_uart_runtime_t runtime;
  bccam_uart_link_endpoint_t target;
  bccam_uart_service_entry_t services[] = {
    { .service_id = BCCAM_UART_SERVICE_CONSOLE, .major = 1, .minor = 0 },
  };
  const uint8_t first_payload[] = { 0x11 };
  const uint8_t second_payload[] = { 0x22 };
  uint8_t service = 0;
  uint8_t payload[sizeof(first_payload)] = { 0 };
  uint16_t payload_len = 0;
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  set_contract_id(services[0].contract_id, "bitcraze.console");
  establish_runtime_link_with_services(&runtime, &target, services, 1);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&runtime.link,
                                                           BCCAM_UART_SERVICE_CONSOLE,
                                                           1));
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONSOLE,
                                                    first_payload,
                                                    sizeof(first_payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        (uint16_t)frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(&runtime.link,
                                                &service,
                                                payload,
                                                sizeof(payload),
                                                &payload_len));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONSOLE, service);
  TEST_ASSERT_EQUAL_MEMORY(first_payload, payload, sizeof(first_payload));
  runtime.link.rx_pending = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&runtime.link,
                                                           BCCAM_UART_SERVICE_CONSOLE,
                                                           1));
  flush_runtime_to_target(&runtime, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONSOLE,
                                                    second_payload,
                                                    sizeof(second_payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NO_CREDIT,
                        bccam_uart_runtime_on_raw_frame(&runtime,
                                                        frame,
                                                        (uint16_t)frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NO_CREDIT, runtime.last_error);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_runtime_get_state(&runtime));
}
