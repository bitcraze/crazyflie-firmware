#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_uart_runtime.h"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_uart_frame.c"
// @MODULE "bccam_uart_link.c"

/* Compatible with Camera Deck vectors at
 * 269140661b420547dbbf42b463db47d7dfc10e39. */
typedef struct {
  uint8_t bytes[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t length;
  unsigned calls;
  bool succeed;
} writer_t;

static bool capture_write(void *context, const uint8_t *data, uint32_t length) {
  writer_t *writer = context;
  writer->calls++;
  writer->length = length;
  memcpy(writer->bytes, data, length);
  return writer->succeed;
}

static void flush(bccam_uart_runtime_t *runtime, writer_t *writer) {
  TEST_ASSERT_EQUAL_INT(writer->succeed ? BCCAM_UART_OK : BCCAM_UART_ERR_LINK_FAULT,
    bccam_uart_runtime_flush_tx(runtime, capture_write, writer));
}

static void decode(const writer_t *writer, bccam_uart_frame_t *frame) {
  bccam_uart_frame_parser_t parser;
  bool ready = false;
  bccam_uart_frame_parser_init(&parser, BCCAM_UART_NORMAL_MAX_PAYLOAD);
  for (size_t i = 0; i < writer->length; i++) {
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_frame_parser_feed(&parser, writer->bytes[i], frame, &ready));
  }
  TEST_ASSERT_TRUE(ready);
}

static void feed_management(bccam_uart_runtime_t *runtime,
                            const uint8_t *payload,
                            uint16_t payload_len) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0, payload, payload_len,
                                    frame, sizeof(frame), &frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(runtime, frame, (uint16_t)frame_len));
}

static uint8_t flush_operation(bccam_uart_runtime_t *runtime,
                               writer_t *writer,
                               bccam_uart_frame_t *frame) {
  writer->length = 0;
  flush(runtime, writer);
  decode(writer, frame);
  return frame->payload[0];
}

static void establish_and_get_count(bccam_uart_runtime_t *runtime,
                                    writer_t *writer,
                                    uint8_t service_count) {
  bccam_uart_frame_t frame;
  bccam_uart_runtime_init(runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_firmware_boot(runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
    flush_operation(runtime, writer, &frame));
  const uint8_t establish_reply[] = {
    BCCAM_UART_LINK_OP_ESTABLISH_REPLY, frame.payload[1],
    BCCAM_UART_ESTABLISH_ACCEPTED, 1, 64, 0
  };
  feed_management(runtime, establish_reply, sizeof(establish_reply));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_COUNT,
    flush_operation(runtime, writer, &frame));
  const uint8_t count_reply[] = {
    BCCAM_UART_LINK_OP_SERVICE_COUNT, frame.payload[1], service_count
  };
  feed_management(runtime, count_reply, sizeof(count_reply));
}

static void feed_descriptor_for_last_request(bccam_uart_runtime_t *runtime,
                                             const bccam_uart_frame_t *request,
                                             uint8_t handle,
                                             uint8_t major,
                                             uint8_t minor,
                                             const char *contract) {
  const uint8_t len = (uint8_t)strlen(contract);
  uint8_t reply[7 + BCCAM_UART_SERVICE_CONTRACT_ID_MAX_LEN] = {
    BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR,
    request->payload[1], request->payload[2], handle, major, minor, len
  };
  memcpy(&reply[7], contract, len);
  feed_management(runtime, reply, (uint16_t)(7 + len));
}

void testRuntimeEstablishesThenStreamsCatalogOneDescriptorAtATime(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 2);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[2]);
  feed_descriptor_for_last_request(&runtime, &frame, 0x33, 1, 0, "vendor.video");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[2]);
  feed_descriptor_for_last_request(&runtime, &frame, 0x91,
                                   BCCAM_UART_APISERVICE_CONTRACT_MAJOR, 0, "bitcraze.api");
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0x91, bccam_uart_runtime_apiservice_id(&runtime));
}

/** Verify Console remains consumable while a compatible API Service probe runs. */
void testConsoleFrameRemainsAvailableWhileApiserviceProbeRuns(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 2);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 1,
                                   BCCAM_UART_APISERVICE_CONTRACT_MAJOR, 0, "bitcraze.api");
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 2, 1, 0,
                                   "bitcraze.console");
  TEST_ASSERT_TRUE(bccam_uart_runtime_console_service_bound(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_probe_done(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_open_console_rx(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[2]);

  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t raw_len = 0u;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 2, (const uint8_t *)"deck", 4,
                                    raw, sizeof(raw), &raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(&runtime, raw, (uint16_t)raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_probe_done(&runtime));

  uint8_t payload[64];
  uint16_t payload_len = 0u;
  bool present = false;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_take_console_rx(&runtime, payload, sizeof(payload),
                                       &payload_len, &present));
  TEST_ASSERT_TRUE(present);
  TEST_ASSERT_EQUAL_UINT16(4, payload_len);
  TEST_ASSERT_EQUAL_MEMORY("deck", payload, 4);
}

/** Verify incompatible API Service metadata does not block Console credit. */
void testConsoleOpensWhenAdvertisedApiserviceVersionIsIncompatible(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 2);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 1,
                                   BCCAM_UART_APISERVICE_CONTRACT_MAJOR + 1u, 0, "bitcraze.api");
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 2, 1, 0,
                                   "bitcraze.console");

  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_bound(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_console_service_bound(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
    bccam_uart_runtime_firmware_startup_result(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_open_console_rx(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[2]);
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[3]);
}

void testIncompatibleApiserviceStopsCleanlyAfterDiscoveryWithoutServiceBurst(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 7,
                                   BCCAM_UART_APISERVICE_CONTRACT_MAJOR + 1u, 0, "bitcraze.api");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
    bccam_uart_runtime_firmware_startup_result(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  writer.length = 0;
  flush(&runtime, &writer);
  TEST_ASSERT_EQUAL_size_t(0, writer.length);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE,
    bccam_uart_runtime_get_state(&runtime));
}

void testCreditArrivingBeforeDescriptorIsAppliedToDiscoveredBinding(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 1);
  {
    const uint8_t credit[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 0x72, 1 };
    feed_management(&runtime, credit, sizeof(credit));
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 0x72,
                                   BCCAM_UART_APISERVICE_CONTRACT_MAJOR, 0, "bitcraze.api");
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_apiservice_tx_credit(&runtime));
}

void testEmptyNormalUnitIsTransmittedExactlyOnce(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  runtime.link.active_link_version = 1;
  runtime.link.negotiated_payload = 64;
  runtime.link.apiservice_binding.valid = true;
  runtime.link.apiservice_binding.descriptor.handle = 0x55;
  runtime.link.apiservice_binding.tx_credit = 1;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_send_normal(&runtime.link, 0x55, NULL, 0));
  flush(&runtime, &writer);
  decode(&writer, &frame);
  TEST_ASSERT_EQUAL_UINT8(0x55, frame.service);
  TEST_ASSERT_EQUAL_UINT16(0, frame.payload_len);
  TEST_ASSERT_EQUAL_UINT(1, writer.calls);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_flush_tx(&runtime, capture_write, &writer));
  TEST_ASSERT_EQUAL_UINT(1, writer.calls);
}

void testEmptyIncomingApiserviceUnitIsMalformedAndRxCreditIsReplenished(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t raw_len = 0;

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  runtime.link.active_link_version = 1;
  runtime.link.negotiated_payload = 64;
  runtime.link.service_count_known = true;
  runtime.link.service_count = 1;
  runtime.link.descriptors_received = 1;
  runtime.link.seen_ordinals[0] = 1;
  runtime.next_descriptor_ordinal = 1;
  runtime.link.apiservice_binding.valid = true;
  runtime.link.apiservice_binding.descriptor.handle = 0x55;
  runtime.link.apiservice_binding.rx_advertised_credit = 1;
  runtime.apiservice_bound = true;
  runtime.apiservice_id = 0x55;
  runtime.apiservice_rx_credit_opened = true;
  runtime.apiservice_request_sent = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0x55, NULL, 0,
                                    raw, sizeof(raw), &raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(&runtime, raw, (uint16_t)raw_len));
  TEST_ASSERT_EQUAL_UINT8(0,
    bccam_uart_link_get_rx_advertised_credit(&runtime.link, 0x55));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1,
    bccam_uart_runtime_get_apiservice_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1,
    bccam_uart_link_get_rx_advertised_credit(&runtime.link, 0x55));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(0x55, frame.payload[2]);
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[3]);
}

/** Verify the completed one-shot probe does not invite unconsumed traffic. */
void testSuccessfulOneShotApiserviceProbeKeepsReceiveCreditClosed(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t raw_len = 0;
  const uint8_t response[] = { 0x81, 0x00, 0x21, 0x01, 0x00 };

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  runtime.link.active_link_version = 1;
  runtime.link.negotiated_payload = 64;
  runtime.link.service_count_known = true;
  runtime.link.service_count = 1;
  runtime.link.descriptors_received = 1;
  runtime.link.seen_ordinals[0] = 1;
  runtime.next_descriptor_ordinal = 1;
  runtime.link.apiservice_binding.valid = true;
  runtime.link.apiservice_binding.descriptor.handle = 0x55;
  runtime.link.apiservice_binding.rx_advertised_credit = 1;
  runtime.apiservice_bound = true;
  runtime.apiservice_id = 0x55;
  runtime.apiservice_rx_credit_opened = true;
  runtime.apiservice_request_sent = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0x55, response, sizeof(response),
                                    raw, sizeof(raw), &raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(&runtime, raw, (uint16_t)raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));

  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_probe_done(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0,
    bccam_uart_link_get_rx_advertised_credit(&runtime.link, 0x55));
  writer.length = 0;
  flush(&runtime, &writer);
  TEST_ASSERT_EQUAL_size_t(0, writer.length);
}

void testReestablishRecoversFaultAndKeepsCounters(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  establish_and_get_count(&runtime, &writer, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_LINK_FAULT,
    bccam_uart_runtime_on_rx_fault(&runtime, BCCAM_UART_RX_FAULT_UART_ERROR));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_runtime_reestablish(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE,
    bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1, runtime.link.counters.link_faults);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
    flush_operation(&runtime, &writer, &frame));
}

void testRetryEstablishmentUsesNewTransactionId(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_firmware_boot(&runtime));

  // The first ESTABLISH has not been sent yet, so nothing is queued.
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_retry_establishment(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
    flush_operation(&runtime, &writer, &frame));
  const uint8_t first_id = frame.payload[1];
  TEST_ASSERT_EQUAL_UINT(1, writer.calls);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_retry_establishment(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_NOT_EQUAL(first_id, frame.payload[1]);
}

void testRetryEstablishmentDoesNothingAfterRejection(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
    flush_operation(&runtime, &writer, &frame));
  const uint8_t reply[] = {
    BCCAM_UART_LINK_OP_ESTABLISH_REPLY, frame.payload[1],
    BCCAM_UART_ESTABLISH_NO_COMMON_LINK_VERSION, 1, 64, 0
  };
  feed_management(&runtime, reply, sizeof(reply));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_retry_establishment(&runtime));
  TEST_ASSERT_FALSE(runtime.link.tx_pending);
}

void testSynchronousTxFailureIsFailStopAndNeverRetriedDuringBootstrap(void) {
  bccam_uart_runtime_t runtime;
  writer_t failing = { .succeed = false };
  writer_t later = { .succeed = true };

  bccam_uart_runtime_init(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_firmware_boot(&runtime));
  flush(&runtime, &failing);
  TEST_ASSERT_EQUAL_UINT(1, failing.calls);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
    bccam_uart_runtime_get_state(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1, runtime.link.counters.tx_failures);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_flush_tx(&runtime, capture_write, &later));
  TEST_ASSERT_EQUAL_UINT(0, later.calls);
}

void testSynchronousTxFailureIsFailStopAndNeverRetriedWhileActive(void) {
  bccam_uart_runtime_t runtime;
  writer_t failing = { .succeed = false };
  writer_t later = { .succeed = true };
  const uint8_t payload[] = { 0x42 };

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  runtime.link.active_link_version = 1;
  runtime.link.negotiated_payload = 64;
  runtime.link.apiservice_binding.valid = true;
  runtime.link.apiservice_binding.descriptor.handle = 7;
  runtime.link.apiservice_binding.tx_credit = 1;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_send_normal(&runtime.link, 7, payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, runtime.link.state);
  TEST_ASSERT_TRUE(runtime.link.tx_pending);
  flush(&runtime, &failing);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, runtime.link.state);
  TEST_ASSERT_EQUAL_UINT(1, failing.calls);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_flush_tx(&runtime, capture_write, &later));
  TEST_ASSERT_EQUAL_UINT(0, later.calls);
}

void testBootNoticeTriggersFreshEstablishmentAndInvalidatesBinding(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  (void)flush_operation(&runtime, &writer, &frame);
  feed_descriptor_for_last_request(&runtime, &frame, 0x66,
                                   BCCAM_UART_APISERVICE_CONTRACT_MAJOR, 0, "bitcraze.api");
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_bound(&runtime));

  {
    const uint8_t boot[] = { BCCAM_UART_LINK_OP_BOOT_NOTICE, 0 };
    feed_management(&runtime, boot, sizeof(boot));
  }
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_bound(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
    flush_operation(&runtime, &writer, &frame));
}

void testCompleteRawPacketWithTrailingByteIsBadLengthAndFaultsActive(void) {
  bccam_uart_runtime_t runtime;
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE + 1u];
  size_t raw_len = 0;
  const uint8_t payload[] = { 0x42 };

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  runtime.link.active_link_version = 1;
  runtime.link.negotiated_payload = 64;
  runtime.link.apiservice_binding.valid = true;
  runtime.link.apiservice_binding.descriptor.handle = 7;
  runtime.link.apiservice_binding.rx_advertised_credit = 1;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 7, payload, sizeof(payload),
                                    raw, sizeof(raw), &raw_len));
  raw[raw_len] = 0xaa;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
    bccam_uart_runtime_on_raw_frame(&runtime, raw, (uint16_t)(raw_len + 1u)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, runtime.link.state);
  TEST_ASSERT_FALSE(runtime.link.apiservice_binding.rx_pending);
}

void testCorruptActiveUartPacketEntersLinkFault(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 0);
  const uint8_t bad_crc[] = {
    0xBC, 0xCD, 0x01, 0x00, 0x02, 0x00, 0x07, 0x00, 0x4F, 0x00
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_CRC,
    bccam_uart_runtime_on_raw_frame(&runtime, bad_crc, sizeof(bad_crc)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, runtime.link.state);
  (void)frame;
}

/* API Service mailbox, between the API Service task and bcCamUartTask. */

#define MAILBOX_HANDLE 0x55u

/** A runtime whose API Service probe has just been answered. */
static void probe_done_runtime(bccam_uart_runtime_t *runtime) {
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t raw_len = 0;
  const uint8_t response[] = { 0x81, 0x00, 0x21, 0x01, 0x00 };

  bccam_uart_runtime_init(runtime);
  runtime->link.state = BCCAM_UART_LINK_ACTIVE;
  runtime->link.active_link_version = 1;
  runtime->link.negotiated_payload = 64;
  runtime->link.service_count_known = true;
  runtime->link.service_count = 1;
  runtime->link.descriptors_received = 1;
  runtime->link.seen_ordinals[0] = 1;
  runtime->link.seen_handles[MAILBOX_HANDLE >> 3] =
    (uint8_t)(1u << (MAILBOX_HANDLE & 7u));
  runtime->next_descriptor_ordinal = 1;
  runtime->link.apiservice_binding.valid = true;
  runtime->link.apiservice_binding.descriptor.handle = MAILBOX_HANDLE;
  runtime->link.apiservice_binding.rx_advertised_credit = 1;
  runtime->apiservice_bound = true;
  runtime->apiservice_id = MAILBOX_HANDLE;
  runtime->apiservice_rx_credit_opened = true;
  runtime->apiservice_request_sent = true;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, MAILBOX_HANDLE, response,
                                    sizeof(response), raw, sizeof(raw),
                                    &raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(runtime, raw, (uint16_t)raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_probe(runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_probe_done(runtime));
}

static void feed_apiservice_credit(bccam_uart_runtime_t *runtime, uint8_t credit) {
  const uint8_t update[] = {
    BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, MAILBOX_HANDLE, credit
  };
  feed_management(runtime, update, sizeof(update));
}

static void feed_apiservice_frame(bccam_uart_runtime_t *runtime,
                               const uint8_t *payload, uint16_t len) {
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t raw_len = 0;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, MAILBOX_HANDLE, payload, len,
                                    raw, sizeof(raw), &raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(runtime, raw, (uint16_t)raw_len));
}

static syslink2_apiservice_frame_t apiservice_frame(const uint8_t *bytes, uint8_t len) {
  syslink2_apiservice_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  memcpy(frame.data, bytes, len);
  frame.len = len;
  return frame;
}

void testMailboxIsNotReadyUntilPublishedAfterProbe(void) {
  bccam_uart_runtime_t runtime;
  probe_done_runtime(&runtime);
  const uint8_t bytes[] = { 0x01, 1, 1, 'q' };
  const syslink2_apiservice_frame_t frame = apiservice_frame(bytes, sizeof(bytes));

  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_ready(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NOT_ACTIVE,
    bccam_uart_runtime_send_apiservice(&runtime, &frame));
  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  TEST_ASSERT_EQUAL_UINT8(0, runtime.apiservice_mailbox.grants_pending);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_ready(&runtime));
}

void testMailboxGrantIsSentAsCreditUpdate(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);

  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(MAILBOX_HANDLE, frame.payload[2]);
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[3]);
  TEST_ASSERT_EQUAL_UINT8(0, runtime.apiservice_mailbox.grants_pending);
}

void testMailboxGrantStaysPendingWhileLinkSlotIsBusy(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);

  runtime.link.tx_pending = true;
  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1, runtime.apiservice_mailbox.grants_pending);
  runtime.link.tx_pending = false;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, runtime.apiservice_mailbox.grants_pending);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    flush_operation(&runtime, &writer, &frame));
}

void testMailboxFrameWaitsForCreditThenIsSent(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;
  const uint8_t bytes[] = { 0x01, 1, 1, 'q' };
  const syslink2_apiservice_frame_t request = apiservice_frame(bytes, sizeof(bytes));
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_send_apiservice(&runtime, &request));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_TRANSACTION_BUSY,
    bccam_uart_runtime_send_apiservice(&runtime, &request));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_TRUE(runtime.apiservice_mailbox.tx_full);
  TEST_ASSERT_FALSE(runtime.link.tx_pending);

  feed_apiservice_credit(&runtime, 1);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_apiservice_tx_credit(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_FALSE(runtime.apiservice_mailbox.tx_full);
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_runtime_apiservice_tx_credit(&runtime));
  writer.length = 0;
  flush(&runtime, &writer);
  decode(&writer, &frame);
  TEST_ASSERT_EQUAL_UINT8(MAILBOX_HANDLE, frame.service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(bytes), frame.payload_len);
  TEST_ASSERT_EQUAL_MEMORY(bytes, frame.payload, sizeof(bytes));
}

void testMailboxRejectsFrameOverMtu(void) {
  bccam_uart_runtime_t runtime;
  syslink2_apiservice_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  probe_done_runtime(&runtime);
  runtime.link.negotiated_payload = 31;
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  frame.len = 32;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_PAYLOAD_TOO_LONG,
    bccam_uart_runtime_send_apiservice(&runtime, &frame));
  frame.len = SYSLINK2_APISERVICE_FRAME_MAX_LEN + 1u;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_PAYLOAD_TOO_LONG,
    bccam_uart_runtime_send_apiservice(&runtime, &frame));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_uart_runtime_send_apiservice(&runtime, NULL));
}

void testMailboxReceivedFrameIsStagedAndTakenOnce(void) {
  bccam_uart_runtime_t runtime;
  syslink2_apiservice_frame_t frame;
  const uint8_t answer[] = { 0x80, 3 };
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  runtime.link.tx_pending = false;

  TEST_ASSERT_FALSE(bccam_uart_runtime_take_apiservice_rx(&runtime, &frame));
  feed_apiservice_frame(&runtime, answer, sizeof(answer));
  bccam_uart_runtime_stage_apiservice_rx(&runtime);
  TEST_ASSERT_TRUE(bccam_uart_runtime_take_apiservice_rx(&runtime, &frame));
  TEST_ASSERT_EQUAL_UINT8(2, frame.len);
  TEST_ASSERT_EQUAL_MEMORY(answer, frame.data, sizeof(answer));
  TEST_ASSERT_FALSE(bccam_uart_runtime_take_apiservice_rx(&runtime, &frame));
  /* Taking does not grant: receive credit is used up until a grant. */
  TEST_ASSERT_EQUAL_UINT8(0,
    bccam_uart_link_get_rx_advertised_credit(&runtime.link, MAILBOX_HANDLE));
}

void testMailboxAnswerIsStagedBeforeLaterCreditUpdate(void) {
  bccam_uart_runtime_t runtime;
  syslink2_apiservice_frame_t frame;
  const uint8_t answer[] = { 0x80, 3 };
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  runtime.link.tx_pending = false;

  /* As dispatch_received_frames does after every RX event. */
  feed_apiservice_frame(&runtime, answer, sizeof(answer));
  bccam_uart_runtime_stage_apiservice_rx(&runtime);
  feed_apiservice_credit(&runtime, 1);
  bccam_uart_runtime_stage_apiservice_rx(&runtime);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_apiservice_tx_credit(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_take_apiservice_rx(&runtime, &frame));
}

void testMailboxSessionResetClearsAndReportsReadyLostOnce(void) {
  bccam_uart_runtime_t runtime;
  syslink2_apiservice_frame_t frame;
  const uint8_t bytes[] = { 0x01, 1, 1, 'q' };
  const syslink2_apiservice_frame_t request = apiservice_frame(bytes, sizeof(bytes));
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_ready(&runtime));
  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_send_apiservice(&runtime, &request));
  runtime.apiservice_mailbox.rx_full = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0, runtime.apiservice_mailbox.grants_pending);
  TEST_ASSERT_FALSE(runtime.apiservice_mailbox.tx_full);
  TEST_ASSERT_FALSE(bccam_uart_runtime_take_apiservice_rx(&runtime, &frame));
  bccam_uart_runtime_grant_apiservice_rx_credit(&runtime);
  TEST_ASSERT_EQUAL_UINT8(0, runtime.apiservice_mailbox.grants_pending);
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_ready(&runtime));
}

void testMailboxReadyLostIsSeenEvenWhenReadyComesBackFirst(void) {
  bccam_uart_runtime_t runtime;
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_ready(&runtime));

  /* The session ends and a new one becomes ready before the API Service task
   * looks again. */
  probe_done_runtime(&runtime);
  runtime.apiservice_mailbox.ready_lost_unobserved = false;
  runtime.apiservice_mailbox.ready = true;
  bccam_uart_runtime_on_firmware_boot(&runtime);
  const bccam_uart_apiservice_mailbox_t lost = runtime.apiservice_mailbox;
  probe_done_runtime(&runtime);
  runtime.apiservice_mailbox.ready_lost_unobserved = lost.ready_lost_unobserved;
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);

  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_ready(&runtime));
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_ready(&runtime));
}

void testMailboxLinkFaultPublishesNotReady(void) {
  bccam_uart_runtime_t runtime;
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_ready(&runtime));
  runtime.link.state = BCCAM_UART_LINK_FAULT;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_ready(&runtime));
}

void testMailboxClosedStaysNotReadyUntilNewSession(void) {
  bccam_uart_runtime_t runtime;
  const uint8_t bytes[] = { 0x01, 1, 1, 'q' };
  const syslink2_apiservice_frame_t request = apiservice_frame(bytes, sizeof(bytes));
  probe_done_runtime(&runtime);
  bccam_uart_runtime_step_apiservice_mailbox(&runtime);
  TEST_ASSERT_TRUE(bccam_uart_runtime_apiservice_ready(&runtime));

  bccam_uart_runtime_close_apiservice(&runtime);
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_ready(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NOT_ACTIVE,
    bccam_uart_runtime_send_apiservice(&runtime, &request));
  /* The old session still looks ready to the Link. */
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_apiservice_mailbox(&runtime));
  TEST_ASSERT_FALSE(bccam_uart_runtime_apiservice_ready(&runtime));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_firmware_boot(&runtime));
  TEST_ASSERT_FALSE(runtime.apiservice_mailbox.closed);
}
