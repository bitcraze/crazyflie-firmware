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
    bccam_uart_runtime_step_control_probe(runtime));
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
    bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[2]);
  feed_descriptor_for_last_request(&runtime, &frame, 0x33, 1, 0, "vendor.video");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[2]);
  feed_descriptor_for_last_request(&runtime, &frame, 0x91, 1, 0,
                                   "bitcraze.control");
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_UINT8(0x91, bccam_uart_runtime_control_service_id(&runtime));
}

void testControlZeroZeroStopsCleanlyAfterDiscoveryWithoutServiceBurst(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  establish_and_get_count(&runtime, &writer, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 7, 0, 0,
                                   "bitcraze.control");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
    bccam_uart_runtime_firmware_startup_result(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_control_probe(&runtime));
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
    bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    flush_operation(&runtime, &writer, &frame));
  feed_descriptor_for_last_request(&runtime, &frame, 0x72, 1, 0,
                                   "bitcraze.control");
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_runtime_control_tx_credit(&runtime));
}

void testEmptyNormalUnitIsTransmittedExactlyOnce(void) {
  bccam_uart_runtime_t runtime;
  writer_t writer = { .succeed = true };
  bccam_uart_frame_t frame;

  bccam_uart_runtime_init(&runtime);
  runtime.link.state = BCCAM_UART_LINK_ACTIVE;
  runtime.link.active_link_version = 1;
  runtime.link.negotiated_payload = 64;
  runtime.link.control_binding.valid = true;
  runtime.link.control_binding.descriptor.handle = 0x55;
  runtime.link.control_binding.tx_credit = 1;
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

void testEmptyIncomingControlUnitIsMalformedAndRxCreditIsReplenished(void) {
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
  runtime.link.control_binding.valid = true;
  runtime.link.control_binding.descriptor.handle = 0x55;
  runtime.link.control_binding.rx_advertised_credit = 1;
  runtime.control_service_bound = true;
  runtime.control_service_id = 0x55;
  runtime.control_rx_credit_opened = true;
  runtime.control_request_sent = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0x55, NULL, 0,
                                    raw, sizeof(raw), &raw_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_on_raw_frame(&runtime, raw, (uint16_t)raw_len));
  TEST_ASSERT_EQUAL_UINT8(0,
    bccam_uart_link_get_rx_advertised_credit(&runtime.link, 0x55));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_control_probe(&runtime));
  TEST_ASSERT_EQUAL_UINT16(1,
    bccam_uart_runtime_get_control_malformed_count(&runtime));
  TEST_ASSERT_EQUAL_UINT8(1,
    bccam_uart_link_get_rx_advertised_credit(&runtime.link, 0x55));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    flush_operation(&runtime, &writer, &frame));
  TEST_ASSERT_EQUAL_UINT8(0x55, frame.payload[2]);
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[3]);
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
  runtime.link.control_binding.valid = true;
  runtime.link.control_binding.descriptor.handle = 7;
  runtime.link.control_binding.tx_credit = 1;
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
    bccam_uart_runtime_step_control_probe(&runtime));
  (void)flush_operation(&runtime, &writer, &frame);
  feed_descriptor_for_last_request(&runtime, &frame, 0x66, 1, 0,
                                   "bitcraze.control");
  TEST_ASSERT_TRUE(bccam_uart_runtime_control_service_bound(&runtime));

  {
    const uint8_t boot[] = { BCCAM_UART_LINK_OP_BOOT_NOTICE, 0 };
    feed_management(&runtime, boot, sizeof(boot));
  }
  TEST_ASSERT_FALSE(bccam_uart_runtime_control_service_bound(&runtime));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_runtime_step_control_probe(&runtime));
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
  runtime.link.control_binding.valid = true;
  runtime.link.control_binding.descriptor.handle = 7;
  runtime.link.control_binding.rx_advertised_credit = 1;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 7, payload, sizeof(payload),
                                    raw, sizeof(raw), &raw_len));
  raw[raw_len] = 0xaa;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
    bccam_uart_runtime_on_raw_frame(&runtime, raw, (uint16_t)(raw_len + 1u)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, runtime.link.state);
  TEST_ASSERT_FALSE(runtime.link.rx_pending);
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
