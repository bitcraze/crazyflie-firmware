#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_uart_link.h"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_uart_frame.c"

static void pump(bccam_uart_link_endpoint_t *from,
                 bccam_uart_link_endpoint_t *to) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(from, frame, sizeof(frame), &frame_len));

  for (size_t i = 0; i < frame_len; i++) {
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_receive_byte(to, frame[i]));
  }
}

static void receive_frame_expect_final_result(bccam_uart_link_endpoint_t *endpoint,
                                              const uint8_t *frame,
                                              size_t frame_len,
                                              int final_result) {
  for (size_t i = 0; i < frame_len; i++) {
    const int expected = (i == frame_len - 1u) ? final_result : BCCAM_UART_OK;
    TEST_ASSERT_EQUAL_INT(expected, bccam_uart_link_receive_byte(endpoint, frame[i]));
  }
}

static void set_contract_id(uint8_t out[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
                            const char *contract_id) {
  memset(out, 0, BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memcpy(out, contract_id, strlen(contract_id));
}

static void assert_contract_id_equals(
  const uint8_t actual[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
  const char *expected) {
  uint8_t expected_contract_id[BCCAM_UART_SERVICE_CONTRACT_ID_LEN];

  set_contract_id(expected_contract_id, expected);
  TEST_ASSERT_EQUAL_MEMORY(expected_contract_id,
                           actual,
                           BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
}

static void test_write_le16(uint8_t *out, uint16_t value) {
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

void testDiscoveryReplyAdvertisesControlContract(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&target));
  TEST_ASSERT_TRUE(bccam_uart_link_service_is_discovered(&initiator,
                                                         BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_FALSE(bccam_uart_link_service_is_discovered(&initiator,
                                                          BCCAM_UART_SERVICE_CONSOLE));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &initiator,
                               BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_UINT8(1, initiator.service_count);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, initiator.services[0].service_id);
  TEST_ASSERT_EQUAL_UINT8(1, initiator.services[0].major);
  TEST_ASSERT_EQUAL_UINT8(0, initiator.services[0].minor);
  assert_contract_id_equals(initiator.services[0].contract_id,
                            "bitcraze.control");
}

void testUnsolicitedCreditUpdateDuringDiscoveryIsIgnored(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t credit_update[] = {
    (uint8_t)BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    0,
    BCCAM_UART_SERVICE_CONTROL,
    1,
  };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                                credit_update,
                                                sizeof(credit_update),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  receive_frame_expect_final_result(&initiator,
                                    frame,
                                    frame_len,
                                    BCCAM_UART_OK);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_UNINITIALIZED,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT16(0,
                           bccam_uart_link_get_counters(&initiator)
                             ->rx_malformed_management_errors);

  pump(&initiator, &target);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_TRUE(bccam_uart_link_service_is_discovered(&initiator,
                                                         BCCAM_UART_SERVICE_CONTROL));
}

void testDiscoverReplyWireLayoutUsesServiceMajorMinorContractOrder(void) {
  bccam_uart_service_entry_t service = {
    .service_id = BCCAM_UART_SERVICE_CONTROL,
    .major = 0,
    .minor = 0,
  };
  uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  size_t payload_len = 0;
  bccam_uart_link_management_message_t decoded;

  set_contract_id(service.contract_id, "bitcraze.control");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_encode_discover_reply(0x31,
                                                              &service,
                                                              1,
                                                              payload,
                                                              sizeof(payload),
                                                              &payload_len));

  TEST_ASSERT_EQUAL_size_t(6 + BCCAM_UART_SERVICE_CONTRACT_ID_LEN + 3, payload_len);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_DISCOVER_REPLY, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x31, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_FRAME_VERSION, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(1, payload[5]);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, payload[6]);
  TEST_ASSERT_EQUAL_UINT8(0, payload[7]);
  TEST_ASSERT_EQUAL_UINT8(0, payload[8]);
  assert_contract_id_equals(&payload[9], "bitcraze.control");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_decode_management(payload,
                                                          (uint16_t)payload_len,
                                                          &decoded));
  TEST_ASSERT_EQUAL_UINT8(1, decoded.body.discover_reply.service_count);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL,
                          decoded.body.discover_reply.services[0].service_id);
  TEST_ASSERT_EQUAL_UINT8(0, decoded.body.discover_reply.services[0].major);
  TEST_ASSERT_EQUAL_UINT8(0, decoded.body.discover_reply.services[0].minor);
  assert_contract_id_equals(decoded.body.discover_reply.services[0].contract_id,
                            "bitcraze.control");
}

void testDiscoverReplyUses32ByteContractSlotsForThreeServices(void) {
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 1, .minor = 0 },
    { .service_id = 2, .major = 1, .minor = 0 },
    { .service_id = 3, .major = 1, .minor = 0 },
  };
  uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  size_t payload_len = 0;

  set_contract_id(services[0].contract_id, "bitcraze.control");
  set_contract_id(services[1].contract_id, "bitcraze.console");
  set_contract_id(services[2].contract_id, "bitcraze.telem");

  TEST_ASSERT_EQUAL_UINT8(32u, BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  TEST_ASSERT_EQUAL_UINT8(3u, BCCAM_UART_MAX_DISCOVERED_SERVICES);
  TEST_ASSERT_EQUAL_UINT16(128u, BCCAM_UART_MANAGEMENT_MAX_PAYLOAD);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_encode_discover_reply(0x42,
                                                              services,
                                                              3,
                                                              payload,
                                                              sizeof(payload),
                                                              &payload_len));
  TEST_ASSERT_EQUAL_size_t(6 + (3 * (3 + 32)), payload_len);
}

void testDiscoverReplyRejectsFourServices(void) {
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 1, .minor = 0 },
    { .service_id = 2, .major = 1, .minor = 0 },
    { .service_id = 3, .major = 1, .minor = 0 },
    { .service_id = 4, .major = 1, .minor = 0 },
  };
  uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  size_t payload_len = 99;

  set_contract_id(services[0].contract_id, "svc.one");
  set_contract_id(services[1].contract_id, "svc.two");
  set_contract_id(services[2].contract_id, "svc.three");
  set_contract_id(services[3].contract_id, "svc.four");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
                        bccam_uart_link_encode_discover_reply(0x42,
                                                              services,
                                                              4,
                                                              payload,
                                                              sizeof(payload),
                                                              &payload_len));
  TEST_ASSERT_EQUAL_size_t(0, payload_len);
}

void testDiscoverReplyFitsMaximumServiceTableCapacity(void) {
  bccam_uart_service_entry_t services[BCCAM_UART_MAX_DISCOVERED_SERVICES] = {
    { .service_id = 1, .major = 1, .minor = 0 },
    { .service_id = 2, .major = 1, .minor = 0 },
    { .service_id = 3, .major = 1, .minor = 0 },
  };
  uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  size_t payload_len = 0;

  set_contract_id(services[0].contract_id, "svc.one");
  set_contract_id(services[1].contract_id, "svc.two");
  set_contract_id(services[2].contract_id, "svc.three");

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_encode_discover_reply(0x42,
                                                              services,
                                                              BCCAM_UART_MAX_DISCOVERED_SERVICES,
                                                              payload,
                                                              sizeof(payload),
                                                              &payload_len));
  TEST_ASSERT_EQUAL_size_t(6 + (BCCAM_UART_MAX_DISCOVERED_SERVICES *
                               (BCCAM_UART_SERVICE_CONTRACT_ID_LEN + 3)),
                           payload_len);
}

void testFindServiceByContractAndVersionReturnsOpaqueServiceId(void) {
  bccam_uart_link_endpoint_t endpoint;
  uint8_t service_id = 0;
  bccam_uart_service_entry_t services[] = {
    { .service_id = 1, .major = 0, .minor = 0 },
    { .service_id = 7, .major = 0, .minor = 0 },
  };

  set_contract_id(services[0].contract_id, "bitcraze.video");
  set_contract_id(services[1].contract_id, "bitcraze.control");

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;
  endpoint.service_count = 2;
  memcpy(endpoint.services, services, sizeof(services));

  TEST_ASSERT_TRUE(bccam_uart_link_find_service_by_contract_version(
    &endpoint,
    "bitcraze.control",
    0,
    0,
    &service_id));
  TEST_ASSERT_EQUAL_UINT8(7, service_id);
}

void testFindServiceByContractAndVersionRejectsWrongVersion(void) {
  bccam_uart_link_endpoint_t endpoint;
  uint8_t service_id = 0;
  bccam_uart_service_entry_t service = {
    .service_id = 7,
    .major = 1,
    .minor = 0,
  };

  set_contract_id(service.contract_id, "bitcraze.control");

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;
  endpoint.service_count = 1;
  endpoint.services[0] = service;

  TEST_ASSERT_FALSE(bccam_uart_link_find_service_by_contract_version(
    &endpoint,
    "bitcraze.control",
    0,
    0,
    &service_id));
  TEST_ASSERT_EQUAL_UINT8(0, service_id);
}

void testControlCreditUpdateAllowsOneControlFrame(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t ping[] = { 0xA5 };
  uint8_t service = 0;
  uint8_t payload[sizeof(ping)] = { 0 };
  uint16_t payload_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&target,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&initiator,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    ping,
                                                    sizeof(ping)));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));
  pump(&initiator, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(&target,
                                                &service,
                                                payload,
                                                sizeof(payload),
                                                &payload_len));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(ping), payload_len);
  TEST_ASSERT_EQUAL_MEMORY(ping, payload, sizeof(ping));
}

void testManagementCreditUpdateCanExceedNegotiatedNormalPayload(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  uint8_t discover_frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  uint8_t discover_payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
  size_t discover_frame_len = 0;
  size_t discover_payload_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&initiator,
                                                discover_frame,
                                                sizeof(discover_frame),
                                                &discover_frame_len));

  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_LINK_MANAGEMENT, discover_frame[3]);
  discover_payload_len = (size_t)discover_frame[4] |
                         ((size_t)discover_frame[5] << 8);
  TEST_ASSERT_EQUAL_size_t(6, discover_payload_len);
  memcpy(discover_payload,
         &discover_frame[BCCAM_UART_FRAME_HEADER_SIZE],
         discover_payload_len);
  test_write_le16(&discover_payload[4], 1u);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                                discover_payload,
                                                (uint16_t)discover_payload_len,
                                                discover_frame,
                                                sizeof(discover_frame),
                                                &discover_frame_len));

  receive_frame_expect_final_result(&target,
                                    discover_frame,
                                    discover_frame_len,
                                    BCCAM_UART_OK);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&target));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_negotiated_payload(&initiator));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_negotiated_payload(&target));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&target,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_link_get_counters(&initiator)->rx_length_errors);
}

void testCreditUpdateWithNonzeroTransactionFaultsActiveLink(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t malformed_credit_update[] = {
    (uint8_t)BCCAM_UART_LINK_OP_CREDIT_UPDATE,
    7,
    BCCAM_UART_SERVICE_CONTROL,
    1,
  };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                                malformed_credit_update,
                                                sizeof(malformed_credit_update),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  receive_frame_expect_final_result(&initiator,
                                    frame,
                                    frame_len,
                                    BCCAM_UART_ERR_MALFORMED_MANAGEMENT);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&initiator)->link_faults);
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL));
}

void testReleaseRxSlotReplenishesControlCredit(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t ping[] = { 0x42 };
  uint8_t service = 0;
  uint8_t payload[sizeof(ping)] = { 0 };
  uint16_t payload_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&target,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&initiator,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    ping,
                                                    sizeof(ping)));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &target,
                               BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(&target,
                                                &service,
                                                payload,
                                                sizeof(payload),
                                                &payload_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_release_rx_slot(&target,
                                                        BCCAM_UART_SERVICE_CONTROL));

  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &target,
                               BCCAM_UART_SERVICE_CONTROL));
}

void testTargetTreatsDuplicateDiscoverAsRetry(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&target));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_retry_discovery(&initiator));
  pump(&initiator, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&target));
  TEST_ASSERT_EQUAL_UINT16(0, bccam_uart_link_get_counters(&target)->link_faults);
}

static bool dispatch_accepts(void *context,
                             uint8_t service,
                             const uint8_t *payload,
                             uint16_t payload_len) {
  uint8_t *accepted = (uint8_t *)context;
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, service);
  TEST_ASSERT_NOT_NULL(payload);
  TEST_ASSERT_EQUAL_UINT16(1, payload_len);
  TEST_ASSERT_EQUAL_UINT8(0x5A, payload[0]);
  (*accepted)++;
  return true;
}

static bool dispatch_refuses(void *context,
                             uint8_t service,
                             const uint8_t *payload,
                             uint16_t payload_len) {
  (void)context;
  (void)service;
  (void)payload;
  (void)payload_len;
  return false;
}

void testRawFrameReceiveDispatchesNormalFrameDirectly(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_receive_raw_frame(&initiator,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(1, accepted);
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &initiator,
                               BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE,
                        bccam_uart_link_get_state(&initiator));
}

void testDispatchRefusalFaultsLink(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NO_CREDIT,
                        bccam_uart_link_receive_raw_frame(&initiator,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_refuses,
                                                          NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &initiator,
                               BCCAM_UART_SERVICE_CONTROL));
}

void testRawFrameReceiveWithoutDispatchDoesNotConsumeCreditWhenRxSlotPending(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  initiator.rx_pending = true;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NO_CREDIT,
                        bccam_uart_link_receive_raw_frame(&initiator,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          NULL,
                                                          NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &initiator,
                               BCCAM_UART_SERVICE_CONTROL));
}

void testRawFrameReceiveTooShortFaultsLinkAndCountsLengthError(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t frame[BCCAM_UART_FRAME_OVERHEAD - 1u] = { 0 };

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          sizeof(frame),
                                                          dispatch_accepts,
                                                          NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&endpoint));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&endpoint)->rx_length_errors);
}

void testStoreRxPayloadRejectsOversizedPayload(void) {
  bccam_uart_link_endpoint_t endpoint;
  uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD + 1u] = { 0 };

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_link_store_rx_payload(
                          &endpoint,
                          BCCAM_UART_SERVICE_CONTROL,
                          payload,
                          sizeof(payload)));
  TEST_ASSERT_FALSE(endpoint.rx_pending);
  TEST_ASSERT_EQUAL_UINT16(0, endpoint.rx_payload_len);
}

void testRawFrameReceiveEnforcesNegotiatedPayloadLimit(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  initiator.parser.max_payload_len = 0u;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_link_receive_raw_frame(&initiator,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&initiator)->rx_length_errors);
}

void testRawFrameReceiveRejectsConcatenatedFramesWithoutDispatch(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t first_frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  uint8_t raw[2u * BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t first_frame_len = 0;
  size_t second_frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           2));
  pump(&initiator, &target);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                first_frame,
                                                sizeof(first_frame),
                                                &first_frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                raw,
                                                sizeof(raw),
                                                &second_frame_len));
  memcpy(raw + second_frame_len, first_frame, first_frame_len);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_link_receive_raw_frame(
                          &initiator,
                          raw,
                          (uint16_t)(second_frame_len + first_frame_len),
                          dispatch_accepts,
                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&initiator)->rx_length_errors);
}

void testRawFrameReceiveRejectsTrailingNoiseWithoutDispatch(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE + 1u];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                raw,
                                                sizeof(raw),
                                                &frame_len));

  raw[frame_len] = 0x00;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_link_receive_raw_frame(
                          &initiator,
                          raw,
                          (uint16_t)(frame_len + 1u),
                          dispatch_accepts,
                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&initiator)->rx_length_errors);
}

void testRawFrameReceiveRejectsLeadingNoiseWithoutDispatch(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t encoded[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  uint8_t raw[BCCAM_UART_FRAME_MAX_ENCODED_SIZE + 1u];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                encoded,
                                                sizeof(encoded),
                                                &frame_len));

  raw[0] = 0x00;
  memcpy(&raw[1], encoded, frame_len);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
                        bccam_uart_link_receive_raw_frame(
                          &initiator,
                          raw,
                          (uint16_t)(frame_len + 1u),
                          dispatch_accepts,
                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_rx_advertised_credit(
                               &initiator,
                               BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&initiator)->rx_length_errors);
}

void testRawFrameReceiveBadCrcFaultsLinkAndCountsCrcError(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_CONTROL,
                                                payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  frame[frame_len - 1u] ^= 0x01u;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_CRC,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&endpoint));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&endpoint)->rx_crc_errors);
}

void testRawFrameReceiveBadVersionFaultsLinkAndCountsVersionError(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_CONTROL,
                                                payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  frame[2] = (uint8_t)(BCCAM_UART_FRAME_VERSION + 1u);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_VERSION,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&endpoint));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&endpoint)->rx_version_errors);
}

void testRawFrameParseCounterSurvivesLaterByteReceive(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_CONTROL,
                                                payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  frame[frame_len - 1u] ^= 0x01u;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_CRC,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          NULL));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&endpoint)->rx_crc_errors);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_receive_byte(&endpoint, 0x00));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&endpoint)->rx_crc_errors);
}

void testRawFrameReceiveHandlesDiscoveryManagementExchange(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&initiator,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_receive_raw_frame(&target,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&target));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_receive_raw_frame(&initiator,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          NULL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, bccam_uart_link_get_state(&initiator));
  TEST_ASSERT_TRUE(bccam_uart_link_service_is_discovered(&initiator,
                                                         BCCAM_UART_SERVICE_CONTROL));
}

void testRawFrameReceiveWithoutDispatchStoresPayloadAndConsumesCredit(void) {
  bccam_uart_link_endpoint_t initiator;
  bccam_uart_link_endpoint_t target;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  uint8_t received[sizeof(payload)] = { 0 };
  uint8_t service = 0;
  uint16_t received_len = 0;
  size_t frame_len = 0;

  bccam_uart_link_init(&initiator, BCCAM_UART_LINK_ROLE_INITIATOR);
  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_discovery(&initiator));
  pump(&initiator, &target);
  pump(&target, &initiator);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(&initiator,
                                                           BCCAM_UART_SERVICE_CONTROL,
                                                           1));
  pump(&initiator, &target);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_normal(&target,
                                                    BCCAM_UART_SERVICE_CONTROL,
                                                    payload,
                                                    sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(&target,
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_receive_raw_frame(&initiator,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          NULL,
                                                          NULL));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_rx_advertised_credit(
                               &initiator,
                               BCCAM_UART_SERVICE_CONTROL));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_rx(&initiator,
                                                &service,
                                                received,
                                                sizeof(received),
                                                &received_len));
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_CONTROL, service);
  TEST_ASSERT_EQUAL_UINT16(sizeof(payload), received_len);
  TEST_ASSERT_EQUAL_MEMORY(payload, received, sizeof(payload));
}

void testRawFrameReceiveRejectsNormalFrameWhenInactiveWithoutDispatch(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_CONTROL,
                                                payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NOT_ACTIVE,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_UNINITIALIZED,
                        bccam_uart_link_get_state(&endpoint));
}

void testRawFrameReceiveRejectsUnknownServiceWithoutDispatch(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_CONTROL,
                                                payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_UNKNOWN_SERVICE,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&endpoint));
  TEST_ASSERT_EQUAL_UINT16(1, bccam_uart_link_get_counters(&endpoint)->rx_unknown_service_errors);
}

void testRawFrameReceiveRejectsZeroRxCreditWithoutDispatch(void) {
  bccam_uart_link_endpoint_t endpoint;
  const uint8_t payload[] = { 0x5A };
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  uint8_t accepted = 0;

  bccam_uart_link_init(&endpoint, BCCAM_UART_LINK_ROLE_INITIATOR);
  endpoint.state = BCCAM_UART_LINK_ACTIVE;
  endpoint.service_count = 1;
  endpoint.services[0].service_id = BCCAM_UART_SERVICE_CONTROL;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_frame_encode(BCCAM_UART_SERVICE_CONTROL,
                                                payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame),
                                                &frame_len));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_NO_CREDIT,
                        bccam_uart_link_receive_raw_frame(&endpoint,
                                                          frame,
                                                          (uint16_t)frame_len,
                                                          dispatch_accepts,
                                                          &accepted));
  TEST_ASSERT_EQUAL_UINT8(0, accepted);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT,
                        bccam_uart_link_get_state(&endpoint));
}
