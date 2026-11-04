#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_uart_frame.h"
#include "bccam_uart_link.h"
// @MODULE "bccam_uart_crc.c"

/* Wire vectors duplicated from Camera Deck firmware
 * 269140661b420547dbbf42b463db47d7dfc10e39. */
static const uint8_t golden_establish_frame[] = {
  0xBC, 0xCD, 0x01, 0x00, 0x06, 0x00,
  0x01, 0x22, 0x01, 0x01, 0x40, 0x00, 0x68, 0x4B
};
static const uint8_t golden_get_service_count_frame[] = {
  0xBC, 0xCD, 0x01, 0x00, 0x02, 0x00, 0x03, 0x22, 0xAB, 0xF7
};
static const uint8_t golden_get_service_descriptor_frame[] = {
  0xBC, 0xCD, 0x01, 0x00, 0x03, 0x00, 0x05, 0x22, 0x00, 0x09, 0x2C
};

static void take_unit(bccam_uart_link_endpoint_t *link,
                      uint8_t *version,
                      uint8_t *service,
                      uint8_t *payload,
                      uint16_t *payload_len) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_take_tx_unit(link, version, service, payload,
                                 BCCAM_UART_NORMAL_MAX_PAYLOAD, payload_len));
  TEST_ASSERT_TRUE(*payload_len > 0u);
}

static void accept_establishment(bccam_uart_link_endpoint_t *link,
                                 uint8_t transaction_id,
                                 uint16_t mtu) {
  const uint8_t reply[] = {
    BCCAM_UART_LINK_OP_ESTABLISH_REPLY, transaction_id,
    BCCAM_UART_ESTABLISH_ACCEPTED, BCCAM_UART_LINK_VERSION_1,
    (uint8_t)(mtu & 0xffu), (uint8_t)(mtu >> 8)
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(link, BCCAM_UART_LINK_VERSION_1,
                                 BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                 reply, sizeof(reply)));
}

static void establish(bccam_uart_link_endpoint_t *link) {
  uint8_t version, service, payload[64];
  uint16_t payload_len;
  bccam_uart_link_init(link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment(link));
  take_unit(link, &version, &service, payload, &payload_len);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_VERSION_1, version);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_SERVICE_LINK_MANAGEMENT, service);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH, payload[0]);
  accept_establishment(link, payload[1], BCCAM_UART_CAMERA_PROFILE_MTU);
}

static void reply_count(bccam_uart_link_endpoint_t *link, uint8_t count) {
  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(link, &version, &service, request, &request_len);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_COUNT, request[0]);
  const uint8_t reply[] = {
    BCCAM_UART_LINK_OP_SERVICE_COUNT, request[1], count
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(link, BCCAM_UART_LINK_VERSION_1, 0,
                                 reply, sizeof(reply)));
}

static void reply_descriptor(bccam_uart_link_endpoint_t *link,
                             uint8_t ordinal,
                             uint8_t handle,
                             uint8_t major,
                             uint8_t minor,
                             const char *contract) {
  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(link, &version, &service, request, &request_len);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR, request[0]);
  TEST_ASSERT_EQUAL_UINT8(ordinal, request[2]);

  const uint8_t contract_len = (uint8_t)strlen(contract);
  uint8_t reply[7u + BCCAM_UART_SERVICE_CONTRACT_ID_MAX_LEN] = {
    BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, request[1], ordinal,
    handle, major, minor, contract_len
  };
  memcpy(&reply[7], contract, contract_len);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(link, BCCAM_UART_LINK_VERSION_1, 0,
                                 reply, (uint16_t)(7u + contract_len)));
}

void testEstablishAndCatalogRequestsMatchPinnedGoldenVectors(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, payload[64], frame[64];
  uint16_t payload_len;
  size_t frame_len;

  bccam_uart_link_init(&link);
  link.next_transaction_id = 0x22;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment(&link));
  take_unit(&link, &version, &service, payload, &payload_len);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(version, service, payload, payload_len,
                                    frame, sizeof(frame), &frame_len));
  TEST_ASSERT_EQUAL_UINT(sizeof(golden_establish_frame), frame_len);
  TEST_ASSERT_EQUAL_MEMORY(golden_establish_frame, frame, frame_len);

  accept_establishment(&link, 0x22, 64);
  link.next_transaction_id = 0x22;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_count(&link));
  take_unit(&link, &version, &service, payload, &payload_len);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(version, service, payload, payload_len,
                                    frame, sizeof(frame), &frame_len));
  TEST_ASSERT_EQUAL_MEMORY(golden_get_service_count_frame, frame, frame_len);

  {
    const uint8_t count_reply[] = { BCCAM_UART_LINK_OP_SERVICE_COUNT, 0x22, 1 };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_link_receive_unit(&link, 1, 0, count_reply, sizeof(count_reply)));
  }
  link.next_transaction_id = 0x22;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  take_unit(&link, &version, &service, payload, &payload_len);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(version, service, payload, payload_len,
                                    frame, sizeof(frame), &frame_len));
  TEST_ASSERT_EQUAL_MEMORY(golden_get_service_descriptor_frame, frame, frame_len);
}

void testCatalogStreamsDescriptorsAndBindsOpaqueControlHandle(void) {
  bccam_uart_link_endpoint_t link;
  bccam_uart_service_descriptor_t descriptor;

  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  reply_count(&link, 2);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  reply_descriptor(&link, 0, 0x31, 1, 4, "vendor.telemetry_long");
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 1));
  reply_descriptor(&link, 1, 0xA5, 1, 0, "bitcraze.control");

  TEST_ASSERT_TRUE(bccam_uart_link_catalog_complete(&link));
  TEST_ASSERT_TRUE(bccam_uart_link_control_binding(&link, &descriptor));
  TEST_ASSERT_EQUAL_UINT8(0xA5, descriptor.handle);
  TEST_ASSERT_EQUAL_UINT8(16, descriptor.contract_id_len);
  TEST_ASSERT_EQUAL_MEMORY("bitcraze.control", descriptor.contract_id, 16);
}

void testExperimentalControlIsDiscoveredButNotBound(void) {
  bccam_uart_link_endpoint_t link;
  bccam_uart_service_descriptor_t descriptor;

  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  reply_count(&link, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  reply_descriptor(&link, 0, 7, 0, 0, "bitcraze.control");

  TEST_ASSERT_TRUE(bccam_uart_link_catalog_complete(&link));
  TEST_ASSERT_FALSE(bccam_uart_link_control_binding(&link, &descriptor));
}

void testOnlyOneAcknowledgedRequestCanBePendingAndEstablishPreemptsIt(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, payload[64];
  uint16_t payload_len;

  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_TRANSACTION_BUSY,
    bccam_uart_link_start_service_count(&link));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment(&link));
  take_unit(&link, &version, &service, payload, &payload_len);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH, payload[0]);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_ESTABLISH, link.pending);
}

void testBootNoticeClearsSessionBindingCreditsAndPendingUnits(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, payload[64];
  uint16_t payload_len;

  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  reply_count(&link, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  reply_descriptor(&link, 0, 9, 1, 0, "bitcraze.control");
  {
    const uint8_t credit[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 9, 1 };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_link_receive_unit(&link, 1, 0, credit, sizeof(credit)));
  }
  TEST_ASSERT_EQUAL_UINT8(1, bccam_uart_link_get_tx_credit(&link, 9));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_send_normal(&link, 9, (const uint8_t *)"x", 1));

  {
    const uint8_t boot[] = { BCCAM_UART_LINK_OP_BOOT_NOTICE, 0 };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_link_receive_unit(&link, 1, 0, boot, sizeof(boot)));
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
  TEST_ASSERT_FALSE(bccam_uart_link_control_binding(&link, NULL));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&link, 9));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_take_tx_unit(&link, &version, &service, payload,
                                 sizeof(payload), &payload_len));
  TEST_ASSERT_EQUAL_UINT16(0, payload_len);
}

void testNegotiatedMtuAndPerServiceCreditAreEnforced(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t payload[65] = {0};

  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  reply_count(&link, 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  reply_descriptor(&link, 0, 0x44, 1, 0, "bitcraze.control");
  {
    const uint8_t credit[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 0x44, 1 };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_link_receive_unit(&link, 1, 0, credit, sizeof(credit)));
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_PAYLOAD_TOO_LONG,
    bccam_uart_link_send_normal(&link, 0x44, payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_send_normal(&link, 0x44, payload, 64));
  TEST_ASSERT_EQUAL_UINT8(0, bccam_uart_link_get_tx_credit(&link, 0x44));
}

void testMalformedDuplicateHandleDescriptorFaultsActiveLink(void) {
  bccam_uart_link_endpoint_t link;
  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  reply_count(&link, 2);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  reply_descriptor(&link, 0, 5, 1, 0, "vendor.one");
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 1));

  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(&link, &version, &service, request, &request_len);
  const uint8_t duplicate[] = {
    BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, request[1], 1, 5, 1, 0, 10,
    'v','e','n','d','o','r','.','t','w','o'
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0, duplicate, sizeof(duplicate)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);
}

void testEstablishmentRejectionStopsInactiveWithoutFault(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, request[64];
  uint16_t request_len;
  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_establishment(&link));
  take_unit(&link, &version, &service, request, &request_len);
  const uint8_t reply[] = {
    BCCAM_UART_LINK_OP_ESTABLISH_REPLY, request[1],
    BCCAM_UART_ESTABLISH_NO_COMMON_LINK_VERSION, 1, 64, 0
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
  TEST_ASSERT_TRUE(link.establishment_rejected);
  TEST_ASSERT_EQUAL_UINT16(0, link.counters.link_faults);
}

static uint8_t begin_descriptor_request(bccam_uart_link_endpoint_t *link,
                                        uint8_t count,
                                        uint8_t ordinal) {
  uint8_t version, service, request[64];
  uint16_t request_len;
  establish(link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_count(link));
  reply_count(link, count);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(link, ordinal));
  take_unit(link, &version, &service, request, &request_len);
  return request[1];
}

static void receive_descriptor_bytes(bccam_uart_link_endpoint_t *link,
                                     uint8_t transaction_id,
                                     uint8_t ordinal,
                                     uint8_t handle,
                                     uint8_t major,
                                     uint8_t minor,
                                     const uint8_t *contract,
                                     uint8_t contract_len,
                                     int expected) {
  uint8_t reply[7u + BCCAM_UART_SERVICE_CONTRACT_ID_MAX_LEN] = {
    BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, transaction_id, ordinal,
    handle, major, minor, contract_len
  };
  if (contract_len > 0u) {
    memcpy(&reply[7], contract, contract_len);
  }
  TEST_ASSERT_EQUAL_INT(expected,
    bccam_uart_link_receive_unit(link, 1, 0, reply,
                                 (uint16_t)(7u + contract_len)));
}

void testDuplicateContractIdsOnDifferentHandlesRemainValid(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t tid = begin_descriptor_request(&link, 2, 0);
  receive_descriptor_bytes(&link, tid, 0, 5, 1, 0,
                           (const uint8_t *)"vendor.one", 10, BCCAM_UART_OK);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 1));
  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(&link, &version, &service, request, &request_len);
  receive_descriptor_bytes(&link, request[1], 1, 6, 1, 0,
                           (const uint8_t *)"vendor.one", 10, BCCAM_UART_OK);
  TEST_ASSERT_TRUE(bccam_uart_link_catalog_complete(&link));
}

void testAlreadySeenDescriptorRequestIsRefusedWithoutQueueingTraffic(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t tid = begin_descriptor_request(&link, 1, 0);
  receive_descriptor_bytes(&link, tid, 0, 5, 1, 0,
                           (const uint8_t *)"vendor.one", 10, BCCAM_UART_OK);
  TEST_ASSERT_FALSE(link.tx_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_NONE, link.pending);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_uart_link_start_service_descriptor(&link, 0));
  TEST_ASSERT_FALSE(link.tx_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_NONE, link.pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, link.state);
}

void testBootstrapPayloadBoundariesEightAndNine(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t payload8[8] = { 0xff, 0 };
  uint8_t payload9[9] = { 0xff, 0 };
  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0, payload8, sizeof(payload8)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_LENGTH,
    bccam_uart_link_receive_unit(&link, 1, 0, payload9, sizeof(payload9)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
}

void testEstablishmentAcceptanceVersionAndMtuBoundaries(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, request[64];
  uint16_t request_len;

  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment_with(&link, 1, 1, 31));
  take_unit(&link, &version, &service, request, &request_len);
  accept_establishment(&link, request[1], 31);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, link.state);
  TEST_ASSERT_EQUAL_UINT16(31, link.negotiated_payload);

  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment_with(&link, 1, 1, 64));
  take_unit(&link, &version, &service, request, &request_len);
  {
    const uint8_t too_small[] = {
      BCCAM_UART_LINK_OP_ESTABLISH_REPLY, request[1],
      BCCAM_UART_ESTABLISH_ACCEPTED, 1, 30, 0
    };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
      bccam_uart_link_receive_unit(&link, 1, 0, too_small, sizeof(too_small)));
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);

  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment_with(&link, 1, 1, 64));
  take_unit(&link, &version, &service, request, &request_len);
  {
    const uint8_t wrong_version[] = {
      BCCAM_UART_LINK_OP_ESTABLISH_REPLY, request[1],
      BCCAM_UART_ESTABLISH_ACCEPTED, 2, 64, 0
    };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
      bccam_uart_link_receive_unit(&link, 1, 0,
                                   wrong_version, sizeof(wrong_version)));
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
}

void testGenericEstablishmentHelperRejectsUnsupportedCapabilities(void) {
  bccam_uart_link_endpoint_t link;
  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_uart_link_start_establishment_with(&link, 2, 2, 64));
  TEST_ASSERT_FALSE(link.tx_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_NONE, link.pending);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_uart_link_start_establishment_with(
      &link, 1, 1, BCCAM_UART_NORMAL_MAX_PAYLOAD + 1u));
  TEST_ASSERT_FALSE(link.tx_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_NONE, link.pending);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_uart_link_start_establishment_with(
      &link, 1, 1, BCCAM_UART_LINK_V1_MIN_MTU - 1u));
  TEST_ASSERT_FALSE(link.tx_pending);
}

void testBothDefinedEstablishmentRejectionsLeaveLinkInactive(void) {
  const uint8_t statuses[] = {
    BCCAM_UART_ESTABLISH_MTU_TOO_SMALL,
    BCCAM_UART_ESTABLISH_NO_COMMON_LINK_VERSION,
  };
  for (size_t i = 0; i < sizeof(statuses); i++) {
    bccam_uart_link_endpoint_t link;
    uint8_t version, service, request[64];
    uint16_t request_len;
    bccam_uart_link_init(&link);
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_link_start_establishment(&link));
    take_unit(&link, &version, &service, request, &request_len);
    const uint8_t reply[] = {
      BCCAM_UART_LINK_OP_ESTABLISH_REPLY, request[1], statuses[i], 1, 64, 0
    };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
      bccam_uart_link_receive_unit(&link, 1, 0, reply, sizeof(reply)));
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
    TEST_ASSERT_TRUE(link.establishment_rejected);
    TEST_ASSERT_EQUAL_UINT8(statuses[i], link.establishment_status);
  }
}

void testDefaultCameraProfileRejectsAcceptedMtu63AndAccepts64(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, request[64];
  uint16_t request_len;

  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment(&link));
  take_unit(&link, &version, &service, request, &request_len);
  {
    const uint8_t reply63[] = {
      BCCAM_UART_LINK_OP_ESTABLISH_REPLY, request[1],
      BCCAM_UART_ESTABLISH_ACCEPTED, 1, 63, 0
    };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
      bccam_uart_link_receive_unit(&link, 1, 0, reply63, sizeof(reply63)));
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);

  bccam_uart_link_init(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment(&link));
  take_unit(&link, &version, &service, request, &request_len);
  accept_establishment(&link, request[1], 64);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, link.state);
  TEST_ASSERT_EQUAL_UINT16(64, link.negotiated_payload);
}

void testCatalogCountZeroAndMaximumOrdinalHandleBoundaries(void) {
  bccam_uart_link_endpoint_t link;
  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  reply_count(&link, 0);
  TEST_ASSERT_TRUE(bccam_uart_link_catalog_complete(&link));

  static const uint8_t max_contract[] = "abcdefghijklmnopqrstuvwx";
  const uint8_t tid = begin_descriptor_request(&link, 255, 254);
  receive_descriptor_bytes(&link, tid, 254, 255, 255, 255,
                           max_contract, 24, BCCAM_UART_OK);
  TEST_ASSERT_EQUAL_UINT8(1, link.descriptors_received);
  TEST_ASSERT_TRUE((link.seen_ordinals[31] & 0x40u) != 0u);
  TEST_ASSERT_TRUE((link.seen_handles[31] & 0x80u) != 0u);
}

void testMinimumAndMaximumContractIdLengthsAreAccepted(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t tid = begin_descriptor_request(&link, 2, 0);
  receive_descriptor_bytes(&link, tid, 0, 1, 0, 0,
                           (const uint8_t *)"a", 1, BCCAM_UART_OK);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 1));
  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(&link, &version, &service, request, &request_len);
  receive_descriptor_bytes(&link, request[1], 1, 255, 1, 0,
                           (const uint8_t *)"abcdefghijklmnopqrstuvwx", 24,
                           BCCAM_UART_OK);
  TEST_ASSERT_TRUE(bccam_uart_link_catalog_complete(&link));
}

void testInvalidDescriptorGrammarVersionCorrelationAndLengthFault(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t tid = begin_descriptor_request(&link, 1, 0);
  {
    const uint8_t empty_id[] = {
      BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, tid, 0, 1, 1, 0, 0
    };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
      bccam_uart_link_receive_unit(&link, 1, 0, empty_id, sizeof(empty_id)));
  }

  tid = begin_descriptor_request(&link, 1, 0);
  {
    uint8_t oversized_id[32] = {
      BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, tid, 0, 1, 1, 0, 25
    };
    memset(&oversized_id[7], 'a', 25);
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
      bccam_uart_link_receive_unit(&link, 1, 0,
                                   oversized_id, sizeof(oversized_id)));
  }

  tid = begin_descriptor_request(&link, 1, 0);
  receive_descriptor_bytes(&link, tid, 0, 0, 1, 0,
                           (const uint8_t *)"a", 1,
                           BCCAM_UART_ERR_MALFORMED_MANAGEMENT);

  tid = begin_descriptor_request(&link, 1, 0);
  receive_descriptor_bytes(&link, tid, 0, 1, 1, 0,
                           (const uint8_t *)".bad", 4,
                           BCCAM_UART_ERR_MALFORMED_MANAGEMENT);

  tid = begin_descriptor_request(&link, 1, 0);
  receive_descriptor_bytes(&link, tid, 0, 1, 0, 1,
                           (const uint8_t *)"a", 1,
                           BCCAM_UART_ERR_MALFORMED_MANAGEMENT);

  tid = begin_descriptor_request(&link, 1, 0);
  receive_descriptor_bytes(&link, (uint8_t)(tid + 1u), 0, 1, 1, 0,
                           (const uint8_t *)"a", 1,
                           BCCAM_UART_ERR_MALFORMED_MANAGEMENT);

  tid = begin_descriptor_request(&link, 1, 0);
  receive_descriptor_bytes(&link, tid, 1, 1, 1, 0,
                           (const uint8_t *)"a", 1,
                           BCCAM_UART_ERR_MALFORMED_MANAGEMENT);

  tid = begin_descriptor_request(&link, 1, 0);
  {
    const uint8_t truncated[] = {
      BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, tid, 0, 1, 1, 0, 2, 'a'
    };
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
      bccam_uart_link_receive_unit(&link, 1, 0,
                                   truncated, sizeof(truncated)));
  }
}

void testKnownServiceCountRequestIsRefusedWithoutQueueingTraffic(void) {
  bccam_uart_link_endpoint_t link;
  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_count(&link));
  reply_count(&link, 1);
  TEST_ASSERT_FALSE(link.tx_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_NONE, link.pending);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_BAD_ARGUMENT,
    bccam_uart_link_start_service_count(&link));
  TEST_ASSERT_FALSE(link.tx_pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_NONE, link.pending);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, link.state);
}

void testMalformedServiceCountTlvFaultsActiveLink(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, request[64];
  uint16_t request_len;
  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK, bccam_uart_link_start_service_count(&link));
  take_unit(&link, &version, &service, request, &request_len);
  const uint8_t malformed[] = {
    BCCAM_UART_LINK_OP_SERVICE_COUNT, request[1], 1, 0x80, 2, 0x11
  };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0,
                                 malformed, sizeof(malformed)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);
}

static uint8_t begin_service_count_request(bccam_uart_link_endpoint_t *link) {
  uint8_t version, service, request[64];
  uint16_t request_len;
  establish(link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_count(link));
  take_unit(link, &version, &service, request, &request_len);
  return request[1];
}

void testUnknownPreCatalogCreditZeroFaultsWhenCountIsZero(void) {
  bccam_uart_link_endpoint_t link;
  const uint8_t tid = begin_service_count_request(&link);
  const uint8_t credit[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 9, 0 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, credit, sizeof(credit)));
  const uint8_t count[] = { BCCAM_UART_LINK_OP_SERVICE_COUNT, tid, 0 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0, count, sizeof(count)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);
}

void testUnknownPreCatalogCreditNonzeroFaultsAfterFinalDescriptor(void) {
  bccam_uart_link_endpoint_t link;
  const uint8_t count_tid = begin_service_count_request(&link);
  const uint8_t credit[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 8, 1 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, credit, sizeof(credit)));
  const uint8_t count[] = { BCCAM_UART_LINK_OP_SERVICE_COUNT, count_tid, 1 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, count, sizeof(count)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(&link, &version, &service, request, &request_len);
  receive_descriptor_bytes(&link, request[1], 0, 9, 1, 0,
                           (const uint8_t *)"vendor.one", 10,
                           BCCAM_UART_ERR_MALFORMED_MANAGEMENT);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);
}

void testKnownPreCatalogCreditHandleRemainsValidAfterFinalDescriptor(void) {
  bccam_uart_link_endpoint_t link;
  const uint8_t count_tid = begin_service_count_request(&link);
  const uint8_t credit[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 9, 0 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, credit, sizeof(credit)));
  const uint8_t count[] = { BCCAM_UART_LINK_OP_SERVICE_COUNT, count_tid, 1 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, count, sizeof(count)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_descriptor(&link, 0));
  uint8_t version, service, request[64];
  uint16_t request_len;
  take_unit(&link, &version, &service, request, &request_len);
  receive_descriptor_bytes(&link, request[1], 0, 9, 1, 0,
                           (const uint8_t *)"vendor.one", 10, BCCAM_UART_OK);
  TEST_ASSERT_TRUE(bccam_uart_link_catalog_complete(&link));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_ACTIVE, link.state);
}

void testCreditBoundariesAndForbiddenDecrease(void) {
  bccam_uart_link_endpoint_t link;
  establish(&link);
  const uint8_t zero[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 1, 0 };
  const uint8_t max[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 1, 127 };
  const uint8_t too_large[] = { BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0, 1, 128 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, zero, sizeof(zero)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, max, sizeof(max)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0,
                                 too_large, sizeof(too_large)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);

  establish(&link);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_receive_unit(&link, 1, 0, max, sizeof(max)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0, zero, sizeof(zero)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);
}

void testNonzeroBootNoticeTransactionFaultsActiveLink(void) {
  bccam_uart_link_endpoint_t link;
  establish(&link);
  const uint8_t invalid[] = { BCCAM_UART_LINK_OP_BOOT_NOTICE, 1 };
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    bccam_uart_link_receive_unit(&link, 1, 0, invalid, sizeof(invalid)));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_FAULT, link.state);
}

void testReestablishmentClearsPendingRxTxCreditAndCatalogState(void) {
  bccam_uart_link_endpoint_t link;
  establish(&link);
  link.service_count_known = true;
  link.service_count = 1;
  link.descriptors_received = 1;
  link.seen_ordinals[0] = 1;
  link.seen_handles[0] = 0x80;
  link.observed_credit_handles[0] = 0x80;
  link.control_binding.valid = true;
  link.control_binding.descriptor.handle = 7;
  link.control_binding.tx_credit = 4;
  link.control_binding.rx_advertised_credit = 1;
  link.rx_pending = true;
  link.rx_service = 7;
  link.rx_payload_len = 1;
  link.tx_pending = true;
  link.tx_payload_len = 1;
  link.pending = BCCAM_UART_PENDING_SERVICE_COUNT;
  link.pending_transaction_id = 9;

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_establishment(&link));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, link.state);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_PENDING_ESTABLISH, link.pending);
  TEST_ASSERT_FALSE(link.rx_pending);
  TEST_ASSERT_FALSE(link.control_binding.valid);
  TEST_ASSERT_FALSE(link.service_count_known);
  TEST_ASSERT_EQUAL_UINT8(0, link.seen_ordinals[0]);
  TEST_ASSERT_EQUAL_UINT8(0, link.seen_handles[0]);
  TEST_ASSERT_EQUAL_UINT8(0, link.observed_credit_handles[0]);
  TEST_ASSERT_EQUAL_UINT8(0, link.pending_tx_credit[7]);
  TEST_ASSERT_TRUE(link.tx_pending);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH, link.tx_payload[0]);
}

void testTransactionIdWrapSkipsZero(void) {
  bccam_uart_link_endpoint_t link;
  uint8_t version, service, request[64];
  uint16_t request_len;
  establish(&link);
  link.next_transaction_id = 255;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_link_start_service_count(&link));
  take_unit(&link, &version, &service, request, &request_len);
  TEST_ASSERT_EQUAL_UINT8(255, request[1]);
  TEST_ASSERT_EQUAL_UINT8(1, link.next_transaction_id);
}

static const uint8_t pinned_incoming_vectors[][39] = {
  { 0xBC,0xCD,0x01,0x01,0x00,0x00,0x44,0xC5 },
  { 0xBC,0xCD,0x01,0x00,0x06,0x00,0x02,0x22,0x00,0x01,0x40,0x00,0x3C,0xF3 },
  { 0xBC,0xCD,0x01,0x00,0x04,0x00,0x08,0x00,0x01,0x01,0x9F,0xD6 },
  { 0xBC,0xCD,0x01,0x00,0x03,0x00,0x04,0x22,0x01,0x18,0x0B },
  { 0xBC,0xCD,0x01,0x00,0x08,0x00,0x06,0x22,0x00,0x01,0x00,0x00,0x01,0x61,0xFE,0xA6 },
  { 0xBC,0xCD,0x01,0x00,0x02,0x00,0x07,0x00,0x4F,0x3F },
  { 0xBC,0xCD,0x01,0x00,0x06,0x00,0x02,0x22,0x01,0x01,0x40,0x00,0x88,0x85 },
  { 0xBC,0xCD,0x01,0x00,0x06,0x00,0x02,0x22,0x02,0x01,0x40,0x00,0x54,0x1E },
  { 0xBC,0xCD,0x01,0x00,0x1F,0x00,0x06,0x22,0xFF,0xFF,0xFF,0xFF,0x18,
    0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,
    0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x6C,0xB1 },
  { 0xBC,0xCD,0x01,0x00,0x06,0x00,0x01,0x22,0x01,0x01,0x40,0x00,0x68,0x03 },
  { 0xBC,0xCD,0x01,0x01,0x01,0x01,0x00,0x00 }
};
static const uint8_t pinned_incoming_lengths[] = { 8, 14, 12, 11, 16, 10, 14, 14, 39, 14, 8 };
static const uint8_t pinned_incoming_services[] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
static const int pinned_incoming_results[] = {
  BCCAM_UART_OK, BCCAM_UART_OK, BCCAM_UART_OK, BCCAM_UART_OK,
  BCCAM_UART_OK, BCCAM_UART_OK, BCCAM_UART_OK, BCCAM_UART_OK,
  BCCAM_UART_OK, BCCAM_UART_ERR_BAD_CRC, BCCAM_UART_ERR_BAD_LENGTH
};

void testRemainingPinnedIncomingFullFrameVectorsDecodeExactly(void) {
  for (size_t vector = 0; vector < sizeof(pinned_incoming_lengths); vector++) {
    bccam_uart_frame_parser_t parser;
    bccam_uart_frame_t frame;
    bool ready = false;
    bccam_uart_frame_parser_init(&parser, 64);
    int result = BCCAM_UART_OK;
    for (uint8_t i = 0; i < pinned_incoming_lengths[vector]; i++) {
      result = bccam_uart_frame_parser_feed(&parser,
        pinned_incoming_vectors[vector][i], &frame, &ready);
      if (result != BCCAM_UART_OK) {
        break;
      }
    }
    TEST_ASSERT_EQUAL_INT(pinned_incoming_results[vector], result);
    if (result == BCCAM_UART_OK) {
      TEST_ASSERT_TRUE(ready);
      TEST_ASSERT_EQUAL_UINT8(1, frame.link_version);
      TEST_ASSERT_EQUAL_UINT8(pinned_incoming_services[vector], frame.service);
    }
  }
}

void testPinnedMaximumNormalFrameEncodesAllPayloadBytes(void) {
  uint8_t payload[256];
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;
  for (size_t i = 0; i < sizeof(payload); i++) {
    payload[i] = (uint8_t)i;
  }
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 1, payload, sizeof(payload),
                                    frame, sizeof(frame), &frame_len));
  TEST_ASSERT_EQUAL_size_t(264, frame_len);
  TEST_ASSERT_EQUAL_UINT8(0xBC, frame[0]);
  TEST_ASSERT_EQUAL_UINT8(0xCD, frame[1]);
  TEST_ASSERT_EQUAL_UINT8(0x00, frame[4]);
  TEST_ASSERT_EQUAL_UINT8(0x01, frame[5]);
  TEST_ASSERT_EQUAL_MEMORY(payload, &frame[6], sizeof(payload));
  TEST_ASSERT_EQUAL_UINT8(0x97, frame[262]);
  TEST_ASSERT_EQUAL_UINT8(0xC3, frame[263]);
}
