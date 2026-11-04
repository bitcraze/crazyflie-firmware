#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_firmware_uart_client.h"
#include "bccam_uart_service.h"
// @MODULE "bccam_bootloader_uart_client.c"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_deck_controller.c"
// @MODULE "bccam_uart_frame.c"
// @MODULE "bccam_uart_link.c"
// @MODULE "bccam_uart_runtime.c"

void bccam_bootloader_uart_client_test_queue_rx(const uint8_t *bytes,
                                                uint32_t length);

void setUp(void) { bccam_uart_service_test_reset(); }
void tearDown(void) {}

static const bccam_firmware_uart_client_test_trace_entry_t *last_uart_send(void) {
  const bccam_firmware_uart_client_test_trace_entry_t *result = NULL;
  for (uint8_t i = 0; i < bccam_firmware_uart_client_test_trace_count(); i++) {
    const bccam_firmware_uart_client_test_trace_entry_t *entry =
      bccam_firmware_uart_client_test_trace_entry(i);
    if (entry != NULL &&
        entry->event == BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND) {
      result = entry;
    }
  }
  TEST_ASSERT_NOT_NULL(result);
  return result;
}

static void service_receive_management(const uint8_t *payload,
                                       uint16_t payload_len) {
  bccam_uart_rx_event_t event = { .type = BCCAM_UART_RX_EVENT_RAW_FRAME };
  size_t frame_len = 0;
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
    bccam_uart_frame_encode_version(1, 0, payload, payload_len,
                                    event.raw_frame.bytes,
                                    sizeof(event.raw_frame.bytes),
                                    &frame_len));
  event.raw_frame.length = (uint16_t)frame_len;
  bccam_uart_service_test_handle_rx_event(&event);
}

static uint8_t uart_send_count(void) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < bccam_firmware_uart_client_test_trace_count(); i++) {
    const bccam_firmware_uart_client_test_trace_entry_t *entry =
      bccam_firmware_uart_client_test_trace_entry(i);
    if (entry != NULL &&
        entry->event == BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND) {
      count++;
    }
  }
  return count;
}

static void establish_service_link(void) {
  bccam_uart_service_test_start_firmware_establishment();
  bccam_uart_service_test_poll_once();
  const bccam_firmware_uart_client_test_trace_entry_t *establish =
    last_uart_send();
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH, establish->bytes[6]);
  const uint8_t reply[] = {
    BCCAM_UART_LINK_OP_ESTABLISH_REPLY, establish->bytes[7],
    BCCAM_UART_ESTABLISH_ACCEPTED, 1, 64, 0
  };
  service_receive_management(reply, sizeof(reply));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
}

void testInitialStateIsUninitialized(void) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_UNINITIALIZED,
                        bccam_uart_service_get_state());
}

void testBootloaderIsActiveOnlyWhileReadyOrBusy(void) {
  TEST_ASSERT_FALSE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_ENTERING));
  TEST_ASSERT_TRUE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_READY));
  TEST_ASSERT_TRUE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_BUSY));
  TEST_ASSERT_FALSE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_COMPLETE));
}

void testCompatibleStartupBecomesFirmwareActive(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_READY);
  bccam_uart_service_test_poll_once();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
}

void testControlZeroZeroIncompatibilityStopsWithoutResetLoop(void) {
  bccam_uart_service_test_startup_recovery_report_t report = {0};
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE);

  bccam_uart_service_test_poll_once();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(100000,
                                                                    &report));
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
}

void testIncompatibleSessionBootNoticeStartsFreshEstablishment(void) {
  bccam_uart_service_test_startup_recovery_report_t report = {0};
  establish_service_link();

  const bccam_firmware_uart_client_test_trace_entry_t *count_request =
    last_uart_send();
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_COUNT,
                          count_request->bytes[6]);
  const uint8_t count_reply[] = {
    BCCAM_UART_LINK_OP_SERVICE_COUNT, count_request->bytes[7], 1
  };
  service_receive_management(count_reply, sizeof(count_reply));

  const bccam_firmware_uart_client_test_trace_entry_t *descriptor_request =
    last_uart_send();
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
                          descriptor_request->bytes[6]);
  const uint8_t descriptor_reply[] = {
    BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR, descriptor_request->bytes[7], 0,
    7, 0, 0, 16,
    'b','i','t','c','r','a','z','e','.','c','o','n','t','r','o','l'
  };
  const uint8_t sends_before_incompatible = uart_send_count();
  service_receive_management(descriptor_reply, sizeof(descriptor_reply));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT8(sends_before_incompatible, uart_send_count());
  TEST_ASSERT_EQUAL_UINT16(1,
    bccam_uart_service_test_incompatible_report_count());

  bccam_uart_service_test_poll_once();
  bccam_uart_service_test_poll_once();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT8(sends_before_incompatible, uart_send_count());
  TEST_ASSERT_EQUAL_UINT16(1,
    bccam_uart_service_test_incompatible_report_count());
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(100000,
                                                                    &report));
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);

  const uint8_t boot_notice[] = { BCCAM_UART_LINK_OP_BOOT_NOTICE, 0 };
  service_receive_management(boot_notice, sizeof(boot_notice));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT8((uint8_t)(sends_before_incompatible + 1u),
                          uart_send_count());
  const bccam_firmware_uart_client_test_trace_entry_t *new_establish =
    last_uart_send();
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
                          new_establish->bytes[6]);
}

void testCompatibleActiveBootNoticeMapsServiceToFreshEstablishment(void) {
  establish_service_link();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
  const uint8_t sends_before_boot = uart_send_count();
  const uint8_t boot_notice[] = { BCCAM_UART_LINK_OP_BOOT_NOTICE, 0 };
  service_receive_management(boot_notice, sizeof(boot_notice));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT8((uint8_t)(sends_before_boot + 1u), uart_send_count());
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_OP_ESTABLISH,
                          last_uart_send()->bytes[6]);
}

void testAbnormalLinkFaultRequestsExternalFirmwareRecovery(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL);
  bccam_uart_service_test_poll_once();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxFaultBeforeEstablishmentDoesNotCreateLinkFault(void) {
  bccam_uart_service_test_start_firmware_establishment();
  const bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_UART_ERROR,
  };
  bccam_uart_service_test_handle_rx_event(&event);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING,
                        bccam_uart_service_get_state());
}

void testBootloaderRequestRemainsAvailableFromIncompatibleState(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);
  bccam_uart_service_test_set_bootloader_enter_result(true);
  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  bccam_uart_service_test_poll_once();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        bccam_uart_service_get_state());
}

void testFlashWriteRequiresReadyBootloader(void) {
  const uint8_t bytes[] = {1, 2, 3, 4};
  TEST_ASSERT_FALSE(bccam_uart_service_write_flash(0, sizeof(bytes), bytes,
                                                    sizeof(bytes)));
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  TEST_ASSERT_TRUE(bccam_uart_service_test_flash_write_allowed());
}

void testPollFailureFormattingKeepsLinkDiagnostics(void) {
  char text[256];
  bccam_firmware_uart_client_observation_t observation = {0};
  observation.link_state = BCCAM_UART_LINK_FAULT;
  observation.control_probe_phase =
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  observation.startup_result = BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL;
  observation.counters.tx_failures = 1;
  observation.counters.link_faults = 1;
  bccam_uart_service_test_format_poll_failure(text, sizeof(text),
                                               BCCAM_UART_ERR_LINK_FAULT,
                                               &observation, true);
  TEST_ASSERT_NOT_NULL(strstr(text, "LINK_FAULT"));
  TEST_ASSERT_NOT_NULL(strstr(text, "FAULT"));
}

void testRxFaultWhileLinkActiveMovesServiceToExternalRecovery(void) {
  establish_service_link();
  const bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };
  bccam_uart_service_test_handle_rx_event(&event);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxFaultEventsAreIgnoredOutsideRxFirmwareStates(void) {
  const bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };
  const bccam_uart_service_state_t states[] = {
    BCCAM_UART_SERVICE_STATE_FW_RESETTING,
    BCCAM_UART_SERVICE_STATE_ISP_READY,
  };
  for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
    bccam_uart_service_test_set_state(states[i]);
    bccam_uart_service_test_handle_rx_event(&event);
    TEST_ASSERT_EQUAL_INT(states[i], bccam_uart_service_get_state());
  }
}

void testRxFaultInTransportAttachedIncompatibleStateTriggersRecovery(void) {
  establish_service_link();
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_poll_once();
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  const bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };
  bccam_uart_service_test_handle_rx_event(&event);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxEventBatchPollsFirmwareOnceAfterDrain(void) {
  const bccam_uart_rx_event_t events[] = {
    { .type = BCCAM_UART_RX_EVENT_FAULT,
      .fault = BCCAM_UART_RX_FAULT_UART_ERROR },
    { .type = BCCAM_UART_RX_EVENT_FAULT,
      .fault = BCCAM_UART_RX_FAULT_SYNC_LOST },
  };
  bccam_uart_service_test_start_firmware_establishment();
  bccam_uart_service_test_handle_rx_events(events, 2);
  TEST_ASSERT_EQUAL_UINT16(1,
    bccam_uart_service_test_rx_post_drain_poll_count());
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING,
                        bccam_uart_service_get_state());
}

void testBootloaderAndFirmwareLifecycleRequestTransitions(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        bccam_uart_service_get_state());

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  TEST_ASSERT_FALSE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        bccam_uart_service_get_state());

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_BUSY);
  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testResetRequestRestartsAllFirmwareLifecycleStates(void) {
  const bccam_uart_service_state_t states[] = {
    BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING,
    BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
    BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
  };
  for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
    bccam_uart_service_test_set_state(states[i]);
    TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
      BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));
    TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                          bccam_uart_service_get_state());
  }
}

void testServiceStatusSnapshotAndCacheTrackState(void) {
  bccam_uart_service_status_t status;
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_READY);
  memset(&status, 0xff, sizeof(status));
  bccam_uart_service_get_status(&status);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        status.service_state);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_ACTIVE, status.link_state);
  TEST_ASSERT_EQUAL_UINT8(1, status.control_probe_done);
  TEST_ASSERT_EQUAL_INT32(BCCAM_UART_OK, status.last_error);

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  bccam_uart_service_get_status(&status);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        status.service_state);
}

void testBootloaderEntrySuspendsFirmwareSession(void) {
  bccam_uart_service_status_t status;
  bccam_uart_service_test_start_firmware_establishment();
  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  bccam_uart_service_get_status(&status);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_INACTIVE, status.link_state);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        status.service_state);
}

void testStartupDeadlineEdgesAndPhaseDiagnostics(void) {
  bccam_uart_service_test_startup_recovery_report_t report = {0};
  bccam_uart_service_test_start_firmware_establishment();
  TEST_ASSERT_EQUAL_STRING(
    "startup stopped while waiting for Camera Link establishment or catalog enumeration.",
    bccam_uart_service_test_startup_phase_message());
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(2999,
                                                                    &report));
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
  TEST_ASSERT_TRUE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                   &report));
  TEST_ASSERT_EQUAL_UINT32(1, report.reset_count);
  TEST_ASSERT_EQUAL_UINT32(3000, report.timeout_ms);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testIncompatibleAndReadyStartupDisableDeadlineRecovery(void) {
  bccam_uart_service_test_startup_recovery_report_t report = {0};
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                    &report));
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_READY);
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(6000,
                                                                    &report));
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
}

void testPollFailureFormattingBoundariesRemainTerminated(void) {
  char one[1] = { 'x' };
  char short_text[8];
  char debug[96];
  memset(short_text, 'x', sizeof(short_text));
  bccam_uart_service_test_format_poll_failure(one, sizeof(one),
                                               BCCAM_UART_ERR_BAD_CRC,
                                               NULL, true);
  bccam_uart_service_test_format_poll_failure(short_text, sizeof(short_text),
                                               BCCAM_UART_ERR_BAD_CRC,
                                               NULL, true);
  bccam_uart_service_test_format_poll_failure(debug, sizeof(debug),
                                               (int)INT32_MIN,
                                               NULL, true);
  TEST_ASSERT_EQUAL_CHAR('\0', one[0]);
  TEST_ASSERT_EQUAL_STRING("UART li", short_text);
  TEST_ASSERT_EQUAL_CHAR('\0', short_text[sizeof(short_text) - 1u]);
  TEST_ASSERT_EQUAL_STRING("UART link poll failed: UNKNOWN (-2147483648)", debug);
}

void testInactiveDiagnosticUsesFinalizedLifecycleName(void) {
  char text[256];
  bccam_firmware_uart_client_observation_t observation = {0};
  observation.link_state = BCCAM_UART_LINK_INACTIVE;
  observation.control_probe_phase =
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  observation.startup_result = BCCAM_UART_FIRMWARE_STARTUP_WAITING;
  bccam_uart_service_test_format_poll_failure(text, sizeof(text),
                                               BCCAM_UART_ERR_BAD_CRC,
                                               &observation, true);
  TEST_ASSERT_NOT_NULL(strstr(text, "link=INACTIVE"));
}
