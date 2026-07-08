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

void setUp(void) {
  bccam_uart_service_test_reset();
}

void tearDown(void) {
}

static void set_service_contract(uint8_t out[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
                                 const char *contract_id) {
  TEST_ASSERT_TRUE(strlen(contract_id) <= BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memset(out, 0, BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
  memcpy(out, contract_id, strlen(contract_id));
}

static void queue_bootloader_ok(void) {
  const uint8_t ok[] = { 'O', 'K' };
  bccam_bootloader_uart_client_test_queue_rx(ok, sizeof(ok));
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

typedef struct {
  uint8_t bytes[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  uint16_t length;
} test_rx_frame_t;

static void target_take_tx_frame(bccam_uart_link_endpoint_t *target,
                                 test_rx_frame_t *frame_out) {
  size_t frame_len = 0;

  TEST_ASSERT_NOT_NULL(frame_out);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_take_tx(target,
                                                frame_out->bytes,
                                                sizeof(frame_out->bytes),
                                                &frame_len));
  TEST_ASSERT_TRUE(frame_len > 0);
  frame_out->length = (uint16_t)frame_len;
}

void testInitialStateIsUninitialized(void) {
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_UNINITIALIZED,
                        bccam_uart_service_get_state());
}

void testServiceEntersFirmwareModeThroughFirmwareClient(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_RESETTING);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_READY);

  bccam_uart_service_test_poll_once();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
}

void testServiceEntersBootloaderModeThroughBootloaderClient(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_bootloader_enter_result(true);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  bccam_uart_service_test_poll_once();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        bccam_uart_service_get_state());
}

void testIncompatibleFirmwareStopsAutomaticRecoveryButAllowsBootloaderRequest(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_DISCOVERY);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_set_bootloader_enter_result(true);

  bccam_uart_service_test_poll_once();
  bccam_uart_service_test_poll_once();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  bccam_uart_service_test_poll_once();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        bccam_uart_service_get_state());
}

void testAbnormalFirmwareStartupRequestsFirmwareRetry(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_DISCOVERY);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL);

  bccam_uart_service_test_poll_once();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxFaultEventMovesFirmwareStartupTowardResetting(void) {
  bccam_uart_service_test_reset();
  bccam_uart_service_test_start_firmware_discovery();

  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };

  bccam_uart_service_test_handle_rx_event(&event);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxFaultEventIsIgnoredInIspState(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);

  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };

  bccam_uart_service_test_handle_rx_event(&event);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        bccam_uart_service_get_state());
}

void testRxFaultEventMovesFirmwareActiveTowardResetting(void) {
  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };

  bccam_uart_service_test_handle_rx_event(&event);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxEventBatchPollsFirmwareLinkOnceAfterDrain(void) {
  bccam_uart_link_endpoint_t target;
  test_rx_frame_t frames[2];

  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_poll_once();

  const bccam_firmware_uart_client_test_trace_entry_t *discover =
    trace_entry(find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                 (uint8_t)-1));
  feed_bytes_to_endpoint(&target, discover->bytes, discover->length);
  target_take_tx_frame(&target, &frames[0]);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_OK,
                        bccam_uart_link_send_credit_update(
                          &target,
                          BCCAM_UART_SERVICE_CONTROL,
                          1));
  target_take_tx_frame(&target, &frames[1]);

  bccam_uart_service_test_handle_rx_frames(frames[0].bytes,
                                           frames[0].length,
                                           frames[1].bytes,
                                           frames[1].length);

  TEST_ASSERT_EQUAL_UINT16(
    1,
    bccam_uart_service_test_rx_post_drain_poll_count());
}

void testCompactRawFrameEventUsesPoolAndReleasesSlot(void) {
  bccam_uart_link_endpoint_t target;
  test_rx_frame_t frame;
  uint8_t slot = BCCAM_UART_RX_SLOT_INVALID;

  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_poll_once();

  const bccam_firmware_uart_client_test_trace_entry_t *discover =
    trace_entry(find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                 (uint8_t)-1));
  feed_bytes_to_endpoint(&target, discover->bytes, discover->length);
  target_take_tx_frame(&target, &frame);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_rx_frame_via_pool(
    frame.bytes,
    frame.length,
    &slot));
  TEST_ASSERT_NOT_EQUAL_UINT8(BCCAM_UART_RX_SLOT_INVALID, slot);
  TEST_ASSERT_FALSE(bccam_uart_service_test_rx_frame_slot_in_use(slot));
}

void testRxDrainHandlesDequeuedRawFrameBeforeQueueOverflow(void) {
  bccam_uart_link_endpoint_t target;
  test_rx_frame_t frame;
  uint8_t slot = BCCAM_UART_RX_SLOT_INVALID;

  bccam_uart_link_init(&target, BCCAM_UART_LINK_ROLE_TARGET);
  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_poll_once();

  const bccam_firmware_uart_client_test_trace_entry_t *discover =
    trace_entry(find_event_after(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
                                 (uint8_t)-1));
  feed_bytes_to_endpoint(&target, discover->bytes, discover->length);
  target_take_tx_frame(&target, &frame);

  TEST_ASSERT_TRUE(
    bccam_uart_service_test_handle_rx_frame_via_pool_then_queue_overflow(
      frame.bytes,
      frame.length,
      &slot));
  TEST_ASSERT_NOT_EQUAL_UINT8(BCCAM_UART_RX_SLOT_INVALID, slot);
  TEST_ASSERT_FALSE(bccam_uart_service_test_rx_frame_slot_in_use(slot));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testInvalidCompactRawFrameSlotMovesFirmwareTowardResetting(void) {
  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  const bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_RAW_FRAME,
    .raw_frame = {
      .slot = BCCAM_UART_RX_SLOT_INVALID,
      .length = 1,
    },
  };

  bccam_uart_service_test_handle_rx_event(&event);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testRxQueueResetClearsFramePoolState(void) {
  uint8_t slot = BCCAM_UART_RX_SLOT_INVALID;

  TEST_ASSERT_TRUE(bccam_uart_service_test_reserve_rx_frame_slot(&slot));
  TEST_ASSERT_NOT_EQUAL_UINT8(BCCAM_UART_RX_SLOT_INVALID, slot);
  TEST_ASSERT_TRUE(bccam_uart_service_test_rx_frame_slot_in_use(slot));

  bccam_uart_service_test_reset_rx_queue_state();

  TEST_ASSERT_FALSE(bccam_uart_service_test_rx_frame_slot_in_use(slot));
}

void testRxFaultEventIsIgnoredWhileFirmwareIsResetting(void) {
  bccam_uart_service_status_t status;

  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_RESETTING);

  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };

  bccam_uart_service_test_handle_rx_event(&event);
  bccam_uart_service_get_status(&status);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT16(0, status.link_faults);
}

void testRxFaultEventIsIgnoredAfterResetToFirmwareRequest(void) {
  bccam_uart_service_status_t status;

  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_BUSY);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));

  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };

  bccam_uart_service_test_handle_rx_event(&event);
  bccam_uart_service_get_status(&status);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT16(0, status.link_faults);
}

void testRxFaultEventIsIgnoredInFirmwareIncompatibleState(void) {
  bccam_uart_service_test_start_firmware_discovery();
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);

  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_SYNC_LOST,
  };

  bccam_uart_service_test_handle_rx_event(&event);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
}

void testSubmitRxEventFromIsrRejectsNullEventInUnitTest(void) {
  BaseType_t higher_priority_task_woken = pdFALSE;

  TEST_ASSERT_FALSE(bccam_uart_service_submit_rx_event_from_isr(
    NULL,
    &higher_priority_task_woken));
}

void testFlashWriteRejectedUnlessServiceStateIsIspReady(void) {
  const uint8_t data[4] = { 1, 2, 3, 4 };

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  TEST_ASSERT_FALSE(bccam_uart_service_write_flash(0x23000000,
                                                   sizeof(data),
                                                   data,
                                                   sizeof(data)));

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  TEST_ASSERT_TRUE(bccam_uart_service_test_flash_write_allowed());
}

void testBootloaderRequestFromFirmwareActiveEntersIspEntering(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        bccam_uart_service_get_state());
}

void testBootloaderRequestFromFirmwareIncompatibleEntersIspEntering(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        bccam_uart_service_get_state());
}

void testBootloaderRequestFromIspReadyIsRejected(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);

  TEST_ASSERT_FALSE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        bccam_uart_service_get_state());
}

void testFirmwareRequestFromIspBusyEntersFwResetting(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_BUSY);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testResetToFirmwareRequestFromFirmwareActiveRestartsFirmwareBoot(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testResetToFirmwareRequestFromFirmwareDiscoveryRestartsFirmwareBoot(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_DISCOVERY);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testResetToFirmwareRequestFromFirmwareIncompatibleRetriesFirmwareBoot(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testFirmwareIncompatibleIsNotPolledAsActiveFirmwareState(void) {
  bccam_uart_service_test_startup_recovery_report_t report;

  memset(&report, 0, sizeof(report));
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);

  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                    &report));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
}

void testPublicBootloaderRequestDoesNotRunInlineWithoutQueue(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  bccam_uart_service_request_bootloader();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
}

void testPublicBootloaderRequestPersistsUntilServiceHandlesIt(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  bccam_uart_service_request_bootloader();

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_pending_mode_request());
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        bccam_uart_service_get_state());
  TEST_ASSERT_FALSE(bccam_uart_service_test_handle_pending_mode_request());
}

void testPublicModeRequestsUseLastRequestWins(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  bccam_uart_service_request_bootloader();
  bccam_uart_service_request_firmware();

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_pending_mode_request());
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
}

void testFlashWriteOnlyAcceptedInIspReady(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  TEST_ASSERT_FALSE(bccam_uart_service_test_flash_write_allowed());

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  TEST_ASSERT_TRUE(bccam_uart_service_test_flash_write_allowed());
}

void testBootloaderActivePropertyIsOnlySetAfterBootloaderEntryCompletes(void) {
  TEST_ASSERT_FALSE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_ENTERING));
  TEST_ASSERT_TRUE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_READY));
  TEST_ASSERT_TRUE(bccam_uart_service_state_has_active_bootloader(
    BCCAM_UART_SERVICE_STATE_ISP_BUSY));
}

void testFinalFlashWriteRequiresFreshBootloaderEntryBeforeNextFlash(void) {
  const uint8_t image[4] = { 1, 2, 3, 4 };

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  queue_bootloader_ok();
  queue_bootloader_ok();

  TEST_ASSERT_TRUE(bccam_uart_service_write_flash(0x23000000,
                                                  sizeof(image),
                                                  image,
                                                  sizeof(image)));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_COMPLETE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_FALSE(bccam_uart_service_test_flash_write_allowed());
  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        bccam_uart_service_get_state());
}

void testPublicFlashWriteReportsCopiedRequestResult(void) {
  const uint8_t image[4] = { 1, 2, 3, 4 };

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  queue_bootloader_ok();
  queue_bootloader_ok();

  TEST_ASSERT_TRUE(bccam_uart_service_write_flash(0x23000000,
                                                  sizeof(image),
                                                  image,
                                                  sizeof(image)));
}

void testPublicFlashReadReportsCopiedRequestResult(void) {
  uint8_t read_buffer[4];
  const uint8_t response[] = { 'O', 'K', 4, 0, 0xAA, 0xBB, 0xCC, 0xDD };

  memset(read_buffer, 0, sizeof(read_buffer));
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  bccam_bootloader_uart_client_test_queue_rx(response, sizeof(response));

  TEST_ASSERT_TRUE(bccam_uart_service_read_flash(0x23000000,
                                                 sizeof(read_buffer),
                                                 read_buffer));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&response[4],
                                read_buffer,
                                sizeof(read_buffer));
}

void testFirmwareIncompatibleIsNotIspReadyForFlashWrites(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE);

  TEST_ASSERT_FALSE(bccam_uart_service_test_flash_write_allowed());
}

void testFlashWriteRejectedOutsideIspReady(void) {
  const uint8_t data[4] = { 1, 2, 3, 4 };

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);

  TEST_ASSERT_FALSE(bccam_uart_service_write_flash(0x23000000,
                                                   sizeof(data),
                                                   data,
                                                   sizeof(data)));
}

void testServiceStatusSnapshotContainsStateLinkCountersAndControlProgress(void) {
  bccam_uart_service_status_t status;

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_READY);

  memset(&status, 0xff, sizeof(status));
  bccam_uart_service_get_status(&status);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        status.service_state);
  TEST_ASSERT_EQUAL_UINT8(BCCAM_UART_LINK_ACTIVE, status.link_state);
  TEST_ASSERT_EQUAL_UINT8(0, status.tx_credit);
  TEST_ASSERT_EQUAL_UINT8(0, status.rx_slots);
  TEST_ASSERT_EQUAL_UINT16(BCCAM_UART_NORMAL_MAX_PAYLOAD,
                           status.negotiated_payload);
  TEST_ASSERT_EQUAL_UINT16(0, status.rx_frames);
  TEST_ASSERT_EQUAL_UINT16(0, status.tx_frames);
  TEST_ASSERT_EQUAL_UINT16(0, status.rx_bytes);
  TEST_ASSERT_EQUAL_UINT16(0, status.tx_bytes);
  TEST_ASSERT_EQUAL_UINT16(0, status.rx_crc_errors);
  TEST_ASSERT_EQUAL_UINT16(0, status.link_faults);
  TEST_ASSERT_EQUAL_UINT16(0, status.tx_flushes);
  TEST_ASSERT_EQUAL_INT32(0, status.last_error);
  TEST_ASSERT_EQUAL_UINT8(1, status.control_probe_done);
  TEST_ASSERT_EQUAL_UINT8(0, status.control_schema_count);
}

void testCachedServiceStatusTracksSynchronousStateChanges(void) {
  bccam_uart_service_status_t status;

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_get_status(&status);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        status.service_state);

  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  bccam_uart_service_get_status(&status);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_READY,
                        status.service_state);
}

void testBootloaderEntrySuspendsFirmwareLink(void) {
  bccam_uart_service_status_t status;

  bccam_uart_service_test_start_firmware_discovery();

  TEST_ASSERT_TRUE(bccam_uart_service_test_handle_request(
    BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER));

  bccam_uart_service_get_status(&status);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_LINK_UNINITIALIZED, status.link_state);
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
                        bccam_uart_service_get_state());
}

void testStartupReportExplainsDiscoveryWait(void) {
  bccam_uart_service_test_start_firmware_discovery();

  TEST_ASSERT_EQUAL_STRING(
    "startup stopped while waiting for the camera deck discovery reply.",
    bccam_uart_service_test_startup_phase_message());
}

void testStartupReportExplainsControlCreditWait(void) {
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_TARGET_TX_CREDIT);

  TEST_ASSERT_EQUAL_STRING(
    "startup stopped while waiting for the camera deck to grant Control communication.",
    bccam_uart_service_test_startup_phase_message());
}

void testStartupWatchdogDoesNotResetBeforeDeadline(void) {
  bccam_uart_service_test_startup_recovery_report_t report;

  memset(&report, 0, sizeof(report));
  bccam_uart_service_test_start_firmware_discovery();

  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(2999,
                                                                    &report));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_DISCOVERY,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
}

void testStartupWatchdogResetsAfterDeadlineWithHumanReport(void) {
  bccam_uart_service_test_startup_recovery_report_t report;

  memset(&report, 0, sizeof(report));
  bccam_uart_service_test_start_firmware_discovery();

  TEST_ASSERT_TRUE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                   &report));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_RESETTING,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT32(1, report.reset_count);
  TEST_ASSERT_EQUAL_UINT32(3000, report.timeout_ms);
  TEST_ASSERT_EQUAL_STRING(
    "startup stopped while waiting for the camera deck discovery reply.",
    report.phase_message);
}

void testStartupWatchdogDoesNotRecoverAlreadyClassifiedIncompatibleFirmware(void) {
  bccam_uart_service_test_startup_recovery_report_t report;
  bccam_uart_service_entry_t service = {
    .service_id = 3,
    .major = 2,
    .minor = 1,
  };

  memset(&report, 0, sizeof(report));
  set_service_contract(service.contract_id, "bitcraze.video");
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE);
  bccam_uart_service_test_set_firmware_advertised_services(&service, 1);
  bccam_uart_service_test_update_startup_progress(&report);

  memset(&report, 0, sizeof(report));
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                    &report));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
  TEST_ASSERT_EQUAL_UINT32(0, report.timeout_ms);
}

void testWellFormedMissingControlServiceEntersIncompatibleBeforeDeadline(void) {
  bccam_uart_service_test_startup_recovery_report_t report;
  bccam_uart_service_entry_t service = {
    .service_id = 3,
    .major = 1,
    .minor = 0,
  };

  memset(&report, 0, sizeof(report));
  set_service_contract(service.contract_id, "bitcraze.video");
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE);
  bccam_uart_service_test_set_firmware_advertised_services(&service, 1);

  bccam_uart_service_test_update_startup_progress(&report);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_TRUE(report.incompatible);
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
  TEST_ASSERT_EQUAL_UINT32(0, report.timeout_ms);
  TEST_ASSERT_EQUAL_STRING(
    "startup stopped because a compatible Control service was not advertised.",
    report.phase_message);
  TEST_ASSERT_EQUAL_UINT8(1, report.advertised_service_count);
  TEST_ASSERT_EQUAL_UINT8(3, report.first_advertised_service.service_id);
}

void testWellFormedWrongControlVersionEntersIncompatibleBeforeDeadline(void) {
  bccam_uart_service_test_startup_recovery_report_t report;
  bccam_uart_service_entry_t service = {
    .service_id = 1,
    .major = 2,
    .minor = 0,
  };

  memset(&report, 0, sizeof(report));
  set_service_contract(service.contract_id, "bitcraze.control");
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE);
  bccam_uart_service_test_set_firmware_advertised_services(&service, 1);

  bccam_uart_service_test_update_startup_progress(&report);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_TRUE(report.incompatible);
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
  TEST_ASSERT_EQUAL_STRING(
    "startup stopped because the advertised Control service version is not supported.",
    report.phase_message);
  TEST_ASSERT_EQUAL_UINT8(1, report.advertised_service_count);
  TEST_ASSERT_EQUAL_UINT8(1, report.first_advertised_service.service_id);
  TEST_ASSERT_EQUAL_UINT8(2, report.first_advertised_service.major);
  TEST_ASSERT_EQUAL_UINT8(0, report.first_advertised_service.minor);
}

void testFirmwareIncompatibleDisablesStartupWatchdogRecovery(void) {
  bccam_uart_service_test_startup_recovery_report_t report;
  bccam_uart_service_entry_t service = {
    .service_id = 3,
    .major = 1,
    .minor = 0,
  };

  memset(&report, 0, sizeof(report));
  set_service_contract(service.contract_id, "bitcraze.video");
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE);
  bccam_uart_service_test_set_firmware_advertised_services(&service, 1);

  bccam_uart_service_test_update_startup_progress(&report);
  memset(&report, 0, sizeof(report));

  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                    &report));
  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
  TEST_ASSERT_EQUAL_UINT32(0, report.timeout_ms);
}

void testCompatibleControlServiceDoesNotEnterIncompatibleBeforeProbeCompletes(void) {
  bccam_uart_service_test_startup_recovery_report_t report;
  bccam_uart_service_entry_t service = {
    .service_id = 7,
    .major = 1,
    .minor = 0,
  };

  memset(&report, 0, sizeof(report));
  set_service_contract(service.contract_id, "bitcraze.control");
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_WAITING);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_RESPONSE);
  bccam_uart_service_test_set_firmware_advertised_services(&service, 1);

  bccam_uart_service_test_update_startup_progress(&report);

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_FALSE(report.incompatible);
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
}

void testStartupWatchdogDisablesAfterControlProbeCompletes(void) {
  bccam_uart_service_test_startup_recovery_report_t report;
  bccam_uart_service_status_t status;

  memset(&report, 0, sizeof(report));
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_FW_ACTIVE);
  bccam_uart_service_test_set_firmware_startup_result(
    BCCAM_UART_FIRMWARE_STARTUP_READY);
  bccam_uart_service_test_set_firmware_control_probe_phase(
    BCCAM_UART_CONTROL_PROBE_DONE);

  bccam_uart_service_get_status(&status);

  TEST_ASSERT_EQUAL_UINT8(1, status.control_probe_done);
  TEST_ASSERT_FALSE(bccam_uart_service_test_update_startup_watchdog(3000,
                                                                    &report));

  TEST_ASSERT_EQUAL_INT(BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
                        bccam_uart_service_get_state());
  TEST_ASSERT_EQUAL_UINT32(0, report.reset_count);
}

void testPollFailureFormattingUsesSymbolicResultInNormalMode(void) {
  char line[96];

  bccam_uart_service_test_format_poll_failure(line,
                                              sizeof(line),
                                              BCCAM_UART_ERR_BAD_CRC,
                                              NULL,
                                              false);

  TEST_ASSERT_EQUAL_STRING("UART link poll failed: BAD_CRC", line);
}

void testPollFailureFormattingUsesSymbolicUnknownForInt32MinInNormalMode(void) {
  char line[96];

  bccam_uart_service_test_format_poll_failure(line,
                                              sizeof(line),
                                              (int)INT32_MIN,
                                              NULL,
                                              false);

  TEST_ASSERT_EQUAL_STRING("UART link poll failed: UNKNOWN", line);
}

void testPollFailureFormattingIncludesInt32MinInDebugMode(void) {
  char line[96];

  bccam_uart_service_test_format_poll_failure(line,
                                              sizeof(line),
                                              (int)INT32_MIN,
                                              NULL,
                                              true);

  TEST_ASSERT_EQUAL_STRING("UART link poll failed: UNKNOWN (-2147483648)",
                           line);
}

void testPollFailureFormattingKeepsTinyBuffersTerminated(void) {
  char one_byte[1] = { 'x' };
  char short_line[8];

  memset(short_line, 'x', sizeof(short_line));

  bccam_uart_service_test_format_poll_failure(one_byte,
                                              sizeof(one_byte),
                                              BCCAM_UART_ERR_BAD_CRC,
                                              NULL,
                                              true);
  bccam_uart_service_test_format_poll_failure(short_line,
                                              sizeof(short_line),
                                              BCCAM_UART_ERR_BAD_CRC,
                                              NULL,
                                              true);

  TEST_ASSERT_EQUAL_CHAR('\0', one_byte[0]);
  TEST_ASSERT_EQUAL_STRING("UART li", short_line);
  TEST_ASSERT_EQUAL_CHAR('\0', short_line[sizeof(short_line) - 1u]);
}

void testPollFailureFormattingIncludesStatePhaseAndCountersInDebugMode(void) {
  char line[256];
  bccam_firmware_uart_client_observation_t observation;

  memset(&observation, 0, sizeof(observation));
  observation.link_state = BCCAM_UART_LINK_UNINITIALIZED;
  observation.startup_result = BCCAM_UART_FIRMWARE_STARTUP_WAITING;
  observation.control_probe_phase =
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_DISCOVERY;
  observation.counters.rx_crc_errors = 1;
  observation.counters.rx_malformed_management_errors = 2;
  observation.counters.rx_resyncs = 12;
  observation.counters.link_faults = 0;
  observation.control_malformed = 3;

  bccam_uart_service_test_format_poll_failure(
    line,
    sizeof(line),
    BCCAM_UART_ERR_MALFORMED_MANAGEMENT,
    &observation,
    true);

  TEST_ASSERT_EQUAL_STRING(
    "UART link poll failed: MALFORMED_MANAGEMENT (-12), link=UNINITIALIZED, phase=WAITING_FOR_DISCOVERY, startup=WAITING, crc=1, mgmt=2, control=3, resync=12, faults=0",
    line);
}
