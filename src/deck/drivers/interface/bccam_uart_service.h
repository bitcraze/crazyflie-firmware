#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bccam_firmware_uart_client.h"
#include "FreeRTOS.h"
#include "bccam_uart_link.h"
#include "bccam_uart_rx_collector.h"
#include "bccam_uart_runtime.h"
#include "deck_core.h"

typedef enum {
  BCCAM_UART_SERVICE_STATE_UNINITIALIZED = 0,
  BCCAM_UART_SERVICE_STATE_FW_RESETTING,
  BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING,
  BCCAM_UART_SERVICE_STATE_FW_ACTIVE,
  BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE,
  BCCAM_UART_SERVICE_STATE_ISP_ENTERING,
  BCCAM_UART_SERVICE_STATE_ISP_READY,
  BCCAM_UART_SERVICE_STATE_ISP_BUSY,
  BCCAM_UART_SERVICE_STATE_ISP_COMPLETE,
  BCCAM_UART_SERVICE_STATE_FAULT,
} bccam_uart_service_state_t;

typedef struct {
  bccam_uart_service_state_t service_state;
  uint8_t link_state;
  uint8_t tx_credit;
  uint8_t rx_slots;
  uint16_t negotiated_payload;
  uint16_t rx_frames;
  uint16_t tx_frames;
  uint16_t rx_bytes;
  uint16_t tx_bytes;
  uint16_t rx_crc_errors;
  uint16_t link_faults;
  uint16_t tx_flushes;
  int32_t last_error;
  uint8_t control_probe_done;
  uint8_t control_schema_count;
} bccam_uart_service_status_t;

void bccam_uart_service_init(DeckInfo *deck_info);

void bccam_uart_service_request_bootloader(void);

void bccam_uart_service_request_firmware(void);

bool bccam_uart_service_write_flash(uint32_t mem_addr,
                                    uint8_t write_len,
                                    const uint8_t *buffer,
                                    uint32_t new_fw_size);

bool bccam_uart_service_read_flash(uint32_t mem_addr,
                                   uint8_t read_len,
                                   uint8_t *buffer);

bool bccam_uart_service_state_has_active_bootloader(
  bccam_uart_service_state_t state);

bccam_uart_service_state_t bccam_uart_service_get_state(void);

void bccam_uart_service_get_status(bccam_uart_service_status_t *status);

bool bccam_uart_service_submit_rx_event_from_isr(
  const bccam_uart_rx_event_t *event,
  BaseType_t *higher_priority_task_woken);

// Backing storage for legacy deck.bcCamLink param reads.
extern uint8_t bccam_uart_service_link_state_log;

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
typedef enum {
  BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER,
  BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE,
} bccam_uart_service_test_request_t;

typedef struct {
  uint32_t reset_count;
  uint32_t timeout_ms;
  const char *phase_message;
  bool incompatible;
  uint8_t advertised_service_count;
  bool control_descriptor_present;
  bccam_uart_service_descriptor_t control_descriptor;
} bccam_uart_service_test_startup_recovery_report_t;

void bccam_uart_service_test_reset(void);
void bccam_uart_service_test_set_state(bccam_uart_service_state_t state);
bool bccam_uart_service_test_handle_request(bccam_uart_service_test_request_t request);
void bccam_uart_service_test_set_firmware_startup_result(
  bccam_uart_firmware_startup_result_t result);
void bccam_uart_service_test_set_firmware_control_probe_phase(
  bccam_uart_control_probe_phase_t phase);
void bccam_uart_service_test_set_bootloader_enter_result(bool result);
void bccam_uart_service_test_poll_once(void);
void bccam_uart_service_test_handle_rx_event(
  const bccam_uart_rx_event_t *event);
void bccam_uart_service_test_handle_rx_events(
  const bccam_uart_rx_event_t *events,
  uint8_t event_count);
uint16_t bccam_uart_service_test_rx_post_drain_poll_count(void);
uint16_t bccam_uart_service_test_incompatible_report_count(void);
bool bccam_uart_service_test_flash_write_allowed(void);
void bccam_uart_service_test_start_firmware_establishment(void);
const char *bccam_uart_service_test_startup_phase_message(void);
void bccam_uart_service_test_update_startup_progress(
  bccam_uart_service_test_startup_recovery_report_t *report);
bool bccam_uart_service_test_update_startup_watchdog(
  uint32_t now_ticks,
  bccam_uart_service_test_startup_recovery_report_t *report);
void bccam_uart_service_test_format_poll_failure(
  char *buffer,
  size_t buffer_size,
  int result,
  const bccam_firmware_uart_client_observation_t *observation,
  bool include_debug_detail);
#endif
