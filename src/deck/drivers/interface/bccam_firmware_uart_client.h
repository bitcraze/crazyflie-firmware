#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bccam_deck_controller.h"
#include "bccam_uart_rx_collector.h"
#include "bccam_uart_runtime.h"

typedef struct {
  bccam_uart_runtime_t runtime;
  bccam_deck_controller_t *deck_controller;
} bccam_firmware_uart_client_t;

typedef struct {
  bccam_uart_link_state_t link_state;
  bccam_uart_firmware_startup_result_t startup_result;
  bccam_uart_control_probe_phase_t control_probe_phase;
  bccam_uart_link_counters_t counters;
  uint16_t negotiated_payload;
  uint16_t rx_bytes;
  uint16_t tx_bytes;
  uint16_t tx_flushes;
  uint16_t control_malformed;
  int32_t last_error;
  uint8_t control_tx_credit;
  uint8_t control_rx_slots;
  uint8_t control_schema_module_count;
  uint8_t advertised_service_count;
  bool control_probe_done;
} bccam_firmware_uart_client_observation_t;

void bccam_firmware_uart_client_init(
  bccam_firmware_uart_client_t *client,
  bccam_deck_controller_t *deck_controller);

bool bccam_firmware_uart_client_enter(
  bccam_firmware_uart_client_t *client,
  uint32_t now_ticks);

void bccam_firmware_uart_client_suspend(
  bccam_firmware_uart_client_t *client);

int bccam_firmware_uart_client_poll(
  bccam_firmware_uart_client_t *client,
  uint32_t now_ticks);

int bccam_firmware_uart_client_on_rx_event(
  bccam_firmware_uart_client_t *client,
  const bccam_uart_rx_event_t *event);

int bccam_firmware_uart_client_pump_tx(
  bccam_firmware_uart_client_t *client);

bccam_uart_firmware_startup_result_t
bccam_firmware_uart_client_startup_result(
  const bccam_firmware_uart_client_t *client);

bccam_uart_control_probe_phase_t
bccam_firmware_uart_client_control_probe_phase(
  const bccam_firmware_uart_client_t *client);

void bccam_firmware_uart_client_observe(
  const bccam_firmware_uart_client_t *client,
  bccam_firmware_uart_client_observation_t *observation);

bool bccam_firmware_uart_client_control_descriptor(
  const bccam_firmware_uart_client_t *client,
  bccam_uart_service_descriptor_t *descriptor);

const bccam_uart_control_schema_module_t *
bccam_firmware_uart_client_control_schema_module(
  const bccam_firmware_uart_client_t *client,
  uint8_t index);

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
typedef enum {
  BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_BEGIN_BOOT = 0,
  BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_RELEASE_BOOT,
  BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SET_BAUDRATE,
  BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND,
  BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_RECEIVE,
  BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_DRAIN,
  BCCAM_FIRMWARE_UART_CLIENT_TEST_RUNTIME_START_ESTABLISHMENT,
} bccam_firmware_uart_client_test_event_t;

typedef struct {
  bccam_firmware_uart_client_test_event_t event;
  bccam_deck_boot_mode_t boot_mode;
  uint32_t value;
  uint32_t length;
  uint8_t byte;
  uint8_t bytes[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
} bccam_firmware_uart_client_test_trace_entry_t;

void bccam_firmware_uart_client_test_trace_reset(void);
void bccam_firmware_uart_client_test_set_uart_send_result(bool result);
uint8_t bccam_firmware_uart_client_test_trace_count(void);
const bccam_firmware_uart_client_test_trace_entry_t *
bccam_firmware_uart_client_test_trace_entry(uint8_t index);
void bccam_firmware_uart_client_test_queue_rx(const uint8_t *bytes,
                                              uint32_t length);
int bccam_firmware_uart_client_test_consume_rx(
  bccam_firmware_uart_client_t *client);
#endif
