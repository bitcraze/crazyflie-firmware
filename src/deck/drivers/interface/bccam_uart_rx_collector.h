#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bccam_uart_frame.h"

typedef enum {
  BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED = 0,
  BCCAM_UART_RX_COLLECTOR_SYNCHRONIZED,
} bccam_uart_rx_collector_state_t;

typedef enum {
  BCCAM_UART_RX_EVENT_RAW_FRAME = 0,
  BCCAM_UART_RX_EVENT_FAULT,
} bccam_uart_rx_event_type_t;

typedef enum {
  BCCAM_UART_RX_FAULT_UART_ERROR = 0,
  BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW,
  BCCAM_UART_RX_FAULT_BAD_LENGTH,
  BCCAM_UART_RX_FAULT_SYNC_LOST,
  BCCAM_UART_RX_FAULT_COLLECTOR_STATE,
} bccam_uart_rx_fault_t;

typedef struct {
  uint16_t length;
  uint8_t bytes[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
} bccam_uart_rx_raw_frame_t;

typedef struct {
  bccam_uart_rx_event_type_t type;
  union {
    bccam_uart_rx_raw_frame_t raw_frame;
    bccam_uart_rx_fault_t fault;
  };
} bccam_uart_rx_event_t;

typedef bool (*bccam_uart_rx_collector_emit_t)(
  void *context,
  const bccam_uart_rx_event_t *event);

typedef struct {
  bccam_uart_rx_collector_state_t sync_state;
  uint16_t index;
  uint16_t expected_length;
  uint8_t buffer[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  bccam_uart_rx_collector_emit_t emit;
  void *emit_context;
} bccam_uart_rx_collector_t;

void bccam_uart_rx_collector_init(bccam_uart_rx_collector_t *collector,
                                  bccam_uart_rx_collector_emit_t emit,
                                  void *emit_context);

void bccam_uart_rx_collector_feed_byte(bccam_uart_rx_collector_t *collector,
                                       uint8_t byte);

void bccam_uart_rx_collector_report_uart_error(
  bccam_uart_rx_collector_t *collector);

bccam_uart_rx_collector_state_t bccam_uart_rx_collector_state(
  const bccam_uart_rx_collector_t *collector);
