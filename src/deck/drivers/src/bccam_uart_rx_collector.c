#include "bccam_uart_rx_collector.h"

#include <string.h>

static uint16_t read_le16(const uint8_t *in) {
  return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static void reset_scan(bccam_uart_rx_collector_t *collector) {
  collector->sync_state = BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED;
  collector->index = 0u;
  collector->expected_length = 0u;
}

static void emit_fault(bccam_uart_rx_collector_t *collector,
                       bccam_uart_rx_fault_t fault) {
  if (collector->emit == NULL) {
    return;
  }
  const bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = fault,
  };
  (void)collector->emit(collector->emit_context, &event);
}

static void emit_packet(bccam_uart_rx_collector_t *collector) {
  if (collector->emit == NULL) {
    return;
  }
  bccam_uart_rx_event_t event = { .type = BCCAM_UART_RX_EVENT_RAW_FRAME };
  event.raw_frame.length = collector->expected_length;
  memcpy(event.raw_frame.bytes, collector->buffer, collector->expected_length);
  if (!collector->emit(collector->emit_context, &event)) {
    emit_fault(collector, BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW);
  }
}

void bccam_uart_rx_collector_init(bccam_uart_rx_collector_t *collector,
                                  bccam_uart_rx_collector_emit_t emit,
                                  void *emit_context) {
  if (collector == NULL) {
    return;
  }
  memset(collector, 0, sizeof(*collector));
  collector->emit = emit;
  collector->emit_context = emit_context;
  reset_scan(collector);
}

void bccam_uart_rx_collector_feed_byte(bccam_uart_rx_collector_t *collector,
                                       uint8_t byte) {
  if (collector == NULL) {
    return;
  }

  if (collector->index == 0u) {
    if (byte != BCCAM_UART_MAGIC0) {
      return;
    }
    collector->buffer[collector->index++] = byte;
    return;
  }

  if (collector->index == 1u) {
    if (byte == BCCAM_UART_MAGIC1) {
      collector->buffer[collector->index++] = byte;
    } else if (byte != BCCAM_UART_MAGIC0) {
      collector->index = 0u;
    }
    return;
  }

  if (collector->index >= sizeof(collector->buffer)) {
    emit_fault(collector, BCCAM_UART_RX_FAULT_COLLECTOR_STATE);
    reset_scan(collector);
    return;
  }
  collector->buffer[collector->index++] = byte;

  if (collector->index == BCCAM_UART_FRAME_HEADER_SIZE) {
    const uint16_t payload_len = read_le16(&collector->buffer[4]);
    if (payload_len > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
      emit_fault(collector, BCCAM_UART_RX_FAULT_BAD_LENGTH);
      reset_scan(collector);
      return;
    }
    collector->expected_length =
      (uint16_t)(BCCAM_UART_FRAME_OVERHEAD + payload_len);
  }

  if (collector->expected_length > 0u &&
      collector->index == collector->expected_length) {
    emit_packet(collector);
    collector->sync_state = BCCAM_UART_RX_COLLECTOR_SYNCHRONIZED;
    collector->index = 0u;
    collector->expected_length = 0u;
  }
}

void bccam_uart_rx_collector_report_uart_error(
  bccam_uart_rx_collector_t *collector) {
  if (collector == NULL) {
    return;
  }
  emit_fault(collector, BCCAM_UART_RX_FAULT_UART_ERROR);
  reset_scan(collector);
}

bccam_uart_rx_collector_state_t bccam_uart_rx_collector_state(
  const bccam_uart_rx_collector_t *collector) {
  return collector == NULL ? BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED :
    collector->sync_state;
}
