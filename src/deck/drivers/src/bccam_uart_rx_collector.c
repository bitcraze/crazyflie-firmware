#include "bccam_uart_rx_collector.h"

#include <string.h>

static uint16_t read_le16(const uint8_t *in) {
  return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static void reset_unsynchronized(bccam_uart_rx_collector_t *collector) {
  collector->sync_state = BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED;
  collector->index = 0;
  collector->expected_length = 0;
}

static void emit_fault(bccam_uart_rx_collector_t *collector,
                       bccam_uart_rx_fault_t fault) {
  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = fault,
  };
  if (collector->emit != NULL && !collector->emit(collector->context, &event)) {
    event.fault = BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW;
    (void)collector->emit(collector->context, &event);
  }
}

static void release_active_slot(bccam_uart_rx_collector_t *collector) {
  if (collector->active_slot != BCCAM_UART_RX_SLOT_INVALID &&
      collector->release != NULL) {
    collector->release(collector->context, collector->active_slot);
  }

  collector->active_slot = BCCAM_UART_RX_SLOT_INVALID;
  collector->active_bytes = NULL;
  collector->active_capacity = 0;
}

static bool reserve_active_slot(bccam_uart_rx_collector_t *collector) {
  if (collector->active_slot != BCCAM_UART_RX_SLOT_INVALID) {
    return true;
  }

  if (collector->reserve == NULL) {
    emit_fault(collector, BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW);
    reset_unsynchronized(collector);
    return false;
  }

  bccam_uart_rx_slot_t slot = {
    .slot = BCCAM_UART_RX_SLOT_INVALID,
    .bytes = NULL,
    .capacity = 0,
  };

  if (!collector->reserve(collector->context, &slot) ||
      slot.bytes == NULL ||
      slot.capacity < BCCAM_UART_FRAME_MAX_ENCODED_SIZE ||
      slot.slot == BCCAM_UART_RX_SLOT_INVALID) {
    emit_fault(collector, BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW);
    reset_unsynchronized(collector);
    return false;
  }

  collector->active_slot = slot.slot;
  collector->active_bytes = slot.bytes;
  collector->active_capacity = slot.capacity;
  return true;
}

static void emit_frame_or_fault(bccam_uart_rx_collector_t *collector) {
  bccam_uart_rx_event_t event = {
    .type = BCCAM_UART_RX_EVENT_RAW_FRAME,
    .raw_frame = {
      .slot = collector->active_slot,
      .length = collector->expected_length,
    },
  };

  if (collector->emit == NULL ||
      !collector->emit(collector->context, &event)) {
    release_active_slot(collector);
    emit_fault(collector, BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW);
    return;
  }

  collector->active_slot = BCCAM_UART_RX_SLOT_INVALID;
  collector->active_bytes = NULL;
  collector->active_capacity = 0;
}

static bool complete_header(bccam_uart_rx_collector_t *collector) {
  const uint16_t payload_len = read_le16(&collector->active_bytes[4]);
  if (payload_len > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
    emit_fault(collector, BCCAM_UART_RX_FAULT_BAD_LENGTH);
    release_active_slot(collector);
    reset_unsynchronized(collector);
    return false;
  }

  collector->expected_length =
    (uint16_t)(BCCAM_UART_FRAME_OVERHEAD + payload_len);
  return true;
}

void bccam_uart_rx_collector_init(bccam_uart_rx_collector_t *collector,
                                  bccam_uart_rx_collector_emit_t emit,
                                  bccam_uart_rx_collector_reserve_t reserve,
                                  bccam_uart_rx_collector_release_t release,
                                  void *context) {
  if (collector == NULL) {
    return;
  }

  memset(collector, 0, sizeof(*collector));
  collector->emit = emit;
  collector->reserve = reserve;
  collector->release = release;
  collector->context = context;
  collector->active_slot = BCCAM_UART_RX_SLOT_INVALID;
  reset_unsynchronized(collector);
}

void bccam_uart_rx_collector_feed_byte(bccam_uart_rx_collector_t *collector,
                                       uint8_t byte) {
  if (collector == NULL) {
    return;
  }

  if (collector->index == 0) {
    if (collector->sync_state == BCCAM_UART_RX_COLLECTOR_SYNCHRONIZED &&
        byte != BCCAM_UART_MAGIC0) {
      emit_fault(collector, BCCAM_UART_RX_FAULT_SYNC_LOST);
      reset_unsynchronized(collector);
      return;
    }

    if (byte != BCCAM_UART_MAGIC0) {
      return;
    }

    if (!reserve_active_slot(collector)) {
      return;
    }
  } else if (collector->index == 1 && byte != BCCAM_UART_MAGIC1) {
    if (collector->sync_state == BCCAM_UART_RX_COLLECTOR_SYNCHRONIZED) {
      emit_fault(collector, BCCAM_UART_RX_FAULT_SYNC_LOST);
      release_active_slot(collector);
      reset_unsynchronized(collector);
      return;
    }

    release_active_slot(collector);
    collector->index = 0;
    if (byte == BCCAM_UART_MAGIC0 && reserve_active_slot(collector)) {
      collector->index = 1;
      collector->active_bytes[0] = byte;
    }
    return;
  }

  if (collector->active_bytes == NULL ||
      collector->index >= collector->active_capacity) {
    emit_fault(collector, BCCAM_UART_RX_FAULT_COLLECTOR_STATE);
    release_active_slot(collector);
    reset_unsynchronized(collector);
    return;
  }

  collector->active_bytes[collector->index++] = byte;

  if (collector->index == BCCAM_UART_FRAME_HEADER_SIZE &&
      !complete_header(collector)) {
    return;
  }

  if (collector->expected_length > 0 &&
      collector->index == collector->expected_length) {
    emit_frame_or_fault(collector);
    collector->sync_state = BCCAM_UART_RX_COLLECTOR_SYNCHRONIZED;
    collector->index = 0;
    collector->expected_length = 0;
  }
}

void bccam_uart_rx_collector_report_uart_error(
  bccam_uart_rx_collector_t *collector) {
  if (collector == NULL) {
    return;
  }
  emit_fault(collector, BCCAM_UART_RX_FAULT_UART_ERROR);
  release_active_slot(collector);
  reset_unsynchronized(collector);
}

bccam_uart_rx_collector_state_t bccam_uart_rx_collector_state(
  const bccam_uart_rx_collector_t *collector) {
  if (collector == NULL) {
    return BCCAM_UART_RX_COLLECTOR_UNSYNCHRONIZED;
  }
  return collector->sync_state;
}
