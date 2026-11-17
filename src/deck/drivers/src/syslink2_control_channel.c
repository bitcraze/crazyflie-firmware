#include "syslink2_control_channel.h"

#include <stddef.h>

#include "bccam_uart_link.h"

static bccam_uart_runtime_t *runtime;

void syslink2_control_channel_init(bccam_uart_runtime_t *runtime_arg) {
  runtime = runtime_arg;
}

bool syslink2_control_channel_runtime_ready(void) {
  return bccam_uart_runtime_control_ready(runtime);
}

void syslink2_control_channel_open(void) {
  bccam_uart_runtime_grant_control_rx_credit(runtime);
}

bool syslink2_control_channel_can_send(void) {
  return bccam_uart_runtime_control_tx_credit(runtime) > 0u;
}

syslink2_control_send_result_t syslink2_control_channel_send(
  const syslink2_control_frame_t *frame) {
  switch (bccam_uart_runtime_send_control(runtime, frame)) {
    case BCCAM_UART_OK:
      return SYSLINK2_CONTROL_SEND_SENT;
    case BCCAM_UART_ERR_NO_CREDIT:
    case BCCAM_UART_ERR_TRANSACTION_BUSY:
      return SYSLINK2_CONTROL_SEND_BUSY;
    case BCCAM_UART_ERR_LINK_FAULT:
    case BCCAM_UART_ERR_NOT_ACTIVE:
    case BCCAM_UART_ERR_UNKNOWN_SERVICE:
      return SYSLINK2_CONTROL_SEND_CLOSED;
    default:
      return SYSLINK2_CONTROL_SEND_REJECTED;
  }
}

bool syslink2_control_channel_receive(syslink2_control_frame_t *frame) {
  if (!bccam_uart_runtime_take_control_rx(runtime, frame)) {
    return false;
  }
  bccam_uart_runtime_grant_control_rx_credit(runtime);
  return true;
}
