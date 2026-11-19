#include "syslink2_apiservice_channel.h"

#include <stddef.h>

#include "bccam_uart_link.h"

static bccam_uart_runtime_t *runtime;

void syslink2_apiservice_channel_init(bccam_uart_runtime_t *runtime_arg) {
  runtime = runtime_arg;
}

bool syslink2_apiservice_channel_runtime_ready(void) {
  return bccam_uart_runtime_apiservice_ready(runtime);
}

void syslink2_apiservice_channel_open(void) {
  bccam_uart_runtime_grant_apiservice_rx_credit(runtime);
}

bool syslink2_apiservice_channel_can_send(void) {
  return bccam_uart_runtime_apiservice_tx_credit(runtime) > 0u;
}

syslink2_apiservice_send_result_t syslink2_apiservice_channel_send(
  const syslink2_apiservice_frame_t *frame) {
  switch (bccam_uart_runtime_send_apiservice(runtime, frame)) {
    case BCCAM_UART_OK:
      return SYSLINK2_APISERVICE_SEND_SENT;
    case BCCAM_UART_ERR_NO_CREDIT:
    case BCCAM_UART_ERR_TRANSACTION_BUSY:
      return SYSLINK2_APISERVICE_SEND_BUSY;
    case BCCAM_UART_ERR_LINK_FAULT:
    case BCCAM_UART_ERR_NOT_ACTIVE:
    case BCCAM_UART_ERR_UNKNOWN_SERVICE:
      return SYSLINK2_APISERVICE_SEND_CLOSED;
    default:
      return SYSLINK2_APISERVICE_SEND_REJECTED;
  }
}

bool syslink2_apiservice_channel_receive(syslink2_apiservice_frame_t *frame) {
  if (!bccam_uart_runtime_take_apiservice_rx(runtime, frame)) {
    return false;
  }
  bccam_uart_runtime_grant_apiservice_rx_credit(runtime);
  return true;
}
