#pragma once

#include <stdbool.h>
#include <stdint.h>

// Host unit tests include this private helper without the STM32F4 peripheral
// headers. These values mirror the USART SR flag bits used by the driver.
#ifndef USART_FLAG_PE
#define USART_FLAG_PE   ((uint16_t)0x0001)
#endif
#ifndef USART_FLAG_FE
#define USART_FLAG_FE   ((uint16_t)0x0002)
#endif
#ifndef USART_FLAG_NE
#define USART_FLAG_NE   ((uint16_t)0x0004)
#endif
#ifndef USART_FLAG_ORE
#define USART_FLAG_ORE  ((uint16_t)0x0008)
#endif
#ifndef USART_FLAG_RXNE
#define USART_FLAG_RXNE ((uint16_t)0x0020)
#endif

typedef enum {
  UART1_ISR_RX_IDLE = 0,
  UART1_ISR_RX_DELIVER_BYTE,
  UART1_ISR_RX_DISCARD_BYTE_AND_REPORT_ERROR,
  UART1_ISR_RX_CLEAR_ERROR_AND_REPORT_ERROR,
} uart1IsrRxAction_t;

static inline bool uart1IsrStatusHasError(uint32_t status) {
  return ((status & USART_FLAG_ORE) != 0) ||
         ((status & USART_FLAG_NE) != 0) ||
         ((status & USART_FLAG_FE) != 0) ||
         ((status & USART_FLAG_PE) != 0);
}

static inline uart1IsrRxAction_t uart1IsrGetRxAction(uint32_t status) {
  const bool has_rx_data = (status & USART_FLAG_RXNE) != 0;
  const bool has_error = uart1IsrStatusHasError(status);

  if (has_error && has_rx_data) {
    return UART1_ISR_RX_DISCARD_BYTE_AND_REPORT_ERROR;
  }

  if (has_error) {
    return UART1_ISR_RX_CLEAR_ERROR_AND_REPORT_ERROR;
  }

  if (has_rx_data) {
    return UART1_ISR_RX_DELIVER_BYTE;
  }

  return UART1_ISR_RX_IDLE;
}
