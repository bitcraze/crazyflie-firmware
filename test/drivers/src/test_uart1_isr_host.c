#include <stdint.h>

#include "unity.h"

#define USART_FLAG_PE   ((uint16_t)0x0001)
#define USART_FLAG_FE   ((uint16_t)0x0002)
#define USART_FLAG_NE   ((uint16_t)0x0004)
#define USART_FLAG_ORE  ((uint16_t)0x0008)
#define USART_FLAG_RXNE ((uint16_t)0x0020)

#include "uart1_isr.h"

void setUp(void) {
}

void tearDown(void) {
}

void testRxByteWithoutErrorIsDelivered(void) {
  TEST_ASSERT_EQUAL_INT(UART1_ISR_RX_DELIVER_BYTE,
                        uart1IsrGetRxAction(USART_FLAG_RXNE));
}

void testRxByteWithFramingErrorIsDiscardedBeforeReportingError(void) {
  TEST_ASSERT_EQUAL_INT(UART1_ISR_RX_DISCARD_BYTE_AND_REPORT_ERROR,
                        uart1IsrGetRxAction(USART_FLAG_RXNE |
                                            USART_FLAG_FE));
}

void testErrorWithoutRxByteClearsErrorSequenceAndReportsError(void) {
  TEST_ASSERT_EQUAL_INT(UART1_ISR_RX_CLEAR_ERROR_AND_REPORT_ERROR,
                        uart1IsrGetRxAction(USART_FLAG_ORE));
}
