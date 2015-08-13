#ifndef IR_RX_H_INCLUDED
#define IR_RX_H_INCLUDED

#include "ring_buffer.h"
#include "ir_code.h"

#define IR_RX_CAPTURE_BUFFER_MAX_LEN  100

typedef struct {
  int16_t bufferLength;
  uint16_t buffer[IR_RX_CAPTURE_BUFFER_MAX_LEN];
} IrRecv;

void ir_rx_setup();
IrRecv* ir_rx_recv();

#endif // IR_RX_H_INCLUDED
