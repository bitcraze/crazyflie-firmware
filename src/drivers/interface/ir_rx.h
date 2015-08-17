#ifndef IR_RX_H_INCLUDED
#define IR_RX_H_INCLUDED

#include "ir_code.h"

#define IR_RX_CAPTURE_BUFFER_MAX_LEN  100

typedef struct {
  int16_t bufferLength;
  uint16_t buffer[IR_RX_CAPTURE_BUFFER_MAX_LEN];
} IrRecv;

void irRxInit();
IrRecv* irRxReceive();

#endif // IR_RX_H_INCLUDED
