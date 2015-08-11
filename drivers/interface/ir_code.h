#ifndef IR_CODE_H
#define	IR_CODE_H

#define BRAND_TIVO         0x0001
#define BRAND_WESTINGHOUSE 0x0002
#define BRAND_MOTOROLA     0x0003

typedef struct {
  uint16_t brand;
  uint16_t key;
  uint8_t repeatCount;
  uint32_t gap;
  uint16_t codeLength;
  uint16_t* code;
} IrCode;

void ir_code_setup();
IrCode* ir_code_decode(uint16_t* buffer, uint16_t bufferLen);
IrCode* ir_code_getByIndex(uint16_t codeIndex);

#endif // IR_CODE_H
