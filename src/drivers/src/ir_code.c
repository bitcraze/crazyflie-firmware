
#include <stdlib.h>
#include <stdint.h>
#include "debug.h"
#include "ir_code.h"

//#define DEBUG_DECODE

#define NO_MATCH         -1
#define MATCH_THRESHOLD  100
#define MIN_SIGNAL_COUNT 5

int _ir_code_match(IrCode* code, uint16_t* buffer, uint16_t bufferLen);

IrCode codes[CODE_COUNT];

uint16_t code0[] = { 2440, 516, 1232, 541, 616, 541, 615, 542, 1231, 542, 615, 542, 614, 543, 614, 542, 1230, 543, 615, 542, 613, 543, 614, 543, 614, 551 };
uint16_t code1[] = { 2440, 515, 640, 517, 615, 541, 615, 541, 616, 542, 615, 542, 615, 541, 614, 542, 1230, 542, 614, 542, 615, 542, 614, 543, 612, 551 };
uint16_t code2[] = { 2416, 539, 1231, 541, 615, 541, 614, 542, 614, 543, 614, 542, 614, 542, 613, 543, 1230, 543, 614, 542, 614, 544, 613, 544, 613, 551 };
uint16_t code3[] = { 2438, 518, 616, 540, 1231, 541, 615, 541, 615, 541, 615, 542, 615, 542, 615, 541, 1230, 542, 614, 543, 614, 543, 614, 542, 613, 551 };
uint16_t code4[] = { 2439, 517, 1232, 541, 1231, 541, 615, 542, 614, 542, 615, 542, 614, 543, 614, 543, 1229, 543, 614, 543, 614, 543, 614, 543, 614, 551 };
uint16_t code5[] = { 2416, 540, 616, 541, 616, 541, 1231, 541, 615, 542, 614, 542, 613, 542, 614, 543, 1231, 542, 614, 543, 615, 542, 614, 543, 613, 551 };
uint16_t code6[] = { 2416, 540, 1232, 541, 616, 541, 1231, 541, 615, 542, 615, 542, 614, 542, 615, 543, 1230, 542, 614, 542, 614, 542, 614, 542, 614, 551 };
uint16_t code7[] = { 2415, 539, 616, 541, 1231, 541, 1231, 542, 615, 542, 614, 542, 615, 542, 615, 542, 1230, 542, 614, 543, 614, 542, 615, 542, 614, 551 };
uint16_t code8[] = { 2440, 516, 1253, 520, 1231, 541, 1230, 542, 614, 543, 614, 542, 615, 542, 613, 543, 1230, 543, 614, 543, 614, 543, 613, 544, 613, 551 };
uint16_t code9[] = { 2417, 538, 616, 541, 616, 541, 615, 542, 1231, 542, 615, 542, 614, 542, 614, 542, 1231, 542, 614, 542, 615, 542, 614, 543, 614, 551 };
uint16_t codePwr[] = { 2433, 543, 1260, 551, 611, 551, 1260, 551, 611, 551, 1260, 551, 611, 551, 611, 551, 1231, 551, 611, 551, 611, 551, 611, 551, 611, 551 };

uint16_t motoCode0[] = { 8912,4406,509,2188,510,2188,509,2188,510,2187,510,2188,510,2187,510,2188,510,2188,511,2187,510,2187,511,2188,510,2187,511,2187,510,2187,511,2187,510,2187,511 };
uint16_t motoCode1[] = { 8892,4428,510,4411,511,2187,510,2187,509,2187,510,2187,510,2188,510,2188,511,2188,510,2187,511,2187,510,2187,511,2186,511,4411,511,4410,510,4411,510,4411,511 };

void ir_code_setup() {
  int i = 0;
  for(; i<11; i++) {
    codes[i].brand = BRAND_WESTINGHOUSE;
    codes[i].key = i;
    codes[i].repeatCount = 4;
    codes[i].gap = 45000;
    codes[i].codeLength = 26;
    switch(i) {
      case 0: codes[i].code = code0; break;
      case 1: codes[i].code = code1; break;
      case 2: codes[i].code = code2; break;
      case 3: codes[i].code = code3; break;
      case 4: codes[i].code = code4; break;
      case 5: codes[i].code = code5; break;
      case 6: codes[i].code = code6; break;
      case 7: codes[i].code = code7; break;
      case 8: codes[i].code = code8; break;
      case 9: codes[i].code = code9; break;
      case 10: codes[i].code = codePwr; break;
    }
  }

  for(; i<CODE_COUNT; i++) {
    codes[i].brand = BRAND_MOTOROLA;
    codes[i].key = i - 11;
    codes[i].repeatCount = 1;
    codes[i].gap = 100000;
    codes[i].codeLength = 35;
    switch(i) {
      case 11: codes[i].code = motoCode0; break;
      case 12: codes[i].code = motoCode1; break;
    }
  }
}

IrCode* ir_code_getByIndex(uint16_t codeIndex)
{
  return &codes[codeIndex];
}

IrCode* ir_code_decode(uint16_t* buffer, uint16_t bufferLen)
{
  if (bufferLen < MIN_SIGNAL_COUNT)
  {
    return NULL;
  }

#ifdef DEBUG_DECODE
  debug_write("?signal: ");
  for(int i = 0; i < bufferLen; i++)
  {
    debug_write_u16(buffer[i], 10);
    debug_write(", ");
  }
  debug_write_line("");
#endif

  int bestMatch = -1;
  int bestValue = 0;
  int i;

  for (i = 0; i < CODE_COUNT; i++)
  {
    int matchValue = _ir_code_match(&codes[i], buffer, bufferLen);
    if (matchValue != NO_MATCH && matchValue > bestValue)
    {
      bestMatch = i;
      bestValue = matchValue;
    }
  }
#ifdef DEBUG_DECODE
  if(bestMatch >= 0)
  {
    debug_write("?match: ");
    debug_write_u16(codes[bestMatch].brand, 16);
    debug_write_u16(codes[bestMatch].key, 16);
    debug_write_line("");
  }
  else
  {
    debug_write_line("?no match");
  }
#endif
  if (bestMatch >= 0)
  {
    return &codes[bestMatch];
  }
  else
  {
    return NULL;
  }
}

int _ir_code_match(IrCode* code, uint16_t* buffer, uint16_t bufferLen)
{
  int minLen = bufferLen < code->codeLength ? bufferLen : code->codeLength;
  int i;
  uint32_t diff = 0;
  uint32_t matchThreshold = minLen * MATCH_THRESHOLD;

  for (i = 0; i < minLen; i++)
  {
    diff += abs((int) buffer[i] - (int) code->code[i]);
    if (diff > matchThreshold)
    {
      return NO_MATCH;
    }
  }

#ifdef DEBUG_DECODE
  debug_write("?diff: ");
  debug_write_u16(code->brand, 16);
  debug_write_u16(code->key, 16);
  debug_write(" ");
  debug_write_u16(diff, 10);
  debug_write_line("");
#endif

  return 0xffff - diff;
}
