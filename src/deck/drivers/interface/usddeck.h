#ifndef __USDDECK_H__
#define __USDDECK_H__

#include <stdint.h>
#include <stdbool.h>

enum usddeckLoggingMode_e
{
  usddeckLoggingMode_Disabled = 0,
  usddeckLoggingMode_SynchronousStabilizer,
  usddeckLoggingMode_Asyncronous,
};

// returns true if logging is enabled
bool usddeckLoggingEnabled(void);

// returns the current logging mode
enum usddeckLoggingMode_e usddeckLoggingMode(void);

// returns the desired logging frequency
int usddeckFrequency(void);

// For synchronous logging: add a new log entry
void usddeckTriggerLogging(void);

// returns size of current file if logging is stopped (0 otherwise)
uint32_t usddeckFileSize(void);

// Read "length" number of bytes at "offset" into "buffer" of current file
// Only works if logging is stopped
bool usddeckRead(uint32_t offset, uint8_t* buffer, uint16_t length);

#endif //__USDDECK_H__
