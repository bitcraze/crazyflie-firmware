/*-----------------------------------------------------------------------/
/  Low level disk interface module include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#pragma once

#define SD_DISK_TIMER_PERIOD_MS 100

#include "diskio.h"

typedef struct {
  void (*initSpi) (void);
  void (*setSlowSpiMode) (void);
  void (*setFastSpiMode) (void);
  BYTE (*xchgSpi) (BYTE dat);
  void (*rcvrSpiMulti) (BYTE *buff, UINT btr);
  void (*xmitSpiMulti) (const BYTE *buff, UINT btx);
  void (*csHigh) (BYTE doDummyClock);
  void (*csLow) (void);
  void (*delayMs) (UINT ms);

  volatile DSTATUS stat;
  BYTE cardType;

  // (Formally 1kHz) 10 Hz decrement timers stopped at zero (disk_timerproc())
  volatile UINT timer1;
  volatile UINT timer2;
} sdSpiContext_t;

/* Drivers function declarations */
DSTATUS SD_disk_initialize(void *);
DSTATUS SD_disk_status(void *);
DRESULT SD_disk_ioctl(BYTE cmd, void *buff, void *);
DRESULT SD_disk_read(BYTE *buff, DWORD sector, UINT count, void *);
DRESULT SD_disk_write(const BYTE *buff, DWORD sector, UINT count, void *);
void SD_disk_timerproc(void *);
