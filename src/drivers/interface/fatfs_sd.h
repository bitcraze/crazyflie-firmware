/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef __FATFS_SD_H__
#define __FATFS_SD_H__

#define SD_DISK_TIMER_PERIOD_MS 1

#include "diskio.h"
#include "integer.h"

typedef struct {
  void (*initSpi) (void);
  void (*setSlowSpiMode) (void);
  void (*setFastSpiMode) (void);
  BYTE (*xchgSpi) (BYTE dat);
  void (*rcvrSpiMulti) (BYTE *buff, UINT btr);
  void (*xmitSpiMulti) (const BYTE *buff, UINT btx);
  void (*csHigh) (void);
  void (*csLow) (void);

  volatile DSTATUS stat;
  BYTE cardType;

  // 1kHz decrement timers stopped at zero (disk_timerproc())
  volatile UINT timer1;
  volatile UINT timer2;
} sdSpiContext_t;

#endif // __FATFS_SD_H__

