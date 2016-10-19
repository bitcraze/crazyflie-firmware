/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef __FATFS_SD_H__
#define __FATFS_SD_H__

#define _USE_WRITE	1	/* 1: Enable disk_write function */
#define _USE_IOCTL	1	/* 1: Enable disk_ioctl fucntion */

#define SD_DISK_TIMER_PERIOD_MS 1

#include "diskio.h"
#include "integer.h"

typedef struct {
  void (*init_spi) (void);
  void (*set_slow_spi_mode) (void);
  void (*set_fast_spi_mode) (void);
  BYTE (*xchg_spi) (BYTE dat);
  void (*rcvr_spi_multi) (BYTE *buff, UINT btr);
  void (*xmit_spi_multi) (const BYTE *buff, UINT btx);
  void (*cs_high) (void);
  void (*cs_low) (void);

  volatile DSTATUS Stat;
  BYTE CardType;

  // 1kHz decrement timers stopped at zero (disk_timerproc())
  volatile UINT Timer1;
  volatile UINT Timer2;
} sdSpiOps_t;

#endif // __FATFS_SD_H__

