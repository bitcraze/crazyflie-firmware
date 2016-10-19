/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED_SD
#define _DISKIO_DEFINED_SD

#define _USE_WRITE	1	/* 1: Enable disk_write function */
#define _USE_IOCTL	1	/* 1: Enable disk_ioctl fucntion */

#include "diskio.h"
#include "integer.h"

typedef struct sdSpiOps_s {
  void (*init_spi) (void);
  void (*set_slow_spi_mode) (void);
  void (*set_fast_spi_mode) (void);
  BYTE (*xchg_spi) (BYTE dat);
  void (*rcvr_spi_multi) (BYTE *buff, UINT btr);
  void (*xmit_spi_multi) (const BYTE *buff, UINT btx);
  void (*cs_high) (void);
  void (*cs_low) (void);
} sdSpiOps_t;

#endif

