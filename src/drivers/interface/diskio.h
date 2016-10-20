/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#include <stdint.h>
#include "integer.h"

/* Status of Disk Functions */
typedef BYTE DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;

//#define FATFS_DEBUG_SEND_USART(x)	TM_USART_Puts(USART6, x); TM_USART_Puts(USART6, "\n");
#define FATFS_DEBUG_SEND_USART(x)

/*---------------------------------------*/
/* Prototypes for disk control functions */

/* FATFS related */
DSTATUS disk_initialize(BYTE pdrv);
DSTATUS disk_status(BYTE pdrv);
DRESULT disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff);

/* Driver related functions */
typedef struct {
	DSTATUS (*disk_initialize)(void *);
	DSTATUS (*disk_status)(void *);
	DRESULT (*disk_ioctl)(BYTE, void *, void *);
	DRESULT (*disk_write)(const BYTE *, DWORD, UINT, void *);
	DRESULT (*disk_read)(BYTE *, DWORD, UINT, void *);
	void * usrOps;
} DISKIO_LowLevelDriver_t;

/* Disk Status Bits (DSTATUS) */
#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */

/* Command code for disk_ioctrl function */
/* Generic command (used by FatFs) */
#define CTRL_SYNC			0	/* Flush disk cache (for write functions) */
#define GET_SECTOR_COUNT	1	/* Get media size (for only f_mkfs()) */
#define GET_SECTOR_SIZE		2	/* Get sector size (for multiple sector size (_MAX_SS >= 1024)) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (for only f_mkfs()) */
#define CTRL_ERASE_SECTOR	4	/* Force erased a block of sectors (for only _USE_ERASE) */

/* Generic command (not used by FatFs) */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */
#define CTRL_FORMAT			8	/* Create physical format on the media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

/**
 * @brief  Adds new driver for DISKIO fatfs structure
 * @param  *Driver: Pointer to @ref DISKIO_LowLevelDriver_t with filled structure
 * @param  DriverName: User can use 2 custom drivers. This parameter can be a value of @ref Driver_t enumeration
 * @retval None
 */
void FATFS_AddDriver(DISKIO_LowLevelDriver_t* Driver, uint8_t driverVolume);

/* Drivers function declarations */
DSTATUS SD_disk_initialize(void *);
DSTATUS SD_disk_status(void *);
DRESULT SD_disk_ioctl(BYTE cmd, void *buff, void *);
DRESULT SD_disk_read(BYTE *buff, DWORD sector, UINT count, void *);
DRESULT SD_disk_write(const BYTE *buff, DWORD sector, UINT count, void *);
void SD_disk_timerproc (void*);

#endif
