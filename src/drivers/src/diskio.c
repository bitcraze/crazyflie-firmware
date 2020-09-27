/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "ff.h"
#include "fatfs_sd.h"
#include "cfassert.h"

///* Make driver structure */
DISKIO_LowLevelDriver_t FATFS_LowLevelDrivers[FF_VOLUMES];

void FATFS_AddDriver(DISKIO_LowLevelDriver_t* Driver, uint8_t driverVolume)
{
  ASSERT(driverVolume < FF_VOLUMES);
  /* Add to structure */
  FATFS_LowLevelDrivers[driverVolume].disk_initialize = Driver->disk_initialize;
  FATFS_LowLevelDrivers[driverVolume].disk_status = Driver->disk_status;
  FATFS_LowLevelDrivers[driverVolume].disk_ioctl = Driver->disk_ioctl;
  FATFS_LowLevelDrivers[driverVolume].disk_read = Driver->disk_read;
  FATFS_LowLevelDrivers[driverVolume].disk_write = Driver->disk_write;
  FATFS_LowLevelDrivers[driverVolume].usrOps = Driver->usrOps;
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	/* Return low level status */
	if (FATFS_LowLevelDrivers[pdrv].disk_initialize) {
		return FATFS_LowLevelDrivers[pdrv].disk_initialize(FATFS_LowLevelDrivers[pdrv].usrOps);
	}
	
	/* Return parameter error */
	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	/* Return low level status */
	if (FATFS_LowLevelDrivers[pdrv].disk_status) {
		return FATFS_LowLevelDrivers[pdrv].disk_status(FATFS_LowLevelDrivers[pdrv].usrOps);
	}
	
	/* Return parameter error */
	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	/* Check count */
	if (!count) {
		return RES_PARERR;
	}
	
	/* Return low level status */
	if (FATFS_LowLevelDrivers[pdrv].disk_read) {
		return FATFS_LowLevelDrivers[pdrv].disk_read(buff, sector, count,
		                                             FATFS_LowLevelDrivers[pdrv].usrOps);
	}
	
	/* Return parameter error */
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	/* Check count */
	if (!count) {
		return RES_PARERR;
	}
	
	/* Return low level status */
	if (FATFS_LowLevelDrivers[pdrv].disk_write) {
		return FATFS_LowLevelDrivers[pdrv].disk_write(buff, sector, count,
		                                              FATFS_LowLevelDrivers[pdrv].usrOps);
	}
	
	/* Return parameter error */
	return RES_PARERR;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	/* Return low level status */
	if (FATFS_LowLevelDrivers[pdrv].disk_ioctl) {
		return FATFS_LowLevelDrivers[pdrv].disk_ioctl(cmd, buff,
		                                              FATFS_LowLevelDrivers[pdrv].usrOps);
	}
	
	/* Return parameter error */
	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Get time for fatfs for files                                          */
/*-----------------------------------------------------------------------*/
__attribute__((weak)) DWORD get_fattime(void) {
	/* Returns current time packed into a DWORD variable */
	return	  ((DWORD)(2016 - 1980) << 25)	/* Year 2016 */
			| ((DWORD)1 << 21)				/* Month 1 */
			| ((DWORD)1 << 16)				/* Mday 1 */
			| ((DWORD)0 << 11)				/* Hour 0 */
			| ((DWORD)0 << 5)				/* Min 0 */
			| ((DWORD)0 >> 1);				/* Sec 0 */
}
