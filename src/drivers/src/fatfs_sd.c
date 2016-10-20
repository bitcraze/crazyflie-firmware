/*------------------------------------------------------------------------*/
/* STM32F100: MMCv3/SDv1/SDv2 (SPI mode) control module                   */
/*------------------------------------------------------------------------*/
/*
/  Copyright (C) 2014, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

#include "diskio.h"
#include "fatfs_sd.h"

// MMC/SD command
#define CMD0    (0)       // GO_IDLE_STATE
#define CMD1    (1)       // SEND_OP_COND (MMC)
#define ACMD41  (0x80+41) // SEND_OP_COND (SDC)
#define CMD8    (8)       // SEND_IF_COND
#define CMD9    (9)       // SEND_CSD
#define CMD10   (10)      // SEND_CID
#define CMD12   (12)      // STOP_TRANSMISSION
#define ACMD13  (0x80+13) // SD_STATUS (SDC)
#define CMD16   (16)      // SET_BLOCKLEN
#define CMD17   (17)      // READ_SINGLE_BLOCK
#define CMD18   (18)      // READ_MULTIPLE_BLOCK
#define CMD23   (23)      // SET_BLOCK_COUNT (MMC)
#define ACMD23  (0x80+23) // SET_WR_BLK_ERASE_COUNT (SDC)
#define CMD24   (24)      // WRITE_BLOCK
#define CMD25   (25)      // WRITE_MULTIPLE_BLOCK
#define CMD32   (32)      // ERASE_ER_BLK_START
#define CMD33   (33)      // ERASE_ER_BLK_END
#define CMD38   (38)      // ERASE
#define CMD55   (55)      // APP_CMD
#define CMD58   (58)      // READ_OCR

static const int INT_TIMEOUT = 0;
static const int INT_ERROR = 0;
static const int INT_READY = 1;


static int waitForCardReady(sdSpiOps_t *ops, UINT timeoutMs) {
  BYTE d;

  ops->Timer2 = timeoutMs;
  do {
    d = ops->xchg_spi(0xFF);
  } while (d != 0xFF && ops->Timer2);

  return (d == 0xFF) ? INT_READY : INT_TIMEOUT;
}


// Deselect card and release SPI
static void deselect(sdSpiOps_t *ops) {
  ops->cs_high();

  // Dummy clock (force DO hi-z for multiple slave SPI)
  ops->xchg_spi(0xFF);
}


static int select(sdSpiOps_t *ops) {
  ops->cs_low();

  // Dummy clock (force DO enabled)
  ops->xchg_spi(0xFF);

  if (waitForCardReady(ops, 500)) {
    return INT_READY;
  }

  deselect(ops);
  return INT_TIMEOUT;
}


static int receiveDataBlock(sdSpiOps_t *ops, BYTE *data, UINT length) {
  BYTE token;

  ops->Timer1 = 200;
  do {
    token = ops->xchg_spi(0xFF);
  } while ((token == 0xFF) && ops->Timer1);

  if(token != 0xFE){
    return INT_ERROR;
  }

  // Store trailing data to the buffer
  ops->rcvr_spi_multi(data, length);

  // Discard CRC
  ops->xchg_spi(0xFF);
  ops->xchg_spi(0xFF);

  return INT_READY;
}


static int transmitDataBlock(sdSpiOps_t *ops, const BYTE *data, BYTE token) {
  if (!waitForCardReady(ops, 500)) {
    return INT_TIMEOUT;
  }
  
  ops->xchg_spi(token);

  // Send data if token is other than StopTran
  if (token != 0xFD) {
    ops->xmit_spi_multi(data, 512);

    // Dummy CRC
    ops->xchg_spi(0xFF); ops->xchg_spi(0xFF);

    BYTE resp = ops->xchg_spi(0xFF);

    if ((resp & 0x1F) != 0x05) {
      // Data packet was not accepted
      return INT_ERROR;
    }
  }

  return INT_READY;
}


static BYTE sendCommand(sdSpiOps_t *ops, BYTE cmd, DWORD arg) {
  // Send a CMD55 prior to ACMD<n>
  if (cmd & 0x80) {
    cmd &= 0x7F;
    BYTE res = sendCommand(ops, CMD55, 0);

    if (res > 1) {
      return res;
    }
  }

  // Select the card and wait for ready except to stop multiple block read
  if(cmd == CMD0) {
    deselect(ops);
    ops->cs_low();
  } else if (cmd != CMD12) {
    deselect(ops);
    if (!select(ops)) {
      return 0xFF;
    }
  }

  // Start + command index
  ops->xchg_spi(0x40 | cmd);

  // Argument[31..24]
  ops->xchg_spi((BYTE)(arg >> 24));
  // Argument[23..16]
  ops->xchg_spi((BYTE)(arg >> 16));
  // Argument[15..8]
  ops->xchg_spi((BYTE)(arg >> 8));
  // Argument[7..0]
  ops->xchg_spi((BYTE)arg);

  // Dummy CRC + Stop
  BYTE crc = 0x01;
  if (cmd == CMD0) {
    crc = 0x95;
  }
  if (cmd == CMD8) {
    crc = 0x87;
  }
  ops->xchg_spi(crc);

  if (cmd == CMD12) {
    // Discard one byte when CMD12
    ops->xchg_spi(0xFF);
  }

  // Receive command resp
  {
    BYTE maxTries = 10;
    BYTE res;
    do {
      res = ops->xchg_spi(0xFF);
    } while ((res & 0x80) && --maxTries);

    // Return value: R1 resp (bit7==1:Failed to send)
    return res;
  }
}


DSTATUS SD_disk_initialize(void* usrOps) {
  sdSpiOps_t *ops = (sdSpiOps_t *)usrOps;

  if (!usrOps) {
    return RES_PARERR;
  }

  ops->init_spi();

  // Check is card is inserted
  if (ops->Stat & STA_NODISK) {
    return ops->Stat;
  }

  ops->set_slow_spi_mode();

  // Send 80 dummy clocks (10 * 8)
  for (BYTE n = 10; n; n--) {
    ops->xchg_spi(0xFF);
  }

  // TODO krri: Is this needed
  for (BYTE n = 100, res = 0; n && res != 1; n--) {
    res = sendCommand(ops, CMD0, 0);
  }

  BYTE cardType = 0;

  // Put the card SPI/Idle state
  if (sendCommand(ops, CMD0, 0) == 1) {
    ops->Timer1 = 1000;

    // SDv2?
    if (sendCommand(ops, CMD8, 0x1AA) == 1) {
      BYTE ocr[4];

      // Get 32 bit return value of R7 resp
      for (BYTE n = 0; n < 4; n++) {
        ocr[n] = ops->xchg_spi(0xFF);
      }

      // Does the card support vcc of 2.7-3.6V?
      if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
        // Wait for end of initialization with ACMD41(HCS)
        while (ops->Timer1 && sendCommand(ops, ACMD41, 1UL << 30)) { /* Do nothing */ }

        // Check CCS bit in the OCR
        if (ops->Timer1 && sendCommand(ops, CMD58, 0) == 0) {
          for (BYTE n = 0; n < 4; n++) {
            ocr[n] = ops->xchg_spi(0xFF);
          }

          // Card id SDv2
          cardType = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
        }
      }
    } else {
      BYTE cmd;

      // SDv1 or MMC?
      if (sendCommand(ops, ACMD41, 0) <= 1)  {
        // SDv1
        cardType = CT_SD1;
        cmd = ACMD41;
      } else {
        // MMCv3
        cardType = CT_MMC;
        cmd = CMD1;
      }

      // Wait for end of initialization
      while (ops->Timer1 && sendCommand(ops, cmd, 0)) { /* Do nothing */ }

      // Set block length to 512
      if (!ops->Timer1 || sendCommand(ops, CMD16, 512) != 0) {
        cardType = 0;
      }
    }
  }

  ops->CardType = cardType;
  deselect(ops);

  if (cardType) {
    // OK
    ops->set_fast_spi_mode();

    // Clear STA_NOINIT flag
    ops->Stat &= ~STA_NOINIT;
  } else {   /* Failed */
    ops->Stat = STA_NOINIT;
  }

  return ops->Stat;
}


DSTATUS SD_disk_status(void * usrOps) {
 sdSpiOps_t *ops = (sdSpiOps_t *)usrOps;

 return ops->Stat;
}


DRESULT SD_disk_read(BYTE *buff, DWORD sector, UINT count, void * usrOps) {
  sdSpiOps_t *ops = (sdSpiOps_t *)usrOps;

  if (!usrOps || !count) {
    return RES_PARERR;
  }

  // Check if drive is ready
  if (ops->Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  // LBA ot BA conversion (byte addressing cards)
  if (!(ops->CardType & CT_BLOCK)) {
    sector *= 512;
  }

  if (count == 1) {
    // Single sector read
    // READ_SINGLE_BLOCK
    if ((sendCommand(ops, CMD17, sector) == 0) && receiveDataBlock(ops, buff, 512)) {
      count = 0;
    }
  } else {
    // Multiple sector read
    // READ_MULTIPLE_BLOCK
    if (sendCommand(ops, CMD18, sector) == 0) {
      do {
        if (!receiveDataBlock(ops, buff, 512)) {
          break;
        }
        buff += 512;
      } while (--count);

      // STOP_TRANSMISSION
      sendCommand(ops, CMD12, 0);
    }
  }

  deselect(ops);

  return count ? RES_ERROR : RES_OK;
}


DRESULT SD_disk_write(const BYTE *buff, DWORD sector, UINT count, void * usrOps) {
  sdSpiOps_t *ops = (sdSpiOps_t *)usrOps;

  if (!usrOps || !count) {
    return RES_PARERR;
  }

  // Check drive status
  if (ops->Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  // Check write protect
  if (ops->Stat & STA_PROTECT) {
    return RES_WRPRT;
  }

  // LBA ==> BA conversion (byte addressing cards)
  if (!(ops->CardType & CT_BLOCK)) {
    sector *= 512;
  }

  if (count == 1) {
    // Single sector write
    // WRITE_BLOCK
    if ((sendCommand(ops, CMD24, sector) == 0) && transmitDataBlock(ops, buff, 0xFE)) {
      count = 0;
    }
  } else {
    // Multiple sector write
    // Set number of sectors
    if (ops->CardType & CT_SDC) sendCommand(ops, ACMD23, count);

    // WRITE_MULTIPLE_BLOCK
    if (sendCommand(ops, CMD25, sector) == 0) {
      do {
        if (!transmitDataBlock(ops, buff, 0xFC)) {
          break;
        }

        buff += 512;
      } while (--count);

      // STOP_TRAN
      if (!transmitDataBlock(ops, 0, 0xFD)) {
        count = 1;
      }
    }
  }

  deselect(ops);

  return count ? RES_ERROR : RES_OK;
}


DRESULT SD_disk_ioctl(BYTE cmd, void* buff, void* usrOps) {
  sdSpiOps_t *ops = (sdSpiOps_t *)usrOps;
  BYTE csd[16];

  if (!usrOps) {
    return RES_PARERR;
  }

  // Check if drive is ready
  if (ops->Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  DRESULT res = RES_ERROR;

  switch (cmd) {
    case CTRL_SYNC :
      // Wait for end of internal write process of the drive
      if (select(ops)) {
        res = RES_OK;
      }
      break;

    case GET_SECTOR_COUNT :
      // Get drive capacity in unit of sector (DWORD)
      if ((sendCommand(ops, CMD9, 0) == 0) && receiveDataBlock(ops, csd, 16)) {
        if ((csd[0] >> 6) == 1) {
          // SDC ver 2.00
          DWORD csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
          *(DWORD*)buff = csize << 10;
        } else {
          // SDC ver 1.XX or MMC ver 3
          BYTE n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          DWORD csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
          *(DWORD*)buff = csize << (n - 9);
        }
        res = RES_OK;
      }
      break;

    case GET_BLOCK_SIZE : /* Get erase block size in unit of sector (DWORD) */
      if (ops->CardType & CT_SD2) {
        // SDC ver 2.00
        if (sendCommand(ops, ACMD13, 0) == 0) {
          // Read SD status
          ops->xchg_spi(0xFF);
          if (receiveDataBlock(ops, csd, 16)) {
            // Read partial block
            for (BYTE n = 64 - 16; n; n--) {
              // Purge trailing data
              ops->xchg_spi(0xFF);
            }

            *(DWORD*)buff = 16UL << (csd[10] >> 4);
            res = RES_OK;
          }
        }
      } else {
        // SDC ver 1.XX or MMC
        if ((sendCommand(ops, CMD9, 0) == 0) && receiveDataBlock(ops, csd, 16)) {
          // Read CSD
          if (ops->CardType & CT_SD1) {
            // SDC ver 1.XX
            *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
          } else {
            // MMC
            *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
          }

          res = RES_OK;
        }
      }
      break;

    case CTRL_ERASE_SECTOR :
      // Erase a block of sectors (used when _USE_ERASE == 1)
      // Check if the card is SDC
      if (!(ops->CardType & CT_SDC)) {
        break;
      }

      // Get CSD
      if (SD_disk_ioctl(MMC_GET_CSD, csd, ops)) {
        break;
      }

      // Check if sector erase can be applied to the card
      if (!(csd[0] >> 6) && !(csd[10] & 0x40)) {
        break;
      }

      // Load sector block
      {
        DWORD* dp = buff;
        DWORD st = dp[0];
        DWORD ed = dp[1];
        if (!(ops->CardType & CT_BLOCK)) {
          st *= 512; ed *= 512;
        }

        /* Erase sector block */
        if (sendCommand(ops, CMD32, st) == 0 && sendCommand(ops, CMD33, ed) == 0 && sendCommand(ops, CMD38, 0) == 0 && waitForCardReady(ops, 30000)) {
          // FatFs does not check result of this command
          res = RES_OK;
        }
      }
      break;

    default:
      res = RES_PARERR;
  }

  deselect(ops);

  return res;
}


// This function must be called from timer interrupt routine in period of 1 ms to generate card control timing.
void SD_disk_timerproc (void* usrOps) {
  sdSpiOps_t *ops = (sdSpiOps_t *)usrOps;

  {
    WORD n = ops->Timer1;
    if (n) {
      ops->Timer1 = --n;
    }
  }

  {
    WORD n = ops->Timer2;
    if (n) {
      ops->Timer2 = --n;
    }
  }


  BYTE s = ops->Stat;

  // Write enabled
  s &= ~STA_PROTECT;

  // Card is in socket
  s &= ~STA_NODISK;

  ops->Stat = s;
}

