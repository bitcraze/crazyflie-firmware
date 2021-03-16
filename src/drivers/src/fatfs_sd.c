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

#include "ff.h"
#include "diskio.h"
#include "fatfs_sd.h"

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC 0x01              /* MMC ver 3 */
#define CT_SD1 0x02              /* SD ver 1 */
#define CT_SD2 0x04              /* SD ver 2 */
#define CT_SDC (CT_SD1 | CT_SD2) /* SD */
#define CT_BLOCK 0x08            /* Block addressing */

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

static uint8_t powerFlag;

static int waitForCardReady(sdSpiContext_t *context, UINT timeoutMs) {
  BYTE d;
  uint32_t timeout = timeoutMs;

  while ((d = context->xchgSpi(0xFF)) != 0xFF && timeout)
  {
    // Waiting can take a while so release the SPI bus in between
    context->csHigh(0);
    context->delayMs(1);
    context->csLow();
    timeout--;
  }

  return (d == 0xFF) ? INT_READY : INT_TIMEOUT;
}

// Deselect card and release SPI
static void deselect(sdSpiContext_t *context) {
  context->csHigh(1);

  // Dummy clock (force DO hi-z for multiple slave SPI)
  // which has been moved to higher layer.
  //context->xchgSpi(0xFF);
}


static int select(sdSpiContext_t *context) {
  context->csLow();

  // Dummy clock (force DO enabled)
  context->xchgSpi(0xFF);

  if (waitForCardReady(context, 500)) {
    return INT_READY;
  }

  deselect(context);
  return INT_TIMEOUT;
}

/* power on */
static void powerOn(sdSpiContext_t *context)
{
  uint8_t cmdbuff[6];
  uint32_t cnt = 0x1FFF;

  deselect(context);
  // Send 80 dummy clocks (10 * 8) to wake up
  for (BYTE n = 10; n; n--) {
    context->xchgSpi(0xFF);
  }
  /* slave select without wait*/
  context->csLow();

  /* make idle state */
  cmdbuff[0] = 0x40 | CMD0;   /* CMD0:GO_IDLE_STATE */
  cmdbuff[1] = 0;
  cmdbuff[2] = 0;
  cmdbuff[3] = 0;
  cmdbuff[4] = 0;
  cmdbuff[5] = 0x95;   /* CRC */

  context->xmitSpiMulti(cmdbuff, sizeof(cmdbuff));

  /* wait response */
  while ((context->xchgSpi(0xFF) != 0x01) && cnt)
  {
    cnt--;
  }

  deselect(context);

  powerFlag = 1;
}

/* power off */
static void powerOff(sdSpiContext_t *context)
{
  powerFlag = 0;
}

/* check power flag */
static uint8_t checkPower(sdSpiContext_t *context)
{
  return powerFlag;
}


static int receiveDataBlock(sdSpiContext_t *context, BYTE *data, UINT length) {
  BYTE token;

  context->timer1 = 2; // In centi-seconds
  do {
    token = context->xchgSpi(0xFF);
  } while ((token == 0xFF) && context->timer1);

  if(token != 0xFE){
    return INT_ERROR;
  }

  // Store trailing data to the buffer
  context->rcvrSpiMulti(data, length);

  // Discard CRC
  context->xchgSpi(0xFF);
  context->xchgSpi(0xFF);

  return INT_READY;
}


static int transmitDataBlock(sdSpiContext_t *context, const BYTE *data, BYTE token) {
  if (!waitForCardReady(context, 500)) {
    return INT_TIMEOUT;
  }
  
  context->xchgSpi(token);

  // Send data if token is other than StopTran
  if (token != 0xFD) {
    context->xmitSpiMulti(data, 512);

    // Dummy CRC
    context->xchgSpi(0xFF);
    context->xchgSpi(0xFF);

    BYTE resp = context->xchgSpi(0xFF);

    if ((resp & 0x1F) != 0x05) {
      // Data packet was not accepted
      return INT_ERROR;
    }
  }

  return INT_READY;
}


static BYTE sendCommand(sdSpiContext_t *context, BYTE cmd, DWORD arg) {
  BYTE cmdbuff[6];

  // Send a CMD55 prior to ACMD<n>
  if (cmd & 0x80) {
    cmd &= 0x7F;
    BYTE res = sendCommand(context, CMD55, 0);

    if (res > 1) {
      return res;
    }
  }


  if (waitForCardReady(context, 500) != INT_READY) {
    return INT_ERROR;
  }


  // Start + command index
  cmdbuff[0] = (0x40 | cmd);

  // Argument[31..24]
  cmdbuff[1] = ((BYTE)(arg >> 24));
  // Argument[23..16]
  cmdbuff[2] = ((BYTE)(arg >> 16));
  // Argument[15..8]
  cmdbuff[3] = ((BYTE)(arg >> 8));
  // Argument[7..0]
  cmdbuff[4] = ((BYTE)arg);

  // Dummy CRC + Stop
  BYTE crc = 0x01;
  if (cmd == CMD0) {
    crc = 0x95;
  }
  if (cmd == CMD8) {
    crc = 0x87;
  }
  cmdbuff[5] = (crc);

  context->xmitSpiMulti(cmdbuff, 6);


  if (cmd == CMD12) {
    // Discard one byte when CMD12
    context->xchgSpi(0xFF);
  }

  // Receive command resp
  {
    BYTE maxTries = 10;
    BYTE res;
    do {
      res = context->xchgSpi(0xFF);
    } while ((res & 0x80) && --maxTries);

    // Return value: R1 resp (bit7==1:Failed to send)
    return res;
  }
}


DSTATUS SD_disk_initialize(void* usrOps) {
  sdSpiContext_t *context = (sdSpiContext_t *)usrOps;
  BYTE cardType = 0;

  if (!usrOps) {
    return RES_PARERR;
  }

  context->initSpi();

  // Check is card is inserted
  if (context->stat & STA_NODISK) {
    return context->stat;
  }

  if (checkPower(context))
  {
    powerOff(context);
  }

  context->setSlowSpiMode();

  powerOn(context);

  if (select(context) == INT_READY)
  {
    // Put the card SPI/Idle state
    if (sendCommand(context, CMD0, 0) == 1) {
      context->timer1 = 10;

      // SDv2?
      if (sendCommand(context, CMD8, 0x1AA) == 1) {
        BYTE ocr[4];

        // Get 32 bit return value of R7 resp
        for (BYTE n = 0; n < 4; n++) {
          ocr[n] = context->xchgSpi(0xFF);
        }

        // Does the card support vcc of 2.7-3.6V?
        if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
          // Wait for end of initialization with ACMD41(HCS)
          while (context->timer1 && sendCommand(context, ACMD41, 1UL << 30)) { /* Do nothing */ }

          // Check CCS bit in the OCR
          if (context->timer1 && sendCommand(context, CMD58, 0) == 0) {
            for (BYTE n = 0; n < 4; n++) {
              ocr[n] = context->xchgSpi(0xFF);
            }

            // Card id SDv2
            cardType = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
          }
        }
      } else {
        BYTE cmd;

        // SDv1 or MMC?
        if (sendCommand(context, ACMD41, 0) <= 1)  {
          // SDv1
          cardType = CT_SD1;
          cmd = ACMD41;
        } else {
          // MMCv3
          cardType = CT_MMC;
          cmd = CMD1;
        }

        // Wait for end of initialization
        while (context->timer1 && sendCommand(context, cmd, 0)) { /* Do nothing */ }

        // Set block length to 512
        if (!context->timer1 || sendCommand(context, CMD16, 512) != 0) {
          cardType = 0;
        }
      }
    }

    context->cardType = cardType;
    deselect(context);
  }

  if (cardType) {
    // OK
    context->setFastSpiMode();

    // Clear STA_NOINIT flag
    context->stat &= ~STA_NOINIT;
  } else {   /* Failed */
    context->stat = STA_NOINIT;
  }

  return context->stat;
}


DSTATUS SD_disk_status(void * usrOps) {
 sdSpiContext_t *context = (sdSpiContext_t *)usrOps;

 return context->stat;
}


DRESULT SD_disk_read(BYTE *buff, DWORD sector, UINT count, void * usrOps) {
  sdSpiContext_t *context = (sdSpiContext_t *)usrOps;

  if (!usrOps || !count) {
    return RES_PARERR;
  }

  // Check if drive is ready
  if (context->stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  // LBA ot BA conversion (byte addressing cards)
  if (!(context->cardType & CT_BLOCK)) {
    sector *= 512;
  }

  if (select(context) == INT_READY)
  {

    if (count == 1) {
      // Single sector read
      // READ_SINGLE_BLOCK
      if ((sendCommand(context, CMD17, sector) == 0) && receiveDataBlock(context, buff, 512)) {
        count = 0;
      }
    } else {
      // Multiple sector read
      // READ_MULTIPLE_BLOCK
      if (sendCommand(context, CMD18, sector) == 0) {
        do {
          if (!receiveDataBlock(context, buff, 512)) {
            break;
          }
          buff += 512;
        } while (--count);

        // STOP_TRANSMISSION
        sendCommand(context, CMD12, 0);
      }
    }

    deselect(context);
  }

  return count ? RES_ERROR : RES_OK;
}


DRESULT SD_disk_write(const BYTE *buff, DWORD sector, UINT count, void * usrOps) {
  sdSpiContext_t *context = (sdSpiContext_t *)usrOps;

  if (!usrOps || !count) {
    return RES_PARERR;
  }

  // Check drive status
  if (context->stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  // Check write protect
  if (context->stat & STA_PROTECT) {
    return RES_WRPRT;
  }

  // LBA ==> BA conversion (byte addressing cards)
  if (!(context->cardType & CT_BLOCK)) {
    sector *= 512;
  }

  if (select(context) == INT_READY)
  {
    if (count == 1) {
      // Single sector write
      // WRITE_BLOCK
      if ((sendCommand(context, CMD24, sector) == 0) && transmitDataBlock(context, buff, 0xFE)) {
        count = 0;
      }
    } else {
      // Multiple sector write
      // Set number of sectors
      if (context->cardType & CT_SDC) sendCommand(context, ACMD23, count);

      // WRITE_MULTIPLE_BLOCK
      if (sendCommand(context, CMD25, sector) == 0) {
        do {
          if (!transmitDataBlock(context, buff, 0xFC)) {
            break;
          }

          buff += 512;
        } while (--count);

        // STOP_TRAN
        if (!transmitDataBlock(context, 0, 0xFD)) {
          count = 1;
        }
      }
    }

    deselect(context);
  }

  return count ? RES_ERROR : RES_OK;
}


DRESULT SD_disk_ioctl(BYTE cmd, void* buff, void* usrOps) {
  sdSpiContext_t *context = (sdSpiContext_t *)usrOps;
  BYTE csd[16];

  if (!usrOps) {
    return RES_PARERR;
  }

  // Check if drive is ready
  if (context->stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  DRESULT res = RES_ERROR;

  if (select(context) == INT_READY)
  {
    switch (cmd) {
      case CTRL_SYNC :
        // Wait for end of internal write process of the drive
        if (waitForCardReady(context, 500)) {
          res = RES_OK;
        }
        break;

      case GET_SECTOR_COUNT :
        // Get drive capacity in unit of sector (DWORD)
        if ((sendCommand(context, CMD9, 0) == 0) &&
             receiveDataBlock(context, csd, 16))
        {
          if ((csd[0] >> 6) == 1)
          {
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
        if (context->cardType & CT_SD2)
        {
          // SDC ver 2.00
          if (sendCommand(context, ACMD13, 0) == 0)
          {
            // Read SD status
            context->xchgSpi(0xFF);
            if (receiveDataBlock(context, csd, 16))
            {
              // Read partial block
              for (BYTE n = 64 - 16; n; n--)
              {
                // Purge trailing data
                context->xchgSpi(0xFF);
              }

              *(DWORD*)buff = 16UL << (csd[10] >> 4);
              res = RES_OK;
            }
          }
        } else {
          // SDC ver 1.XX or MMC
          if ((sendCommand(context, CMD9, 0) == 0) &&
               receiveDataBlock(context, csd, 16))
          {
            // Read CSD
            if (context->cardType & CT_SD1)
            {
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

      case CTRL_TRIM :
        // Erase a block of sectors (used when _USE_ERASE == 1)
        // Check if the card is SDC
        if (!(context->cardType & CT_SDC)) {
          break;
        }

        // Get CSD
        if (SD_disk_ioctl(MMC_GET_CSD, csd, context)) {
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
          if (!(context->cardType & CT_BLOCK)) {
            st *= 512;
            ed *= 512;
          }

          /* Erase sector block */
          if (sendCommand(context, CMD32, st) == 0 &&
              sendCommand(context, CMD33, ed) == 0 &&
              sendCommand(context, CMD38, 0) == 0 &&
              waitForCardReady(context, 30000)) {
            // FatFs does not check result of this command
            res = RES_OK;
          }
        }
        break;

      default:
        res = RES_PARERR;
    }

    deselect(context);
  }

  return res;
}


// This function must be called from timer interrupt routine in period of 1 ms to generate card control timing.
// Changed to 100ms to relax timers as all waits are 200ms or more --TA
void SD_disk_timerproc (void* usrOps) {
  sdSpiContext_t *context = (sdSpiContext_t *)usrOps;

  {
    WORD n = context->timer1;
    if (n) {
      context->timer1 = --n;
    }
  }

  {
    WORD n = context->timer2;
    if (n) {
      context->timer2 = --n;
    }
  }


  BYTE s = context->stat;

  // Write enabled
  s &= ~STA_PROTECT;

  // Card is in socket
  s &= ~STA_NODISK;

  context->stat = s;
}

