/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * lighthouse_deck_flasher.h - handles flashing of FPGA binaries on the
 * Lighthouse deck
 */

#include <string.h>
#include <stdlib.h>

#define DEBUG_MODULE "LHFL"
#include "debug.h"
#include "lh_bootloader.h"
#include "lighthouse_deck_flasher.h"
#include "crc32.h"
#include "mem.h"

#ifdef LH_FLASH_BOOTLOADER
#include "lh_flasher.h"
#endif

#define BITSTREAM_CRC 0xe2889216
#define BITSTREAM_SIZE 104092

static uint32_t getFirmwareSize(void);
static bool readFirmware(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool writeFirmware(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

static const MemoryHandlerDef_t flasherMemory = {
  .type = MEM_TYPE_DECK_FW,
  .getSize = getFirmwareSize,
  .read = readFirmware,
  .write = writeFirmware,
};

void lighthouseDeckFlasherInit()
{
  // Register access to the flash in the memory subsystem
  memoryRegisterHandler(&flasherMemory);
}

bool lighthouseDeckFlasherCheckVersionAndBoot() {
  lhblInit(I2C1_DEV);

  #ifdef LH_FLASH_BOOTLOADER
  // Flash deck bootloader using SPI (factory and recovery flashing)
  lhflashInit();
  lhflashFlashBootloader();
  #endif

  uint8_t bootloaderVersion = 0;
  if (lhblGetVersion(&bootloaderVersion) == false) {
    DEBUG_PRINT("Cannot communicate with lighthouse bootloader, aborting!\n");
    return false;
  }
  DEBUG_PRINT("Lighthouse bootloader version: %d\n", bootloaderVersion);

  // Wakeup mem
  lhblFlashWakeup();
  vTaskDelay(M2T(1));

  // Decoding bitstream version for console
  static char deckBitstream[65];
  lhblFlashRead(LH_FW_ADDR, 64, (uint8_t*)deckBitstream);
  deckBitstream[64] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);

  // Checking that the bitstream has the right checksum
  crc32Context_t crcContext;
  crc32ContextInit(&crcContext);

  for (int i=0; i<=BITSTREAM_SIZE; i+=64) {
    int length = ((i+64)<BITSTREAM_SIZE)?64:BITSTREAM_SIZE-i;
    lhblFlashRead(LH_FW_ADDR + i, length, (uint8_t*)deckBitstream);
    crc32Update(&crcContext, deckBitstream, length);
  }

  uint32_t crc = crc32Out(&crcContext);
  bool pass = crc == BITSTREAM_CRC;
  DEBUG_PRINT("Bitstream CRC32: %x %s\n", (int)crc, pass?"[PASS]":"[FAIL]");

  // Launch LH deck FW
  if (pass) {
    DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
    lhblBootToFW();
  } else {
    DEBUG_PRINT("The deck bitstream does not match the required bitstream.\n");
    DEBUG_PRINT("We require lighthouse bitstream of size %d and CRC32 %x.\n", BITSTREAM_SIZE, BITSTREAM_CRC);
    DEBUG_PRINT("Leaving the deck in bootloader mode ...\n");
  }
  
  return pass;
}

static uint32_t getFirmwareSize(void)
{
  return 256*1024;
}

static bool readFirmware(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer)
{
  return lhblFlashRead(LH_FW_ADDR + memAddr, readLen, buffer);
}

static bool writeFirmware(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer)
{
  return false;
}
