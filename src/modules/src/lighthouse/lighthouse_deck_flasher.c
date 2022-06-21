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
#include <stdbool.h>

#define DEBUG_MODULE "LHFL"
#include "debug.h"

#include "lighthouse.h"
#include "deck_core.h"
#include "lh_bootloader.h"
#include "lighthouse_deck_flasher.h"
#include "crc32.h"
#include "mem.h"

#include "FreeRTOS.h"
#include "task.h"

#ifdef LH_FLASH_BOOTLOADER
#include "lh_flasher.h"
#endif

#define READ_BUFFER_LENGTH 64
// We read the version string in the read buffer and we need one byte for null termination
#define VERSION_STRING_MAX_LENGTH (READ_BUFFER_LENGTH - 1)

static bool inBootloaderMode = true;
static bool hasStarted = false;

bool lighthouseDeckFlasherCheckVersionAndBoot() {
  lhblInit();

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
  static char deckBitstream[READ_BUFFER_LENGTH];
  lhblFlashRead(LH_FW_ADDR, VERSION_STRING_MAX_LENGTH, (uint8_t*)deckBitstream);
  deckBitstream[VERSION_STRING_MAX_LENGTH] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);

  // Checking that the bitstream has the right checksum
  crc32Context_t crcContext;
  crc32ContextInit(&crcContext);

  for (int i=0; i<=LIGHTHOUSE_BITSTREAM_SIZE; i+=READ_BUFFER_LENGTH) {
    int length = ((i + READ_BUFFER_LENGTH) < LIGHTHOUSE_BITSTREAM_SIZE)?READ_BUFFER_LENGTH:LIGHTHOUSE_BITSTREAM_SIZE-i;
    lhblFlashRead(LH_FW_ADDR + i, length, (uint8_t*)deckBitstream);
    crc32Update(&crcContext, deckBitstream, length);
  }

  uint32_t crc = crc32Out(&crcContext);
  bool pass = crc == LIGHTHOUSE_BITSTREAM_CRC;
  DEBUG_PRINT("Bitstream CRC32: %x %s\n", (int)crc, pass?"[PASS]":"[FAIL]");

  // Launch LH deck FW
  if (pass) {
    DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
    lhblBootToFW();
    inBootloaderMode = false;
  } else {
    DEBUG_PRINT("The deck bitstream does not match the required bitstream.\n");
    DEBUG_PRINT("We require lighthouse bitstream of size %d and CRC32 %x.\n", LIGHTHOUSE_BITSTREAM_SIZE, LIGHTHOUSE_BITSTREAM_CRC);
    DEBUG_PRINT("Leaving the deck in bootloader mode ...\n");
  }

  hasStarted = true;

  return pass;
}

bool lighthouseDeckFlasherRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer)
{
  if (inBootloaderMode) {
    return lhblFlashRead(LH_FW_ADDR + memAddr, readLen, buffer);
  } else {
    return false;
  }
}

bool lighthouseDeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer, const DeckMemDef_t* memDef)
{
  bool pass;
  if (memAddr == 0) {
    pass = lhblFlashEraseFirmware();

    if (pass == false) {
      return false;
    }
  }

  uint32_t address = LH_FW_ADDR + memAddr;

  // This function can be called with a buffer spanning 2 pages
  // Try to find the first byte of the second page if this is the case
  int pageSplit = 0;
  for (int i=0; i<writeLen; i++) {
    if (((address + i) & 0x0ff) == 0) {
      pageSplit = i;
      break;
    }
  }

  // Flashing first page
  if (pageSplit > 0) {
    pass = lhblFlashWritePage(address, pageSplit, buffer);

    if (pass == false) {
      return pass;
    }
  }

  // Flashing second page if necessary
  if ((writeLen-pageSplit) > 0) {
    pass = lhblFlashWritePage(address+pageSplit, writeLen-pageSplit, &buffer[pageSplit]);
  }

  return pass;
}

uint8_t lighthouseDeckFlasherPropertiesQuery() {
  uint8_t result = 0;

  if (hasStarted) {
    result |= DECK_MEMORY_MASK_STARTED;
  }

  if (inBootloaderMode) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE | DECK_MEMORY_MASK_UPGRADE_REQUIRED;
  }

  return result;
}
