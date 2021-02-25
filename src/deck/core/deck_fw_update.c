/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 * deck_fw_update.c - implements functionality for updating firmware
 *                    and binary data in decks.
 *
 */

#include <string.h>
#include "deck_fw_update.h"
#include "deck_core.h"
#include "mem.h"
#include "test_support.h"

static const uint32_t DECK_FW_MAX_SIZE = 0x10000000;

// TODO krri is size actually used? What to return?
static uint32_t handleMemGetSize(void) { return 0x1000000; }
TESTABLE_STATIC bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_DECK_MEM,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

// TODO krri document
// uint32_t requiredHash;
// uint32_t requiredSize;
// uint32_t uploadAddress;
// bool isUpdateRequired;
// bool isUpdateable;
#define INFO_SIZE 14


static void populateDeckFwInfo(uint8_t fwInfo[], const uint8_t deckNr) {
    DeckInfo* deckInf = deckInfo(deckNr);
    memset(fwInfo, 0, INFO_SIZE);

    if (deckInf->driver->fwUpdate) {
        memcpy(&fwInfo[0], &deckInf->driver->fwUpdate->requiredHash, 4);
        memcpy(&fwInfo[4], &deckInf->driver->fwUpdate->requiredSize, 4);

        uint32_t startAddr = deckNr * DECK_FW_MAX_SIZE;
        memcpy(&fwInfo[8], &startAddr, 4);

        // TODO krri how to find out?
        // IsUpdateRequired
        fwInfo[12] = false;

        // isUpdateable
        fwInfo[13] = 1;
    }
}

TESTABLE_STATIC bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
    int index = 0;
    uint8_t bytesLeft = readLen;
    int nrOfDecks = deckCount();

    // First byte is the nr of decks
    if (0 == (memAddr + index) && bytesLeft > 0) {
        buffer[index] = nrOfDecks;
        index++;
        bytesLeft--;
    }

    while (bytesLeft > 0) {
        int deckNr = (memAddr + index - 1) / INFO_SIZE;
        if (deckNr >= nrOfDecks) {
            return true;
        }

        uint8_t fwInfo[INFO_SIZE];
        populateDeckFwInfo(fwInfo, deckNr);

        int startAddrOfThisInfo = deckNr * INFO_SIZE + 1;
        int firstByteToUse = memAddr + index - startAddrOfThisInfo;

        int bytesToUse = INFO_SIZE - firstByteToUse;
        if (bytesLeft < bytesToUse) {
            bytesToUse = bytesLeft;
        }

        memcpy(buffer + index, (uint8_t*)(&fwInfo) + firstByteToUse, bytesToUse);
        bytesLeft -= bytesToUse;
        index += bytesToUse;
    }

    // ToDo: call the read function for the deck if this is called within a deck address space

    return true;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
    const int deckNr = memAddr / DECK_FW_MAX_SIZE;
    DeckInfo* deckInf = deckInfo(deckNr);
    if (deckInf) {
        if (deckInf->driver->fwUpdate) {
            const uint32_t uploadAddress = deckNr * DECK_FW_MAX_SIZE;
            deckInf->driver->fwUpdate->write(memAddr - uploadAddress, writeLen, buffer);
        }
    }

    return true;
}

void deckFwUpdateInit() {
    memoryRegisterHandler(&memDef);
}
