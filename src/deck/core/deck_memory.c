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
 * deck_memory.c - implements functionality for reading and writing deck memory
 *                 using the memory subsystem, including updating firmware
 *                 and binary data in decks.
 *
 */

#include <string.h>
#include "deck_memory.h"
#include "deck_core.h"
#include "mem.h"
#include "test_support.h"


static const uint32_t DECK_MEM_MAX_SIZE = 0x10000000;

static uint32_t handleMemGetSize(void) { return DECK_MEM_MAX_SIZE * (DECK_MAX_COUNT + 1); }
TESTABLE_STATIC bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
TESTABLE_STATIC bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memoryDef = {
  .type = MEM_TYPE_DECK_MEM,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

typedef enum {
    MEM_PRIMARY = 0,
    MEM_SECONDARY = 1
} MemSelector;

static const uint8_t VERSION = 3;

static const int DECK_MEMORY_INFO_SIZE = 0x20;

static const int OFFS_BITFIELD1 = 0x00;
static const int OFFS_BITFIELD2 = 0x01;
static const int OFFS_REQ_HASH = 0x02;
static const int OFFS_REQ_LEN = 0x06;
static const int OFFS_BASE_ADDR = 0x0A;
static const int OFFS_NAME = 0x0E;

static const int NAME_LEN_EX_ZERO_TREM = 17;

static const uint8_t MASK_IS_VALID                = 1;
static const uint8_t MASK_IS_STARTED              = 2;
static const uint8_t MASK_SUPPORTS_READ           = 4;
static const uint8_t MASK_SUPPORTS_WRITE          = 8;
static const uint8_t MASK_SUPPORTS_UPGRADE        = 16;
static const uint8_t MASK_UPGRADE_REQUIRED        = 32;
static const uint8_t MASK_BOOTLOADER_ACTIVE       = 64;

static const uint32_t COMMAND_BASE_ADR = 0x1000;
static const uint32_t DECK_MEMORY_COMMAND_SIZE = 0x20;

static const uint8_t DECK_MEMORY_MASK_SUPPORTS_RESET_TO_FW         = 1;
static const uint8_t DECK_MEMORY_MASK_SUPPORTS_RESET_TO_BOOTLOADER = 2;

static const uint8_t DECK_MEMORY_MASK_COMMAND_RESET_TO_FW         = 1;
static const uint8_t DECK_MEMORY_MASK_COMMAND_RESET_TO_BOOTLOADER = 2;

static const uint32_t COMMAND_BITFIELD_ADR = 0x4;


static uint8_t populateBitfield1(const DeckMemDef_t* memDef) {
    uint8_t result = MASK_IS_VALID;

    const uint8_t properties = memDef->properties();
    if (properties & DECK_MEMORY_MASK_STARTED) {
        result |= MASK_IS_STARTED;
    }

    if (properties & DECK_MEMORY_MASK_UPGRADE_REQUIRED) {
        result |= MASK_UPGRADE_REQUIRED;
    }

    if (properties & DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE) {
        result |= MASK_BOOTLOADER_ACTIVE;
    }

    if (memDef->read) {
        result |= MASK_SUPPORTS_READ;
    }

    if (memDef->write) {
        result |= MASK_SUPPORTS_WRITE;
    }

    if (memDef->supportsUpgrade) {
        result |= MASK_SUPPORTS_UPGRADE;
    }

    return result;
}

static uint8_t populateBitfield2(const DeckMemDef_t* memDef) {
    uint8_t result = 0;

    if (memDef->commandResetToFw) {
        result |= DECK_MEMORY_MASK_SUPPORTS_RESET_TO_FW;
    }

    if (memDef->commandResetToBootloader) {
        result |= DECK_MEMORY_MASK_SUPPORTS_RESET_TO_BOOTLOADER;
    }

    return result;
}

static void populateDeckMemoryInfoBuffer(const DeckMemDef_t *deckMemDef,
                                         const char *name,
                                         uint32_t baseAddress,
                                         uint8_t buffer[])
{
    buffer[OFFS_BITFIELD1] = populateBitfield1(deckMemDef);
    buffer[OFFS_BITFIELD2] = populateBitfield2(deckMemDef);
    memcpy(&buffer[OFFS_REQ_HASH], &deckMemDef->requiredHash, 4);
    memcpy(&buffer[OFFS_REQ_LEN], &deckMemDef->requiredSize, 4);
    memcpy(&buffer[OFFS_BASE_ADDR], &baseAddress, 4);
    strncpy((char*)&buffer[OFFS_NAME], name, NAME_LEN_EX_ZERO_TREM);

    //
    // If the memory definition has an id we append it to the name using
    // a colon:
    // name "bcAI" + id "gap8" => "bcAI:gap8"
    //
    if (deckMemDef->id) {
        size_t namelen = strlen(name);
        buffer[OFFS_NAME + namelen] = ':';
        memcpy(
            &buffer[OFFS_NAME] + namelen + 1,
            deckMemDef->id,
            strlen(deckMemDef->id)
        );
    }
}

//
// Fill in information to the deck memory areas primary and secondary, if
// present. The information is taken from the deck driver.
//
static void populateDeckMemoryInfos(uint8_t buffer[], const int deckNr) {
    DeckInfo* info = deckInfo(deckNr);
    memset(buffer, 0, DECK_MEMORY_INFO_SIZE * 2); // primary plus secondary

    const DeckMemDef_t* deckMemDef = info->driver->memoryDef;
    if (deckMemDef) {
        uint32_t baseAddress = (deckNr + 1) * DECK_MEM_MAX_SIZE;
        populateDeckMemoryInfoBuffer(deckMemDef, info->driver->name,
                                     baseAddress, buffer);
    }

    const DeckMemDef_t* deckMemDefSecondary = info->driver->memoryDefSecondary;
    if (deckMemDefSecondary) {
        uint32_t baseAddress = (deckNr + 2) * DECK_MEM_MAX_SIZE;
        populateDeckMemoryInfoBuffer(deckMemDefSecondary, info->driver->name,
                                     baseAddress, buffer + DECK_MEMORY_INFO_SIZE);
    }
}

static bool handleInfoSectionRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer, const int nrOfDecks) {
    uint32_t index = 0;
    uint8_t bytesLeft = readLen;

    memset(buffer, 0, readLen);

    // First byte is the version
    if (0 == (memAddr + index) && bytesLeft > 0) {
        buffer[index] = VERSION;
        index++;
        bytesLeft--;
    }

    // Deck memory infos, it is times 2 because we can have a secondary memDef
    while (bytesLeft > 0) {
        int deckNr = (memAddr + index - 1) / (DECK_MEMORY_INFO_SIZE * 2);

        if (deckNr >= nrOfDecks) {
            break;
        }

        uint8_t deckMemoryInfo[DECK_MEMORY_INFO_SIZE * 2];

        populateDeckMemoryInfos(deckMemoryInfo, deckNr);

        int startAddrOfThisInfo = deckNr * (DECK_MEMORY_INFO_SIZE * 2) + 1;
        int firstByteToUse = memAddr + index - startAddrOfThisInfo;

        int bytesToUse = (DECK_MEMORY_INFO_SIZE * 2) - firstByteToUse;
        if (bytesLeft < bytesToUse) {
            bytesToUse = bytesLeft;
        }

        memcpy(buffer + index, (uint8_t*)(&deckMemoryInfo) + firstByteToUse, bytesToUse);
        bytesLeft -= bytesToUse;
        index += bytesToUse;
    }

    return true;
}

static bool handleDeckSectionRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer, const int deckNr, const MemSelector selector) {
    bool result = false;

    const DeckInfo* info = deckInfo(deckNr);

    const DeckMemDef_t* deckMemDef = info->driver->memoryDef;
    if (selector == MEM_SECONDARY) {
        deckMemDef = info->driver->memoryDefSecondary;
    }

    if (deckMemDef) {
        if (deckMemDef->read) {
            uint32_t baseAddress = (deckNr + 1) * DECK_MEM_MAX_SIZE + selector * DECK_MEM_MAX_SIZE;
            uint32_t deckAddress = memAddr - baseAddress;
            result = deckMemDef->read(deckAddress, readLen, buffer);
        }
    }

    return result;
}

static void handleCommandForDevice(const DeckMemDef_t* memoryDef, const uint32_t adr, const uint8_t value) {
    if (adr < COMMAND_BITFIELD_ADR) {
        if (memoryDef->newFwSizeP) {
            uint8_t* newFwSizePtr = (uint8_t*)memoryDef->newFwSizeP;
            newFwSizePtr[adr] = value;
        }
    }
    else if (adr == COMMAND_BITFIELD_ADR) {
        if (value & DECK_MEMORY_MASK_COMMAND_RESET_TO_FW && memoryDef->commandResetToFw) {
            memoryDef->commandResetToFw();
        }
        if (value & DECK_MEMORY_MASK_COMMAND_RESET_TO_BOOTLOADER && memoryDef->commandResetToBootloader) {
            memoryDef->commandResetToBootloader();
        }
    }
}

static bool handleCommandSectionWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer, const int nrOfDecks) {
    for (uint32_t i = 0; i < writeLen; i++) {
        uint32_t adr = memAddr + i;
        if (adr >= COMMAND_BASE_ADR) {
            uint32_t relAdr = adr - COMMAND_BASE_ADR;
            int deckNr = relAdr / (DECK_MEMORY_COMMAND_SIZE * 2);
            if (deckNr < nrOfDecks) {
                const DeckInfo* info = deckInfo(deckNr);
                uint32_t commandAdr = relAdr % DECK_MEMORY_COMMAND_SIZE;
                uint32_t selector = (relAdr / DECK_MEMORY_COMMAND_SIZE) % 2;
                uint8_t value = buffer[i];
                if (selector == 0) {
                    handleCommandForDevice(info->driver->memoryDef, commandAdr, value);
                } else {
                    handleCommandForDevice(info->driver->memoryDefSecondary, commandAdr, value);
                }
            }
        }
    }

    return true;
}

static bool handleDeckSectionWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer, const int deckNr, const MemSelector selector) {
    bool result = false;

    const DeckInfo* info = deckInfo(deckNr);

    const DeckMemDef_t* deckMemDef = info->driver->memoryDef;
    if (selector == MEM_SECONDARY) {
        deckMemDef = info->driver->memoryDefSecondary;
    }

    if (deckMemDef) {
        if (deckMemDef->write) {
            uint32_t baseAddress = (deckNr + 1) * DECK_MEM_MAX_SIZE + selector * DECK_MEM_MAX_SIZE;
            uint32_t deckAddress = memAddr - baseAddress;
            result = deckMemDef->write(deckAddress, writeLen, buffer, deckMemDef);
        }
    }

    return result;
}

TESTABLE_STATIC bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
    bool result = false;
    int nrOfDecks = deckCount();

    // Assume the buffer is fully within one section. It may not be true, but unlikely
    // and not an important use case to support
    const uint32_t section = memAddr / DECK_MEM_MAX_SIZE;
    if (section == 0) {
        result = handleInfoSectionRead(memAddr, readLen, buffer, nrOfDecks);
    } else {
        MemSelector selector = (section - 1) % 2;
        int deckNr = (section - 1) / 2;
        if (deckNr < nrOfDecks * 2) {
            result = handleDeckSectionRead(memAddr, readLen, buffer, deckNr, selector);
        }
    }

    return result;
}

TESTABLE_STATIC bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
    bool result = false;
    int nrOfDecks = deckCount();

    // Assume the buffer is fully within one section. It may not be true, but unlikely
    // and not an important use case to support
    const uint32_t section = memAddr / DECK_MEM_MAX_SIZE;
    if (section == 0) {
        result = handleCommandSectionWrite(memAddr, writeLen, buffer, nrOfDecks);
    } else {
        MemSelector selector = (section - 1) % 2;
        int deckNr = (section - 1) / 2;
        if (deckNr < nrOfDecks * 2) {
            result = handleDeckSectionWrite(memAddr, writeLen, buffer, deckNr, selector);
        }
    }

    return result;
}

void deckMemoryInit() {
    memoryRegisterHandler(&memoryDef);
}
