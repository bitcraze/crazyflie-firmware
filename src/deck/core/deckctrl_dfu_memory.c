/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 * deck_ctrl_dfu_memory.h - interface for reading/writing and managing DFU
 *                          memory on a DeckCtrl managed deck
 *
 */

#include <string.h>
#include "deck_memory.h"
#include "deck_core.h"
#include "mem.h"
#include "i2cdev.h"
#include "i2c_dfu.h"
#include "syslink.h"
#include "worker.h"

#define DEBUG_MODULE "DECK_CTRL_DFU"
#include "debug.h"

// STM32C011 is 32 KB, so 0x8000 bytes
// Address 0 starts at 0x10000
static const uint32_t DECK_CTRL_MEM_SIZE = 0x8000;
static const uint32_t DECK_CTRL_MEM_OFFSET = 0x10000;
static const uint32_t DECK_MEM_MAX_SIZE = DECK_CTRL_MEM_OFFSET + DECK_CTRL_MEM_SIZE;

static uint32_t handleMemGetSize(const uint8_t internal_id) {
  return DECK_MEM_MAX_SIZE;
}

bool handleMemRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
bool handleMemWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

static const MemoryHandlerDef_t memoryDef = {
  .type = MEM_TYPE_DECKCTRL_DFU,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static bool shouldEnableDFU = false;

static const uint8_t VERSION = 1;

#define VERSION_MEMORY_BYTE   0x00
#define DECKCTRL_MEMORY_BYTE  0x01
#define STATUS_MEMORY_BYTE    0x02
#define CMD_MEMORY_BYTE       0x03

#define STATUS_IN_DFU_MODE    (1 << 0)
#define STATUS_CAN_ENABLE_DFU (1 << 1)

#define CMD_ENTER_DFU_MODE      (1 << 0)
#define CMD_ENTER_FIRMWARE_MODE (1 << 1)

static uint8_t nbrOfDeckCtrl(void) {
    uint8_t deckCtrlCount = 0;
    for (int i = 0; i < deckCount(); i++) {
        DeckInfo *deck = deckInfo(i);
        if (strcmp(deck->discoveryBackend->name, "deckctrl") == 0) {
            deckCtrlCount++;
        }
    }

    return deckCtrlCount;
}

static uint8_t getStatus(void) {
    uint8_t status = 0;

    if (nbrOfDeckCtrl() <= 1) {
        status = status | STATUS_CAN_ENABLE_DFU;
    } else {
        status = status & (uint8_t)~STATUS_CAN_ENABLE_DFU;
    }

    uint8_t dummy;
    if (dfu_i2c_read(DFU_STM32C0_I2C_ADDRESS, 0, &dummy, 1)) {
        status = status | STATUS_IN_DFU_MODE;
        status = status & (uint8_t)~STATUS_CAN_ENABLE_DFU;
    } else {
        status = status & (uint8_t)~STATUS_IN_DFU_MODE;
    }

    return status;
}

// Calling this function will cut the power to VCC including this MCU and the
// DeckCtrl, causing a reset. The DeckCtrl will then enter DFU mode on next
// power-up.
static void enableDFUViaNRF(void * arg) {
    bool enable = *((bool *) arg);
    vTaskDelay(pdMS_TO_TICKS(10)); // Give some time for the ongoing write to complete
    SyslinkPacket slp;
    slp.type = SYSLINK_PM_DECKCTRL_DFU;
    slp.length = 1;
    slp.data[0] = enable?0x01:0x00; // 1 to enter DFU, 0 to enter firmware mode
    syslinkSendPacket(&slp);
}

bool handleMemRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {

    if (memAddr < DECK_CTRL_MEM_OFFSET) {
        for (unsigned int i = 0; i < readLen; i++) {
            if (memAddr + i == VERSION_MEMORY_BYTE) {
                buffer[i] = VERSION;
            } else if (memAddr + i == DECKCTRL_MEMORY_BYTE) {
                buffer[i] = nbrOfDeckCtrl();
            } else if (memAddr + i == STATUS_MEMORY_BYTE) {
                buffer[i] = getStatus();
            } else {
                buffer[i] = 0x00;
            }
        }
    } else if (memAddr >= DECK_CTRL_MEM_OFFSET &&
               memAddr < (DECK_CTRL_MEM_OFFSET + DECK_CTRL_MEM_SIZE)) {

        uint32_t dfuMemAddr = memAddr - DECK_CTRL_MEM_OFFSET;
        bool result = dfu_i2c_read(DFU_STM32C0_I2C_ADDRESS, dfuMemAddr, buffer, readLen);
        if (!result) {
            return false;
        }
    } else {
        return false;
    }
    
    return true;
}

bool handleMemWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {

    if (memAddr < DECK_CTRL_MEM_OFFSET) {
        for (unsigned int i = 0; i < writeLen; i++) {
            if (memAddr + i == CMD_MEMORY_BYTE) {
                if (
                    (buffer[i] & CMD_ENTER_FIRMWARE_MODE) != 0 &&
                    (buffer[i] & CMD_ENTER_DFU_MODE) != 0)
                {
                    return false;
                } else if ((buffer[i] & CMD_ENTER_FIRMWARE_MODE) != 0) {
                    shouldEnableDFU = false;
                    workerSchedule(enableDFUViaNRF, (void *) &shouldEnableDFU);
                } else if ((buffer[i] & CMD_ENTER_DFU_MODE) != 0) {
                    if (nbrOfDeckCtrl() <= 1) {
                        shouldEnableDFU = true;
                        workerSchedule(enableDFUViaNRF, (void *) &shouldEnableDFU);
                    } else {
                        return false;
                    }
                }
            }
        }
    } else if (memAddr >= DECK_CTRL_MEM_OFFSET &&
               memAddr < (DECK_CTRL_MEM_OFFSET + DECK_CTRL_MEM_SIZE)) {

        uint32_t dfuMemAddr = memAddr - DECK_CTRL_MEM_OFFSET;
        bool result = dfu_i2c_write(DFU_STM32C0_I2C_ADDRESS, dfuMemAddr, buffer, writeLen);
        if (!result) {
            return false;
        }
    } else {
        return false;
    }

    return true;
}

void deckCtrlDFUMemoryInit() {
    memoryRegisterHandler(&memoryDef);
}
