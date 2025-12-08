/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2020 BitCraze AB
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
 * mem.h - Memory sub system
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  MEM_TYPE_EEPROM         = 0x00,
  MEM_TYPE_OW             = 0x01,
  MEM_TYPE_LED12          = 0x10,
  MEM_TYPE_LOCO           = 0x11,
  MEM_TYPE_TRAJ           = 0x12,
  MEM_TYPE_LOCO2          = 0x13,
  MEM_TYPE_LH             = 0x14,
  MEM_TYPE_TESTER         = 0x15,
  MEM_TYPE_USD            = 0x16,
  MEM_TYPE_LEDMEM         = 0x17,
  MEM_TYPE_APP            = 0x18,
  MEM_TYPE_DECK_MEM       = 0x19,
  MEM_TYPE_DECKCTRL_DFU   = 0x20,
  MEM_TYPE_DECKCTRL       = 0x21
} MemoryType_t;

#define MEMORY_SERIAL_LENGTH 12

typedef struct {
  MemoryType_t type;
  uint32_t (*getSize)(const uint8_t internal_id);
  bool (*getSerialNbr)(const uint8_t internal_id, const uint8_t max_length, uint8_t* len, uint8_t* buffer);
  bool (*read)(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
  bool (*write)(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
  uint8_t internal_id;
} MemoryHandlerDef_t;

// Public functions
void memInit(void);
bool memTest(void);

/**
 * @brief Register a memory handler
 *
 * @param handlerDef A pointer to a memory handler definition
 */
void memoryRegisterHandler(const MemoryHandlerDef_t* handlerDef);

/**
 * @brief Block the possibility to register any more memory handlers.
 * This function can be called by a protocol implementation to "freeze" the available set of handlers.
 */
void memBlockHandlerRegistration();

/**
 * @brief Get the number of registered handlers
 *
 * @return uint16_t The number of registered memory handlers
 */
uint16_t memGetNrOfMems();

/**
 * @brief Get the type of a specific memory handler
 *
 * @param memId The id of the memory handler to get the type for. The id is an index based on the registration order, between 0 and memGetNrOfMems() - 1.
 * @return MemoryType_t The type of the memory handler
 */
MemoryType_t memGetType(const uint16_t memId);

/**
 * @brief Get the size of the mapped memory for a specific memory handler
 *
 * @param memId The id of the memory handler to get the type for. The id is an index based on the registration order, between 0 and memGetNrOfMems() - 1.
 * @return uint32_t The size of the mapped memory
 */
uint32_t memGetSize(const uint16_t memId);

/**
 * @brief Get the serial number of a specific memory
 *
 * @param memId The id of the memory to get the type for. The id is an index based on the registration order, between 0 and memGetNrOfMems() - 1.
 * @param maxLen The maximum length of the provided buffer
 * @param len The length of the serial number returned
 * @param buffer The buffer to copy the serial number into
 * @return true If successful
 * @return false If failure
 */
bool memSerialNbr(const uint16_t memId, const uint8_t maxLen, uint8_t* len, uint8_t* buffer);

/**
 * @brief Read data from a memory handler
 *
 * @param memId The id of the memory handler to get the type for. The id is an index based on the registration order, between 0 and memGetNrOfMems() - 1.
 * @param memAddr The mapped start address to read from. Starts from 0 for each mapped memory.
 * @param readLen The number of bytes to read
 * @param buffer The buffer to copy the data into
 * @return true If successful
 * @return false If failure
 */
bool memRead(const uint16_t memId, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);

/**
 * @brief Write data to a memory handler
 *
 * @param memId The id of the memory handler to get the type for. The id is an index based on the registration order, between 0 and memGetNrOfMems() - 1.
 * @param memAddr The mapped start address to read from. Starts from 0 for each mapped memory.
 * @param writeLen The number of bytes to write
 * @param buffer The buffer to copy data from
 * @return true If successful
 * @return false If failure
 */
bool memWrite(const uint16_t memId, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

#ifdef UNIT_TEST_MODE
/**
 * @brief Reset function for unit testing
 */
void memReset();
#endif
