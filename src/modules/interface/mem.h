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
  MEM_TYPE_EEPROM   = 0x00,
  MEM_TYPE_OW       = 0x01,
  MEM_TYPE_LED12    = 0x10,
  MEM_TYPE_LOCO     = 0x11,
  MEM_TYPE_TRAJ     = 0x12,
  MEM_TYPE_LOCO2    = 0x13,
  MEM_TYPE_LH       = 0x14,
  MEM_TYPE_TESTER   = 0x15,
  MEM_TYPE_USD      = 0x16,
  MEM_TYPE_LEDMEM   = 0x17,
  MEM_TYPE_APP      = 0x18,
  MEM_TYPE_DECK_MEM = 0x19,
} MemoryType_t;

#define MEMORY_SERIAL_LENGTH 8

typedef struct {
  const MemoryType_t type;
  uint32_t (*getSize)(void);
  bool (*read)(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
  bool (*write)(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
} MemoryHandlerDef_t;

typedef struct {
  uint8_t nrOfMems;
  const uint32_t size;
  bool (*getSerialNr)(const uint8_t selectedMem, uint8_t* serialNr);
  bool (*read)(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
  bool (*write)(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
} MemoryOwHandlerDef_t;

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
 * @brief Register the One Wire memory handler.
 * It is only possible to register a single One Wire memory handler.
 *
 * @param handlerDef A pointer to a One Wire memory handler definition.
 */
void memoryRegisterOwHandler(const MemoryOwHandlerDef_t* handlerDef);

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
 * @brief Get the number of registered One Wire memory handlers
 *
 * @return uint16_t The number of registered handlers
 */
uint16_t memGetNrOfOwMems();

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
 * @brief Get the size of a One Wire memory. Note: at the time of writing, only one type of One Wire memory is supported.
 *
 * @return uint32_t The size of a One Wire memory
 */
uint32_t memGetOwSize();

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

/**
 * @brief Read from a One Wire memory
 *
 * @param owMemId The id of the one wire memory. The id is an index based on the available One Wire memories on decks for instance, between 0 and memGetNrOfOwMems() - 1.
 * @param memAddr The address to start reading from
 * @param readLen The number of bytes to read
 * @param buffer The buffer to copy data into
 * @return true If successful
 * @return false If failure
 */
bool memReadOw(const uint16_t owMemId, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);

/**
 * @brief Get the serial number of a specific One Wire memory
 *
 * @param owMemId The id of the one wire memory. The id is an index based on the available One Wire memories on decks for instance, between 0 and memGetNrOfOwMems() - 1.
 * @param serialNr A buffer of minimum 8 bytes to write the serial number to
 * @return true If successful
 * @return false If failure
 */
bool memGetOwSerialNr(const uint8_t owMemId, uint8_t* serialNr);

/**
 * @brief Write to a One Wire
 *
 * @param owMemId The id of the one wire memory. The id is an index based on the available One Wire memories on decks for instance, between 0 and memGetNrOfOwMems() - 1.
 * @param memAddr The address to start writing to
 * @param writeLen The number of bytes to write
 * @param buffer The buffer to copy data from
 * @return true If successful
 * @return false If failure
 */
bool memWriteOw(const uint16_t owMemId, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

#ifdef UNIT_TEST_MODE
/**
 * @brief Reset function for unit testing
 */
void memReset();
#endif
