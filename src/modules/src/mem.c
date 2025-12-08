/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2023 Bitcraze AB
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
 * mem.c: Memory module. Handles one-wire, eeprom and other memory mapping functions.
 */

#include "mem.h"

#include "cfassert.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#define MEM_TESTER_SIZE            0x1000

// Private functions, mem tester
static uint32_t handleMemTesterGetSize(const uint8_t internal_id) { return MEM_TESTER_SIZE; }
static bool handleMemTesterRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemTesterWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData);
static uint32_t memTesterWriteErrorCount = 0;
static uint8_t memTesterWriteReset = 0;
static const MemoryHandlerDef_t memTesterDef = {
  .type = MEM_TYPE_TESTER,
  .getSize = handleMemTesterGetSize,
  .read = handleMemTesterRead,
  .write = handleMemTesterWrite,
};

static bool isInit = false;
static bool registrationEnabled = true;

#define MAX_NR_HANDLERS 20
static const MemoryHandlerDef_t* handlers[MAX_NR_HANDLERS];
static uint8_t nrOfHandlers = 0;


void memInit(void)
{
  if(isInit) {
    return;
  }

  memoryRegisterHandler(&memTesterDef);

  isInit = true;
}

bool memTest(void) {
  if (!isInit) {
    return false;
  }

  return true;
}

#ifdef UNIT_TEST_MODE
void memReset() {
  isInit = false;

  registrationEnabled = true;
  nrOfHandlers = 0;
}
#endif

void memoryRegisterHandler(const MemoryHandlerDef_t* handlerDef){
  ASSERT(nrOfHandlers < MAX_NR_HANDLERS);
  ASSERT(registrationEnabled);
  handlers[nrOfHandlers] = handlerDef;
  nrOfHandlers++;
}

void memBlockHandlerRegistration() {
  registrationEnabled = false;
}

MemoryType_t memGetType(const uint16_t memId) {
  ASSERT(memId < nrOfHandlers);
  return handlers[memId]->type;
}

uint32_t memGetSize(const uint16_t memId) {
  ASSERT(memId < nrOfHandlers);
  return handlers[memId]->getSize(handlers[memId]->internal_id);
}

bool memRead(const uint16_t memId, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  ASSERT(memId < nrOfHandlers);
  if (handlers[memId]->read) {
    result = handlers[memId]->read(handlers[memId]->internal_id, memAddr, readLen, buffer);
  }

  return result;
}

bool memWrite(const uint16_t memId, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  ASSERT(memId < nrOfHandlers);
  if (handlers[memId]->write) {
    result = handlers[memId]->write(handlers[memId]->internal_id, memAddr, writeLen, buffer);
  }

  return result;
}

bool memSerialNbr(const uint16_t memId, const uint8_t maxLen, uint8_t* len, uint8_t* buffer) {
  bool result = false;

  ASSERT(memId < nrOfHandlers);
  if (handlers[memId]->getSerialNbr) {
    result = handlers[memId]->getSerialNbr(handlers[memId]->internal_id, maxLen, len, buffer);
  }

  return result;
}

uint16_t memGetNrOfMems() {
  return nrOfHandlers;
}

/**
 * @brief The memory tester is used to verify the functionality of the memory sub system.
 * It supports "virtual" read and writes that are used by a test script to
 * check that a client (for instance the python lib) is working as expected.
 *
 * When reading data from the tester, it simply fills up a buffer with known data so that the
 * client can examine the data and verify that buffers have been correctly assembled.
 *
 * @param memAddr - the virtual address to read from
 * @param readLen - nr of bytes to read
 * @param startOfData - address to write result to
 * @return Always returns true
 */
static bool handleMemTesterRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData) {
  for (int i = 0; i < readLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t data = addr & 0xff;
    startOfData[i] = data;
  }

  return true;
}

/**
 * @brief When writing data to the tester, the tester verifies that the received data
 * contains the expected values.
 *
 * @param memAddr - the virtual address to write to
 * @param writeLen - nr of bytes to write
 * @param startOfData - pointer to the data in the packet that is provided by the client
 * @return Always returns true
 */
static bool handleMemTesterWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData) {
  if (memTesterWriteReset) {
    memTesterWriteReset = 0;
    memTesterWriteErrorCount = 0;
  }

  for (int i = 0; i < writeLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t expectedData = addr & 0xff;
    uint8_t actualData = startOfData[i];
    if (actualData != expectedData) {
      // Log first error
      if (memTesterWriteErrorCount == 0) {
        DEBUG_PRINT("Verification failed: expected: %d, actual: %d, addr: %lu\n", expectedData, actualData, addr);
      }

      memTesterWriteErrorCount++;
      break;
    }
  }

  return true;
}

PARAM_GROUP_START(memTst)
  PARAM_ADD(PARAM_UINT8, resetW, &memTesterWriteReset)
PARAM_GROUP_STOP(memTst)

LOG_GROUP_START(memTst)
  LOG_ADD(LOG_UINT32, errCntW, &memTesterWriteErrorCount)
LOG_GROUP_STOP(memTst)
