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
static uint32_t handleMemTesterGetSize(void) { return MEM_TESTER_SIZE; }
static bool handleMemTesterRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemTesterWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData);
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

static uint8_t nrOfOwMems = 0;

#define MAX_NR_HANDLERS 20
static const MemoryHandlerDef_t* handlers[MAX_NR_HANDLERS];
static uint8_t nrOfHandlers = 0;
static const MemoryOwHandlerDef_t* owMemHandler = 0;


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

  if (owMemHandler == 0) {
    return false;
  }

  return true;
}

#ifdef UNIT_TEST_MODE
void memReset() {
  isInit = false;

  registrationEnabled = true;
  nrOfHandlers = 0;
  owMemHandler = 0;
}
#endif

void memoryRegisterHandler(const MemoryHandlerDef_t* handlerDef){
  for (int i = 0; i < nrOfHandlers; i++) {
    ASSERT(handlerDef->type != handlers[i]->type);
  }
  ASSERT(nrOfHandlers < MAX_NR_HANDLERS);
  ASSERT(registrationEnabled);
  handlers[nrOfHandlers] = handlerDef;
  nrOfHandlers++;
}

void memoryRegisterOwHandler(const MemoryOwHandlerDef_t* handlerDef){
  ASSERT(owMemHandler == 0);
  ASSERT(registrationEnabled);
  owMemHandler = handlerDef;

  nrOfOwMems = handlerDef->nrOfMems;
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
  return handlers[memId]->getSize();
}

uint32_t memGetOwSize() {
  ASSERT(owMemHandler);
  return owMemHandler->size;
}

bool memRead(const uint16_t memId, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  ASSERT(memId < nrOfHandlers);
  if (handlers[memId]->read) {
    result = handlers[memId]->read(memAddr, readLen, buffer);
  }

  return result;
}

bool memWrite(const uint16_t memId, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  ASSERT(memId < nrOfHandlers);
  if (handlers[memId]->write) {
    result = handlers[memId]->write(memAddr, writeLen, buffer);
  }

  return result;
}

bool memReadOw(const uint16_t owMemId, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  ASSERT(owMemHandler);
  return owMemHandler->read(owMemId, memAddr, readLen, buffer);
}

bool memGetOwSerialNr(const uint8_t owMemId, uint8_t* serialNr) {
  ASSERT(owMemHandler);
  return owMemHandler->getSerialNr(owMemId, serialNr);
}

bool memWriteOw(const uint16_t owMemId, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  ASSERT(owMemHandler);
  return owMemHandler->write(owMemId, memAddr, writeLen, buffer);
}

uint16_t memGetNrOfMems() {
  return nrOfHandlers;
}

uint16_t memGetNrOfOwMems() {
  return nrOfOwMems;
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
static bool handleMemTesterRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData) {
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
static bool handleMemTesterWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData) {
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
