/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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
 * crtp_mem.c: CRTP implementation of memory mapping.
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mem.h"
#include "crtp.h"
#include "system.h"

#include "console.h"
#include "assert.h"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "static_mem.h"

#if 0
#define MEM_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)
#define MEM_ERROR(fmt, ...) DEBUG_PRINT("E/log " fmt, ## __VA_ARGS__)
#else
#define MEM_DEBUG(...)
#define MEM_ERROR(...)
#endif


// Maximum payload length
#define MEM_MAX_LEN 30

#define MEM_SETTINGS_CH     0
#define MEM_READ_CH         1
#define MEM_WRITE_CH        2

#define MEM_CMD_GET_NBR     1
#define MEM_CMD_GET_INFO    2

#define STATUS_OK 0


// Private functions
static void memTask(void * prm);
static void memSettingsProcess(CRTPPacket* p);
static void memWriteProcess(CRTPPacket* p);
static void memReadProcess(CRTPPacket* p);
static void createNbrResponse(CRTPPacket* p);
static void createInfoResponse(CRTPPacket* p, uint8_t memId);
static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]);

static bool isInit = false;

static const uint8_t NoSerialNr[MEMORY_SERIAL_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0};
static CRTPPacket packet;

STATIC_MEM_TASK_ALLOC(memTask, MEM_TASK_STACKSIZE);

void crtpMemInit(void)
{
  if(isInit) {
    return;
  }

  //Start the mem task
  STATIC_MEM_TASK_CREATE(memTask, memTask, MEM_TASK_NAME, NULL, MEM_TASK_PRI);

  isInit = true;
}

bool crtpMemTest(void) {
  if (!isInit) {
    return false;
  }

  return true;
}

static void memTask(void* param) {
	crtpInitTaskQueue(CRTP_PORT_MEM);

  // This should be synced with decks starting up, otherwise
  // there might be late arrivals for the registration that will
  // trigger assert.

  systemWaitStart();

  // Do not allow registration of new handlers after this point as clients now can start
  // to query for available memories
  memBlockHandlerRegistration();

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_MEM, &packet);

		switch (packet.channel) {
      case MEM_SETTINGS_CH:
        memSettingsProcess(&packet);
        break;
      case MEM_READ_CH:
        memReadProcess(&packet);
        break;
      case MEM_WRITE_CH:
        memWriteProcess(&packet);
        break;
      default:
        // Do nothing
        break;
		}
	}
}

static void memSettingsProcess(CRTPPacket* p) {
  switch (p->data[0]) {
    case MEM_CMD_GET_NBR:
      createNbrResponse(p);
      crtpSendPacketBlock(p);
      break;

    case MEM_CMD_GET_INFO:
      {
        uint8_t memId = p->data[1];
        createInfoResponse(p, memId);
        crtpSendPacketBlock(p);
      }
      break;

    default:
      // Do nothing
      break;
  }
}

static void createNbrResponse(CRTPPacket* p) {
  const uint8_t nrOfMems = memGetNrOfMems();
  const uint8_t nbrOwMems = memGetNrOfOwMems();

  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_NBR;
  p->data[1] = nbrOwMems + nrOfMems;
}

static void createInfoResponse(CRTPPacket* p, uint8_t memId) {
  const uint8_t nrOfMems = memGetNrOfMems();

  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_INFO;
  p->data[1] = memId;

  if (memId < nrOfMems) {
    createInfoResponseBody(p, memGetType(memId), memGetSize(memId), NoSerialNr);
  } else {
    const uint8_t selectedMem = memId - nrOfMems;
    uint8_t serialNr[MEMORY_SERIAL_LENGTH];

    // No error code if we fail, just send an empty packet back
    if (memGetOwSerialNr(selectedMem, serialNr)) {
      createInfoResponseBody(p, MEM_TYPE_OW, memGetOwSize(), serialNr);
    }
  }
}

static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]) {
  p->data[2] = type;
  p->size += 1;

  memcpy(&p->data[3], &memSize, 4);
  p->size += 4;

  memcpy(&p->data[7], data, 8);
  p->size += 8;
}


static void memReadProcess(CRTPPacket* p) {
  uint32_t memAddr;
  const uint8_t nrOfMems = memGetNrOfMems();

  MEM_DEBUG("Packet is MEM READ\n");
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_READ_CH);

  uint8_t memId = p->data[0];
  memcpy(&memAddr, &p->data[1], 4);
  uint8_t readLen = p->data[5];
  uint8_t* startOfData = &p->data[6];

  bool result = false;
  if (memId < nrOfMems) {
    result = memRead(memId, memAddr, readLen, startOfData);
  } else {
    uint8_t owMemId = memId - nrOfMems;
    result = memReadOw(owMemId, memAddr, readLen, startOfData);
  }

  p->data[5] = result ? STATUS_OK : EIO;
  if (result) {
    p->size = 6 + readLen;
  } else {
    p->size = 6;
  }

  crtpSendPacketBlock(p);
}

static void memWriteProcess(CRTPPacket* p) {
  uint32_t memAddr;
  const uint8_t nrOfMems = memGetNrOfMems();

  uint8_t memId = p->data[0];
  memcpy(&memAddr, &p->data[1], 4);
  uint8_t* startOfData = &p->data[5];

  uint8_t writeLen = p->size - 5;

  MEM_DEBUG("Packet is MEM WRITE\n");
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_WRITE_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  bool result = false;
  if (memId < nrOfMems) {
    result = memWrite(memId, memAddr, writeLen, startOfData);
  } else {
    uint8_t owMemId = memId - nrOfMems;
    result = memWriteOw(owMemId, memAddr, writeLen, startOfData);
  }

  p->data[5] = result ? STATUS_OK : EIO;
  p->size = 6;

  crtpSendPacketBlock(p);
}
