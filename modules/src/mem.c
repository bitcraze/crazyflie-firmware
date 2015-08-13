/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * mem.c: Memory module. Handles one-wire and eeprom memory functions over crtp link.
 */

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"
#include "crtp.h"
#include "mem.h"
#include "ow.h"
#include "eeprom.h"
#ifdef PLATFORM_CF2
#include "ledring12.h"
#endif


#include "console.h"
#include "cfassert.h"
#include "debug.h"

#if 0
#define MEM_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)
#define MEM_ERROR(fmt, ...) DEBUG_PRINT("E/log " fmt, ## __VA_ARGS__)
#else
#define MEM_DEBUG(...)
#define MEM_ERROR(...)
#endif


// Maximum log payload length
#define MEM_MAX_LEN 30

#define SETTINGS_CH     0
#define READ_CH         1
#define WRITE_CH        2

#define CMD_GET_NBR     1
#define CMD_GET_INFO    2

#ifdef PLATFORM_CF1
#define NBR_EEPROM      0
#else
#define NBR_EEPROM      1
#endif

#define EEPROM_ID       0x00
#ifdef PLATFORM_CF1
  #define NBR_LEDMEM      0
  uint8_t ledringmem[1];
#else
  #define NBR_LEDMEM      1
#endif
#define LEDMEM_ID       0x01

#define NBR_STATIC_MEM  (NBR_EEPROM + NBR_LEDMEM)

#define MEM_TYPE_EEPROM 0x00
#define MEM_TYPE_OW     0x01
#define MEM_TYPE_LED12  0x10


//Private functions
static void memTask(void * prm);
static void memSettingsProcess(int command);
static void memWriteProcess(void);
static void memReadProcess(void);


static bool isInit = false;

static uint8_t nbrOwMems = 0;
static OwSerialNum serialNbr;
static const OwSerialNum eepromSerialNum =
{
  .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, EEPROM_I2C_ADDR}
};
static uint32_t memSize;
static CRTPPacket p;

void memInit(void)
{
  if(isInit)
    return;

  if (owScan(&nbrOwMems))
    isInit = true;
  else
    isInit = false;
  
  //Start the mem task
  xTaskCreate(memTask, (const signed char * const)MEM_TASK_NAME,
              MEM_TASK_STACKSIZE, NULL, MEM_TASK_PRI, NULL);
}

bool memTest(void)
{
  return isInit;
}

void memTask(void * param)
{
	crtpInitTaskQueue(CRTP_PORT_MEM);
	
	while(1)
	{
		crtpReceivePacketBlock(CRTP_PORT_MEM, &p);

		switch (p.channel)
		{
      case SETTINGS_CH:
        memSettingsProcess(p.data[0]);
        break;
      case READ_CH:
        memReadProcess();
        break;
      case WRITE_CH:
        memWriteProcess();
        break;
      default:
        break;
		}
	}
}

void memSettingsProcess(int command)
{
  uint8_t memId;
  
  switch (command)
  {
    case CMD_GET_NBR:
      p.header = CRTP_HEADER(CRTP_PORT_MEM, SETTINGS_CH);
      p.size = 2;
      p.data[0] = CMD_GET_NBR;
      p.data[1] = nbrOwMems + NBR_STATIC_MEM;
      crtpSendPacket(&p);
      break;

    case CMD_GET_INFO:
      memId = p.data[1];
      p.header = CRTP_HEADER(CRTP_PORT_MEM, SETTINGS_CH);
      p.size = 2;
      p.data[0] = CMD_GET_INFO;
      p.data[1] = memId;
      // No error code if we fail, just send an empty packet back
      if (memId == EEPROM_ID)
      {
        // Memory type (eeprom)
        p.data[2] = MEM_TYPE_EEPROM;
        p.size += 1;
        // Size of the memory
        memSize = EEPROM_SIZE;
        memcpy(&p.data[3], &memSize, 4);
        p.size += 4;
        memcpy(&p.data[7], eepromSerialNum.data, 8);
        p.size += 8;
      }
      else if (memId == LEDMEM_ID)
      {
        // Memory type virtual ledring mem
        p.data[2] = MEM_TYPE_LED12;
        p.size += 1;
        // Size of the memory
        memSize = sizeof(ledringmem);
        memcpy(&p.data[3], &memSize, 4);
        p.size += 4;
        memcpy(&p.data[7], eepromSerialNum.data, 8); //TODO
        p.size += 8;
      }
      else
      {
        if (owGetinfo(memId - NBR_STATIC_MEM, &serialNbr))
        {
          // Memory type (1-wire)
          p.data[2] = MEM_TYPE_OW;
          p.size += 1;
          // Size of the memory TODO: Define length type
          memSize = OW_MAX_SIZE;
          memcpy(&p.data[3], &memSize, 4);
          p.size += 4;
          memcpy(&p.data[7], serialNbr.data, 8);
          p.size += 8;
        }
      }
      crtpSendPacket(&p);

      break;
  }
}

void memReadProcess()
{
  uint8_t memId = p.data[0];
  uint8_t readLen = p.data[5];
  uint32_t memAddr;
  uint8_t status = 0;

  memcpy(&memAddr, &p.data[1], 4);

  MEM_DEBUG("Packet is MEM READ\n");
  p.header = CRTP_HEADER(CRTP_PORT_MEM, READ_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  if (memId == EEPROM_ID)
  {
    if (memAddr + readLen <= EEPROM_SIZE &&
        eepromReadBuffer(&p.data[6], memAddr, readLen))
      status = 0;
    else
      status = EIO;
  }
  else if (memId == LEDMEM_ID)
  {
    if (memAddr + readLen <= sizeof(ledringmem) &&
        memcpy(&p.data[6], &(ledringmem[memAddr]), readLen))
      status = 0;
    else
      status = EIO;
  }
  else
  {
    memId = memId - NBR_STATIC_MEM;
    if (memAddr + readLen <= OW_MAX_SIZE &&
        owRead(memId, memAddr, readLen, &p.data[6]))
      status = 0;
    else
      status = EIO;
  }

#if 0
  {
    int i;
    for (i = 0; i < readLen; i++)
      consolePrintf("%X ", p.data[i+6]);

    consolePrintf("\nStatus %i\n", status);
  }
#endif

  p.data[5] = status;
  if (status == 0)
    p.size = 6 + readLen;
  else
    p.size = 6;


  crtpSendPacket(&p);
}

void memWriteProcess()
{
  uint8_t memId = p.data[0];
  uint8_t writeLen;
  uint32_t memAddr;
  uint8_t status = 0;

  memcpy(&memAddr, &p.data[1], 4);
  writeLen = p.size - 5;

  MEM_DEBUG("Packet is MEM WRITE\n");
  p.header = CRTP_HEADER(CRTP_PORT_MEM, WRITE_CH);
  // Dont' touch the first 5 bytes, they will be the same.
  if (memId == EEPROM_ID)
  {
    if (memAddr + writeLen <= EEPROM_SIZE &&
        eepromWriteBuffer(&p.data[5], memAddr, writeLen))
      status = 0;
    else
      status = EIO;
  }
  else if(memId == LEDMEM_ID)
  {
    if ((memAddr + writeLen) <= sizeof(ledringmem))
    {
      memcpy(&(ledringmem[memAddr]), &p.data[5], writeLen);
      MEM_DEBUG("LED write addr:%i, led:%i\n", memAddr, writeLen);
    }
    else
    {
      MEM_DEBUG("\LED write failed! addr:%i, led:%i\n", memAddr, writeLen);
    }
  }
  else
  {
    memId = memId - NBR_STATIC_MEM;
    if (memAddr + writeLen <= OW_MAX_SIZE &&
        owWrite(memId, memAddr, writeLen, &p.data[5]))
      status = 0;
    else
      status = EIO;
  }

  p.data[5] = status;
  p.size = 6;

  crtpSendPacket(&p);
}
