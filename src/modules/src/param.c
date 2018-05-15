/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * param.h - Crazy parameter system source file.
 */
#include <string.h>
#include <errno.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "crtp.h"
#include "param.h"
#include "crc.h"
#include "console.h"

#if 0
#include "debug.h"
#define PARAM_DEBUG(fmt, ...) DEBUG_PRINT("D/param " fmt, ## __VA_ARGS__)
#define PARAM_ERROR(fmt, ...) DEBUG_PRINT("E/param " fmt, ## __VA_ARGS__)
#else
#define PARAM_DEBUG(...)
#define PARAM_ERROR(...)
#endif


#define TOC_CH 0
#define READ_CH 1
#define WRITE_CH 2
#define MISC_CH 3

#define CMD_RESET 0
#define CMD_GET_NEXT 1
#define CMD_GET_CRC 2

#define CMD_GET_ITEM 0
#define CMD_GET_INFO 1

#define MISC_SETBYNAME 0

//Private functions
static void paramTask(void * prm);
void paramTOCProcess(int command);


//These are set by the Linker
extern struct param_s _param_start;
extern struct param_s _param_stop;

//The following two function SHALL NOT be called outside paramTask!
static void paramWriteProcess(int id, void*);
static void paramReadProcess(int id);
static int variableGetIndex(int id);
static char paramWriteByNameProcess(char* group, char* name, int type, void *valptr);

//Pointer to the parameters list and length of it
static struct param_s * params;
static int paramsLen;
static uint32_t paramsCrc;
static int paramsCount = 0;

static CRTPPacket p;

static bool isInit = false;

void paramInit(void)
{
  int i;
  const char* group = NULL;
  int groupLength = 0;

  if(isInit)
    return;

  params = &_param_start;
  paramsLen = &_param_stop - &_param_start;

  // Calculate a hash of the toc by chaining description of each elements
  // Using the CRTP packet as temporary buffer
  paramsCrc = 0;
  for (int i=0; i<paramsLen; i++)
  {
    int len = 5;
    memcpy(&p.data[0], &paramsCrc, 4);
    p.data[4] = params[i].type;
    if (params[i].type & PARAM_GROUP) {
      if (params[i].type & PARAM_START) {
        group = params[i].name;
        groupLength = strlen(group);
      }
    } else {
      // CMD_GET_ITEM result's size is: 3 + strlen(params[i].name) + groupLength + 2
      if (strlen(params[i].name) + groupLength + 2 > 27) {
        PARAM_ERROR("'%s.%s' too long\n", group, params[i].name);
        ASSERT_FAILED();
      }
    }

    if (params[i].name) {
      memcpy(&p.data[5], params[i].name, strlen(params[i].name));
      len += strlen(params[i].name);
    }
    paramsCrc = crcSlow(p.data, len);
  }

  for (i=0; i<paramsLen; i++)
  {
    if(!(params[i].type & PARAM_GROUP))
      paramsCount++;
  }


  //Start the param task
	xTaskCreate(paramTask, PARAM_TASK_NAME,
	            PARAM_TASK_STACKSIZE, NULL, PARAM_TASK_PRI, NULL);

  //TODO: Handle stored parameters!

  isInit = true;
}

bool paramTest(void)
{
  return isInit;
}

void paramTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_PARAM);

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_PARAM, &p);

		if (p.channel==TOC_CH)
		  paramTOCProcess(p.data[0]);
	  else if (p.channel==READ_CH)
		  paramReadProcess(p.data[0]);
		else if (p.channel==WRITE_CH)
		  paramWriteProcess(p.data[0], &p.data[1]);
    else if (p.channel==MISC_CH) {
      if (p.data[0] == MISC_SETBYNAME) {
        int i, nzero = 0;
        char *group;
        char *name;
        uint8_t type;
        void * valPtr;
        int error;

        // If the packet contains at least 2 zeros in the first 28 bytes
        // The packet decoding algorithm will not crash
        for (i=0; i<CRTP_MAX_DATA_SIZE; i++) {
          if (p.data[i] == '\0') nzero++;
        }

        if (nzero < 2) return;

        group = (char*)&p.data[1];
        name = (char*)&p.data[1+strlen(group)+1];
        type = p.data[1+strlen(group)+1+strlen(name)+1];
        valPtr = &p.data[1+strlen(group)+1+strlen(name)+2];

        error = paramWriteByNameProcess(group, name, type, valPtr);

        p.data[1+strlen(group)+1+strlen(name)+1] = error;
        p.size = 1+strlen(group)+1+strlen(name)+1+1;
        crtpSendPacket(&p);
      }
    }
	}
}

void paramTOCProcess(int command)
{
  int ptr = 0;
  char * group = "";
  int n=0;

  switch (command)
  {
  case CMD_GET_INFO: //Get info packet about the param implementation
    ptr = 0;
    group = "";
    p.header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
    p.size=6;
    p.data[0]=CMD_GET_INFO;
    p.data[1]=paramsCount;
    memcpy(&p.data[2], &paramsCrc, 4);
    crtpSendPacket(&p);
    break;
  case CMD_GET_ITEM:  //Get param variable
    for (ptr=0; ptr<paramsLen; ptr++) //Ptr points a group
    {
      if (params[ptr].type & PARAM_GROUP)
      {
        if (params[ptr].type & PARAM_START)
          group = params[ptr].name;
        else
          group = "";
      }
      else                          //Ptr points a variable
      {
        if (n==p.data[1])
          break;
        n++;
      }
    }

    if (ptr<paramsLen)
    {
      p.header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.data[1]=n;
      p.data[2]=params[ptr].type;
      p.size=3+2+strlen(group)+strlen(params[ptr].name);
      ASSERT(p.size <= CRTP_MAX_DATA_SIZE); // Too long! The name of the group or the parameter may be too long.
      memcpy(p.data+3, group, strlen(group)+1);
      memcpy(p.data+3+strlen(group)+1, params[ptr].name, strlen(params[ptr].name)+1);
      crtpSendPacket(&p);
    } else {
      p.header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.size=1;
      crtpSendPacket(&p);
    }
    break;
  }
}

static void paramWriteProcess(int ident, void* valptr)
{
  int id;

  id = variableGetIndex(ident);

  if (id<0) {
    p.data[0] = -1;
    p.data[1] = ident;
    p.data[2] = ENOENT;
    p.size = 3;

    crtpSendPacket(&p);
    return;
  }

	if (params[id].type & PARAM_RONLY)
		return;

  switch (params[id].type & PARAM_BYTES_MASK)
  {
 	case PARAM_1BYTE:
 		*(uint8_t*)params[id].address = *(uint8_t*)valptr;
 		break;
    case PARAM_2BYTES:
  	  *(uint16_t*)params[id].address = *(uint16_t*)valptr;
      break;
 	case PARAM_4BYTES:
      *(uint32_t*)params[id].address = *(uint32_t*)valptr;
      break;
 	case PARAM_8BYTES:
      *(uint64_t*)params[id].address = *(uint64_t*)valptr;
      break;
  }

  crtpSendPacket(&p);
}

static char paramWriteByNameProcess(char* group, char* name, int type, void *valptr) {
  int ptr;
  char *pgroup = "";

  for (ptr=0; ptr<paramsLen; ptr++) //Ptr points a group
  {
    if (params[ptr].type & PARAM_GROUP)
    {
      if (params[ptr].type & PARAM_START)
        pgroup = params[ptr].name;
      else
        pgroup = "";
    }
    else                          //Ptr points a variable
    {
      if (!strcmp(params[ptr].name, name) && !strcmp(pgroup, group))
        break;
    }
  }

  if (ptr >= paramsLen) {
    return ENOENT;
  }

  if (type != params[ptr].type) {
    return EINVAL;
  }

  if (params[ptr].type & PARAM_RONLY) {
    return EACCES;
  }

  switch (params[ptr].type & PARAM_BYTES_MASK)
  {
 	case PARAM_1BYTE:
 		*(uint8_t*)params[ptr].address = *(uint8_t*)valptr;
 		break;
    case PARAM_2BYTES:
  	  *(uint16_t*)params[ptr].address = *(uint16_t*)valptr;
      break;
 	case PARAM_4BYTES:
      *(uint32_t*)params[ptr].address = *(uint32_t*)valptr;
      break;
 	case PARAM_8BYTES:
      *(uint64_t*)params[ptr].address = *(uint64_t*)valptr;
      break;
  }

  return 0;
}

static void paramReadProcess(int ident)
{
  int id;

  id = variableGetIndex(ident);

  if (id<0) {
    p.data[0] = -1;
    p.data[1] = ident;
    p.data[2] = ENOENT;
    p.size = 3;

    crtpSendPacket(&p);
    return;
  }

  switch (params[id].type & PARAM_BYTES_MASK)
  {
 	case PARAM_1BYTE:
   		memcpy(&p.data[1], params[id].address, sizeof(uint8_t));
   		p.size = 1+sizeof(uint8_t);
   		break;
 		break;
    case PARAM_2BYTES:
   		memcpy(&p.data[1], params[id].address, sizeof(uint16_t));
   		p.size = 1+sizeof(uint16_t);
   		break;
    case PARAM_4BYTES:
      memcpy(&p.data[1], params[id].address, sizeof(uint32_t));
   		p.size = 1+sizeof(uint32_t);
   		break;
 	  case PARAM_8BYTES:
      memcpy(&p.data[1], params[id].address, sizeof(uint64_t));
   		p.size = 1+sizeof(uint64_t);
   		break;
  }

  crtpSendPacket(&p);
}

static int variableGetIndex(int id)
{
  int i;
  int n=0;

  for (i=0; i<paramsLen; i++)
  {
    if(!(params[i].type & PARAM_GROUP))
    {
      if(n==id)
        break;
      n++;
    }
  }

  if (i>=paramsLen)
    return -1;

  return i;
}
