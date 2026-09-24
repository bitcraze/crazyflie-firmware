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
 * console.c - Used to send console data to client
 */

#include <string.h>
#include <errno.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "console.h"

#include "crtp.h"
#include "crc32.h"

/** CRTP channel for the legacy local console stream. */
#define CONSOLE_CHANNEL_LOCAL 0u
/** CRTP channel for source-tagged console stream packets. */
#define CONSOLE_CHANNEL_SOURCED 1u
/** CRTP channel for source runtime-control requests. */
#define CONSOLE_CHANNEL_CONTROL 2u
/** CRTP channel for source-catalog requests. */
#define CONSOLE_CHANNEL_TOC 3u
/** Source ID selecting every catalog entry in a control request. */
#define CONSOLE_SOURCE_ALL 0xffu
/** Number of assignable wire IDs; 0xff selects all sources. */
#define CONSOLE_SOURCE_MAX 255u
/** Runtime-control command that changes source enable state. */
#define CONSOLE_CMD_SET_ENABLED 0x00u
/** Source-catalog command that returns one entry. */
#define CONSOLE_TOC_GET_ITEM 0x00u
/** Source-catalog command that returns count and CRC. */
#define CONSOLE_TOC_GET_INFO 0x01u

/** Registered source catalog in stable source-ID order. */
static ConsoleSource *sourceHead;
static ConsoleSource *sourceTail;
/** Count can reach 255, one past the highest assignable source ID. */
static uint16_t sourceCount;
/** True after source registration has permanently closed. */
static bool sourcesFrozen;

#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#ifndef SCB_ICSR_VECTACTIVE_Msk
#define SCB_ICSR_VECTACTIVE_Msk 0x1FFUL
#endif
#endif

static CRTPPacket messageToPrint;
static bool messageSendingIsPending = false;
static xSemaphoreHandle synch = NULL;

static const char bufferFullMsg[] = "<F>\n";
static bool isInit;

static void addBufferFullMarker();
/** Handle sourced Console control and catalog CRTP requests. */
static void consoleCrtpCallback(CRTPPacket *packet);
/** Echo the request command followed by an error number. */
static void consoleSendCommandError(CRTPPacket *packet, uint8_t error);

/**
 * Validate one NUL-terminated UTF-8 source path.
 *
 * @param text Path to validate.
 * @return true when text contains canonical UTF-8, otherwise false.
 */
static bool validUtf8(const char *text)
{
  const uint8_t *p = (const uint8_t *)text;
  while (*p != 0u) {
    if (*p < 0x80u) {
      p++;
    } else if (*p >= 0xc2u && *p <= 0xdfu &&
               p[1] >= 0x80u && p[1] <= 0xbfu) {
      p += 2;
    } else if (*p == 0xe0u && p[1] >= 0xa0u && p[1] <= 0xbfu &&
               p[2] >= 0x80u && p[2] <= 0xbfu) {
      p += 3;
    } else if (((*p >= 0xe1u && *p <= 0xecu) || (*p >= 0xeeu && *p <= 0xefu)) &&
               p[1] >= 0x80u && p[1] <= 0xbfu &&
               p[2] >= 0x80u && p[2] <= 0xbfu) {
      p += 3;
    } else if (*p == 0xedu && p[1] >= 0x80u && p[1] <= 0x9fu &&
               p[2] >= 0x80u && p[2] <= 0xbfu) {
      p += 3;
    } else if (*p == 0xf0u && p[1] >= 0x90u && p[1] <= 0xbfu &&
               p[2] >= 0x80u && p[2] <= 0xbfu &&
               p[3] >= 0x80u && p[3] <= 0xbfu) {
      p += 4;
    } else if (*p >= 0xf1u && *p <= 0xf3u &&
               p[1] >= 0x80u && p[1] <= 0xbfu &&
               p[2] >= 0x80u && p[2] <= 0xbfu &&
               p[3] >= 0x80u && p[3] <= 0xbfu) {
      p += 4;
    } else if (*p == 0xf4u && p[1] >= 0x80u && p[1] <= 0x8fu &&
               p[2] >= 0x80u && p[2] <= 0xbfu &&
               p[3] >= 0x80u && p[3] <= 0xbfu) {
      p += 4;
    } else {
      return false;
    }
  }
  return true;
}


/**
 * Send the data to the client
 * returns TRUE if successful otherwise FALSE
 */
static bool consoleSendMessage(void)
{
  if (crtpSendPacket(&messageToPrint) == pdTRUE)
  {
    messageToPrint.size = 0;
    messageSendingIsPending = false;
  }
  else
  {
    return false;
  }

  return true;
}

void consoleInit()
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, CONSOLE_CHANNEL_LOCAL);
  vSemaphoreCreateBinary(synch);
  messageSendingIsPending = false;
  crtpRegisterPortCB(CRTP_PORT_CONSOLE, consoleCrtpCallback);

  isInit = true;
}

int consoleSourceRegister(ConsoleSource *source)
{
  if (source == NULL) {
    return EINVAL;
  }
  const char *path = source->path;
  if (path == NULL || path[0] == '\0' || !validUtf8(path)) {
    return EINVAL;
  }

  bool segmentStart = true;
  size_t length = 0u;
  for (const char *cursor = path; *cursor != '\0'; cursor++) {
    if (*cursor == ':') {
      if (segmentStart) {
        return EINVAL;
      }
      segmentStart = true;
    } else {
      segmentStart = false;
    }
    length++;
  }
  if (segmentStart || length > (CRTP_MAX_DATA_SIZE - 3u)) {
    return EINVAL;
  }
  taskENTER_CRITICAL();
  if (sourcesFrozen) {
    taskEXIT_CRITICAL();
    return EBUSY;
  }
  for (ConsoleSource *entry = sourceHead; entry != NULL; entry = entry->next) {
    if (entry == source || strcmp(entry->path, path) == 0) {
      taskEXIT_CRITICAL();
      return EEXIST;
    }
  }
  if (sourceCount >= CONSOLE_SOURCE_MAX) {
    taskEXIT_CRITICAL();
    return ENOSPC;
  }

  source->next = NULL;
  source->id = (uint8_t)sourceCount++;
  source->pathLength = (uint8_t)length;
  source->enabled = false;
  if (sourceTail == NULL) {
    sourceHead = source;
  } else {
    sourceTail->next = source;
  }
  sourceTail = source;
  taskEXIT_CRITICAL();
  return 0;
}

void consoleSourceFreeze(void)
{
  taskENTER_CRITICAL();
  sourcesFrozen = true;
  taskEXIT_CRITICAL();
}

bool consoleSourceIsEnabled(const ConsoleSource *source)
{
  bool enabled = false;
  taskENTER_CRITICAL();
  enabled = source->enabled;
  taskEXIT_CRITICAL();
  return enabled;
}

bool consoleSourceSend(const ConsoleSource *source, const uint8_t *data, size_t length)
{
  if (data == NULL || length > (CRTP_MAX_DATA_SIZE - 1u)) {
    return false;
  }

  CRTPPacket packet = {
    .header = CRTP_HEADER(CRTP_PORT_CONSOLE, CONSOLE_CHANNEL_SOURCED),
    .size = (uint8_t)(length + 1u),
  };
  packet.data[0] = source->id;
  memcpy(&packet.data[1], data, length);

  bool accepted = false;
  taskENTER_CRITICAL();
  if (source->enabled) {
    accepted = crtpSendPacket(&packet) == pdTRUE;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

#ifdef UNIT_TEST_MODE
void consoleResetForTest(void)
{
  sourceHead = NULL;
  sourceTail = NULL;
  sourceCount = 0u;
  sourcesFrozen = false;
  messageSendingIsPending = false;
  synch = NULL;
  isInit = false;
}
#endif

/** Return the firmware CRC-32 of the frozen source catalog. */
static uint32_t consoleCatalogCrc(void)
{
  crc32Context_t context;
  crc32ContextInit(&context);
  for (const ConsoleSource *source = sourceHead; source != NULL; source = source->next) {
    crc32Update(&context, &source->id, sizeof(source->id));
    crc32Update(&context, source->path, source->pathLength);
  }
  return crc32Out(&context);
}

static void consoleSendCommandError(CRTPPacket *packet, uint8_t error)
{
  packet->size = 2u;
  packet->data[1] = error;
  (void)crtpSendPacketBlock(packet);
}

static ConsoleSource *consoleSourceById(uint8_t id)
{
  for (ConsoleSource *source = sourceHead; source != NULL; source = source->next) {
    if (source->id == id) {
      return source;
    }
  }
  return NULL;
}

static void consoleHandleControl(CRTPPacket *packet)
{
  if (packet->size == 0u) {
    return;
  }
  switch (packet->data[0]) {
    case CONSOLE_CMD_SET_ENABLED:
      break;
    default:
      consoleSendCommandError(packet, ENOSYS);
      return;
  }
  if (packet->size != 3u || packet->data[2] > 1u) {
    consoleSendCommandError(packet, EINVAL);
    return;
  }

  const uint8_t sourceId = packet->data[1];
  uint8_t result = 0u;
  taskENTER_CRITICAL();
  if (!sourcesFrozen) {
    result = EAGAIN;
  } else if (sourceId == CONSOLE_SOURCE_ALL) {
    for (ConsoleSource *source = sourceHead; source != NULL; source = source->next) {
      source->enabled = packet->data[2] != 0u;
    }
  } else {
    ConsoleSource *source = consoleSourceById(sourceId);
    if (source != NULL) {
      source->enabled = packet->data[2] != 0u;
    } else {
      result = ENOENT;
    }
  }
  taskEXIT_CRITICAL();
  if (result != 0u) {
    consoleSendCommandError(packet, result);
  } else {
    packet->size = 4u;
    packet->data[3] = packet->data[2];
    packet->data[2] = sourceId;
    packet->data[1] = 0u;
    packet->data[0] = CONSOLE_CMD_SET_ENABLED;
    (void)crtpSendPacketBlock(packet);
  }
}

static void consoleHandleCatalog(CRTPPacket *packet)
{
  if (packet->size == 0u) {
    return;
  }
  switch (packet->data[0]) {
    case CONSOLE_TOC_GET_INFO:
      if (packet->size != 1u) {
        consoleSendCommandError(packet, EINVAL);
        return;
      }
      break;
    case CONSOLE_TOC_GET_ITEM:
      if (packet->size != 2u) {
        consoleSendCommandError(packet, EINVAL);
        return;
      }
      break;
    default:
      consoleSendCommandError(packet, ENOSYS);
      return;
  }

  bool frozen;
  taskENTER_CRITICAL();
  frozen = sourcesFrozen;
  taskEXIT_CRITICAL();
  if (!frozen) {
    consoleSendCommandError(packet, EAGAIN);
    return;
  }
  if (packet->data[0] == CONSOLE_TOC_GET_INFO) {
    const uint32_t crc = consoleCatalogCrc();
    packet->size = 7u;
    packet->data[0] = CONSOLE_TOC_GET_INFO;
    packet->data[1] = 0u;
    packet->data[2] = (uint8_t)sourceCount;
    memcpy(&packet->data[3], &crc, sizeof(crc));
    (void)crtpSendPacketBlock(packet);
  } else {
    const uint8_t sourceId = packet->data[1];
    const ConsoleSource *source = consoleSourceById(sourceId);
    if (source == NULL) {
      consoleSendCommandError(packet, ENOENT);
      return;
    } else {
      const size_t length = source->pathLength;
      memcpy(&packet->data[3], source->path, length);
      packet->size = (uint8_t)(length + 3u);
      packet->data[0] = CONSOLE_TOC_GET_ITEM;
      packet->data[1] = 0u;
      packet->data[2] = sourceId;
    }
    (void)crtpSendPacketBlock(packet);
  }
}

static void consoleCrtpCallback(CRTPPacket *packet)
{
  switch (packet->channel) {
    case CONSOLE_CHANNEL_CONTROL:
      consoleHandleControl(packet);
      break;
    case CONSOLE_CHANNEL_TOC:
      consoleHandleCatalog(packet);
      break;
    default:
      break;
  }
}

bool consoleTest(void)
{
  return isInit;
}

int consolePutchar(int ch)
{
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (!isInit) {
    return 0;
  }

  if (isInInterrupt) {
    return consolePutcharFromISR(ch);
  }

  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    // Try to send if we already have a pending message
    if (messageSendingIsPending)
    {
      consoleSendMessage();
    }

    if (! messageSendingIsPending)
    {
      if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
      {
        messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
        messageToPrint.size++;
      }

      if (ch == '\n' || messageToPrint.size >= CRTP_MAX_DATA_SIZE)
      {
        if (crtpGetFreeTxQueuePackets() == 1)
        {
          addBufferFullMarker();
        }
        messageSendingIsPending = true;
        consoleSendMessage();
      }
    }
    xSemaphoreGive(synch);
  }

  return (unsigned char)ch;
}

int consolePutcharFromISR(int ch) {
  BaseType_t higherPriorityTaskWoken;

  if (xSemaphoreTakeFromISR(synch, &higherPriorityTaskWoken) == pdTRUE) {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
    {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    xSemaphoreGiveFromISR(synch, &higherPriorityTaskWoken);
  }

  return ch;
}

int consolePuts(const char *str)
{
  int ret = 0;

  while(*str)
    ret |= consolePutchar(*str++);

  return ret;
}

void consoleFlush(void)
{
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    consoleSendMessage();
    xSemaphoreGive(synch);
  }
}


static int findMarkerStart()
{
  int start = messageToPrint.size;

  // If last char is new line, rewind one char since the marker contains a new line.
  if (start > 0 && messageToPrint.data[start - 1] == '\n')
  {
    start -= 1;
  }

  return start;
}

static void addBufferFullMarker()
{
  // Try to add the marker after the message if it fits in the buffer, otherwise overwrite the end of the message
  int endMarker = findMarkerStart() + sizeof(bufferFullMsg);
  if (endMarker >= (CRTP_MAX_DATA_SIZE))
  {
    endMarker = CRTP_MAX_DATA_SIZE;
  }

  int startMarker = endMarker - sizeof(bufferFullMsg);
  memcpy(&messageToPrint.data[startMarker], bufferFullMsg, sizeof(bufferFullMsg));
  messageToPrint.size = startMarker + sizeof(bufferFullMsg);
}
