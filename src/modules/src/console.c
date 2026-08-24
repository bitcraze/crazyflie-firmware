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
/** Maximum number of boot-lifetime console sources. */
#define CONSOLE_SOURCE_MAX 8u
/** Runtime-control command that changes source enable state. */
#define CONSOLE_CMD_SET_ENABLED 0x00u
/** Source-catalog command that returns one entry. */
#define CONSOLE_TOC_GET_ITEM 0x00u
/** Source-catalog command that returns count and CRC. */
#define CONSOLE_TOC_GET_INFO 0x01u

/** One entry in the immutable boot-lifetime source catalog. */
typedef struct {
  char path[CRTP_MAX_DATA_SIZE - 1u];  ///< Owned NUL-terminated source path.
  uint8_t pathLength;                 ///< Encoded path length without NUL.
  volatile bool enabled;              ///< Current client-controlled state.
} ConsoleSource;

/** Registered source catalog in stable source-ID order. */
static ConsoleSource sources[CONSOLE_SOURCE_MAX];
/** Number of populated entries in sources. */
static uint8_t sourceCount;
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
/** Send a command-level errno response. */
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

int consoleSourceRegister(const char *path)
{
  if (path == NULL || path[0] == '\0' || !validUtf8(path)) {
    return -1;
  }

  bool segmentStart = true;
  size_t length = 0u;
  for (const char *cursor = path; *cursor != '\0'; cursor++) {
    if (*cursor == ':') {
      if (segmentStart) {
        return -1;
      }
      segmentStart = true;
    } else {
      segmentStart = false;
    }
    length++;
  }
  if (segmentStart || length > (CRTP_MAX_DATA_SIZE - 2u)) {
    return -1;
  }
  int result = -1;
  taskENTER_CRITICAL();
  if (sourcesFrozen) {
    taskEXIT_CRITICAL();
    return result;
  }
  for (uint8_t i = 0u; i < sourceCount; i++) {
    if (strcmp(sources[i].path, path) == 0) {
      result = i;
      taskEXIT_CRITICAL();
      return result;
    }
  }
  if (sourceCount >= CONSOLE_SOURCE_MAX) {
    taskEXIT_CRITICAL();
    return result;
  }

  const uint8_t id = sourceCount++;
  memcpy(sources[id].path, path, length + 1u);
  sources[id].pathLength = (uint8_t)length;
  sources[id].enabled = false;
  result = id;
  taskEXIT_CRITICAL();
  return result;
}

void consoleSourceFreeze(void)
{
  taskENTER_CRITICAL();
  sourcesFrozen = true;
  taskEXIT_CRITICAL();
}

bool consoleSourceIsEnabled(uint8_t sourceId)
{
  bool enabled = false;
  taskENTER_CRITICAL();
  if (sourceId < sourceCount) {
    enabled = sources[sourceId].enabled;
  }
  taskEXIT_CRITICAL();
  return enabled;
}

bool consoleSourceSend(uint8_t sourceId, const uint8_t *data, size_t length)
{
  if (data == NULL || length > (CRTP_MAX_DATA_SIZE - 1u)) {
    return false;
  }

  CRTPPacket packet = {
    .header = CRTP_HEADER(CRTP_PORT_CONSOLE, CONSOLE_CHANNEL_SOURCED),
    .size = (uint8_t)(length + 1u),
  };
  packet.data[0] = sourceId;
  memcpy(&packet.data[1], data, length);

  bool accepted = false;
  taskENTER_CRITICAL();
  if (sourceId < sourceCount && sources[sourceId].enabled) {
    accepted = crtpSendPacket(&packet) == pdTRUE;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

#ifdef UNIT_TEST_MODE
void consoleResetForTest(void)
{
  memset(sources, 0, sizeof(sources));
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
  for (uint8_t id = 0u; id < sourceCount; id++) {
    crc32Update(&context, &id, sizeof(id));
    crc32Update(&context, sources[id].path, sources[id].pathLength);
  }
  return crc32Out(&context);
}

static void consoleSendCommandError(CRTPPacket *packet, uint8_t error)
{
  packet->size = 2u;
  packet->data[1] = error;
  (void)crtpSendPacketBlock(packet);
}

static void consoleCrtpCallback(CRTPPacket *packet)
{
  if (packet->channel == CONSOLE_CHANNEL_CONTROL) {
    if (packet->size == 0u) {
      return;
    }
    if (packet->data[0] != CONSOLE_CMD_SET_ENABLED) {
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
      for (uint8_t i = 0u; i < sourceCount; i++) {
        sources[i].enabled = packet->data[2] != 0u;
      }
    } else if (sourceId < sourceCount) {
      sources[sourceId].enabled = packet->data[2] != 0u;
    } else {
      result = ENOENT;
    }
    taskEXIT_CRITICAL();
    packet->size = 4u;
    packet->data[3] = result;
    (void)crtpSendPacketBlock(packet);
    return;
  }

  if (packet->channel != CONSOLE_CHANNEL_TOC || packet->size == 0u) {
    return;
  }
  if (packet->data[0] == CONSOLE_TOC_GET_INFO) {
    if (packet->size != 1u) {
      consoleSendCommandError(packet, EINVAL);
      return;
    }
  } else if (packet->data[0] == CONSOLE_TOC_GET_ITEM) {
    if (packet->size != 2u) {
      consoleSendCommandError(packet, EINVAL);
      return;
    }
  } else {
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
    packet->size = 6u;
    packet->data[1] = sourceCount;
    memcpy(&packet->data[2], &crc, sizeof(crc));
    (void)crtpSendPacketBlock(packet);
  } else {
    const uint8_t sourceId = packet->data[1];
    if (sourceId >= sourceCount) {
      consoleSendCommandError(packet, ENOENT);
      return;
    } else {
      const size_t length = sources[sourceId].pathLength;
      memcpy(&packet->data[2], sources[sourceId].path, length);
      packet->size = (uint8_t)(length + 2u);
    }
    (void)crtpSendPacketBlock(packet);
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
