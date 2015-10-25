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

#include <stdbool.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "semphr.h"

#include "crtp.h"

CRTPPacket messageToPrint;
xSemaphoreHandle synch = NULL;

static const char fullMsg[] = "<F>\n";
static bool isInit;

/**
 * Send the data to the client
 * returns TRUE if successful otherwise FALSE
 */
static bool consoleSendMessage(void)
{

  if (crtpSendPacket(&messageToPrint) == pdTRUE)
  {
    messageToPrint.size = 0;
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
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  vSemaphoreCreateBinary(synch);
  
  isInit = true;
}

bool consoleTest(void)
{
  return isInit;
}

int consolePutchar(int ch)
{
  int i;

  if (!isInit)
    return 0;

  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
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
        for (i = 0; i < sizeof(fullMsg) && (messageToPrint.size - i) > 0; i++)
        {
          messageToPrint.data[messageToPrint.size - i] =
              (uint8_t)fullMsg[sizeof(fullMsg) - i - 1];
        }
      }
      consoleSendMessage();
    }
    xSemaphoreGive(synch);
  }
  
  return (unsigned char)ch;
}

int consolePuts(char *str)
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
