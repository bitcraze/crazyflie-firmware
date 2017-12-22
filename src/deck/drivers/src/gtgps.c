/*
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
 * exptest.c - Testing of expansion port.
 */
#define DEBUG_MODULE "GTGPS"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"

static bool isInit;

#define LEN_TOKEN 5
#define MAX_LEN_SENTANCE 100
char buff[MAX_LEN_SENTANCE];
uint8_t bi;

//typedef bool (*SentanceParser)(char * buff);

/*typedef struct {
  const char * token;
  SentanceParser parser;
} ParserConfig;*/

typedef enum {
  FixNone = 1,
  Fix2D = 2,
  Fix3D = 3
} FixQuality;

typedef enum {
  NoFix = 1,
  GPSFix = 2
} FixType;

typedef struct {
  FixQuality fix;
  uint32_t locks[20];
  float pdop;
  float hdop;
  float vdop;
} Basic;

typedef struct {
  uint32_t fixtime;
  uint32_t latitude_d;
  uint32_t latitude_m;
  uint8_t NS;
  uint32_t longitude_d;
  uint32_t longitude_m;
  uint8_t EW;
  FixType fixtype;
  uint8_t nsat;
  float hdop;
  float alt;
  float height;
} MeasData;

//static Basic b;
//static MeasData m;
bool result = false;
uint8_t messages = 0;

/*static bool gnrmcParser(char * buff) {
  int i = 0;
  consolePutchar('$');
  consolePutchar('G');
  consolePutchar('N');
  consolePutchar('R');
  consolePutchar('M');
  consolePutchar('C');
  while (buff[i] != 0) {
      consolePutchar(buff[i]);
      i++;
  }
  consolePutchar(0x0c);
  if (messages == 1) {
    for (i = 0; i< MAX_LEN_SENTANCE; i++){  // longueur de buff, plutôt,  // strlen ?
        consolePutchar(buff[i]);}
  }
  return false;
}*/

/*static ParserConfig parsers[] = {
  {.token = "GNRMC", .parser = gnrmcParser}
};*/
/*
static bool verifyChecksum(const char * buff) {
  uint8_t test_chksum = 0;
  uint32_t ref_chksum = 0;
  uint8_t i = 0;
  while (buff[i] != '*' && i < MAX_LEN_SENTANCE-3) {
    test_chksum ^= buff[i++];
  }
  ref_chksum = strtol(&buff[i+1], 0, 16);

  return (test_chksum == ref_chksum);
}*/

void gtgpsTask(void *param)
{
  char ch;
//  int j;

  vTaskDelay(500);
//  uart1Init(115200);
  vTaskDelay(500);

//  uart1SendData(sizeof(updaterate), updaterate);
//  uart1SendData(sizeof(updaterate2), updaterate2);

//  uart1SendData(sizeof(updaterate3), updaterate3);
//  uart1SendData(sizeof(updaterate4), updaterate4);


  while(1)
  {
    uart1Getchar(&ch);
//    if (messages == 1) {consolePutchar(ch);}
    if (ch != 0){consolePutchar(ch);}
//    if (ch == 10){consolePuts("FIN");}
//    if (ch == '*'){consolePutchar(ch);}
//    if (ch == '\n'){consolePutchar(ch);}
//    consolePutchar('$');
//    consolePutchar(10);
//    consolePutchar('\n');
//    consolePutchar('\r');
//    consolePutchar('*');
//    consolePutchar('A');
//    if (ch == '\r'){consolePuts("slash r");}
/*    if (ch == '$') {
      buff[0] = '$';
      buff[1] = '$';
      bi = 1;
//      consolePutchar('$');
    } else if ((ch == '\r') | (ch == '\n')){
      buff[bi] = 0; // Terminate with null
      if (verifyChecksum(buff)) {
//          consolePuts(buff);
          DEBUG_PRINT("yyyyyyyyyyyyyyyyyyyyy%s",buff);
          buff[0] = 0;
      }
    } else if (bi < MAX_LEN_SENTANCE) {
      buff[bi++] = ch;
//      consolePutchar(ch);
    }
    if (strlen(buff) > 0){consolePuts(buff);}*/
  }
}


static void gtgpsInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Enabling reading from GlobalTop GPS\n");
  uart1Init(9600);

  xTaskCreate(gtgpsTask, "GTGPS",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}

static bool gtgpsTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver gtgps_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcGTGPS",

  .usedPeriph = 0,
  .usedGpio = DECK_USING_TX1 | DECK_USING_RX1,               // FIXME: Edit the used GPIOs

  .init = gtgpsInit,
  .test = gtgpsTest,
};

DECK_DRIVER(gtgps_deck);
/*
LOG_GROUP_START(gps_base)
LOG_ADD(LOG_FLOAT, time, &m.fixtime)
LOG_ADD(LOG_FLOAT, hAcc, &b.hdop)
LOG_ADD(LOG_UINT8, nsat, &m.nsat)
LOG_ADD(LOG_UINT8, fix, &b.fix)
LOG_GROUP_STOP(gps_base)

LOG_GROUP_START(gps_track)
LOG_ADD(LOG_UINT32, lat_d, &m.latitude_d)
LOG_ADD(LOG_UINT32, lat_m, &m.latitude_m)
LOG_ADD(LOG_UINT8, NS, &m.NS)
LOG_ADD(LOG_UINT32, lon_d, &m.longitude_d)
LOG_ADD(LOG_UINT32, lon_m, &m.longitude_m)
LOG_ADD(LOG_UINT8, EW, &m.EW)
LOG_ADD(LOG_FLOAT, hMSL, &m.alt)
LOG_GROUP_STOP(gps_track)

PARAM_GROUP_START(gps)
PARAM_ADD(PARAM_INT8, messages, &messages)
PARAM_GROUP_STOP(gps)*/
