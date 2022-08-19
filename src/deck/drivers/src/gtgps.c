/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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

typedef bool (*SentanceParser)(char * buff);

typedef struct {
  const char * token;
  SentanceParser parser;
} ParserConfig;

typedef enum {
  FixNone = 1,
  Fix2D = 2,
  Fix3D = 3
} FixQuality;

typedef enum {
  NoFix = 1,
  GPSFix = 2
} FixType;

typedef enum {FIELD_COORD, FIELD_FLOAT, FIELD_INT} FieldType;

typedef struct {
  FixQuality fix;
  uint32_t locks[12];
  float pdop;
  float hdop;
  float vdop;
} Basic;

typedef struct {
  uint32_t fixtime;
  int32_t latitude;
  int32_t longitude;
  FixType fixtype;
  uint32_t nsat;
  float hdop;
  float alt;
  float height;
} MeasData;

static Basic b;
static MeasData m;

// Only use on 0-terminated strings!
static int skip_to_next(char ** sp, const char ch) {
  int steps=0;
  while (ch != 0 && (**sp) != ch) {
    (*sp)++;
    steps++;
  }
  if (ch != 0)
    (*sp)++;
  return (ch != 0 ? steps : -1);
}

static int32_t parse_coordinate(char ** sp) {
  int32_t dm;
  int32_t degree;
  int32_t minute;
  int32_t second;
  int32_t ret;
  char * i;
  char * j;

  // Format as DDDMM.SSSS converted by long or lat = DDD + MM / 100 + SSSS/3600
  // To avoid inaccuracy caused by float representation save this value as
  // a large number * 10 M

  // 32 18.0489 N = 32 degrees + 18.0489 / 60 = 32.300815 N
  dm = strtol(*sp, &i, 10);
  degree = (dm / 100) * 10000000;
  minute = ((dm % 100) * 10000000) / 60;
  second = (strtol(i+1, &j, 10) * 1000) / 60;
  ret = degree + minute + second;
  skip_to_next(sp, ',');
  if (**sp == 'S' || **sp == 'W')
    ret *= -1;
  return ret;
}

static float parse_float(char * sp) {
  float ret = 0;
  int major = 0;
  int minor = 0;
  int deci_nbr = 0;
  char * i;
  char * j;

  major = strtol(sp, &i, 10);
  // Do decimals
  if (strncmp(i, ".", 1) == 0) {
    minor = strtol(i+1, &j, 10);
    deci_nbr = j - i - 1;
  }
  ret = (major * pow(10, deci_nbr) + minor) / pow(10, deci_nbr);
  //printf("%i.%i == %f (%i) (%c)\n", major, minor, ret, deci_nbr, (int) *i);
  return ret;
}

static void parse_next(char ** sp, FieldType t, void * value) {
  skip_to_next(sp, ',');
  //DEBUG_PRINT("[%s]\n", (*sp));
  switch (t) {
    case FIELD_INT:
      *((uint32_t*) value) = strtol(*sp, 0, 10);
      break;
    case FIELD_FLOAT:
      *((float*) value) = parse_float(*sp);
      break;
    case FIELD_COORD:
      *((int32_t*) value) = parse_coordinate(sp);
  }
}

static bool gpgsaParser(char * buff) {
  int i = 0;
  char * sp = buff;

  // Skip leading A/M
  skip_to_next(&sp, ',');

  parse_next(&sp, FIELD_INT, &b.fix);
  for (i = 0; i < 12; i++) {
    parse_next(&sp, FIELD_INT, &b.locks[i]);
  }
  parse_next(&sp, FIELD_FLOAT, &b.pdop);
  parse_next(&sp, FIELD_FLOAT, &b.hdop);
  parse_next(&sp, FIELD_FLOAT, &b.vdop);

  //dbg_print_basic(&b);
  return false;
}

static bool gpggaParser(char * buff) {
  char * sp = buff;

  parse_next(&sp, FIELD_INT, &m.fixtime);
  parse_next(&sp, FIELD_COORD, &m.latitude);
  parse_next(&sp, FIELD_COORD, &m.longitude);
  parse_next(&sp, FIELD_INT, &m.fixtype);
  parse_next(&sp, FIELD_INT, &m.nsat);
  parse_next(&sp, FIELD_FLOAT, &m.hdop);
  parse_next(&sp, FIELD_FLOAT, &m.alt);
  skip_to_next(&sp, ',');
  // Unit for altitude (not used yet)
  parse_next(&sp, FIELD_FLOAT, &m.height);
  skip_to_next(&sp, ',');
  // Unit for height (not used yet)
  skip_to_next(&sp, ',');
  //consolePutchar('.');
  //consoleFlush();
  return false;
}

static ParserConfig parsers[] = {
  {.token = "GPGSA", .parser = gpgsaParser},
  {.token = "GPGGA", .parser = gpggaParser}
};

static bool verifyChecksum(const char * buff) {
  uint8_t test_chksum = 0;
  uint32_t ref_chksum = 0;
  uint8_t i = 0;
  while (buff[i] != '*' && i < MAX_LEN_SENTANCE-3) {
    test_chksum ^= buff[i++];
  }
  ref_chksum = strtol(&buff[i+1], 0, 16);

  return (test_chksum == ref_chksum);
}

static uint8_t baudcmd[] = "$PMTK251,115200*1F\r\n";

// 5 Hz
static uint8_t updaterate[] = "$PMTK220,200*2C\r\n";
static uint8_t updaterate2[] = "$PMTK300,200,0,0,0,0*2F\r\n";

// 10 Hz
//static uint8_t updaterate3[] = "$PMTK220,100*2F\r\n";
//static uint8_t updaterate4[] = "$PMTK300,100,0,0,0,0*2C\r\n";


void gtgpsTask(void *param)
{
  char ch;
  int j;

  uart1SendData(sizeof(baudcmd), baudcmd);

  vTaskDelay(500);
  uart1Init(115200);
  vTaskDelay(500);

  uart1SendData(sizeof(updaterate), updaterate);
  uart1SendData(sizeof(updaterate2), updaterate2);

//  uart1SendData(sizeof(updaterate3), updaterate3);
//  uart1SendData(sizeof(updaterate4), updaterate4);


  while(1)
  {
    uart1Getchar(&ch);
    consolePutchar(ch);

    if (ch == '$') {
      bi = 0;
    } else if (ch == '\n') {
      buff[bi] = 0; // Terminate with null
      if (verifyChecksum(buff)) {
        //DEBUG_PRINT("O");
        for (j = 0; j < sizeof(parsers)/sizeof(parsers[0]); j++) {
          if (strncmp(parsers[j].token, buff, LEN_TOKEN) == 0) {
            parsers[j].parser(&buff[LEN_TOKEN]);
          }
        }
      }
    } else if (bi < MAX_LEN_SENTANCE) {
      buff[bi++] = ch;
    }
  }
}


static void gtgpsInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Enabling reading from GlobalTop GPS\n");
  uart1Init(9600);

  xTaskCreate(gtgpsTask, GTGPS_DECK_TASK_NAME,
              GTGPS_DECK_TASK_STACKSIZE, NULL, GTGPS_DECK_TASK_PRI, NULL);

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
  .vid = 0xBC,
  .pid = 0x07,
  .name = "bcGTGPS",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,

  .init = gtgpsInit,
  .test = gtgpsTest,
};

DECK_DRIVER(gtgps_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(gps)
LOG_ADD(LOG_INT32, lat, &m.latitude)
LOG_ADD(LOG_INT32, lon, &m.longitude)
LOG_ADD(LOG_FLOAT, hMSL, &m.height)
LOG_ADD(LOG_FLOAT, hAcc, &b.pdop)
LOG_ADD(LOG_INT32, nsat, &m.nsat)
LOG_ADD(LOG_INT32, fix, &b.fix)
LOG_GROUP_STOP(gps)
