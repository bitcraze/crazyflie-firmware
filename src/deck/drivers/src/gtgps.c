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

typedef enum {FIELD_COORD, FIELD_FLOAT, FIELD_INT, FIELD_CHAR} FieldType;

typedef enum {
  NoFix = 1,
  GPSFix = 2
} FixType;

typedef struct {
  uint32_t locks[12];
  float pdop;
  float hdop;
  float vdop;
  float height;
  FixType fixtype;
} Basic;

typedef struct {
  uint32_t fixtime;
  uint8_t status;
  uint32_t latitude_d;
  uint32_t latitude_m;
  uint8_t NS;
  uint32_t longitude_d;
  uint32_t longitude_m;
  uint8_t EW;
  uint32_t fix;
  uint8_t nsat;
  float alt;
//  uint8_t posMode;
//  uint8_t navStatus;
} MeasData;

static Basic b;
static MeasData m;
//static float empty = 0.;
bool longitude = false;
bool correct = false; // cas longitude > 180
uint8_t messages = 0;

// Only use on 0-terminated strings!
static int skip_to_next(char ** sp, const char ch) {
  int steps;
  while (ch != 0 && (**sp) != ch) {
    (*sp)++;
    steps++;
  }
  if (ch != 0)
    (*sp)++;
  return (ch != 0 ? steps : -1);
}

static uint32_t parse_coordinate(char ** sp) {
  uint32_t dm;
  uint16_t degree;
  uint32_t minute;
  uint32_t second;
  uint32_t ret;
  char * i;
  char * j;

//  *sp = "27833.914843";
  dm = strtoul(*sp, &i, 10);
  degree = dm / 100;
  if (longitude){m.longitude_d = degree;}
  else {m.latitude_d = degree;}
  second = strtoul(i+1, &j, 10);
  minute = (dm % 100) * 10000000;// * 100000);// / 60;
  ret = minute + second; //Transmets les degrés d'une part, les minutes avec décimales d'autre part
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
    case FIELD_CHAR:
      value = *sp;
      break;
    case FIELD_INT:
      *((uint32_t*) value) = strtoul(*sp, 0, 10);
      break;
    case FIELD_FLOAT:
      *((float*) value) = parse_float(*sp);
      break;
    case FIELD_COORD:
      *((uint32_t*) value) = parse_coordinate(sp);
  }
}

static bool gnrmcParser(char * buff) {
/*  char * sp = buff;

  parse_next(&sp, FIELD_INT, &m.fixtime);
  parse_next(&sp, FIELD_CHAR, &m.status);
  longitude = false;
  parse_next(&sp, FIELD_COORD, &m.latitude_m);//minutes and seconds only
  parse_next(&sp, FIELD_CHAR, &m.NS);
  longitude = true;
  parse_next(&sp, FIELD_COORD, &m.longitude_m);//minutes and seconds only
  parse_next(&sp, FIELD_CHAR, &m.EW);
  parse_next(&sp, FIELD_FLOAT, &empty);//speed
  parse_next(&sp, FIELD_FLOAT, &empty);//cog
  parse_next(&sp, FIELD_INT, &empty);//date
  parse_next(&sp, FIELD_FLOAT, &empty);//magnetic
  parse_next(&sp, FIELD_CHAR, &empty);//magnetic indicator
  parse_next(&sp, FIELD_CHAR, &m.posMode);
  parse_next(&sp, FIELD_CHAR, &m.navStatus);*/
  return false;
}

static bool gnggaParser(char * buff) {
  char * sp = buff;

  parse_next(&sp, FIELD_INT, &m.fixtime);
  longitude = false;
  parse_next(&sp, FIELD_COORD, &m.latitude_m);//minutes and seconds only
  parse_next(&sp, FIELD_CHAR, &m.NS);
//  m.NS = 'N';
  longitude = true;
  parse_next(&sp, FIELD_COORD, &m.longitude_m);//minutes and seconds only
  parse_next(&sp, FIELD_CHAR, &m.EW);
//  m.EW = 'E';
  parse_next(&sp, FIELD_INT, &m.fix);
//  m.fix = 1;
  parse_next(&sp, FIELD_INT, &m.nsat);
//  m.nsat = 8;
  parse_next(&sp, FIELD_FLOAT, &b.hdop);
  parse_next(&sp, FIELD_FLOAT, &m.alt);
  return false;
}


static bool gngsaParser(char * buff) {
  char * sp = buff;
  // Skip leading A/M
  skip_to_next(&sp, ',');
  skip_to_next(&sp, ',');
  parse_next(&sp, FIELD_INT, &m.nsat);
  return false;
}

static ParserConfig parsers[] = {
  {.token = "GNGSA", .parser = gngsaParser},
  {.token = "GNGGA", .parser = gnggaParser},
  {.token = "GNRMC", .parser = gnrmcParser}
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

void gtgpsTask(void *param)
{
  char ch;
  int j;

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
    if ((messages == 1) & (ch != 0)) {consolePutchar(ch);}

    if (ch == '$') {
      bi = 0;
//      consolePutchar('$');////
    } else if (ch == '\n') {
      buff[bi] = 0; // Terminate with null
      if (verifyChecksum(buff)) {
        for (j = 0; j < sizeof(parsers)/sizeof(parsers[0]); j++) {
          if (strncmp(parsers[j].token, buff, LEN_TOKEN) == 0) {
            parsers[j].parser(&buff[LEN_TOKEN]);
          }
        }
      }
    } else if (bi < MAX_LEN_SENTANCE) {
      buff[bi++] = ch;
//      consolePutchar(ch);
    }
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

LOG_GROUP_START(gps_base)
LOG_ADD(LOG_FLOAT, time, &m.fixtime)
LOG_ADD(LOG_FLOAT, hAcc, &b.hdop)
LOG_ADD(LOG_UINT8, nsat, &m.nsat)
LOG_ADD(LOG_UINT8, fixquality, &m.fix)
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
PARAM_GROUP_STOP(gps)
