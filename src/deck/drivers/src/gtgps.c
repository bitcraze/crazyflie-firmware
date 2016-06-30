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
 * gtgps.c - Titan-2 GNSS Receiver Expansion Deck Driver.
 */
#define DEBUG_MODULE "GTGPS"

#include <stdint.h>
#include <string.h>
//#include <stdio.h> //conflict with stabilizer_types.h via #include "compass.h"

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

#include "compass.h"

/** 
 * This prototype driver supports different configurations of the Titan-2  
 * Depending upon the firmware inside the Titan-2
 * The default factory firmware only supports NMEA mode output (FFWNMEA)
 * The customized firmware supports NMEA mode and Binary mode (CFWNMEA & CFWBIN) ) 
 */

#define ENABLE_GPS_DECK

// Note - These are mutually exclusive 
//#define FFWNMEA
//#define CFWNMEA
#define CFWBIN

#ifdef FFWNMEA
// Default is 9600 baudrate and update rate is 1 Hz
// Default NMEA output GGA(1), GSA(1), GSV(5), RMC(1), VTG(1)
// Default is WGS84, SBAS Mode enabled, DGPS Mode enabled
// Lat/Lon data in NMEA 4 decimal places
// Note - GGA is $GPGGA format
#endif

#ifdef CFWNMEA
// Default is 115200 baudrate and update rate is 5 Hz
// Default NMEA output GLL(1), RMC(1), VTG(1), GSA(1), GSV(1), GGA(1)
// Default is WGS84, SBAS Mode enabled, DGPS Mode enabled
// Lat/Lon data in NMEA 6 decimal places
// Note - GGA is $GNGGA format
#endif

/**
 * gtgps.h is needed for the Position Hold capability
 * and when gygps.c is called by stabilizer.c
 */ 

#include "gtgps.h" 

static bool isInit;

#if defined(FFWNMEA) || defined(CFWNMEA)
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
  NoFix = 0,
  GPSFix = 1,
  DGPSFix = 2
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
#endif

#ifdef CFWBIN
typedef struct {
  uint8_t fix;
  uint32_t pdop;
} Basic;

typedef struct {
  uint32_t utc;
  uint32_t date;
  int32_t latitude;
  uint8_t N_S;
  int32_t longitude;
  uint8_t E_W;
  uint8_t fixtype;
  uint32_t alt;
  uint32_t cog;
  uint32_t smg;
  uint8_t sats;
  uint8_t nsat;
  uint32_t hdop;
  uint32_t temp32;
  uint16_t temp16;
} MeasData;

struct nav_onesentence {
    uint32_t utc;            //hhmmss.sss * 10^3
    uint32_t date;           //ddmmyy
    uint32_t lat;            //dd.dddddd * 10^6 (deg)
    uint8_t N_S;             //North = 1, South = 2
    uint32_t lon;            //ddd.dddddd * 10^6 (deg)
    uint8_t E_W;             //East = 1, West = 2
    uint8_t fixquality;      //No GPS = 1, 2D Fix = 2, 3D-GPS = 3
    uint8_t fixmode;         //No GPS = 0, SPS Fix = 1, DGPS = 2    
    uint32_t altitude;       //mm.mm * 10^2 (m)
    uint32_t cog;            //dd.dd * 10^2 (deg)
    uint32_t smg;            //v.vv * 10^2 (km/hr)
    uint8_t sats;            //number of satellites in view
    uint8_t nsat;            //number of satellites in use
    uint16_t hdop;           //v.vv * 10^2
    uint16_t epe;            //v.vv * 10^2
} __attribute__ ((packed));

struct gtbin_message {
    union {
    	void * payload;
    	struct nav_onesentence* nav_onesentence;
    };
} __attribute__ ((packed));

#endif

static Basic b;
static MeasData m;

static uint8_t  gps_fixType;
static uint8_t  gps_fixStatus;
static bool     NewFrame = false;
static float    gps_hMSL;
static float    gps_hAcc;
static float    gps_pAcc;
static float    gps_scaleLat = 0.011113295f; //degrees * 1e+7 convert to meters
static float    gps_scaleLon;
static float    D2R;
static uint32_t gps_fixTime;
static bool     gps_setHome = false;
static int32_t  gps_latHome = 0;
static int32_t  gps_lonHome = 0;

static uint32_t timestamp;
static float pos_x;
static float pos_y;
static float pos_z;

void gtgpsGetFrameData(uint32_t* ts, float* px, float* py, float* pz)
{
  *ts = timestamp;
  *px = pos_x;
  *py = pos_y;
  *pz = pos_z;
}

static void saveFrameData(void)
{
  static float phi;
    
//Assume 3D Fix or 3D Fix/DGPS    
  if ((gps_fixType > 2) && (m.nsat >= 5) && (gps_hAcc <= 2.0f))
  {
    if (!gps_setHome)  
    {
      gps_latHome = m.latitude;   //deg * 1e+7
      gps_lonHome = m.longitude;   //deg * 1e+7
      gps_setHome = true;
      phi = m.latitude / 10000000.0f * D2R;
      gps_scaleLon = cosf(phi) * gps_scaleLat;    
    }
    
//Assume right-hand xyz cartesian coordinate system
//Assume x = latitude pointing to North geographic or plus latitude  
//Assume y = longitude pointing to West geographic or minus longitude
//Assume z = vertical pointing downward or minus hMSL
//Assume yawgeo is +/- 180 degrees and + is counterclockwise
//Assume yawgeo in degrees is available to map xyz to level cf1/cf2 fwd direction   
//Preserve 1 cm significance by using Home position offset

    else if (compassCaled())
    {
      timestamp = 1;  
      pos_x = (m.latitude - gps_latHome) * gps_scaleLat;   //meters
      pos_y = -(m.longitude - gps_lonHome) * gps_scaleLon;  //meters
      pos_z = -gps_hMSL;                              //meters
    }
  }  
  else
  {
    timestamp = 0;     
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_z = 0.0f;
  }

//  In converting position (desired-measured) to roll/pitch
//  D2R = (float) M_PI/180.0f
//  cos = cosf(yawgeo * D2R)
//  sin = sinf(yawgeo * D2R)
//  pitch = - position.x * cos - position.y * sin
//  roll  = - position.y * cos + position.x * sin
  
}

#if defined(FFWNMEA) || defined(CFWNMEA)
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

static int32_t parse_coordinate(char ** sp) {
  int32_t dm;
  int32_t degree;
  int32_t minute;
  int32_t second;
  int32_t ret;
  char * i;
  char * j;

  // Format as DDDMM.SSSS converted by long or lat = DDD + MM / 100 + SSSS/3600
  // Format as DDDMM.SSSSSS converted by long or lat = DDD + MM / 100 + SSSSSS/3600
  // To avoid inaccuracy caused by float representation save this value as
  // a large number * 10 M

  // 32 18.0489 N = 32 degrees + 18.0489 / 60 = 32.300815 N
  dm = strtol(*sp, &i, 10);
  degree = (dm / 100) * 10000000;
  minute = ((dm % 100) * 10000000) / 60;
  //second = (strtol(i+1, &j, 10) * 1000) / 60;
#endif
#ifdef FFWNMEA  
  second = (strtol(i+1, &j, 10) * 1000) / 60;
#endif  
#ifdef CFWNMEA
  second = (strtol(i+1, &j, 10) * 10) / 60;
#endif  
#if defined(FFWNMEA) || defined(CFWNMEA)
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
  if(b.fix == 1) b.fix = 0;
  for (i = 0; i < 12; i++) {
    parse_next(&sp, FIELD_INT, &b.locks[i]);
  }
  parse_next(&sp, FIELD_FLOAT, &b.pdop);
  gps_pAcc = b.pdop;
  parse_next(&sp, FIELD_FLOAT, &b.hdop);
  parse_next(&sp, FIELD_FLOAT, &b.vdop);

  //dbg_print_basic(&b);
  return false;
}

// Used by both $GPGGA & $GNGGA
static bool gxggaParser(char * buff) {
    
  char * sp = buff;

  parse_next(&sp, FIELD_INT, &m.fixtime);
  gps_fixTime = m.fixtime;
  parse_next(&sp, FIELD_COORD, &m.latitude);
  parse_next(&sp, FIELD_COORD, &m.longitude);
  parse_next(&sp, FIELD_INT, &m.fixtype);
  gps_fixStatus = m.fixtype;
  if(m.fixtype > 1) m.fixtype = 1; else m.fixtype = 0;
  parse_next(&sp, FIELD_INT, &m.nsat);
  parse_next(&sp, FIELD_FLOAT, &m.hdop);
  gps_hAcc = m.hdop;
  parse_next(&sp, FIELD_FLOAT, &m.alt);
  gps_hMSL = m.alt;
  skip_to_next(&sp, ',');
  // Unit for altitude (not used yet)
  parse_next(&sp, FIELD_FLOAT, &m.height);
  skip_to_next(&sp, ',');
  // Unit for height (not used yet)
  skip_to_next(&sp, ',');
  //consolePutchar('.');
  //consoleFlush();
  if(m.fixtype == 1)
  {
    if(b.fix == 2)
    {
      gps_fixType = 6;
    }
    else  if(b.fix == 3)
    {
      gps_fixType = 7;
    }
  }
  else
  {
    gps_fixType = b.fix;
  }
  saveFrameData();
  NewFrame = false;
}

static ParserConfig parsers[] = {
#endif
#ifdef FFWNMEA    
  {.token = "GPGSA", .parser = gpgsaParser},  
  {.token = "GPGGA", .parser = gxggaParser}
#endif
#ifdef CFWNMEA
  {.token = "GPGSA", .parser = gpgsaParser},  
  {.token = "GNGGA", .parser = gxggaParser}
#endif
#if defined(FFWNMEA) || defined(CFWNMEA)  
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
#endif

#ifdef CFWBIN
static char gtgpsGetc()
{
  char c;
  uart1Getchar(&c);
  return c;
}

static void gtgpsRead(void *buffer, int length)
{
  int i;

  for (i=0; i<length; i++)
  {
    ((char*)buffer)[i] = gtgpsGetc();
  }
}

static void gtgpsSwitch(void *buffer_in, void *buffer_out, int length)
{
  int i, j;
  
  j = length - 1;
  for (i=0; i<length; i++)
  {  
    ((char*)buffer_out)[i] = ((char*)buffer_in)[j--];
  }
}

uint8_t msg_len = 38;
uint8_t ref_chksum = 0;
static void gtgpsReceiveBin(struct gtbin_message* msg, int maxPayload)
{
  while (!NewFrame)
  {
    if ((uint8_t)gtgpsGetc() != 0x04)
      continue;
    if ((uint8_t)gtgpsGetc() != 0x24)
      continue;
    if ((uint8_t)gtgpsGetc() != 0x02)
      continue;
    gtgpsRead(msg->payload, msg_len); 
    if ((uint8_t)gtgpsGetc() != 0x2A)
      continue;
    ref_chksum = gtgpsGetc();
    if ((uint8_t)gtgpsGetc() != 0x0D)
      continue;
    if ((uint8_t)gtgpsGetc() != 0x0A)
      continue;

    NewFrame = true;;
  }    
}
#endif

#ifdef FFWNMEA
//static uint8_t baudcmd[] = "$PMTK251,115200*1F\r\n";

// 5 Hz
//static uint8_t updaterate[] = "$PMTK220,200*2C\r\n";
//static uint8_t updaterate2[] = "$PMTK300,200,0,0,0,0*2F\r\n";

// 10 Hz - Note SBAS Mode cannot be enabled at this update rate
//static uint8_t updaterate3[] = "$PMTK220,100*2F\r\n";
//static uint8_t updaterate4[] = "$PMTK300,100,0,0,0,0*2C\r\n";

static uint8_t setnmeatypescmd[] = "$PMTK314,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
#endif

#ifdef CFWNMEA
static uint8_t setnmeatypescmd[] = "$PMTK314,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
#endif

#ifdef CFWBIN
static uint8_t setbinarymodecmd[] = "$PGCMD,21,1*6F\r\n";
#endif

void gtgpsTask(void *param)
{
#if defined(FFWNMEA) || defined(CFWNMEA)
  char ch;
  int j;
#endif
  
#ifdef FFWNMEA
//  uart1SendData(sizeof(baudcmd), baudcmd);
//  vTaskDelay(500);
//  uart1Init(115200);
//  vTaskDelay(2000);

//  uart1SendData(sizeof(setnmeatypescmd), setnmeatypescmd);

//  uart1SendData(sizeof(updaterate), updaterate);
//  uart1SendData(sizeof(updaterate2), updaterate2);

//  uart1SendData(sizeof(updaterate3), updaterate3);
//  uart1SendData(sizeof(updaterate4), updaterate4);
#endif

#if defined(CFWNMEA) || defined(CFWBIN)
  uart1Init(115200);
  vTaskDelay(2000);
#endif
#if defined(FFWNMEA) || defined(CFWNMEA)
  uart1SendData(sizeof(setnmeatypescmd), setnmeatypescmd);
#endif
  
#ifdef CFWBIN
  uart1SendData(sizeof(setbinarymodecmd), setbinarymodecmd);
#endif  
  
#if defined(FFWNMEA) || defined(CFWNMEA)
  while(1)
  {
    if(!NewFrame)
    {
      uart1Getchar(&ch);
      //consolePutchar(ch);

      if (ch == '$')
      {
        bi = 0;
      }
      else if (ch == '\n')
      {
        buff[bi] = 0; // Terminate with null
        if (verifyChecksum(buff))
        {
          for (j = 0; j < sizeof(parsers)/sizeof(parsers[0]); j++)
          {
            if (strncmp(parsers[j].token, buff, LEN_TOKEN) == 0)
            {
              parsers[j].parser(&buff[LEN_TOKEN]);
            }
          }
          NewFrame = true;
        } 
      }
      else if (bi < MAX_LEN_SENTANCE)
      {
        buff[bi++] = ch;
      }
    }
  } 
}
#elif defined(CFWBIN)
  struct gtbin_message msg;
  char payload[50];

  msg.payload = payload;

  while(1)
  {
    gtgpsReceiveBin(&msg, 50);

    m.temp32 = msg.nav_onesentence->utc;
    gtgpsSwitch(&m.temp32, &m.utc, 4);
    gps_fixTime = m.utc;
    m.date = msg.nav_onesentence->date;
    m.temp32 = msg.nav_onesentence->lat;
    gtgpsSwitch(&m.temp32, &m.latitude, 4);
    m.N_S = msg.nav_onesentence->N_S;
    if(m.N_S == 2) m.latitude *= -10; else m.latitude *= 10;
    m.temp32 = msg.nav_onesentence->lon;
    gtgpsSwitch(&m.temp32, &m.longitude, 4);
    m.E_W = msg.nav_onesentence->E_W;
    if(m.E_W == 2) m.longitude *= -10; else m.longitude *= 10;
    b.fix = msg.nav_onesentence->fixquality;
    if(b.fix == 1) b.fix = 0;  
    m.fixtype = msg.nav_onesentence->fixmode;
    gps_fixStatus = m.fixtype;
    if(m.fixtype > 1) m.fixtype = 1; else m.fixtype = 0;   
    m.temp32 = msg.nav_onesentence->altitude;
    gtgpsSwitch(&m.temp32, &m.alt, 4); 
    gps_hMSL = m.alt / 100.0;
    m.temp32 = msg.nav_onesentence->cog;
    gtgpsSwitch(&m.temp32, &m.cog, 4); 
    m.temp32 = msg.nav_onesentence->smg;
    gtgpsSwitch(&m.temp32, &m.smg, 4);
    m.sats = msg.nav_onesentence->sats;
    m.nsat = msg.nav_onesentence->nsat;
    m.temp16 = msg.nav_onesentence->hdop;
    gtgpsSwitch(&m.temp16, &m.hdop, 2);
    gps_hAcc = m.hdop / 100.0; 
    m.temp16 = msg.nav_onesentence->epe;
    gtgpsSwitch(&m.temp16, &b.pdop, 2);
    gps_pAcc = b.pdop / 100.0;
    if (ref_chksum == 0) ref_chksum = 0;  //dummy until chksum is verified
    if(m.fixtype == 1)
    {
      if(b.fix == 2)
      {
        gps_fixType = 6;
      }
      else  if(b.fix == 3)
      {
        gps_fixType = 7;
      }
    }
    else
    {
      gps_fixType = b.fix;
    }
    saveFrameData();
    NewFrame = false; 
  } 
}  
#endif
 
static void gtgpsInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Enabling reading from GlobalTop GPS\n");
  uart1Init(9600);

  xTaskCreate(gtgpsTask, "GTGPS",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
  D2R = (float) M_PI/180.0;
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
  .vid = 0xBC,             //prototype .vid value selection 
  .pid = 0x07,             //prototype .pid value selection 
  .name = "bcGTGPS",

  .usedPeriph = 0,
  .usedGpio = DECK_USING_PA2 | DECK_USING_PA3,

  .init = gtgpsInit,
  .test = gtgpsTest,
};

#ifdef ENABLE_GPS_DECK
DECK_DRIVER(gtgps_deck);
#endif

LOG_GROUP_START(gps)
LOG_ADD(LOG_INT32, lat, &m.latitude)         //degrees * 1e+7
LOG_ADD(LOG_INT32, lon, &m.longitude)        //degrees * 1e+7
LOG_ADD(LOG_FLOAT, hMSL, &gps_hMSL)          //meters
LOG_ADD(LOG_FLOAT, hAcc, &gps_hAcc)          //meters
LOG_ADD(LOG_UINT8, nsat, &m.nsat)
LOG_ADD(LOG_INT32, fix, &b.fix)              //0-no DGPS, 1-DGPS
LOG_ADD(LOG_FLOAT, pAcc, &gps_pAcc)          //meters
LOG_ADD(LOG_UINT8, fixType, &gps_fixType)    //NMEA 0183 Protocol (0,2,3,6,7)
LOG_ADD(LOG_UINT8, fixStatus, &gps_fixStatus)//0-no, 1-fix, 2-dgps 
LOG_ADD(LOG_UINT32, fixtime, &gps_fixTime)
LOG_GROUP_STOP(gps)

LOG_GROUP_START(gps_dpos)
LOG_ADD(LOG_FLOAT, pos_x, &pos_x)
LOG_ADD(LOG_FLOAT, pos_y, &pos_y)
LOG_ADD(LOG_FLOAT, pos_z, &pos_z)
LOG_ADD(LOG_UINT32, timestamp, &timestamp)
LOG_GROUP_STOP(gps_dpos)
