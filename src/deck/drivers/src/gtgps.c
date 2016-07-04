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

/**
 * gtgps.h is needed for the Position Hold Mode capability
 * and when gtgps.c is called by sensors_stock.c to pick up
 * 3D fix positions
 */ 

#include "gtgps.h"

#if defined(COMPASS_ENABLED)
/**
 * compass.h is needed for the Position Hold Mode capability
 * and when gtgps.c calls compass.c to see if is calibrated
 */  

#include "compass.h"
#endif

#define ENABLE_GPS_DECK

/**
 * gtgps.h is needed for the Position Hold Mode capability
 * and when gtgps.c is called by sensors_stock.c
 */ 

#include "gtgps.h" 

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


static Basic b;
static MeasData m;

static bool isInit;

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
      gps_latHome = m.latitude;    //deg * 1e+7
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
//don't output positions until compass is calibrated
    
#if defined(COMPASS_ENABLED)    
    else if (compassCaled())
    {
      timestamp = 1;  
      pos_x = (m.latitude - gps_latHome) * gps_scaleLat;   //meters
      pos_y = -(m.longitude - gps_lonHome) * gps_scaleLon; //meters
      pos_z = -gps_hMSL;                                   //meters
    }
#endif    
  }  
  else
  {
    timestamp = 0;     
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_z = 0.0f;
  }

//This code can be found in position_controller_pid.c
//  In converting position (desired-measured) to roll/pitch
//  D2R = (float) M_PI/180.0f
//  cos = cosf(yawgeo * D2R)
//  sin = sinf(yawgeo * D2R)
//  pitch = - position.x * cos - position.y * sin
//  roll  = - position.y * cos + position.x * sin 
  
}

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

static uint8_t setbinarymodecmd[] = "$PGCMD,21,1*6F\r\n";

void gtgpsTask(void *param)
{
  uart1Init(115200);
  vTaskDelay(2000);

  uart1SendData(sizeof(setbinarymodecmd), setbinarymodecmd);
  
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
LOG_ADD(LOG_UINT32, timestamp, &timestamp)
LOG_ADD(LOG_FLOAT, pos_x, &pos_x)
LOG_ADD(LOG_FLOAT, pos_y, &pos_y)
LOG_ADD(LOG_FLOAT, pos_z, &pos_z)
LOG_GROUP_STOP(gps_dpos)
