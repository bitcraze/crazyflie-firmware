/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * compass.c - compassController - Crazyflie yaw angle relative to True North
 */
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
 
#include "compass.h"
#include "param.h"
#include "log.h"
#include "num.h"
 
#include "configblock.h" 

#define COMPASS_RATE RATE_5_HZ
#define COMPASS_CAL_RATE RATE_100_HZ
 
//if CAL_BUTTONS defined xbox360 buttons are used to step through calibration
//if not defined then manual calibration uses xyz min and max outputs collected
//during intervals shown by log group magcalOn & calRequired states on plotter
//tab and data is collected using cfclient logging function tools
//xyz scale factors and offsets are computed from min max data and then hard
//coded into this module
#define CAL_BUTTONS

//state->attitude.roll and state->attitude.pitch are required inputs
//for platform tilt compensation of the magnetometer sensor data

static float magneticdeclination = 11.97f; //degrees for lat 33.76843 lon -117.49194 

static uint8_t calRequired;
static bool calSeqButton;
static bool calHorzButton;
static bool calVertButton;

static bool cb_write;
static bool cb_read;

static bool magcalOn = false;
static bool calAdvance;
static int16_t  magx;
static int16_t  magy;
static int16_t  magz;
static int16_t  xmax;
static int16_t  xmin;
static int16_t  ymax;
static int16_t  ymin;
static int16_t  zmax;
static int16_t  zmin;
static int32_t  f_magx = 0;
static int32_t  f_magy = 0;
static int32_t  f_magz = 0;
static uint32_t lastOnOffTime  = 0;
static bool lastTestTime;
static float    D2R;
static float    RawMag = 666.7f; 

// xoff = -(xmax+xmin)/2, yoff = -(ymax+ymin)/2, zoff = -(zmax+zmin)/2
// xsf = -1.0, ysf = (xmax+xoff)/(ymax+yoff), zsf = (xmax+xoff)/(zmax+zoff)
// xoff, xsf, yoff, ysf uses 0 deg tilt rotation data
// zoff, zsf uses 90 deg tilt rotation data

// sample CF2 without gps expansion board
//static float  xsf  =   -1.00f; // xmax =   798 (horz)
//static float  xoff =  -677.0f; // xmin =   556 (horz)
//static float  ysf  =   0.992f; // ymax = -1334 (horz)
//static float  yoff =  1456.0f; // ymin = -1578 (horz)
//static float  zsf  =   0.858f; // zmax =  -136 (vert)
//static float  zoff =   277.0f; // zmin =  -418 (vert)

// sample CF2 with gps expansion board
static float  xsf  =  -1.000f; // xmax =       (horz)
static float  xoff = -1081.5f; // xmin =       (horz)
static float  ysf  =   1.104f; // ymax =       (horz)
static float  yoff =   379.5f; // ymin =       (horz)
static float  zsf  =   0.863f; // zmax =       (vert)
static float  zoff =   644.0f; // zmin =       (vert)

static float  theta;
static float  phi;
static float  sinr;
static float  cosr;
static float  sinp;
static float  cosp;
static float  xm;
static float  ym;
static float  fmagx;
static float  fmagy;
static float  fmagz;
static float  yawangle;
static float  yawBias;
static float  yawBias0;
static float  yawBias1;
static float  yawGyroBias = 0.0f;
static float  yawMagBias = 0.0f;
static uint16_t yawBiasCtr = 2000;
static uint16_t updateBias = 4000;
static bool   applyBias = false;

static bool isInit;

void compassInit(void)
{
  if(isInit)
    return;
  isInit = true;
  D2R = (float) M_PI/180.0;
  calRequired = 0;
  lastOnOffTime = xTaskGetTickCount (); // used in magcal
  lastTestTime = xTaskGetTickCount ();
//After flashing new firmware, the default is xoff=yoff=zoff=0, xsf=-1,ysf=zsf=1  
#ifdef CAL_BUTTONS  
  cb_read = configblockGetCalibMag(&xoff, &xsf, &yoff, &ysf, &zoff, &zsf);
#endif
}

bool compassTest(void)
{
 return true;
}

static bool cal_Timeout(const uint32_t timeout)
{
  if ((xTaskGetTickCount() - lastOnOffTime) > timeout) return 1; else return 0;
}

static void AdjAngle(float* angle)
{
  while (*angle > 180.0f)
    *angle -= 360.0f;
  while (*angle < -180.0f)
    *angle += 360.0f;
}

bool compassCalibration(const uint32_t tick)
{
  // Begin 100Hz updates
  if (RATE_DO_EXECUTE(COMPASS_CAL_RATE, tick))
  {
    if (calRequired == 1) {
      magcalOn = false;
      xmax = -10000;
      xmin =  10000;
      ymax = -10000;
      ymin =  10000;
      zmax = -10000;
      zmin =  10000;
    }
    else if (calRequired == 2) {
      magcalOn = true;
      if (f_magx > xmax) xmax = f_magx;
      if (f_magy > ymax) ymax = f_magy;          
      if (f_magx < xmin) xmin = f_magx;
      if (f_magy < ymin) ymin = f_magy;         
    }
    else if (calRequired == 3) {
      magcalOn = false;
    }
    else if (calRequired == 4) {
      magcalOn = true;
      if (f_magz > zmax) zmax = f_magz;
      if (f_magz < zmin) zmin = f_magz;
    }
    else if (calRequired == 5) {
      magcalOn = false;
      xoff = xmax+xmin;
      xoff = -(xoff)/2.0f;
      yoff = ymax+ymin;
      yoff = -(yoff)/2.0f;
      zoff = zmax+zmin;
      zoff = -(zoff)/2.0f;
      xsf = -1.0f;
      ysf = (xmax+xoff)/(ymax+yoff);
      zsf = (xmax+xoff)/(zmax+zoff);
      cb_write = configblockSetCalibMag(xoff, xsf, yoff, ysf, zoff, zsf);
      calRequired = 0;
      applyBias = false;
      yawBiasCtr = 2000;
      yawMagBias = 0.0f;
      updateBias = 4000;
    }
    else {
      magcalOn = false;
      calRequired = 0;
    }
  }
  return true;
}

void compassController(state_t *state, const sensorData_t *sensorData, const uint32_t tick)
{
#ifdef CAL_BUTTONS    
  if (calSeqButton) {
    if ((calRequired % 2) == 0) calRequired++;
  }
  else if (calHorzButton && calRequired == 1) {
    calRequired++;
  }  
  else if (calVertButton && calRequired == 3) {      
    calRequired++;    
  }
#endif  
  // Begin 100Hz updates  
  if (RATE_DO_EXECUTE(COMPASS_CAL_RATE, tick))
  {
    //Magnetometer axis data is used to calculate geodetic cf2 yaw angle
    // which is needed to apply gps lat && lon corrections to cf2 roll and pitch.

    //Manual compensation for magnetic inclination and attitude compensation has been implemented.   
    //In addition the magnetometer requires calibration and repeated again
    // when changing cf2 platform hardware configurations or when there is a change
    // in the local magnetic disturbance nearby that affect the magnetometer.
    //Calibration data is gathered while
    // rotating the cf2 360 degrees or more in the horizontal plane during a ~30 second
    // period when the magcalOn parameter is true and
    // rotating the cf2 360 degrees or more in a 90 deg tilt angle during a second ~30
    // second period when magcalOn is again true.
    //Read the x, y and z (min and max) values during each of the 30 second periods when
    // magcalOn is false
    //cfclient View Tabs Log Blocks utility can be used to capture min && max values while
    // watching the plotter tab of magcalOn switching between 1 (cal period) and 0 (read
    // cal values period)

    //Note: This assumes mag +X axis is aft, +y axis is larboard, +z axia is up
    magx = (sensorData->mag.x * RawMag);
    magy = (sensorData->mag.y * RawMag);
    magz = (sensorData->mag.z * RawMag); 
     
    f_magx = (f_magx * 3 + magx) >> 2;
    f_magy = (f_magy * 3 + magy) >> 2;
    f_magz = (f_magz * 3 + magz) >> 2;

    calAdvance = false;
    if ((calRequired == 2) || (calRequired == 4)) { 
      if (cal_Timeout(30000)) {  //30s to collect horz/vert min & max x, y, z values
        calAdvance = true;
        lastOnOffTime = xTaskGetTickCount ();
      }
    }
    else if (calRequired < 5) {
      if (cal_Timeout(15000)) { //15 seconds between collection periods
        calAdvance = true;
        lastOnOffTime = xTaskGetTickCount ();
      }
    }    
#ifndef CAL_BUTTONS
    if (calAdvance) {
      calRequired++;
      calAdvance = false;
    }
    if (calRequired > 4) calRequired = 0;
#endif
             
    compassCalibration(tick);

  } // end 100Hz updates
  
      
  // Begin 5HZ Updates
  if (RATE_DO_EXECUTE(COMPASS_RATE, tick))
  {  
    fmagx = (f_magx + xoff) * xsf;
    fmagy = (f_magy + yoff) * ysf;
    fmagz = (f_magz + zoff) * zsf;

    theta = -state->attitude.roll; //convert to magnetometer axis
    //Limit roll to +/- 90 degrees
    if (theta > 90.0f)
    {
      theta = 180.0f - theta;
    }
    else if (theta < -90.0f)
    {
      theta = -180.0f - theta;
    }
    theta *= D2R;
    phi  = state->attitude.pitch * D2R; //convert to magnetometer axis
    sinr = sinf(theta);
    cosr = cosf(theta);
    sinp = sinf(phi);
    cosp = cosf(phi);
    //Limit pitch to +/- 90 degrees
    if (cosp < 0.0f) cosp = -cosp;

    //CF2 Yaw +/- 180 degrees, zero at power on, ccw positive rotation
    //yawangle +/- 180 degrees, ccw positive rotation, zero at true north
    //Note roll from eulerActuals (gyro) is negated above

    xm = fmagx*cosp + fmagy*sinr*sinp + fmagz*cosr*sinp;
    ym = fmagy*cosr - fmagz*sinr;

    yawangle = atan2f(ym,xm) / D2R  + magneticdeclination;
    AdjAngle(&yawangle);                 //+ angle cw
    yawangle = -yawangle;                //+ angle ccw    
    state->attitude.yawgeo = yawangle;   //+ angle ccw
//  In converting position (desired-measured) to roll/pitch
//  D2R = (float) M_PI/180.0f
//  cos = cosf(yawgeo * D2R)
//  sin = sinf(yawgeo * D2R)
//  pitch = - position.x * cos - position.y * sin
//  roll  = - position.y * cos + position.x * sin
    
  } //End 5 Hz updates
}

void compassGyroBias(float* yaw)
{
//Compute bias to eliminate drift in gyro based euler yaw actual and
// convert this actual to heading relative to geographic or true north
    yawGyroBias = (yawGyroBias * 99.0f + abs(*yaw)) / 100.0f;
    if (*yaw < 0.0f) yawBias = -yawGyroBias; else yawBias = yawGyroBias;
    yawMagBias = (yawMagBias * 99.0f + abs(yawangle)) / 100.0f;
    if (yawangle < 0.0f) yawBias += yawMagBias; else yawBias -= yawMagBias;
    if (applyBias)
    {
      if (!updateBias--)
      {
        updateBias = 4000;
        yawBias1 = yawBias - yawBias0;
      }
      *yaw = *yaw - yawBias1;
    }
    else
    {
      if ((xoff + yoff + zoff) != 0.0f)  //See if compassCal has been performed
      {
        if (!yawBiasCtr--)
        {
          applyBias = true;
          yawBias0 = yawBias;
          yawBias1 = 0.0f;
        }
      }
    }
}

PARAM_GROUP_START(compass)
PARAM_ADD(PARAM_UINT8, calSeqButton, &calSeqButton)
PARAM_ADD(PARAM_UINT8, calHorzButton, &calHorzButton)
PARAM_ADD(PARAM_UINT8, calVertButton, &calVertButton)
PARAM_GROUP_STOP(compass)

LOG_GROUP_START(compassFac)
LOG_ADD(LOG_FLOAT, xoff, &xoff)
LOG_ADD(LOG_FLOAT, yoff, &yoff)
LOG_ADD(LOG_FLOAT, zoff, &zoff)
LOG_ADD(LOG_FLOAT, xsf, &xsf)
LOG_ADD(LOG_FLOAT, ysf, &ysf)
LOG_ADD(LOG_FLOAT, zsf, &zsf)
LOG_ADD(LOG_UINT8, cb_write, &cb_write)
LOG_ADD(LOG_UINT8, cb_read, &cb_read)
LOG_GROUP_STOP(compassFac)

LOG_GROUP_START(compassCal)
LOG_ADD(LOG_UINT8,  magcalOn, &magcalOn)
LOG_ADD(LOG_UINT8,  calRequired, &calRequired)
LOG_ADD(LOG_INT16, xmax, &xmax)
LOG_ADD(LOG_INT16, xmin, &xmin)
LOG_ADD(LOG_INT16, ymax, &ymax)
LOG_ADD(LOG_INT16, ymin, &ymin)
LOG_ADD(LOG_INT16, zmax, &zmax)
LOG_ADD(LOG_INT16, zmin, &zmin)
LOG_GROUP_STOP(compassCal)

LOG_GROUP_START(compass)
LOG_ADD(LOG_FLOAT,  yawgeo, &yawangle)
LOG_ADD(LOG_FLOAT,  yawBias0, &yawBias0)
LOG_ADD(LOG_FLOAT,  yawBias, &yawBias)
LOG_GROUP_STOP(compass)
