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
 * lighthouse_pulse_processor.h - Handling lighthouse pulses
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "lighthouse_pulse_processor.h"
#include "log.h"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#define FRAME_TIME_US   8333

#define US2TICK(usec) (84*usec) //  84000000MHz * usec/1000000

#define ANGLE_CENTER_TICKS  US2TICK(4000)
#define CYCLE_PERIOD_TICKS  US2TICK(FRAME_TIME_US)

#define MIN_SHORT_PULSE_LEN_TICKS US2TICK(2)
#define MIN_LONG_PULSE_LEN_TICKS  US2TICK(40)
#define MAX_LONG_PULSE_LEN_TICKS  US2TICK(300)

#define SYNC_A_TO_B_TIME_TICKS    US2TICK(400)
#define FRAME_TIME_TICKS          US2TICK(FRAME_TIME_US)

#define SYNC_BITS_BASE_TICKS      4814
#define SYNC_BITS_DEVIDER         875

static void lhppSynchPulse(LhObj* lhObj, LhPulseType *p);
static bool lhppFrameDecode(LhObj* lhObj);
static float calculateAngle(int32_t sweepTime);
static float timeAtoA;
static float timeAtoB;

bool lhppAnalysePulse(LhObj* lhObj, LhPulseType *p)
{
  bool anglesCalculated = false;

  if (p->width >= MAX_LONG_PULSE_LEN_TICKS)
  {
      // Ignore very long pulses.
  }
  else if (p->width >= MIN_LONG_PULSE_LEN_TICKS)
  { // Long pulse - likely sync pulse
    lhppSynchPulse(lhObj, p);
  }
  else
  { // Short pulse - likely laser sweep
    lhObj->frame.sweep.tsRise = p->tsRise;
    lhObj->frame.sweep.width = p->width;
    lhObj->state = PULSE_A;

    anglesCalculated = lhppFrameDecode(lhObj);
  }

  return anglesCalculated;
}

static void lhppSynchPulse(LhObj* lhObj, LhPulseType *p)
{
  switch(lhObj->state)
  {
    case PULSE_A:
      timeAtoA = (p->tsRise - lhObj->frame.syncA.tsRise) / 84.0f;
      lhObj->frame.syncA.tsRise = p->tsRise;
      lhObj->frame.syncA.width = p->width;
      lhObj->state = PULSE_B;
      break;
    case PULSE_B:
      lhObj->frame.syncB.tsRise = p->tsRise;
      lhObj->frame.syncB.width = p->width;
      lhObj->state = PULSE_A;
      break;
    default:
      lhObj->state = PULSE_A;
      break;
  }
}

static bool lhppFrameDecode(LhObj* lhObj)
{
  bool anglesCalculated = false;
  SyncInfo syncInfoA;
  SyncInfo syncInfoB;
  int32_t sweepPulseCenter = lhObj->frame.sweep.tsRise + (lhObj->frame.sweep.width / 2);
  int32_t sweepTimeFromA = sweepPulseCenter - lhObj->frame.syncA.tsRise;
  int32_t sweepTimeFromB = sweepPulseCenter - lhObj->frame.syncB.tsRise;
  int32_t syncToSync = lhObj->frame.syncB.tsRise - lhObj->frame.syncA.tsRise;

  timeAtoB  = (lhObj->frame.syncB.tsRise - lhObj->frame.syncA.tsRise) /84.0f;
  syncInfoA.bits = (lhObj->frame.syncA.width - SYNC_BITS_BASE_TICKS) / SYNC_BITS_DEVIDER;
  syncInfoB.bits = (lhObj->frame.syncB.width - SYNC_BITS_BASE_TICKS) / SYNC_BITS_DEVIDER;
//  printf("SyncA:%d SyncB:%d\n", syncInfoA.bits, syncInfoB.bits);

  // Check that booth sync pulses exist and are correctly spaced
  if (syncToSync > US2TICK(370) && syncToSync < US2TICK(430))
  {
    // Determine sweep and calculate angles
    if      (syncInfoA.axis == 0 && syncInfoA.skip == 0)
    {
      lhObj->angles.x0 = calculateAngle(sweepTimeFromA);
      lhObj->isCalc.x0 = true;
    }
    else if (syncInfoA.axis == 1 && syncInfoA.skip == 0)
    {
      lhObj->angles.y0 = calculateAngle(sweepTimeFromA);
      lhObj->isCalc.y0 = true;
    }
    else if (syncInfoB.axis == 0 && syncInfoB.skip == 0)
    {
      lhObj->angles.x1 = calculateAngle(sweepTimeFromB);
      lhObj->isCalc.x1 = true;
    }
    else if (syncInfoB.axis == 1 && syncInfoB.skip == 0)
    {
      lhObj->angles.y1 = calculateAngle(sweepTimeFromB);
      lhObj->isCalc.y1 = true;
    }
  }

  if ((lhObj->isCalc.x0) &&
      (lhObj->isCalc.y0) &&
      (lhObj->isCalc.x1) &&
      (lhObj->isCalc.y1))
    {
//      printf("x0:%f, y0:%f, x1:%f, y1:%f\n", lhObj->angles.x0, lhObj->angles.y0, lhObj->angles.x1, lhObj->angles.y1);
      memset(&lhObj->isCalc, 0,  sizeof(LhAnglesCalc));
      anglesCalculated = true;
    }

  return anglesCalculated;
}

static float calculateAngle(int32_t sweepTime)
{
  return ((int32_t)sweepTime - ANGLE_CENTER_TICKS) * (float)M_PI / CYCLE_PERIOD_TICKS;
}

LOG_GROUP_START(lhpp)
LOG_ADD(LOG_FLOAT, timeAtoA, &timeAtoA)
LOG_ADD(LOG_FLOAT, timeAtoB, &timeAtoB)
LOG_GROUP_STOP(lhpp)

