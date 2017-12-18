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

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#define US2TICK(usec) (84*usec) //  84000000MHz * usec/1000000

#define ANGLE_CENTER_TICKS  US2TICK(4000)
#define CYCLE_PERIOD_TICKS  US2TICK(8333)

#define MIN_SHORT_PULSE_LEN_TICKS US2TICK(2)
#define MIN_LONG_PULSE_LEN_TICKS  US2TICK(40)
#define MAX_LONG_PULSE_LEN_TICKS  US2TICK(300)

#define SYNC_A_TO_B_TIME_TICKS    US2TICK(400)
#define FRAME_TIME_TICKS          US2TICK(8333)

#define SYNC_BITS_BASE_TICKS      4814
#define SYNC_BITS_DEVIDER         875

enum {PULSE_A, PULSE_B, PULSE_SWEEP} pulseState;

static void lhppSynchPulse(LhObj* lhObj, LhPulseType *p);
static bool lhppFrameDecode(LhObj* lhObj);
static float calculateAngle(int32_t sweepTime);

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
    pulseState = PULSE_A;

    anglesCalculated = lhppFrameDecode(lhObj);
  }

  return anglesCalculated;
}

static void lhppSynchPulse(LhObj* lhObj, LhPulseType *p)
{
  switch(pulseState)
  {
    case PULSE_A:
      lhObj->frame.syncA.tsRise = p->tsRise;
      lhObj->frame.syncA.width = p->width;
      pulseState = PULSE_B;
      break;
    case PULSE_B:
      lhObj->frame.syncB.tsRise = p->tsRise;
      lhObj->frame.syncB.width = p->width;
      pulseState = PULSE_A;
      break;
    default:
      pulseState = PULSE_A;
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
  int32_t syncToSync = lhObj->frame.syncB.tsRise - lhObj->frame.syncA.tsRise;

  syncInfoA.bits = (lhObj->frame.syncA.width - SYNC_BITS_BASE_TICKS) / SYNC_BITS_DEVIDER;
  syncInfoB.bits = (lhObj->frame.syncB.width - SYNC_BITS_BASE_TICKS) / SYNC_BITS_DEVIDER;

  // Check that booth sync pulses exist and are correctly spaced
  if (syncToSync > US2TICK(370) && syncToSync < US2TICK(430))
  {
    // Determine sweep and calculate angles
    if      (syncInfoA.axis == 0 && syncInfoA.skip == 0)
    {
      lhObj->workAngles.x0 = calculateAngle(sweepTimeFromA);
    }
    else if (syncInfoA.axis == 1 && syncInfoA.skip == 0)
    {
      lhObj->workAngles.y0 = calculateAngle(sweepTimeFromA);
    }
    else if (syncInfoB.axis == 0 && syncInfoB.skip == 0)
    {
      lhObj->workAngles.x1 = calculateAngle(sweepTimeFromA);
    }
    else if (syncInfoB.axis == 1 && syncInfoB.skip == 0)
    {
      lhObj->workAngles.y1 = calculateAngle(sweepTimeFromA);
    }
  }

  if ((lhObj->workAngles.x0 != 0.0f) &&
      (lhObj->workAngles.y0 != 0.0f) &&
      (lhObj->workAngles.x1 != 0.0f) &&
      (lhObj->workAngles.y1 != 0.0f))
    {
      memcpy(&lhObj->workAngles, &lhObj->angles, sizeof(LhAngles));
      memset(&lhObj->workAngles, 0,  sizeof(LhAngles));
      anglesCalculated = true;
    }

  return anglesCalculated;
}

static float calculateAngle(int32_t sweepTime)
{
  return ((int32_t)sweepTime - ANGLE_CENTER_TICKS) * (float)M_PI / CYCLE_PERIOD_TICKS;
}
