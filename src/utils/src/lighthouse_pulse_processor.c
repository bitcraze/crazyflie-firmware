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

#include "lighthouse_pulse_processor.h"

#define US2TICK(usec) (84*usec) //  84000000MHz * usec/1000000

#define ANGLE_CENTER_TICKS  US2TICK(4000)
#define CYCLE_PERIOD_TICKS  US2TICK(8333)

#define MIN_SHORT_PULSE_LEN_TICKS US2TICK(2)
#define MIN_LONG_PULSE_LEN_TICKS  US2TICK(40)
#define MAX_LONG_PULSE_LEN_TICKS  US2TICK(300)

#define SYNC_A_TO_B_TIME_TICKS    US2TICK(400)
#define FRAME_TIME_TICKS          US2TICK(8333)

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
  int32_t sweepPulseCentre = lhObj->frame.sweep.tsRise + (lhObj->frame.sweep.width / 2);
  int32_t sweepTimeFromA = sweepPulseCentre - lhObj->frame.syncA.tsRise;
  int32_t sweepTimeFromB = sweepPulseCentre - lhObj->frame.syncB.tsRise;
  int32_t syncToSync = lhObj->frame.syncB.tsRise - lhObj->frame.syncA.tsRise;

  syncInfoA.bits = (lhObj->frame.syncA.width - 4814) / 875;
  syncInfoB.bits = (lhObj->frame.syncB.width - 4814) / 875;

  // Check sync pulses
  if (syncToSync > US2TICK(370) && syncToSync < US2TICK(430))
  {
    // Determine sweep
    if      (syncInfoA.axis == 0 && syncInfoA.skip == 0) { lhObj->angles.x0 = calculateAngle(sweepTimeFromA); }
    else if (syncInfoA.axis == 1 && syncInfoA.skip == 0) { lhObj->angles.y0 = calculateAngle(sweepTimeFromA); }
    else if (syncInfoB.axis == 0 && syncInfoB.skip == 0) { lhObj->angles.x1 = calculateAngle(sweepTimeFromA); }
    else if (syncInfoB.axis == 1 && syncInfoB.skip == 0)
    {
      lhObj->angles.y1 = calculateAngle(sweepTimeFromA);
      anglesCalculated = true;
    }
  }

  return anglesCalculated;
}

static float calculateAngle(int32_t sweepTime)
{
  return ((int32_t)sweepTime - ANGLE_CENTER_TICKS) * (float)M_PI / CYCLE_PERIOD_TICKS;
}
