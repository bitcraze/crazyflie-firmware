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

#define MIN_SHORT_PULSE_LEN_TICKS US2TICK(2)
#define MIN_LONG_PULSE_LEN_TICKS  US2TICK(40)
#define MAX_LONG_PULSE_LEN_TICKS  US2TICK(300)

#define SYNC_A_TO_B_TIME_TICKS    US2TICK(400)
#define FRAME_TIME_TICKS          US2TICK(8333)

typedef struct _LhPulseType
{
  uint32_t tsRise; // Timestamp of rising edge
  uint32_t width;  // Width of the pulse in clock ticks
} LhPulseType;

LhPulseType pulses[18] = {
    {175243,  382}, {479637,  9693}, {514749,  7965},  {815882,  368}, {1180949, 7065}, {1216055, 8831},
    {1546318, 458}, {1882115, 6185}, {1917209, 11456}, {2152833, 384}, {2583431, 8844}, {2618527, 5334},
    {2980187, 384}, {3284600, 9691}, {3319677, 6177},  {3620805, 367}, {3985899, 5345}, {4021009, 10582}
};

typedef struct _LhFrame
{
  LhPulseType syncA;
  LhPulseType syncB;
  LhPulseType sweep;
} LhFrame;

enum {PULSE_A, PULSE_B, PULSE_SWEEP} pulseState;
static LhFrame frame;

lhppAnalysePulse(LhPulseType *p)
{
  if (p.width >= MAX_LONG_PULSE_LEN_TICKS)
  {
      // Ignore very long pulses.
  }
  else if (p.width >= MIN_LONG_PULSE_LEN_TICKS)
  { // Long pulse - likely sync pulse
    lhppSynchPulse(p);
  }
  else
  { // Short pulse - likely laser sweep
    frame.sweep.tsRise = p->tsRise;
    frame.sweep.width = p->width;
    pulseState = PULSE_A;

    lhppFrameDecode(p);
  }
}

lhppSynchPulse(LhPulseType *p)
{
  switch(pulseState)
  {
    case PULSE_A:
      frame.syncA.tsRise = p->tsRise;
      frame.syncA.width = p->width;
      pulseState = PULSE_B;
      break;
    case PULSE_B:
      frame.syncB.tsRise = p->tsRise;
      frame.syncB.width = p->width;
      pulseState = PULSE_A;
      break;
    default:
      pulseState = PULSE_A;
      break;
  }
}

void lhppFrameDecode(void)
{
  uint32_t sweepTimeFromA = frame.sweep.tsRise - frame.syncA.tsRise;
  uint32_t sweepTimeFromB = frame.sweep.tsRise - frame.syncB.tsRise;

  //TODO

}

