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
 * configblock.h - Simple static implementation of the config block
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef __LIGHTHOUSE_PULSE_PROCESSOR_H__
#define __LIGHTHOUSE_PULSE_PROCESSOR_H__

typedef struct _LhPulseType
{
  uint32_t tsRise; // Timestamp of rising edge
  uint32_t width;  // Width of the pulse in clock ticks
} LhPulseType;

typedef struct
{
  union {
    uint8_t bits;
    struct {
      uint8_t axis:1;
      uint8_t data:1;
      uint8_t skip:1;
    };
  };
} SyncInfo;

typedef struct _LhFrame
{
  LhPulseType syncA;
  LhPulseType syncB;
  LhPulseType sweep;
} LhFrame;

typedef struct _LhAngle
{
  float x0;
  float y0;
  float x1;
  float y1;
} LhAngles;

typedef struct _LhAnglesCalc
{
  bool x0;
  bool y0;
  bool x1;
  bool y1;
} LhAnglesCalc;

typedef struct _LhObj
{
  LhFrame       frame;
  LhAngles      angles;
  LhAnglesCalc  isCalc;
} LhObj;

bool lhppAnalysePulse(LhObj* lhObj, LhPulseType *p);

#endif //__LIGHTHOUSE_PULSE_PROCESSOR_H__
