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
 * expbrd.h - Expansion board handling functions
 */
#ifndef __EXPBRD_H__
#define __EXPBRD_H__

#include <stdint.h>

#define EXPBRD_MAX      5
#define EXPBRD_ID       0xEB
#define EXPBRD_OW_ADDR  0x00

// Definition of Vendor ID
#define EXPBRD_VID_BITCRAZE  0xBC

// Definition of expansion board Product ID
#define EXPBRD_PID_LEDRING  0x01
#define EXPBRD_PID_QI       0x02
#define EXPBRD_PID_ET       0xFF


typedef struct _ExpbrdData
{
  uint8_t header;
  uint8_t usedPins[4];
  uint8_t vid;
  uint8_t pid;
  uint8_t crc;
} __attribute__((packed)) ExpbrdData ;

void expbrdInit();
bool expbrdTest();

#endif //__EXPBRD_H__

