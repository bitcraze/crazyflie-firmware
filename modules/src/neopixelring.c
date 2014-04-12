/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * neopixelring.c: NeoPixel Ring 16 Leds effects/driver
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "ws2812.h"
#include "worker.h"

void neopixelringWorker(void * data);

static xTimerHandle timer;

static void neopixelringTimer(xTimerHandle timer)
{
  workerSchedule(neopixelringWorker, NULL);
}

void neopixelringInit(void)
{
  ws2812Init();
  
  timer = xTimerCreate( (const signed char *)"ringTimer", M2T(30), 
                                     pdTRUE, NULL, neopixelringTimer );
  xTimerStart(timer, 100);
}

#define RED {0x10, 0x00, 0x00}
#define GREEN {0x00, 0x10, 0x00}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0x10, 0x10, 0x10}
#define BLACK {0x00, 0x00, 0x00}

static uint8_t color[][3] = {{40, 40, 40}, {32, 32, 32}, {16,16,16}, {8,8,8},
                             {4,4,4}, {2,2,2}, {1,1,1}, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                            };
#define COPY_COLOR(dest, orig) dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2];

void neopixelringWorker(void * data)
{
  int i;
  static uint8_t temp[3];

  ws2812Send(color, 16);

  COPY_COLOR(temp, color[0]);
  for (i=0; i<15; i++) {
    COPY_COLOR(color[i], color[i+1]);
  }
  COPY_COLOR(color[15], temp);
}
 
