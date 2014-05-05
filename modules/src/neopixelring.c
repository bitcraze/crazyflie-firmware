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

#include "neopixelring.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "ws2812.h"
#include "worker.h"
#include "param.h"
#include "pm.h"
#include "log.h"


/*
 * To add a new effect just add it as a static function with the prototype
 * void effect(uint8_t buffer[][3], bool reset)
 *
 * Then add it to the effectsFct[] list bellow. It will automatically be
 * activable using the ring.effect parameter.
 *
 * The ring color needs to be written in the buffer argument. The buffer is not
 * modified in memory as long as reset is not 'true', see the spin effects for
 * and example.
 *
 * The log subsystem can be used to get the value of any log variable of the
 * system. See tiltEffect for an example.
 */


/**************** Some useful macros ***************/

#define RED {0x10, 0x00, 0x00}
#define GREEN {0x00, 0x10, 0x00}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0xff, 0xff, 0xff}
#define BLACK {0x00, 0x00, 0x00}

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define COPY_COLOR(dest, orig) dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2]
#define LIMIT(a) ((a>255)?255:(a<0)?0:a)
#define SIGN(a) ((a>=0)?1:-1)

/**************** Black (LEDs OFF) ***************/

static void blackEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  
  if (reset)
  {
    for (i=0; i<16; i++) {
      buffer[i][0] = 0;
      buffer[i][1] = 0;
      buffer[i][2] = 0;
    }
  }
}

/**************** White spin ***************/

static const uint8_t whiteRing[][3] = {{40, 40, 40}, {32, 32, 32}, {16,16,16}, {8,8,8},
                                       {4,4,4}, {2,2,2}, {1,1,1}, BLACK,
                                       BLACK, BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK, BLACK,
                                      };

static void whiteSpinEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];
  
  if (reset)
  {
    for (i=0; i<16; i++) {
      COPY_COLOR(buffer[i], whiteRing[i]);
    }
  }

  COPY_COLOR(temp, buffer[0]);
  for (i=0; i<15; i++) {
    COPY_COLOR(buffer[i], buffer[i+1]);
  }
  COPY_COLOR(buffer[15], temp);
}

/**************** Color spin ***************/

static const uint8_t colorRing[][3] = {{0,0,32}, {0,0,16}, {0,0,8}, {0,0,4},
                                       {0,0,2}, {0,0,1}, {16,16,16}, {8,8,8},
                                       {4,4,4},{2,2,2},{32,0,0},{16,0,0},
                                       {8,0,0}, {4,0,0}, {2,0,0}, {1,0,0},
                                      };

static void colorSpinEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];
  
  if (reset)
  {
    for (i=0; i<16; i++) {
      COPY_COLOR(buffer[i], colorRing[i]);
    }
  }

  COPY_COLOR(temp, buffer[0]);
  for (i=0; i<15; i++) {
    COPY_COLOR(buffer[i], buffer[i+1]);
  }
  COPY_COLOR(buffer[15], temp);
}

/**************** Dynamic tilt effect ***************/

static void tiltEffect(uint8_t buffer[][3], bool reset)
{
  static int pitchid, rollid, thrust=-1;
  
  if (thrust<0) {
    //Init
    pitchid = logGetVarId("stabilizer", "pitch");
    rollid = logGetVarId("stabilizer", "roll");
    thrust = logGetVarId("stabilizer", "thrust");
  } else {
    const int led_middle = 10;
    float pitch = -1*logGetFloat(pitchid);
    float roll  = -1*logGetFloat(rollid);
    
    pitch = (pitch>20)?20:(pitch<-20)?-20:pitch;
    roll = (roll>20)?20:(roll<-20)?-20:roll;
      
    pitch=SIGN(pitch)*pitch*pitch;
    roll*=SIGN(roll)*roll;
    
    buffer[0][0] = LIMIT(led_middle + pitch);
    buffer[1][0] = LIMIT(led_middle + pitch);
    buffer[2][0] = LIMIT(led_middle + pitch);
    buffer[3][0] = LIMIT(led_middle + pitch);
    
    buffer[4][2] = LIMIT(led_middle - roll);
    buffer[5][2] = LIMIT(led_middle - roll);
    buffer[6][2] = LIMIT(led_middle - roll);
    buffer[7][2] = LIMIT(led_middle - roll);
    
    buffer[8][0] = LIMIT(led_middle - pitch);
    buffer[9][0] = LIMIT(led_middle - pitch);
    buffer[10][0] = LIMIT(led_middle - pitch);
    buffer[11][0] = LIMIT(led_middle - pitch);

    buffer[12][2] = LIMIT(led_middle + roll);
    buffer[13][2] = LIMIT(led_middle + roll);
    buffer[14][2] = LIMIT(led_middle + roll);
    buffer[15][2] = LIMIT(led_middle + roll);
  }
}

/**************** Effect list ***************/

NeopixelRingEffect effectsFct[] = {blackEffect,
                                   whiteSpinEffect, 
                                   colorSpinEffect, 
                                   tiltEffect,
                                  }; //TODO Add more


/********** Ring init and switching **********/

static xTimerHandle timer;

static uint32_t effect;
static uint32_t neffect;

static uint8_t black[][3] = {BLACK, BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                            };

void neopixelringWorker(void * data)
{
  static int current_effect = 0;
  static uint8_t buffer[16][3];
  bool reset = true;
  
  if (!pmIsDischarging() || (effect > neffect)) {
    ws2812Send(black, 16);
    return;
  }
  
  if (current_effect != effect) {
    reset = true;
  } else {
    reset = false;
  }
  current_effect = effect;
  
  effectsFct[current_effect](buffer, reset);
  ws2812Send(buffer, 16);
}

static void neopixelringTimer(xTimerHandle timer)
{
  workerSchedule(neopixelringWorker, NULL);
}

void neopixelringInit(void)
{
  ws2812Init();
  
  neffect = sizeof(effectsFct)/sizeof(effectsFct[0])-1;
  
  timer = xTimerCreate( (const signed char *)"ringTimer", M2T(40), 
                                     pdTRUE, NULL, neopixelringTimer );
  xTimerStart(timer, 100);
}

PARAM_GROUP_START(ring)
PARAM_ADD(PARAM_UINT32, effect, &effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)

 
