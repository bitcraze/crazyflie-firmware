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
 * neopixelring.c: NeoPixel Ring 12 Leds effects/driver
 */

#include "neopixelring.h"

#include <stdint.h>

#include "stm32fxxx.h"

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
 * activated using the ring.effect parameter.
 *
 * The ring color needs to be written in the buffer argument. The buffer is not
 * modified in memory as long as reset is not 'true', see the spin effects for
 * and example.
 *
 * The log subsystem can be used to get the value of any log variable of the
 * system. See tiltEffect for an example.
 */


/**************** Some useful macros ***************/

#define NBR_LEDS  12
#define RED {0x10, 0x00, 0x00}
#define GREEN {0x00, 0x10, 0x00}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0xff, 0xff, 0xff}
#define BLACK {0x00, 0x00, 0x00}

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define COPY_COLOR(dest, orig) dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2]
#define ADD_COLOR(dest, o1, o2) dest[0]=(o1[0]>>1)+(o2[0]>>1);dest[1]=(o1[1]>>1)+(o2[1]>>1);dest[2]=(o1[2]>>1)+(o2[2]>>1);
#define LIMIT(a) ((a>255)?255:(a<0)?0:a)
#define SIGN(a) ((a>=0)?1:-1)
#define DEADBAND(a, b) ((a<b) ? 0:a)
#define LINSCALE(domain_low, domain_high, codomain_low, codomain_high, value) ((codomain_high - codomain_low) / (domain_high - domain_low)) * (value - domain_low) + codomain_low

static uint32_t effect = 9;
static uint32_t neffect;
static uint8_t headlightEnable = 0;
static uint8_t black[][3] = {BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK,
                            };

/**************** Black (LEDs OFF) ***************/

static void blackEffect(uint8_t buffer[][3], bool reset)
{
  int i;

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      buffer[i][0] = 0;
      buffer[i][1] = 0;
      buffer[i][2] = 0;
    }
  }
}

/**************** White spin ***************/
static const uint8_t whiteRing[][3] = {{32, 32, 32}, {8,8,8}, {2,2,2},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };

static const uint8_t blueRing[][3] = {{64, 64, 255}, {32,32,64}, {8,8,16},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };

static const uint8_t greenRing[][3] = {{64, 255, 64}, {32,64,32}, {8,16,8},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };

static const uint8_t redRing[][3] = {{64, 0, 0}, {16,0,0}, {8,0,0},
                                       {4,0,0}, {2,0,0}, {1,0,0},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };

static void whiteSpinEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], greenRing[i]);
    }
  }

  COPY_COLOR(temp, buffer[0]);
  for (i=0; i<(NBR_LEDS-1); i++) {
    COPY_COLOR(buffer[i], buffer[i+1]);
  }
  COPY_COLOR(buffer[(NBR_LEDS-1)], temp);
}

static uint8_t solidRed=20, solidGreen=20, solidBlue=20;
static float glowstep = 0.05;
static void solidColorEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  static float brightness=0;

  if (reset) brightness = 0;

  if (brightness<1) brightness += 0.05;
  else brightness = 1;

  for (i=0; i<NBR_LEDS; i++)
  {
    buffer[i][0] = solidRed*brightness;
    buffer[i][1] = solidGreen*brightness;
    buffer[i][2] = solidBlue*brightness;
  }
}

static const uint8_t green[] = {0x00, 0xFF, 0x00};
static const uint8_t red[] = {0xFF, 0x00, 0x00};
static const uint8_t white[] = WHITE;
static const uint8_t part_black[] = BLACK;

static void boatEffect(uint8_t buffer[][3], bool reset)
{
  int i;

  uint8_t reds[] = {1,2,3,4,5};
  uint8_t greens[] = {7,8,9,10,11};
  uint8_t whites[] = {0};
  uint8_t blacks[] = {6};


  for (i=0; i<sizeof(reds); i++)
  {
    COPY_COLOR(buffer[reds[i]], red);
  }

  for (i=0; i<sizeof(greens); i++)
  {
    COPY_COLOR(buffer[greens[i]], green);
  }

  for (i=0; i<sizeof(whites); i++)
  {
    COPY_COLOR(buffer[whites[i]], white);
  }

  for (i=0; i<sizeof(blacks); i++)
  {
    COPY_COLOR(buffer[blacks[i]], part_black);
  }


}

/**************** Color spin ***************/

static const uint8_t colorRing[][3] = {{0,0,32}, {0,0,16}, {0,0,8},
                                       {0,0,4}, {16,16,16}, {8,8,8},
                                       {4,4,4},{32,0,0},{16,0,0},
                                       {8,0,0}, {4,0,0}, {2,0,0},
                                      };

static void colorSpinEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], colorRing[i]);
    }
  }

  COPY_COLOR(temp, buffer[0]);
  for (i=0; i<(NBR_LEDS-1); i++) {
    COPY_COLOR(buffer[i], buffer[i+1]);
  }
  COPY_COLOR(buffer[(NBR_LEDS-1)], temp);
}

static void spinEffect2(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[(NBR_LEDS-i)%NBR_LEDS], blueRing[i]);
    }
  }



  COPY_COLOR(temp, buffer[(NBR_LEDS-1)]);
  for (i=(NBR_LEDS-1); i>=0; i--) {
    COPY_COLOR(buffer[i], buffer[i-1]);
  }
  COPY_COLOR(buffer[0], temp);
}

static void doubleSpinEffect(uint8_t buffer[][3], bool reset) {
  static uint8_t sub1[NBR_LEDS][3];
  static uint8_t sub2[NBR_LEDS][3];
  int i;
  static int step;

  if (reset) step = 0;

  whiteSpinEffect(sub1, reset);
  spinEffect2(sub2, reset);
  //if ((step%3)) spinEffect2(sub2, false);
  //if (reset) spinEffect2(sub2, true);

  for (i=0; i<NBR_LEDS; i++)
  {
    ADD_COLOR(buffer[i], sub1[i], sub2[i]);
  }

  step ++;
}

/**************** Dynamic tilt effect ***************/

static void tiltEffect(uint8_t buffer[][3], bool reset)
{
  static int pitchid, rollid, thrust=-1;

  // 2014-12-28 chad: Reset LEDs to off to avoid color artifacts
  // when switching from other effects.
  if (reset)
    {
        int i;
        for (i=0; i<NBR_LEDS; i++) {
            buffer[i][0] = 0;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
        }
    }


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

    buffer[11][0] = LIMIT(led_middle + pitch);
    buffer[0][0] = LIMIT(led_middle + pitch);
    buffer[1][0] = LIMIT(led_middle + pitch);

    buffer[2][2] = LIMIT(led_middle - roll);
    buffer[3][2] = LIMIT(led_middle - roll);
    buffer[4][2] = LIMIT(led_middle - roll);

    buffer[5][0] = LIMIT(led_middle - pitch);
    buffer[6][0] = LIMIT(led_middle - pitch);
    buffer[7][0] = LIMIT(led_middle - pitch);

    buffer[8][2] = LIMIT(led_middle + roll);
    buffer[9][2] = LIMIT(led_middle + roll);
    buffer[10][2] = LIMIT(led_middle + roll);
  }
}

#define MAX_RATE 512

static void brightnessEffect(uint8_t buffer[][3], bool reset)
{

  static int gyroYid, gyroZid, gyroXid =- 1;
  static uint8_t brightness = 0;

  if (gyroXid < 0)
  {
    //Init
    gyroXid = logGetVarId("gyro", "x");
    gyroYid = logGetVarId("gyro", "y");
    gyroZid = logGetVarId("gyro", "z");
  }
  else
  {
    int i;
    int gyroX = (int)logGetFloat(gyroXid);
    int gyroY = (int)logGetFloat(gyroYid);
    int gyroZ = (int)logGetFloat(gyroZid);

    // Adjust to interval
    gyroX = (gyroX>MAX_RATE) ? MAX_RATE:(gyroX<-MAX_RATE) ? -MAX_RATE:gyroX;
    gyroY = (gyroY>MAX_RATE) ? MAX_RATE:(gyroY<-MAX_RATE) ? -MAX_RATE:gyroY;
    gyroZ = (gyroZ>MAX_RATE) ? MAX_RATE:(gyroZ<-MAX_RATE) ? -MAX_RATE:gyroZ;

    gyroX = SIGN(gyroX) * gyroX / 2;
    gyroY = SIGN(gyroY) * gyroY / 2;
    gyroZ = SIGN(gyroZ) * gyroZ / 2;

    gyroX = DEADBAND(gyroX, 5);
    gyroY = DEADBAND(gyroY, 5);
    gyroZ = DEADBAND(gyroZ, 5);

    for (i=0; i < NBR_LEDS; i++)
    {
      buffer[i][0] = (uint8_t)(LIMIT(gyroZ));
      buffer[i][1] = (uint8_t)(LIMIT(gyroY));
      buffer[i][2] = (uint8_t)(LIMIT(gyroX));
    }

    brightness++;
  }
}


static void setHeadlightsOn(bool on)
{
  if (on)
    GPIO_SetBits(GPIOB, GPIO_Pin_4);
  else
    GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}


/* LED-ring test effect */
#define TEST_INTENTS 20
static uint8_t test_pat[3][3] = {{TEST_INTENTS, 0, 0}, {0, TEST_INTENTS, 0}, {0, 0, TEST_INTENTS}};
static uint8_t test_eff_nbr = 0;
#define TEST_DELAY 4
static uint8_t test_delay_counter = 0;
static uint8_t headlight_test_counter =0;
static uint8_t test_front = false;
static void ledTestEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  static float brightness=0;

  if (reset) brightness = 0;

  if (brightness<1) brightness += 0.05;
  else brightness = 1;

  for (i=0; i<NBR_LEDS; i++)
  {
    buffer[i][0] = test_pat[test_eff_nbr][0];
    buffer[i][1] = test_pat[test_eff_nbr][1];
    buffer[i][2] = test_pat[test_eff_nbr][2];
  }

  test_delay_counter++;
  headlight_test_counter++;

  if (test_delay_counter > TEST_DELAY) {
    test_delay_counter = 0;
    test_eff_nbr = (test_eff_nbr + 1) % 3;
  }

  if (headlight_test_counter > (TEST_DELAY*3)) {
    headlight_test_counter = 0;
    test_front = !test_front;
    headlightEnable = test_front;
  }
}

/**
 * An effect that shows the battery charge on the LED ring.
 *
 * Red means empty, blue means full.
 */
static float emptyCharge = 3.1, fullCharge = 4.2;
static void batteryChargeEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  static int vbatid;
  float vbat;

  vbatid = logGetVarId("pm", "vbat");
  vbat = logGetFloat(vbatid);

  for (i = 0; i < NBR_LEDS; i++) {
    buffer[i][0] = LIMIT(LINSCALE(emptyCharge, fullCharge, 255, 0, vbat)); // Red (emtpy)
    buffer[i][1] = 0; // Green
    buffer[i][2] = LIMIT(LINSCALE(emptyCharge, fullCharge, 0, 255, vbat)); // Blue (charged)
  }
}

/**************** Effect list ***************/


NeopixelRingEffect effectsFct[] = {blackEffect,
                                   whiteSpinEffect,
                                   colorSpinEffect,
                                   tiltEffect,
                                   brightnessEffect,
                                   spinEffect2,
                                   doubleSpinEffect,
                                   solidColorEffect,
                                   ledTestEffect,
                                   batteryChargeEffect,
                                   boatEffect
                                  }; //TODO Add more

/*
NeopixelRingEffect effectsFct[] = {blackEffect,
                                   doubleSpinEffect,
                                   solidColorEffect,
                                  };
*/
/********** Ring init and switching **********/

static xTimerHandle timer;



void neopixelringWorker(void * data)
{
  static int current_effect = 0;
  static uint8_t buffer[NBR_LEDS][3];
  bool reset = true;

  if (/*!pmIsDischarging() ||*/ (effect > neffect)) {
    ws2812Send(black, NBR_LEDS);
    return;
  }

  if (current_effect != effect) {
    reset = true;
  } else {
    reset = false;
  }
  current_effect = effect;

  effectsFct[current_effect](buffer, reset);
  ws2812Send(buffer, NBR_LEDS);
}

static void neopixelringTimer(xTimerHandle timer)
{
  workerSchedule(neopixelringWorker, NULL);

  setHeadlightsOn(headlightEnable);
}

void neopixelringInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  ws2812Init();

  neffect = sizeof(effectsFct)/sizeof(effectsFct[0])-1;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  timer = xTimerCreate( (const signed char *)"ringTimer", M2T(55),
                                     pdTRUE, NULL, neopixelringTimer );
  xTimerStart(timer, 100);
}

PARAM_GROUP_START(ring)
PARAM_ADD(PARAM_UINT8, effect, &effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD(PARAM_UINT8, solidRed, &solidRed)
PARAM_ADD(PARAM_UINT8, solidGreen, &solidGreen)
PARAM_ADD(PARAM_UINT8, solidBlue, &solidBlue)
PARAM_ADD(PARAM_UINT8, headlightEnable, &headlightEnable)
PARAM_ADD(PARAM_FLOAT, glowstep, &glowstep)
PARAM_ADD(PARAM_FLOAT, emptyCharge, &emptyCharge)
PARAM_ADD(PARAM_FLOAT, fullCharge, &fullCharge)
PARAM_GROUP_STOP(ring)
