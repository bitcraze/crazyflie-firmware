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
 * ledring12.c: RGB Ring 12 Leds effects/driver
 */

#include "ledring12.h"

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "stm32fxxx.h"

#include "deck.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "ledring12.h"
#include "ws2812.h"
#include "worker.h"
#include "param.h"
#include "pm.h"
#include "log.h"
#include "pulse_processor.h"

#define DEBUG_MODULE "LED"
#include "debug.h"

static bool isInit = false;

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

typedef void (*Ledring12Effect)(uint8_t buffer[][3], bool reset);

/**************** Some useful macros ***************/

#define RED {0x10, 0x00, 0x00}
#define GREEN {0x00, 0x10, 0x00}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0xff, 0xff, 0xff}
#define BLACK {0x00, 0x00, 0x00}

#define COPY_COLOR(dest, orig) dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2]
#define ADD_COLOR(dest, o1, o2) dest[0]=(o1[0]>>1)+(o2[0]>>1);dest[1]=(o1[1]>>1)+(o2[1]>>1);dest[2]=(o1[2]>>1)+(o2[2]>>1);
#define LIMIT(a) ((a>255)?255:(a<0)?0:a)
#define SIGN(a) ((a>=0)?1:-1)
#define DEADBAND(a, b) ((a<b) ? 0:a)
#define LINSCALE(domain_low, domain_high, codomain_low, codomain_high, value) ((codomain_high - codomain_low) / (domain_high - domain_low)) * (value - domain_low) + codomain_low
#define SET_WHITE(dest, intensity) dest[0] = intensity; dest[1] = intensity; dest[2] = intensity;
#define RGB565_TO_RGB888(dest, orig)                                           \
  uint8_t R5, G6, B5;                                                          \
  R5 = orig[0] >> 3;                                                           \
  G6 = ((orig[0] & 0x07) << 3) | (orig[1] >> 5);                               \
  B5 = orig[1] & 0x1F;                                                         \
  dest[0] = ((uint16_t)R5 * 527 + 23) >> 6;                                    \
  dest[1] = ((uint16_t)G6 * 259 + 33) >> 6;                                    \
  dest[2] = ((uint16_t)B5 * 527 + 23) >> 6;

#ifndef LEDRING_DEFAULT_EFFECT
#define LEDRING_DEFAULT_EFFECT 6
#endif

#define LEDRING_TIME_MEM_SEC 1000 / 25

static uint32_t effect = LEDRING_DEFAULT_EFFECT;
static uint32_t neffect;
static uint8_t headlightEnable = 0;
static uint8_t black[][3] = {BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK,
                            };

static const uint8_t green[] = {0x00, 0xFF, 0x00};
static const uint8_t red[] = {0xFF, 0x00, 0x00};
static const uint8_t blue[] = {0x00, 0x00, 0xFF};
static const uint8_t white[] = WHITE;
static const uint8_t part_black[] = BLACK;

uint8_t ledringmem[NBR_LEDS * 2];

ledtimings ledringtimingsmem;

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
#if NBR_LEDS > 12
static const uint8_t whiteRing[NBR_LEDS][3] = {{32, 32, 32}, {8,8,8}, {2,2,2},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };
#else
static const uint8_t whiteRing[][3] = {{32, 32, 32}, {8,8,8}, {2,2,2},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };
#endif

#if NBR_LEDS > 12
static const uint8_t blueRing[NBR_LEDS][3] = {{64, 64, 255}, {32,32,64}, {8,8,16},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };
#else
static const uint8_t blueRing[][3] = {{64, 64, 255}, {32,32,64}, {8,8,16},
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                       BLACK, BLACK, BLACK,
                                      };
#endif

// #if NBR_LEDS > 12
// static const uint8_t greenRing[NBR_LEDS][3] = {{64, 255, 64}, {32,64,32}, {8,16,8},
//                                       BLACK, BLACK, BLACK,
//                                       BLACK, BLACK, BLACK,
//                                       BLACK, BLACK, BLACK,
//                                      };
// #else
// static const uint8_t greenRing[][3] = {{64, 255, 64}, {32,64,32}, {8,16,8},
//                                       BLACK, BLACK, BLACK,
//                                       BLACK, BLACK, BLACK,
//                                       BLACK, BLACK, BLACK,
//                                      };
// #endif

// #if NBR_LEDS > 12
// static const uint8_t redRing[NBR_LEDS][3] = {{64, 0, 0}, {16,0,0}, {8,0,0},
//                                       {4,0,0}, {2,0,0}, {1,0,0},
//                                       BLACK, BLACK, BLACK,
//                                       BLACK, BLACK, BLACK,
//                                      };
// #else
// static const uint8_t redRing[][3] = {{64, 0, 0}, {16,0,0}, {8,0,0},
//                                       {4,0,0}, {2,0,0}, {1,0,0},
//                                       BLACK, BLACK, BLACK,
//                                       BLACK, BLACK, BLACK,
//                                      };
// #endif

static void whiteSpinEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], whiteRing[i]);
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

  if (brightness<1) brightness += 0.05f;
  else brightness = 1;

  for (i=0; i<NBR_LEDS; i++)
  {
    buffer[i][0] = solidRed*brightness;
    buffer[i][1] = solidGreen*brightness;
    buffer[i][2] = solidBlue*brightness;
  }
}

static void virtualMemEffect(uint8_t buffer[][3], bool reset)
{
  int i;

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], part_black);
    }
  }

  for (i = 0; i < NBR_LEDS; i++)
  {
    uint8_t R5, G6, B5;
    uint8_t (*led)[2] = (uint8_t (*)[2])ledringmem;
    // Convert from RGB565 to RGB888
    R5 = led[i][0] >> 3;
    G6 = ((led[i][0] & 0x07) << 3) | (led[i][1] >> 5);
    B5 = led[i][1] & 0x1F;
    buffer[i][0] = ((uint16_t)R5 * 527 + 23 ) >> 6;
    buffer[i][1] = ((uint16_t)G6 * 259 + 33 ) >> 6;
    buffer[i][2] = ((uint16_t)B5 * 527 + 23 ) >> 6;
  }
}

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

#if NBR_LEDS > 12
static const uint8_t colorRing[NBR_LEDS][3] = {{0,0,32}, {0,0,16}, {0,0,8},
                                       {0,0,4}, {16,16,16}, {8,8,8},
                                       {4,4,4},{32,0,0},{16,0,0},
                                       {8,0,0}, {4,0,0}, {2,0,0},
                                      };
#else
static const uint8_t colorRing[][3] = {{0,0,32}, {0,0,16}, {0,0,8},
                                       {0,0,4}, {16,16,16}, {8,8,8},
                                       {4,4,4},{32,0,0},{16,0,0},
                                       {8,0,0}, {4,0,0}, {2,0,0},
                                      };
#endif

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
  for (i=(NBR_LEDS-1); i>0; i--) {
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


/*************** Gravity light effect *******************/

static float gravityLightCalculateAngle(float pitch, float roll);
static void gravityLightRender(uint8_t buffer[][3], float led_index, int intensity);

static void gravityLight(uint8_t buffer[][3], bool reset)
{
  static int pitchid, rollid;
  static bool isInitialized = false;

  if (!isInitialized) {
    pitchid = logGetVarId("stabilizer", "pitch");
    rollid = logGetVarId("stabilizer", "roll");
    isInitialized = true;
  }

  float pitch = logGetFloat(pitchid); // -180 to 180
  float roll = logGetFloat(rollid); // -180 to 180

  float angle = gravityLightCalculateAngle(pitch, roll);
  float led_index = NBR_LEDS * angle / (2 * (float) M_PI);
  int intensity = LIMIT(sqrtf(pitch * pitch + roll * roll));
  gravityLightRender(buffer, led_index, intensity);
}

static float gravityLightCalculateAngle(float pitch, float roll) {
  float angle = 0.0;

  if (roll != 0) {
    angle = atanf(pitch / roll) + (float) M_PI_2;

    if (roll < 0.0f) {
      angle += (float) M_PI;
    }
  }

  return angle;
}

static void gravityLightRender(uint8_t buffer[][3], float led_index, int intensity) {
  float width = 5;
  float height = intensity;

  int i;
  for (i = 0; i < NBR_LEDS; i++) {
	float distance = fabsf(led_index - i);
	if (distance > NBR_LEDS / 2) {
		distance = NBR_LEDS - distance;
	}

	int col = height - distance * (height / (width / 2));
	SET_WHITE(buffer[i], LIMIT(col));
  }
}


/*************** Brightness effect ********************/

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

  if (brightness<1) brightness += 0.05f;
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

/**
 * An effect mimicking a blue light siren
 */
static void siren(uint8_t buffer[][3], bool reset)
{
  int i;
  static int tic = 0;

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], part_black);
    }
  }

  if ((tic < 10) && (tic & 1))
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], blue);
    }
  }
  else
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], part_black);
    }
  }
  if (++tic >= 20) tic = 0;
}

/**
 * Display a solid color and fade to the next one in a given time
 */
static uint32_t fadeColor = 0;
static float fadeTime = 0.5;

static float currentFadeTime = 0.5;

#include "log.h"

LOG_GROUP_START(ring)
LOG_ADD(LOG_FLOAT, fadeTime, &currentFadeTime)
LOG_GROUP_STOP(ring)
static void fadeColorEffect(uint8_t buffer[][3], bool reset)
{
  static float currentRed = 255;
  static float currentGreen = 255;
  static float currentBlue = 255;
  static float targetRed, targetGreen, targetBlue;
  static uint32_t previousTargetColor = 0xffffffff;
  static float cachedFadeTime = 0.5;

  if (fadeColor != previousTargetColor) {
    float alpha = currentFadeTime / cachedFadeTime;

    currentRed = (alpha * currentRed) + ((1 - alpha) * targetRed);
    currentGreen = (alpha * currentGreen) + ((1 - alpha) * targetGreen);
    currentBlue = (alpha * currentBlue) + ((1 - alpha) * targetBlue);

    currentFadeTime = fadeTime;
    cachedFadeTime = fadeTime;
    targetRed = (fadeColor >> 16) & 0x0FF;
    targetGreen = (fadeColor >> 8) & 0x0FF;
    targetBlue = (fadeColor >> 0) & 0x0FF;

    previousTargetColor = fadeColor;
  }

  if (currentFadeTime > 0)
  {
    float alpha = currentFadeTime / cachedFadeTime;

    int red = (alpha * currentRed) + ((1-alpha) * targetRed);
    int green = (alpha * currentGreen) + ((1 - alpha) * targetGreen);
    int blue = (alpha * currentBlue) + ((1 - alpha) * targetBlue);

    for (int i = 0; i < NBR_LEDS; i++)
    {
      buffer[i][0] = red;
      buffer[i][1] = green;
      buffer[i][2] = blue;
    }

    currentFadeTime -= 50e-3f;
  } else {
    currentFadeTime = 0;
    currentRed = (fadeColor >> 16) & 0x0FF;
    currentGreen = (fadeColor >> 8) & 0x0FF;
    currentBlue = (fadeColor >> 0) & 0x0FF;

    for (int i = 0; i < NBR_LEDS; i++)
    {
      buffer[i][0] = currentRed;
      buffer[i][1] = currentGreen;
      buffer[i][2] = currentBlue;
    }
  }
}

/**
 * An effect that shows the Signal Strength (RSSI) on the LED ring.
 *
 * Red means bad, green means good.
 */
static float badRssi = 85, goodRssi = 35;
static void rssiEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  static int isConnectedId, rssiId;
  float rssi;
  bool isConnected;

  isConnectedId = logGetVarId("radio", "isConnected");
  isConnected = logGetUint(isConnectedId);

  rssiId = logGetVarId("radio", "rssi");
  rssi = logGetFloat(rssiId);
  uint8_t rssi_scaled = LIMIT(LINSCALE(badRssi, goodRssi, 0, 255, rssi));

  for (i = 0; i < NBR_LEDS; i++) {
    if (isConnected) {
      buffer[i][0] = 255 - rssi_scaled; // Red (bad)
      buffer[i][1] = rssi_scaled; // Green (good)
      buffer[i][2] = 0; // Blue
    } else {
      buffer[i][0] = 100; // Red
      buffer[i][1] = 100; // Green
      buffer[i][2] = 100; // Blue
    }
  }
}

/**
 * An effect that shows the status of the lighthouse.
 *
 * Red means 0 angles, green means 16 angles (2 basestations x 4 crazyflie sensors x 2 sweeping directions).
 */
static void lightHouseEffect(uint8_t buffer[][3], bool reset)
{
  #if DISABLE_LIGHTHOUSE_DRIVER == 1
    uint16_t validAngles = 0;
  #else
    uint16_t validAngles = pulseProcessorAnglesQuality();
  #endif

  for (int i = 0; i < NBR_LEDS; i++) {
    buffer[i][0] = LIMIT(LINSCALE(0.0f, 255.0f, 100.0f, 0.0f, validAngles)); // Red (small validAngles)
    buffer[i][1] = LIMIT(LINSCALE(0.0f, 255.0f, 0.0f, 100.0f, validAngles)); // Green (large validAngles)
    buffer[i][2] = 0;
  }
}

/**
 * An effect that shows the status of the location service.
 *
 * Red means bad, green means good.
 * Blinking means battery was low during flight.
 */
static void locSrvStatus(uint8_t buffer[][3], bool reset)
{
  static int locSrvTickId = -1;
  static int pmStateId = -1;

  static int tic = 0;
  static bool batteryEverLow = false;

  // lazy initialization of the logging variables
  if (locSrvTickId == -1) {
    locSrvTickId = logGetVarId("locSrvZ", "tick");
    pmStateId = logGetVarId("pm", "state");
  }

  // compute time since the last update in milliseconds
  uint16_t time_since_last_update = xTaskGetTickCount() - logGetUint(locSrvTickId);
  if (time_since_last_update > 30) {
    time_since_last_update = 30;
  }

  int8_t pmstate = logGetInt(pmStateId);
  if (pmstate == lowPower) {
    batteryEverLow = true;
  }

  for (int i = 0; i < NBR_LEDS; i++) {
    if (batteryEverLow && tic < 10) {
      buffer[i][0] = 0;
      buffer[i][1] = 0;
    } else {
      buffer[i][0] = LIMIT(LINSCALE(0, 30, 0, 100, time_since_last_update)); // Red (large time_since_last_update)
      buffer[i][1] = LIMIT(LINSCALE(0, 30, 100, 0, time_since_last_update)); // Green (small time_since_last_update)
    }
    buffer[i][2] = 0;
  }

  if (++tic >= 20) {
    tic = 0;
  }
}

static bool isTimeMemDone(ledtiming current)
{
  return current.duration == 0 && current.color[0] == 0 &&
         current.color[1] == 0;
}

static int timeEffectI = 0;
static uint64_t timeEffectTime = 0;
static uint8_t timeEffectPrevBuffer[NBR_LEDS][3];
static float timeEffectRotation = 0;

static void timeMemEffect(uint8_t outputBuffer[][3], bool reset)
{
  // Start timer when going to this
  if (reset) {
    for (int i = 0; i < NBR_LEDS; i++) {
      COPY_COLOR(timeEffectPrevBuffer[i], part_black);
      COPY_COLOR(outputBuffer[i], part_black);
    }

    timeEffectRotation = 0;
    timeEffectTime = usecTimestamp() / 1000;
    timeEffectI = 0;
  }

  ledtiming current = ledringtimingsmem.timings[timeEffectI];

  // Stop when completed
  if (isTimeMemDone(current))
    return;

  // Get the proper index
  uint64_t time = usecTimestamp() / 1000;
  while (timeEffectTime + LEDRING_TIME_MEM_SEC * current.duration < time) {
    // Apply previous commands to the cache
    uint8_t color[3];
    RGB565_TO_RGB888(color, current.color)

    if (current.leds == 0) {
      for (int i = 0; i < NBR_LEDS; i++) {
        COPY_COLOR(timeEffectPrevBuffer[i], color);
      }
    } else {
      COPY_COLOR(timeEffectPrevBuffer[current.leds], color);
    }

    // Goto next effect
    if(current.rotate)
      timeEffectRotation += 1.0f * current.duration * LEDRING_TIME_MEM_SEC / (current.rotate * 1000);
    timeEffectTime += LEDRING_TIME_MEM_SEC * current.duration;
    timeEffectI++;
    current = ledringtimingsmem.timings[timeEffectI];

    if (isTimeMemDone(current))
      return;
  }

  // Apply the current effect
  uint8_t color[3];
  RGB565_TO_RGB888(color, current.color)
  uint8_t currentBuffer[NBR_LEDS][3];
  for (int i = 0; i < NBR_LEDS; i++) {
    COPY_COLOR(currentBuffer[i], timeEffectPrevBuffer[i]);
  }

  if (current.fade) {
    float percent = 1.0 * (time - timeEffectTime) / (current.duration * LEDRING_TIME_MEM_SEC);
    if (current.leds == 0)
      for (int i = 0; i < NBR_LEDS; i++)
        for (int j = 0; j < 3; j++)
          currentBuffer[i][j] = (1.0f - percent) * timeEffectPrevBuffer[i][j] + percent * color[j];
    else
      for (int j = 0; j < 3; j++)
        currentBuffer[current.leds][j] = (1.0f - percent) * timeEffectPrevBuffer[current.leds][j] + percent * color[j];
  }
  else {
    if (current.leds == 0) {
      for (int i = 0; i < NBR_LEDS; i++) {
        COPY_COLOR(currentBuffer[i], color);
      }
    } else {
      COPY_COLOR(currentBuffer[current.leds], color);
    }
  }

  float rotate = timeEffectRotation;
  if(current.rotate) {
    rotate += 1.0f * (time - timeEffectTime) / (current.rotate * 1000);
  }

  int shift = rotate * NBR_LEDS;
  float percentShift = rotate * NBR_LEDS - shift;
  shift = shift % NBR_LEDS;

  // Output current leds
  for (int i = 0; i < NBR_LEDS; i++)
    for (int j = 0; j < 3; j++)
      outputBuffer[(i+shift) % NBR_LEDS][j] =
        percentShift * currentBuffer[i][j] +
        (1-percentShift) * currentBuffer[(i+1) % NBR_LEDS][j];
}

/**************** Effect list ***************/


Ledring12Effect effectsFct[] =
{
  blackEffect,
  whiteSpinEffect,
  colorSpinEffect,
  tiltEffect,
  brightnessEffect,
  spinEffect2,
  doubleSpinEffect,
  solidColorEffect,
  ledTestEffect,
  batteryChargeEffect,
  boatEffect,
  siren,
  gravityLight,
  virtualMemEffect,
  fadeColorEffect,
  rssiEffect,
  locSrvStatus,
  timeMemEffect,
  lightHouseEffect,
};

/********** Ring init and switching **********/
static xTimerHandle timer;



void ledring12Worker(void * data)
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

static void ledring12Timer(xTimerHandle timer)
{
  workerSchedule(ledring12Worker, NULL);

  setHeadlightsOn(headlightEnable);
}

static void ledring12Init(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  GPIO_InitTypeDef GPIO_InitStructure;

  ws2812Init();

  neffect = sizeof(effectsFct)/sizeof(effectsFct[0])-1;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  isInit = true;

  timer = xTimerCreate( "ringTimer", M2T(50),
                                     pdTRUE, NULL, ledring12Timer );
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
PARAM_ADD(PARAM_UINT32, fadeColor, &fadeColor)
PARAM_ADD(PARAM_FLOAT, fadeTime, &fadeTime)
PARAM_GROUP_STOP(ring)

static const DeckDriver ledring12_deck = {
  .vid = 0xBC,
  .pid = 0x01,
  .name = "bcLedRing",

  .usedPeriph = DECK_USING_TIMER3,
  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_3,

  .init = ledring12Init,
};

DECK_DRIVER(ledring12_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &isInit)
PARAM_GROUP_STOP(deck)
