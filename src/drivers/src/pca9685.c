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
 * pca9685.c: 12-bit, 16-channel PWM servo (, LED, ESC, ...) driver
 */

#include "i2cdev.h"
#include "pca9685.h"
#include "math.h" // fmax, fmin

// the pca9685 uses 8-bit internal addresses.
enum Registers
{
  regMode1 = 0x00,
  regMode2,
  regSubAdr1,
  regSubAdr2,
  regSubAdr3,
  regAllCallAdr,
  // Each channel has 4 registers:
  // [ ON_L | ON_H | OFF_L | OFF_H ]
  regLED_FIRST = 0x06,
  regLED_LAST = 0x45,
  regLED_ALL = 0xFA,
  regPreScale = 0xFE,
  regTestMode = 0xFF,
};

int const LED_NBYTES = 4 * (regLED_LAST - regLED_FIRST + 1);

// the bits of the Mode1 register, as masks.
enum Mode1
{
   m1AllCall = 1 << 0,
      m1Sub3 = 1 << 1,
      m1Sub2 = 1 << 2,
      m1Sub1 = 1 << 3,
     m1Sleep = 1 << 4,
  m1AutoIncr = 1 << 5,
    m1ExtClk = 1 << 6,
   m1Restart = 1 << 7,
};

// TODO: mode2

static inline int channelReg(int channel)
{
  return regLED_FIRST + 4 * channel;
}

static inline void u16ToByte(uint16_t i, uint8_t *bytes)
{
  bytes[0] = i & 0xFF;
  bytes[1] = i >> 8;
}

static void durationToBytes(uint16_t duration, uint8_t *bytes)
{
  if (duration >= 4096) {
    u16ToByte(4096, bytes);
    u16ToByte(0, bytes + 2);
  }
  else if (duration == 0) {
    u16ToByte(0, bytes);
    u16ToByte(4096, bytes + 2);
  }
  else {
    u16ToByte(0, bytes);
    u16ToByte(duration, bytes + 2);
  }
}

static uint16_t dutyToDuration(float duty)
{
  duty = fmax(duty, 0.0f);
  duty = fmin(duty, 1.0f);
  return duty * 4096.0f;
}

//
// SYNCHRONOUS
// (see pca9685.h for function descriptions.)
//

bool pca9685init(int addr, float pwmFreq)
{
  // initial state is sleeping.
  // must be asleep to set PWM freq.

  // set up value of Mode1 register. compared to default state, we:
  // - wake up from sleep
  // - enable AutoIncrement
  // - disable AllCall
  uint8_t mode1val = m1AutoIncr;

  static float const OSC_CLOCK = 25.0f * 1000.0f * 1000.0f;
  // according to my measurements with an oscilloscope,
  // i think the datasheet is wrong about the minus one!
  //int const prescale = OSC_CLOCK / (4096.0f * pwmFreq) + 0.5f - 1.0f
  int const prescale = OSC_CLOCK / (4096.0f * pwmFreq) + 0.5f;
  return
    (prescale >= 0x03) && (prescale <= 0xFF) &&
    i2cdevWriteByte(&deckBus, addr, regPreScale, (uint8_t)prescale) &&
    i2cdevWriteByte(&deckBus, addr, regMode1, mode1val);
}

bool pca9685setDuties(
  int addr, int chanBegin, int nChan, float const *duties)
{
  uint8_t data[LED_NBYTES];
  for (int i = 0; i < nChan; ++i) {
    uint16_t duration = dutyToDuration(duties[i]);
    durationToBytes(duration, data + 4*i);
  }
  int const reg = channelReg(chanBegin);
  return i2cdevWriteReg8(&deckBus, addr, reg, 4*nChan, data);
}

bool pca9685setDurations(
  int addr, int chanBegin, int nChan, uint16_t const *durations)
{
  uint8_t data[LED_NBYTES];
  for (int i = 0; i < nChan; ++i) {
    durationToBytes(durations[i], data + 4*i);
  }
  int const reg = channelReg(chanBegin);
  return i2cdevWriteReg8(&deckBus, addr, reg, 4*nChan, data);
}

//
// ASYNCHRONOUS
// (see pca9685.h for function descriptions.)
//

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "static_mem.h"

struct asyncRequest
{
  int addr;
  int chanBegin;
  int nChan;
  uint16_t durations[16];
};

// reqPush is not static for thread safety
static struct asyncRequest reqPop;

static TaskHandle_t task;
STATIC_MEM_TASK_ALLOC(task, PCA9685_TASK_STACKSIZE);

static xQueueHandle queue;
STATIC_MEM_QUEUE_ALLOC(queue, 1, sizeof(struct asyncRequest));

static void asyncTask(__attribute__((unused)) void *param)
{
  while (true) {
    BaseType_t ok = xQueueReceive(queue, &reqPop, portMAX_DELAY);
    if (ok == pdTRUE) {
      // blocking message send.
      pca9685setDurations(
        reqPop.addr, reqPop.chanBegin, reqPop.nChan, reqPop.durations);
    }
  }
}

bool pca9685startAsyncTask()
{
  queue = STATIC_MEM_QUEUE_CREATE(queue);
  if (queue == 0) {
    return false;
  }

  task = STATIC_MEM_TASK_CREATE(task, asyncTask, PCA9685_TASK_NAME, NULL, PCA9685_TASK_PRI);
  return true;
}

bool pca9685setDutiesAsync(
  int addr, int chanBegin, int nChan, float const *duties)
{
  struct asyncRequest reqPush = {
    .addr = addr,
    .chanBegin = chanBegin,
    .nChan = nChan,
  };
  for (int i = 0; i < nChan; ++i) {
    reqPush.durations[i] = dutyToDuration(duties[i]);
  }
  // drop message unless queue is empty!
  xQueueSend(queue, &reqPush, 0);
  return true;
}

bool pca9685setDurationsAsync(
  int addr, int chanBegin, int nChan, uint16_t const *durations)
{
  struct asyncRequest reqPush = {
    .addr = addr,
    .chanBegin = chanBegin,
    .nChan = nChan,
  };
  for (int i = 0; i < nChan; ++i) {
    reqPush.durations[i] = durations[i];
  }
  // drop message unless queue is empty!
  xQueueSend(queue, &reqPush, 0);
  return true;
}
