/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2023 Bitcraze AB
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
 *
 * extrx.c - Module to handle external receiver inputs
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32fxxx.h"
#include "config.h"
#include "system.h"
#include "nvicconf.h"
#include "commander.h"
#include "uart1.h"
#include "cppm.h"
#include "crtp_commander.h"
#include "supervisor.h"

#define DEBUG_MODULE  "EXTRX"
#include "debug.h"
#include "log.h"
#include "static_mem.h"

#define ENABLE_CPPM
#define ENABLE_EXTRX_LOG


#define EXTRX_NR_CHANNELS  8

#ifdef EXTRX_TAER
  #define EXTRX_CH_THRUST     0
  #define EXTRX_CH_ROLL      1
  #define EXTRX_CH_PITCH     2
  #define EXTRX_CH_YAW       3

  #define EXTRX_SIGN_ROLL    (1)
  #define EXTRX_SIGN_PITCH   (-1)
  #define EXTRX_SIGN_YAW     (-1)
#else // default is AETR, like in BetaFlight
  #define EXTRX_CH_THRUST     2
  #define EXTRX_CH_ROLL      0
  #define EXTRX_CH_PITCH     1
  #define EXTRX_CH_YAW       3

  #define EXTRX_SIGN_ROLL    (1)
  #define EXTRX_SIGN_PITCH   (-1)
  #define EXTRX_SIGN_YAW     (-1)
#endif

#define EXTRX_CH_ALTHOLD   4
#define EXTRX_CH_ARM       5
#define EXTRX_CH_MODE     6

#define EXTRX_SIGN_ALTHOLD   (-1)
#define EXTRX_SIGN_ARM       (-1)
#define EXTRX_SIGN_MODE       (-1)

#define EXTRX_DEADBAND_ROLL (0.05f)
#define EXTRX_DEADBAND_PITCH (0.05f)
#define EXTRX_DEADBAND_YAW (0.05f)

#define EXTRX_SWITCH_MIN_CNT 5 // number of identical subsequent switch states before the switch variable is changed

bool extRxArm = false;
bool extRxAltHold = false;
bool extRxModeRate = false;

#if CONFIG_DECK_EXTRX_ARMING
  bool extRxArmPrev = false;
  int8_t arm_cnt = 0;
#endif

#if CONFIG_DECK_EXTRX_ALT_HOLD
  #define EXTRX_DEADBAND_ZVEL  (0.25f)
  bool extRxAltHoldPrev = false;
  int8_t altHold_cnt = 0;
#endif

#if CONFIG_DECK_EXTRX_MODE_RATE
  bool extRxModeRatePrev = false;
  int8_t modeRate_cnt = 0;
#endif

static setpoint_t extrxSetpoint;
static uint16_t ch[EXTRX_NR_CHANNELS] = {0};

static void extRxTask(void *param);
static void extRxDecodeCppm(void);
static void extRxDecodeChannels(void);


STATIC_MEM_TASK_ALLOC(extRxTask, EXTRX_TASK_STACKSIZE);

void extRxInit(void)
{

  extrxSetpoint.mode.roll = modeAbs;
  extrxSetpoint.mode.pitch = modeAbs;
  extrxSetpoint.mode.yaw = modeVelocity;

#ifdef ENABLE_CPPM
  cppmInit();
  #ifdef CONFIG_DECK_EXTRX_TAER
    DEBUG_PRINT("CPPM initialized, expecting TAER channel mapping\n");
  #else
    DEBUG_PRINT("CPPM initialized, expecting AETR channel mapping\n");
  #endif

#endif

#ifdef ENABLE_SPEKTRUM
  uart1Init();
#endif

  STATIC_MEM_TASK_CREATE(extRxTask, extRxTask, EXTRX_TASK_NAME, NULL, EXTRX_TASK_PRI);
}

static void extRxTask(void *param)
{

  //Wait for the system to be fully started
  systemWaitStart();
  DEBUG_PRINT("CPPM Task Started\n");

  while (true)
  {
    extRxDecodeCppm();
  }
}

static void extRxDecodeChannels(void)
{

  #if CONFIG_DECK_EXTRX_ARMING
  if (EXTRX_SIGN_ARM * cppmConvert2Float(ch[EXTRX_CH_ARM], -1, 1, 0.0) > 0.5f) // channel needs to be 75% or more to work correctly with 2/3 way switches
  {
    if (arm_cnt < EXTRX_SWITCH_MIN_CNT) arm_cnt++;
    else extRxArm = true;

    if (extRxArmPrev != extRxArm)
    {
      if (supervisorRequestArming(extRxArm)) {
        DEBUG_PRINT("System armed\n");
      } else {
        DEBUG_PRINT("Arming failed\n");
      }
    }
  }
  else
  {
    if (arm_cnt > 0) arm_cnt--;
    else extRxArm = false;

    if (extRxArmPrev != extRxArm)
    {
      DEBUG_PRINT("System unarmed\n");
      supervisorRequestArming(extRxArm);
    }
  }

  extRxArmPrev = extRxArm;
  #endif

  #if CONFIG_DECK_EXTRX_ALT_HOLD
  if (EXTRX_SIGN_ALTHOLD * cppmConvert2Float(ch[EXTRX_CH_ALTHOLD], -1, 1, 0.0) > 0.5f)
  {
    if (altHold_cnt < EXTRX_SWITCH_MIN_CNT) altHold_cnt++;
    else extRxAltHold=true;

    if (extRxAltHoldPrev != extRxAltHold) DEBUG_PRINT("Althold mode ON\n");
  }
  else
  {
    if (altHold_cnt > 0) altHold_cnt--;
    else extRxAltHold=false;

    if (extRxAltHoldPrev != extRxAltHold) DEBUG_PRINT("Althold mode OFF\n");
  }

  if (extRxAltHold)
  {
    extrxSetpoint.thrust = 0;
    extrxSetpoint.mode.z = modeVelocity;

    extrxSetpoint.velocity.z = cppmConvert2Float(ch[EXTRX_CH_THRUST], -1, 1, EXTRX_DEADBAND_ZVEL);
  }
  else
  {
    extrxSetpoint.mode.z = modeDisable;
    extrxSetpoint.thrust = cppmConvert2uint16(ch[EXTRX_CH_THRUST]);
  }

  extRxAltHoldPrev = extRxAltHold;
  #else

  extrxSetpoint.mode.z = modeDisable;
  extrxSetpoint.thrust = cppmConvert2uint16(ch[EXTRX_CH_THRUST]);
  #endif

  #if CONFIG_DECK_EXTRX_MODE_RATE
  if (EXTRX_SIGN_MODE * cppmConvert2Float(ch[EXTRX_CH_MODE], -1, 1, 0.0) > 0.5f) // channel needs to be 75% or more to work correctly with 2/3 way switches
  {
    if (modeRate_cnt < EXTRX_SWITCH_MIN_CNT) modeRate_cnt++;
    else extRxModeRate = true;

    if (extRxModeRatePrev != extRxModeRate)
    {
      DEBUG_PRINT("Switched to rate mode\n");
      extrxSetpoint.mode.roll = modeVelocity;
      extrxSetpoint.mode.pitch = modeVelocity;
    }
  }
  else
  {
    if (modeRate_cnt > 0) modeRate_cnt--;
    else extRxModeRate = false;

    if (extRxModeRatePrev != extRxModeRate)
    {
      DEBUG_PRINT("Switched to level mode\n");
      extrxSetpoint.mode.roll = modeAbs;
      extrxSetpoint.mode.pitch = modeAbs;
    }
  }

  extRxModeRatePrev = extRxModeRate;
  #endif

  if (extRxModeRate)
  {
    extrxSetpoint.attitudeRate.roll = EXTRX_SIGN_ROLL * getCPPMRollRateScale() * cppmConvert2Float(ch[EXTRX_CH_ROLL], -1, 1, EXTRX_DEADBAND_ROLL);
    extrxSetpoint.attitudeRate.pitch = EXTRX_SIGN_PITCH * getCPPMPitchRateScale() * cppmConvert2Float(ch[EXTRX_CH_PITCH], -1, 1, EXTRX_DEADBAND_PITCH);
  }
  else
  {
    extrxSetpoint.attitude.roll = EXTRX_SIGN_ROLL * getCPPMRollScale() * cppmConvert2Float(ch[EXTRX_CH_ROLL], -1, 1, EXTRX_DEADBAND_ROLL);
    extrxSetpoint.attitude.pitch = EXTRX_SIGN_PITCH * getCPPMPitchScale() * cppmConvert2Float(ch[EXTRX_CH_PITCH], -1, 1, EXTRX_DEADBAND_PITCH);
  }
  extrxSetpoint.attitudeRate.yaw = EXTRX_SIGN_YAW * getCPPMYawRateScale() * cppmConvert2Float(ch[EXTRX_CH_YAW], -1, 1, EXTRX_DEADBAND_YAW);

  commanderSetSetpoint(&extrxSetpoint, COMMANDER_PRIORITY_EXTRX);
}

static void extRxDecodeCppm(void)
{
  uint16_t ppm;
  static uint8_t currChannel = 0;

  if (cppmGetTimestamp(&ppm) == pdTRUE)
  {
    if (cppmIsAvailible() && ppm < 2100)
    {
      if (currChannel < EXTRX_NR_CHANNELS)
      {
        ch[currChannel] = ppm;
      }
      currChannel++;
    }
    else
    {
      extRxDecodeChannels();
      currChannel = 0;
    }
  }
}

#if 0
static void extRxDecodeSpektrum(void)
{
  while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE)
  { // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
    for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++)
    {
      SerialRead(SPEK_SERIAL_PORT);
    }  //Toss one full frame of bytes.
  }
  if (spekFrameFlags == 0x01)
  { //The interrupt handler saw at least one valid frame start since we were last here.
    if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE)
    {  //A complete frame? If not, we'll catch it next time we are called.
      SerialRead(SPEK_SERIAL_PORT);
      SerialRead(SPEK_SERIAL_PORT);        //Eat the header bytes
      for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2)
      {
        uint8_t bh = SerialRead(SPEK_SERIAL_PORT);
        uint8_t bl = SerialRead(SPEK_SERIAL_PORT);
        uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
      if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
    }
    spekFrameFlags = 0x00;
    spekFrameData = 0x01;
#if defined(FAILSAFE)
    if(failsafeCnt > 20) failsafeCnt -= 20;
    else failsafeCnt = 0;   // Valid frame, clear FailSafe counter
#endif
  }
  else
  { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
    uint32_t spekInterval = (timer0_overflow_count << 8)
        * (64 / clockCyclesPerMicrosecond()) - spekTimeLast;
    if (spekInterval > 2500)
    {
      spekFrameFlags = 0;
    }  //If it has been a while, make the interrupt handler start over.
  }
}
#endif

/* Loggable variables */
#ifdef ENABLE_EXTRX_LOG
/**
 * External receiver (RX) log group. This contains received raw
 * channel data
 */
LOG_GROUP_START(extrx_raw)
/**
 * @brief External RX received channel 0 value
 */
LOG_ADD(LOG_UINT16, ch0, &ch[0])
/**
 * @brief External RX received channel 1 value
 */
LOG_ADD(LOG_UINT16, ch1, &ch[1])
/**
 * @brief External RX received channel 2 value
 */
LOG_ADD(LOG_UINT16, ch2, &ch[2])
/**
 * @brief External RX received channel 3 value
 */
LOG_ADD(LOG_UINT16, ch3, &ch[3])
/**
 * @brief External RX received channel 4 value
 */
LOG_ADD(LOG_UINT16, ch4, &ch[4])
/**
 * @brief External RX received channel 5 value
 */
LOG_ADD(LOG_UINT16, ch5, &ch[5])
/**
 * @brief External RX received channel 6 value
 */
LOG_ADD(LOG_UINT16, ch6, &ch[6])
/**
 * @brief External RX received channel 7 value
 */
LOG_ADD(LOG_UINT16, ch7, &ch[7])
LOG_GROUP_STOP(extrx_raw)

/**
 * External receiver (RX) log group. This contains setpoints for
 * thrust, roll, pitch, yaw rate, z velocity, and arming and
 * altitude-hold signals
 */
LOG_GROUP_START(extrx)
/**
 * @brief External RX thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &extrxSetpoint.thrust)
/**
 * @brief External RX roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll, &extrxSetpoint.attitude.roll)
/**
 * @brief External RX pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch, &extrxSetpoint.attitude.pitch)
/**
 * @brief External RX roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate, &extrxSetpoint.attitudeRate.roll)
/**
 * @brief External RX pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &extrxSetpoint.attitudeRate.pitch)
/**
 * @brief External RX yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate, &extrxSetpoint.attitudeRate.yaw)
/**
 * @brief External RX z-velocity setpoint
 */
LOG_ADD(LOG_FLOAT, zVel, &extrxSetpoint.velocity.z)
/**
 * @brief External RX Altitude Hold signal
 */
LOG_ADD(LOG_UINT8, AltHold, &extRxAltHold)
/**
 * @brief External RX Arming signal
 */
LOG_ADD(LOG_UINT8, Arm, &extRxArm)
LOG_GROUP_STOP(extrx)
#endif
