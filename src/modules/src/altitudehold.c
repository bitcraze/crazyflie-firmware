/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 */
#include <math.h>

#include "FreeRTOS.h"
#include "pm.h"
#include "commander.h"
#include "imu.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"

// 500hz / 5 = 100hz
#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))

static uint32_t altHoldCounter = 0;

static PidObject altHoldPID;  // Used for altitute hold mode. I gets reset when the bat status changes
static float vSpeedASL = 0.0; // Vertical speed (world frame) derived from barometer ASL
static float vSpeedAcc = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;   // Output of the PID controller
static float altHoldErr;      // Different between target and current altitude

static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.98; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust

// Baro variables
static float temperature; // temp from barometer in celcius
static float pressure;    // pressure from barometer in bar
static float asl;         // smoothed asl
static float aslRaw;      // raw asl
static float aslLong;     // long term asl
static float aslRef;      // asl reference (ie. offset)


#if defined(SITAW_ENABLED)
// Automatic take-off variables
static bool autoTOActive           = false; // Flag indicating if automatic take-off is active / deactive.
static float autoTOAltBase         = 0.0f;  // Base altitude for the automatic take-off. Set to altHoldTarget when automatic take-off is activated.
static float autoTOAltCurrent      = 0.0f;  // Current target altitude adjustment. Equals 0 when function is activated, increases to autoTOThresh when function is deactivated.
// Automatic take-off parameters
static float autoTOAlpha           = 0.98f; // Smoothing factor when adjusting the altHoldTarget altitude.
static float autoTOTargetAdjust    = 1.5f;  // Meters to add to altHoldTarget to reach auto take-off altitude.
static float autoTOThresh          = 0.97f; // Threshold for when to deactivate auto Take-Off. A value of 0.97 means 97% of the target altitude adjustment.
#endif


static void stabilizerAltHoldIterate(uint16_t* actuatorThrust, float vSpeed);
static void stabilizerPreAltHoldComputeThrustCallOut(bool setAltHold);


void stabilizerAltHoldUpdate(uint16_t* actuatorThrust, float vSpeed)
{
  // Called at 500 Hz
  if (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER)
  {
    // 100 Hz
    stabilizerAltHoldIterate(actuatorThrust, vSpeed);
    altHoldCounter = 0;
  }
}

static void stabilizerAltHoldIterate(uint16_t* actuatorThrust, float vSpeed)
{
  bool altHold = false;
  bool setAltHold = false;      // Hover mode has just been activated
  float altHoldChange = 0;      // Change in target altitude

  // Get altitude hold commands from pilot
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);

  // Get barometer height estimates
  //TODO do the smoothing within getData
#ifdef PLATFORM_CF1
  ms5611GetData(&pressure, &temperature, &aslRaw);
#else
  lps25hGetData(&pressure, &temperature, &aslRaw);
#endif

  aslRaw -= aslRef;

  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeedAcc = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);

  // Reset Integral gain of PID controller if being charged
  if (!pmIsDischarging())
  {
    altHoldPID.integ = 0.0;
  }

  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Cache last integral term for reuse after pid init
    const float pre_integral = altHoldPID.integ;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /

    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
  }

  /* Call out before performing altHold thrust regulation. */
  stabilizerPreAltHoldComputeThrustCallOut(setAltHold);

  // In altitude hold mode
  if (altHold)
  {
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));

    // compute new thrust
    *actuatorThrust = max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitUint16( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));

    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0;
    altHoldErr = 0.0;
    altHoldPIDVal = 0.0;
  }
}


static void stabilizerPreAltHoldComputeThrustCallOut(bool setAltHold)
{
  /* Code that shall run BEFORE each altHold thrust computation, should be placed here. */

#if defined(SITAW_ENABLED)
  /*
   * The number of variables used for automatic Take-Off could be reduced, however that would
   * cause debugging and tuning to become more difficult. The variables currently used ensure
   * that tuning can easily be done through the LOG and PARAM frameworks.
   *
   * Note that while the automatic take-off function is active, it will overrule any other
   * changes to altHoldTarget by the user.
   *
   * The automatic take-off function will automatically deactivate once the take-off has been
   * conducted.
   */
  if(!autoTOActive){
    /*
     * Enabling automatic take-off: When At Rest, Not Tumbled, and the user pressing the AltHold button
     */
    if(sitAwARDetected() && !sitAwTuDetected() && setAltHold) {
      /* Enable automatic take-off. */
      autoTOActive = true;
      autoTOAltBase = altHoldTarget;
      autoTOAltCurrent = 0.0f;
    }
  }

  if(autoTOActive) {
    /*
     * Automatic take-off is quite simple: Slowly increase altHoldTarget until reaching the target altitude.
     */

    /* Calculate the new current setpoint for altHoldTarget. autoTOAltCurrent is normalized to values from 0 to 1. */
    autoTOAltCurrent = autoTOAltCurrent * autoTOAlpha + (1 - autoTOAlpha);

    /* Update the altHoldTarget variable. */
    altHoldTarget = autoTOAltBase + autoTOAltCurrent * autoTOTargetAdjust;

    if((autoTOAltCurrent >= autoTOThresh)) {
      /* Disable the automatic take-off mode if target altitude has been reached. */
      autoTOActive = false;
      autoTOAltBase = 0.0f;
      autoTOAltCurrent = 0.0f;
    }
  }
#endif
}


LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)



PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, aslRef, &aslRef)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

#if defined(SITAW_ENABLED)
// Automatic take-off parameters
LOG_GROUP_START(autoTO)
LOG_ADD(LOG_UINT8, Active, &autoTOActive)
LOG_ADD(LOG_FLOAT, AltBase, &autoTOAltBase)
LOG_ADD(LOG_FLOAT, AltCurrent, &autoTOAltCurrent)
LOG_GROUP_STOP(autoTO)

// Automatic take-off parameters
PARAM_GROUP_START(autoTO)
PARAM_ADD(PARAM_FLOAT, TargetAdjust, &autoTOTargetAdjust)
PARAM_ADD(PARAM_FLOAT, Thresh, &autoTOThresh)
PARAM_ADD(PARAM_FLOAT, Alpha, &autoTOAlpha)
PARAM_GROUP_STOP(autoTO)
#endif
