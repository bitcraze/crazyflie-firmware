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

static PidObject pid;

static float estimatedZ = 0.0; // The current Z estimate, has same offset as asl
static float estAlpha = 0.99;
static float targetZ          = -1;    // Target altitude
static float targetChangeSens = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float velocityFactor = 1.0;

static float pidInitKp = 30000.0;
static float pidInitKi = 0.0;
static float pidInitKd = 10000.0;

static uint16_t thrustBase    = 32000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static float thrust;      // The thrust from regulator, expressed as an offset to thrustBase


#if defined(SITAW_ENABLED)
// Automatic take-off variables
static bool autoTOActive           = false; // Flag indicating if automatic take-off is active / deactive.
static float autoTOAltBase         = 0.0f;  // Base altitude for the automatic take-off. Set to targetZ when automatic take-off is activated.
static float autoTOAltCurrent      = 0.0f;  // Current target altitude adjustment. Equals 0 when function is activated, increases to autoTOThresh when function is deactivated.
// Automatic take-off parameters
static float autoTOAlpha           = 0.98f; // Smoothing factor when adjusting the targetZ altitude.
static float autoTOTargetAdjust    = 1.5f;  // Meters to add to targetZ to reach auto take-off altitude.
static float autoTOThresh          = 0.97f; // Threshold for when to deactivate auto Take-Off. A value of 0.97 means 97% of the target altitude adjustment.
#endif


static void stabilizerPreAltHoldComputeThrustCallOut(bool setAltHold);
static void resetTarget(float newTargetZ, float dt);
static void updateTargetForPilotControl(float altHoldChange);

void altHoldUpdate(uint16_t* actuatorThrust, float asl, float velocityZ, float dt) {
  bool isActive = false;
  bool justActivated = false;
  float pilotChange = 0;

  // Get altitude hold commands from pilot
  commanderGetAltHold(&isActive, &justActivated, &pilotChange);

   // Calculate new estimate. LP measurement from baro and add delta based on speed from accelerometers
  estimatedZ = estAlpha * estimatedZ + (1.0 - estAlpha) * asl +
    velocityFactor * velocityZ * dt;

  // Altitude hold mode just activated
  if (justActivated) {
    resetTarget(estimatedZ, dt);
  }

  // Call out before performing altHold thrust regulation.
  stabilizerPreAltHoldComputeThrustCallOut(justActivated);

  if (isActive) {
    updateTargetForPilotControl(pilotChange);

    thrust = pidUpdate(&pid, estimatedZ, true);
    *actuatorThrust = limitUint16(thrustBase + thrust);
  }
}

static void resetTarget(float newTargetZ, float dt) {
  targetZ = newTargetZ;
  pidInit(&pid, targetZ, pidInitKp, pidInitKi, pidInitKd, dt);
}

static void updateTargetForPilotControl(float pilotChange) {
  float delta = pilotChange / targetChangeSens;
  targetZ += delta;
  pidSetDesired(&pid, targetZ);
}

static void stabilizerPreAltHoldComputeThrustCallOut(bool justActivated) {
  /* Code that shall run BEFORE each altHold thrust computation, should be placed here. */

#if defined(SITAW_ENABLED)
  /*
   * The number of variables used for automatic Take-Off could be reduced, however that would
   * cause debugging and tuning to become more difficult. The variables currently used ensure
   * that tuning can easily be done through the LOG and PARAM frameworks.
   *
   * Note that while the automatic take-off function is active, it will overrule any other
   * changes to targetZ by the user.
   *
   * The automatic take-off function will automatically deactivate once the take-off has been
   * conducted.
   */
  if(!autoTOActive) {
    /*
     * Enabling automatic take-off: When At Rest, Not Tumbled, and the user pressing the AltHold button
     */
    if(sitAwARDetected() && !sitAwTuDetected() && justActivated) {
      /* Enable automatic take-off. */
      autoTOActive = true;
      autoTOAltBase = targetZ;
      autoTOAltCurrent = 0.0f;
    }
  }

  if(autoTOActive) {
    /*
     * Automatic take-off is quite simple: Slowly increase targetZ until reaching the target altitude.
     */

    /* Calculate the new current setpoint for targetZ. autoTOAltCurrent is normalized to values from 0 to 1. */
    autoTOAltCurrent = autoTOAltCurrent * autoTOAlpha + (1 - autoTOAlpha);

    /* Update the targetZ variable. */
    targetZ = autoTOAltBase + autoTOAltCurrent * autoTOTargetAdjust;

    if((autoTOAltCurrent >= autoTOThresh)) {
      /* Disable the automatic take-off mode if target altitude has been reached. */
      autoTOActive = false;
      autoTOAltBase = 0.0f;
      autoTOAltCurrent = 0.0f;
    }
  }
#endif
}


LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, p, &pid.outP)
LOG_ADD(LOG_FLOAT, i, &pid.outI)
LOG_ADD(LOG_FLOAT, d, &pid.outD)
LOG_ADD(LOG_FLOAT, thrust, &thrust)

LOG_ADD(LOG_FLOAT, targetZ, &targetZ)
LOG_ADD(LOG_FLOAT, estimatedZ, &estimatedZ)
LOG_GROUP_STOP(altHold)

PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, targetChangeSens, &targetChangeSens)
PARAM_ADD(PARAM_FLOAT, pidInitKp, &pidInitKp)
PARAM_ADD(PARAM_FLOAT, pidInitKi, &pidInitKi)
PARAM_ADD(PARAM_FLOAT, pidInitKd, &pidInitKd)
PARAM_ADD(PARAM_UINT16, thrustBase, &thrustBase)
PARAM_ADD(PARAM_FLOAT, estAlpha, &estAlpha)
PARAM_ADD(PARAM_FLOAT, velocityFactor, &velocityFactor)
PARAM_GROUP_STOP(altHold)


#if defined(SITAW_ENABLED)
// Automatic take-off logs
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
