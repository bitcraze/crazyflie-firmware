/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * sitAw.h - Implementation of situation awareness.
 */

#include <stddef.h>
#include <stdlib.h>

#include "log.h"
#include "param.h"
#include "trigger.h"
#include "sitaw.h"

/* Trigger object used to detect Free Fall situation. */
static trigger_t sitAwFFAccWZ;

/* Trigger object used to detect At Rest situation. */
static trigger_t sitAwARAccZ;

#if defined(SITAW_ENABLED)

#if defined(SITAW_LOG_ENABLED) /* Enable the log group. */
LOG_GROUP_START(sitAw)
#if defined(SITAW_FF_LOG_ENABLED) /* Log trigger variables for Free Fall detection. */
LOG_ADD(LOG_UINT32, FFAccWZTestCounter, &sitAwFFAccWZ.testCounter)
LOG_ADD(LOG_UINT8, FFAccWZDetected, &sitAwFFAccWZ.released)
#endif
#if defined(SITAW_AR_LOG_ENABLED) /* Log trigger variables for At Rest detection. */
LOG_ADD(LOG_UINT32, ARTestCounter, &sitAwARAccZ.testCounter)
LOG_ADD(LOG_UINT8, ARDetected, &sitAwARAccZ.released)
#endif
#if defined(SITAW_LOG_ALL_DETECT_ENABLED) /* Log the 'Detected' flags. */
LOG_ADD(LOG_UINT8, FFAccWZDetected, &sitAwFFAccWZ.released)
LOG_ADD(LOG_UINT8, ARDetected, &sitAwARAccZ.released)
#endif
LOG_GROUP_STOP(sitAw)
#endif /* SITAW_LOG_ENABLED */

#if defined(SITAW_PARAM_ENABLED) /* Enable the param group. */
PARAM_GROUP_START(sitAw)
#if defined(SITAW_FF_PARAM_ENABLED) /* Param variables for Free Fall detection. */
PARAM_ADD(PARAM_UINT8, FFActive, &sitAwFFAccWZ.active)
PARAM_ADD(PARAM_UINT32, FFTriggerCount, &sitAwFFAccWZ.triggerCount)
PARAM_ADD(PARAM_FLOAT, FFaccWZ, &sitAwFFAccWZ.threshold)
#endif
#if defined(SITAW_AR_PARAM_ENABLED) /* Param variables for At Rest detection. */
PARAM_ADD(PARAM_UINT8, ARActive, &sitAwARAccZ.active)
PARAM_ADD(PARAM_UINT32, ARTriggerCount, &sitAwARAccZ.triggerCount)
PARAM_ADD(PARAM_FLOAT, ARaccZ, &sitAwARAccZ.threshold)
#endif
PARAM_GROUP_STOP(sitAw)
#endif /* SITAW_PARAM_ENABLED */

#endif /* SITAW_ENABLED */

/**
 * abs() equivalent function for floating point numbers.
 *
 * @param x The floating point number to return the absolute value for.
 *
 * @return The absolute value of x.
 */
float sitAwFAbs(float x)
{
  return (x < 0) ? (-x) : x;
}

/**
 * Initialize the Free Fall detection.
 *
 * See the sitAwFFDetect() function for details.
 */
void sitAwFFInit(void)
{
  triggerInit(&sitAwFFAccWZ, triggerFuncIsLE, SITAW_FF_THRESHOLD, SITAW_FF_TRIGGER_COUNT);
  triggerActivate(&sitAwFFAccWZ, true);
}

/**
 * Check for a Free Fall situation.
 *
 * A free fall situation is considered identified when the vertical
 * acceleration of the crazyflie (regardless of orientation - given by
 * AccWZ) is approaching -1 (AccWZ is 0 when crazyflie is at rest). We
 * will look for when AccWZ is within SITAW_FF_THRESHOLD of -1.
 *
 * At the same time, there are should be no other accelerations experienced
 * by the crazyflie.

 * This can be checked by looking at the accMAG (total acceleration). If
 * the accMAG is approaching 0, there are no other accelerations than accWZ.
 * This helps to distinguish free fall situations from other other movements
 * such as shaking.
 *
 * @param accWZ  Vertical acceleration (regardless of orientation)
 * @param accMAG All experienced accelerations.
 *
 * @return True if a Free Fall situation is being identified, otherwise false.
 */
bool sitAwFFDetect(float accWZ, float accMAG)
{
  /* Check that the total acceleration is close to zero. */
  if(sitAwFAbs(accMAG) > SITAW_FF_THRESHOLD) {
    /* If the total acceleration deviates from 0, this is not a free fall situation. */
    triggerReset(&sitAwFFAccWZ);
    return false;
  }

  /**
   * AccWZ approaches -1 in free fall. Check that the value stays close to -1 for
   * the triggerCount specified.
   */
  return(triggerTestValue(&sitAwFFAccWZ, sitAwFAbs(accWZ + 1)));
}

/**
 * Initialize the At Rest detection.
 *
 * See the sitAwARDetect() function for details.
 */
void sitAwARInit(void)
{
  triggerInit(&sitAwARAccZ, triggerFuncIsLE, SITAW_AR_THRESHOLD, SITAW_AR_TRIGGER_COUNT);
  triggerActivate(&sitAwARAccZ, true);
}

/**
 * Check for an At Rest situation.
 *
 * An At Rest situation is considered identified when the crazyflie is
 * placed on its feet (accZ = 1) and with no horizontal accelerations
 * (accX = accY = 0).
 *
 * Since there is always some minor noise in the measurements, we use
 * a margin of SITAW_AR_THRESHOLD from the ideal values.
 *
 * @param accX   Horizontal acceleration (when crazyflie is placed on its feet)
 * @param accY   Horizontal acceleration (when crazyflie is placed on its feet)
 * @param accZ   Vertical acceleration (when crazyflie is placed on its feet)
 *
 * @return True if an At Rest situation is being identified, otherwise false.
 */
bool sitAwARDetect(float accX, float accY, float accZ)
{
  /* Check that there are no horizontal accelerations. At rest, these are 0. */
  if((sitAwFAbs(accX) > SITAW_AR_THRESHOLD) || (sitAwFAbs(accY) > SITAW_AR_THRESHOLD)) {
    /* If the X or Y accelerations are different than 0, the crazyflie is not at rest. */
    triggerReset(&sitAwARAccZ);
    return(false);
  }

  /**
   * If the test above indicates that there are no horizontal movements, test the
   * vertical acceleration value against the trigger.
   *
   * The vertical acceleration must be close to 1, but is allowed to oscillate slightly
   * around 1. Testing that the deviation from 1 stays within SITAW_AR_THRESHOLD.
   */
  return(triggerTestValue(&sitAwARAccZ, sitAwFAbs(accZ - 1)));
}

/**
 * Initialize the situation awareness subsystem.
 */
void sitAwInit(void)
{
  sitAwFFInit();
  sitAwARInit();
}
