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
 * position_estimator_altitude.c: Altitude-only position estimator
 */

#define DEBUG_MODULE "ALTHOLD"
#include "debug.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "sitaw.h"
#include "altitude_hold.h"

struct state_s {
  float targetZ;    // Target altitude
  float targetChangeSens;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes

  bool isActive;
  bool wasActive;
  bool justActivated;
  float pilotChange;

  #if defined(SITAW_ENABLED)
  // Automatic take-off variables
  bool autoTOActive; // Flag indicating if automatic take-off is active / deactive.
  float autoTOAltBase;  // Base altitude for the automatic take-off. Set to targetZ when automatic take-off is activated.
  float autoTOAltCurrent;  // Current target altitude adjustment. Equals 0 when function is activated, increases to autoTOThresh when function is deactivated.
  // Automatic take-off parameters
  float autoTOAlpha; // Smoothing factor when adjusting the targetZ altitude.
  float autoTOTargetAdjust;  // Meters to add to targetZ to reach auto take-off altitude.
  float autoTOThresh; // Threshold for when to deactivate auto Take-Off. A value of 0.97 means 97% of the target altitude adjustment.
  #endif
};

static struct state_s state = {
  .targetZ = -1,
  .targetChangeSens = 200,

  .isActive = false,
  .wasActive = false,
  .justActivated = false,
  .pilotChange = 0.0,

  #if defined(SITAW_ENABLED)
  // Automatic take-off variables
  .autoTOActive          = false,
  .autoTOAltBase         = 0.0f,
  .autoTOAltCurrent      = 0.0f,

  .autoTOAlpha           = 0.98f,
  .autoTOTargetAdjust    = 1.5f,
  .autoTOThresh          = 0.97f,
  #endif
};

static float preAltHoldComputeZCallOut(struct state_s* state);
static bool altHoldIsActiveInternal(struct state_s* state);
static void altHoldGetNewSetPointInternal(setpointZ_t* setpoint, const estimate_t* estimate, struct state_s* state);

bool altHoldIsActive() {
  return altHoldIsActiveInternal(&state);
}

void altHoldGetNewSetPoint(setpointZ_t* setpoint, const estimate_t* estimate) {
  altHoldGetNewSetPointInternal(setpoint, estimate, &state);
}

static bool altHoldIsActiveInternal(struct state_s* state) {
  // Get altitude hold commands from pilot
  commanderGetAltHold(&state->isActive, &state->pilotChange);

  state->justActivated = !state->wasActive && state->isActive;
  state->wasActive = state->isActive;

  return state->isActive;
}

static void altHoldGetNewSetPointInternal(setpointZ_t* setpoint, const estimate_t* estimate, struct state_s* state) {
  if (state->justActivated) {
    setpoint->isUpdate = false;

    state->targetZ = estimate->position.z;
  } else {
    setpoint->isUpdate = true;

    float delta = state->pilotChange / state->targetChangeSens;
    state->targetZ += delta;
  }

  state->targetZ = preAltHoldComputeZCallOut(state);

  setpoint->z = state->targetZ;
}

static float preAltHoldComputeZCallOut(struct state_s* state) {
  float result = state->targetZ;
  /* Code that shall run BEFORE each altHold z target computation, should be placed here. */

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
  if(!state->autoTOActive) {
    /*
     * Enabling automatic take-off: When At Rest, Not Tumbled, and the user pressing the AltHold button
     */
    if(sitAwARDetected() && !sitAwTuDetected() && state->justActivated) {
      /* Enable automatic take-off. */
      state->autoTOActive = true;
      state->autoTOAltBase = state->targetZ;
      state->autoTOAltCurrent = 0.0f;
    }
  }

  if(state->autoTOActive) {
    /*
     * Automatic take-off is quite simple: Slowly increase targetZ until reaching the target altitude.
     */

    /* Calculate the new current setpoint for targetZ. autoTOAltCurrent is normalized to values from 0 to 1. */
    state->autoTOAltCurrent = state->autoTOAltCurrent * state->autoTOAlpha + (1 - state->autoTOAlpha);

    /* Update the targetZ variable. */
    result = state->autoTOAltBase + state->autoTOAltCurrent * state->autoTOTargetAdjust;

    if((state->autoTOAltCurrent >= state->autoTOThresh)) {
      /* Disable the automatic take-off mode if target altitude has been reached. */
      state->autoTOActive = false;
      state->autoTOAltBase = 0.0f;
      state->autoTOAltCurrent = 0.0f;
    }
  }
#endif

  return result;
}


LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, targetZ, &state.targetZ)
LOG_GROUP_STOP(altHold)

PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, targetChangeSens, &state.targetChangeSens)
PARAM_GROUP_STOP(altHold)


#if defined(SITAW_ENABLED)
// Automatic take-off logs
LOG_GROUP_START(autoTO)
LOG_ADD(LOG_UINT8, Active, &state.autoTOActive)
LOG_ADD(LOG_FLOAT, AltBase, &state.autoTOAltBase)
LOG_ADD(LOG_FLOAT, AltCurrent, &state.autoTOAltCurrent)
LOG_GROUP_STOP(autoTO)

// Automatic take-off parameters
PARAM_GROUP_START(autoTO)
PARAM_ADD(PARAM_FLOAT, TargetAdjust, &state.autoTOTargetAdjust)
PARAM_ADD(PARAM_FLOAT, Thresh, &state.autoTOThresh)
PARAM_ADD(PARAM_FLOAT, Alpha, &state.autoTOAlpha)
PARAM_GROUP_STOP(autoTO)
#endif
