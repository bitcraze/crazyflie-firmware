/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * out_of_tree_onboard_guidance.c - Example out-of-tree onboard guidance.
 *
 * This example delegates all functions to the built-in High-Level Commander,
 * effectively wrapping it. Replace the function bodies with your own guidance
 * logic (e.g. neural-network navigation, behavior trees, custom planners).
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "OOTGUIDANCE"
#include "debug.h"

#include "onboard_guidance.h"
#include "crtp_commander_high_level.h"

void appMain() {
  DEBUG_PRINT("OOT onboard guidance example started\n");

  while (1) {
    vTaskDelay(M2T(2000));
  }
}

// Called once at startup. Initialize your guidance state here.
void onboardGuidanceOutOfTreeInit(void) {
  crtpCommanderHighLevelInit();
  DEBUG_PRINT("Out-of-tree onboard guidance initialized\n");
}

// Called once after init. Return true if initialization succeeded.
bool onboardGuidanceOutOfTreeTest(void) {
  return true;
}

// Called every stabilizer loop iteration (~1 kHz).
// Write the desired setpoint and return true if a setpoint was produced.
bool onboardGuidanceOutOfTreeGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep) {
  return crtpCommanderHighLevelGetSetpoint(setpoint, state, stabilizerStep);
}

// Called when a higher-priority setpoint source takes over.
void onboardGuidanceOutOfTreeStop(void) {
  crtpCommanderHighLevelStop();
}

// Called when onboard guidance is re-enabled after a higher-priority source,
// to provide a current state snapshot. Not called periodically.
void onboardGuidanceOutOfTreeTellState(const state_t *state) {
  crtpCommanderHighLevelTellState(state);
}

// Called every stabilizer loop iteration. When doBlock is true, the guidance
// must not produce setpoints (motors are not allowed to run). Critical for safety.
void onboardGuidanceOutOfTreeBlock(bool doBlock) {
  crtpCommanderBlock(doBlock);
}

// Return true if the guidance has finished its current task (e.g. trajectory completed).
bool onboardGuidanceOutOfTreeIsDone(void) {
  return crtpCommanderHighLevelIsTrajectoryFinished();
}
