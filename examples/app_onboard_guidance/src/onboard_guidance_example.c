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
 * onboard_guidance_example.c - Example out-of-tree onboard guidance.
 *
 * This example delegates to the high-level commander for trajectory
 * planning. Replace the body of onboardGuidanceOutOfTreeGetSetpoint()
 * with your own guidance logic (e.g. neural-network navigation,
 * behavior trees, custom planners).
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

void onboardGuidanceOutOfTreeInit(void) {
  DEBUG_PRINT("Out-of-tree onboard guidance initialized\n");
}

bool onboardGuidanceOutOfTreeTest(void) {
  return true;
}

bool onboardGuidanceOutOfTreeGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep) {
  // Delegate to the high-level commander's trajectory planner.
  // Replace this with your own guidance logic.
  return crtpCommanderHighLevelGetSetpoint(setpoint, state, stabilizerStep);
}

void onboardGuidanceOutOfTreeStop(void) {
  crtpCommanderHighLevelStop();
}

void onboardGuidanceOutOfTreeTellState(const state_t *state) {
  crtpCommanderHighLevelTellState(state);
}
