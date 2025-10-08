/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"
#include "crtp_commander_high_level.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#define DEBUG_MODULE "COMSW"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

typedef enum {
  commander,
  highlevelcommander,
} State;

static State state = commander;

static float height_sp = 0.2f;

void appMain()
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));
  DEBUG_PRINT("Starting commander switch demo\n");

  TickType_t start_time = xTaskGetTickCount();

  while(1) {
    TickType_t elapsed = T2M(xTaskGetTickCount() - start_time);

    if (state == commander) {
      setHoverSetpoint(&setpoint, 0, 0, height_sp, 0);
      commanderSetSetpoint(&setpoint, 3); // Priority on 3 like you said
      if (elapsed > 5000) {
        vTaskDelay(M2T(2)); // Wait 2ms for stabilizer to update lastState
        DEBUG_PRINT("Switching to high level commander\n");
        state = highlevelcommander;
        commanderRelaxPriority();
        // vTaskDelay(M2T(10)); // Give HLC time to update state
        crtpCommanderHighLevelGoTo2(0, 0, height_sp, 0, 2.0f, false, true);
      }
      vTaskDelay(M2T(10));
    } else if (state == highlevelcommander) {
      vTaskDelay(M2T(100));

      if (elapsed > 10000) {
        DEBUG_PRINT("Landing\n");
        crtpCommanderHighLevelLand(0.02f, 3.0f);
        vTaskDelay(M2T(4000));
        break; // Exit after landing
      }
    }
  }
}
