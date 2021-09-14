/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 * wall_follower.c - App layer application of the wall following demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 *
 * The same wallfollowing strategy was used in the following paper:

 @article{mcguire2019minimal,
  title={Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment},
  author={McGuire, KN and De Wagter, Christophe and Tuyls, Karl and Kappen, HJ and de Croon, Guido CHE},
  journal={Science Robotics},
  volume={4},
  number={35},
  year={2019},
  publisher={Science Robotics}
}

 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include <math.h>
#include "usec_time.h"

#include "wallfollowing_multiranger_onboard.h"

#define DEBUG_MODULE "WALLFOLLOWING"

static void setVelocitySetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
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

// States
typedef enum
{
  idle,
  lowUnlock,
  unlocked,
  stopping
} StateOuterLoop;

StateOuterLoop stateOuterLoop = idle;
StateWF stateInnerLoop = forward;

// Thresholds for the unlocking procedure of the top sensor of the multiranger
static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

// Handling the height setpoint
static const float spHeight = 0.5f;
static const uint16_t radius = 300;

// Some wallfollowing parameters and logging
bool goLeft = false;
float distanceToWall = 0.5f;
float maxForwardSpeed = 0.5f;

float cmdVelX = 0.0f;
float cmdVelY = 0.0f;
float cmdAngWRad = 0.0f;
float cmdAngWDeg = 0.0f;

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)

void appMain()
{
  vTaskDelay(M2T(3000));
  // Getting Logging IDs of the multiranger
  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");

  // Getting the Logging IDs of the state estimates
  logVarId_t idStabilizerYaw = logGetVarId("stabilizer", "yaw");
  logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");

  // Getting Param IDs of the deck driver initialization
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

  // Initialize the wall follower state machine
  wallFollowerInit(distanceToWall, maxForwardSpeed, stateInnerLoop);

  // Intialize the setpoint structure
  setpoint_t setpoint;

  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(10));

    // Check if decks are properly mounted
    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    // Get the upper range sensor value (used for startup and stopping)
    uint16_t up = logGetUint(idUp);

    // Get Height estimate
    float heightEstimate = logGetFloat(idHeightEstimate);

    // If the crazyflie is unlocked by the hand, continue with state machine
    if (stateOuterLoop == unlocked)
    {

      // Get all multiranger values
      float frontRange = (float)logGetUint(idFront) / 1000.0f;
      float sideRange;
      if (goLeft)
      {
        sideRange = (float)logGetUint(idRight) / 1000.0f;
      }
      else
      {
        sideRange = (float)logGetUint(idLeft) / 1000.0f;
      }

      // Get the heading and convert it to rad
      float estYawDeg = logGetFloat(idStabilizerYaw);
      float estYawRad = estYawDeg * (float)M_PI / 180.0f;

      //Adjust height based on up ranger input
      uint16_t up_o = radius - MIN(up, radius);
      float cmdHeight = spHeight - up_o / 1000.0f;

      cmdVelX = 0.0f;
      cmdVelY = 0.0f;
      cmdAngWRad = 0.0f;
      cmdAngWDeg = 0.0f;

      // Only go to the state machine if the crazyflie has reached a certain height
      if (heightEstimate > spHeight - 0.1f)
      {
        // Set the wall following direction
        int direction;
        if (goLeft)
        {
          direction = 1;
        }
        else
        {
          direction = -1;
        }

        // The wall-following state machine which outputs velocity commands
        float timeNow = usecTimestamp() / 1e6;
        stateInnerLoop = wallFollower(&cmdVelX, &cmdVelY, &cmdAngWRad, frontRange, sideRange, estYawRad, direction, timeNow);
        cmdAngWDeg = cmdAngWRad * 180.0f / (float)M_PI;
      }
      // Turn velocity commands into setpoints and send it to the commander
      setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdAngWDeg);
      commanderSetSetpoint(&setpoint, 3);

      // Handling stopping with hand above the crazyflie
      if (cmdHeight < spHeight - 0.2f)
      {
        stateOuterLoop = stopping;
        DEBUG_PRINT("X\n");
      }
    }
    else
    {

      // Handeling locking and unlocking
      if (stateOuterLoop == stopping && up > stoppedTh)
      {
        DEBUG_PRINT("%i", up);
        stateOuterLoop = idle;
        DEBUG_PRINT("S\n");
      }

      // If the up multiranger is activated for the first time, prepare to be unlocked
      if (up < unlockThLow && stateOuterLoop == idle && up > 0.001f)
      {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        stateOuterLoop = lowUnlock;
      }

      // Unlock CF if hand above is removed, and if the positioningdeckand multiranger deck is initalized.
      if (up > unlockThHigh && stateOuterLoop == lowUnlock && positioningInit && multirangerInit)
      {
        DEBUG_PRINT("Unlocked!\n");
        stateOuterLoop = unlocked;
      }

      // Stop the crazyflie with idle or stopping state
      if (stateOuterLoop == idle || stateOuterLoop == stopping)
      {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}

PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8, goLeft, &goLeft)
PARAM_ADD(PARAM_FLOAT, distanceWall, &distanceToWall)
PARAM_ADD(PARAM_FLOAT, maxSpeed, &maxForwardSpeed)
PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT, cmdVelX, &cmdVelX)
LOG_ADD(LOG_FLOAT, cmdVelY, &cmdVelY)
LOG_ADD(LOG_FLOAT, cmdAngWRad, &cmdAngWRad)
LOG_ADD(LOG_UINT8, stateInnerLoop, &stateInnerLoop)
LOG_ADD(LOG_UINT8, stateOuterLoop, &stateOuterLoop)
LOG_GROUP_STOP(app)
