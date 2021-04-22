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

typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;

// States of the outer state machine (Locking and onlocking)
static State state = idle;

// States of the inner state machine (Wall following: wallfollowing_multiranger.c)
static int state_wf = 0;

// Handling the unlocking procedure of the top sensor of the multiranger
static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

// Handling the height setpoint
static const float height_sp = 0.5f;
static const uint16_t radius = 300;

// Some wallfollowing parameters and logging
bool goLeft = false;
float distance_to_wall = 0.5f;
float max_speed = 0.5f;
float init_wf_state = 1; // start with going forward

float vel_x_cmd = 0.0f;
float vel_y_cmd = 0.0f;
float vel_w_cmd = 0.0f;

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

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
  wall_follower_init(distance_to_wall, max_speed, init_wf_state);

  // Intialize the setpoint structure
  static setpoint_t setpoint;

  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(10));
    
    // Check if decks are properly mounted
    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    // Get the upper range sensor value (used for startup and stopping)
    uint16_t up = logGetUint(idUp);

    // Get Height estimate
    float heightEstimate = logGetFloat(idHeightEstimate);

    // If the crazyflie is unlocked by the hand, continue with state machine
    if (state == unlocked) {

      // Get all multiranger values
      float front_range = (float)logGetUint(idFront) / 1000.0f;
      float side_range;
      if(goLeft)
        side_range = (float)logGetUint(idRight) / 1000.0f;
      else
        side_range = (float)logGetUint(idLeft) / 1000.0f;

      // Get the heading and convert it to rad
      float heading_deg = logGetFloat(idStabilizerYaw);
      float heading_rad = heading_deg * (float)M_PI / 180.0f;

      //Adjust height based on up ranger input
      uint16_t up_o = radius - MIN(up, radius);
      float height_cmd = height_sp - up_o/1000.0f;

      vel_x_cmd = 0.0f;
      vel_y_cmd = 0.0f;
      vel_w_cmd = 0.0f;
      float vel_w_cmd_convert=0.0f;

      // Only go to the state machine if the crazyflie has reached a certain height
      if (heightEstimate > height_sp - 0.1f)
      {
        // Set the wall following direction
        int direction;
        if(goLeft) direction = 1; else direction = -1;

        // The wall-following state machine which outputs velocity commands
        state_wf = wall_follower(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, side_range, heading_rad, direction);
        vel_w_cmd_convert = vel_w_cmd * 180.0f / (float)M_PI;
        //DEBUG_PRINT("state %i\n",state_wf);

      }
      // Turn velocity commands into setpoints and send it to the commander
      setVelocitySetpoint(&setpoint, vel_x_cmd, vel_y_cmd, height_cmd, vel_w_cmd_convert);
      commanderSetSetpoint(&setpoint, 3);

      // Handling stopping with hand above the crazyflie
      if (height_cmd < height_sp - 0.2f) {
        state = stopping;
        DEBUG_PRINT("X\n");
      }

    } else {
      
      // Handeling locking and unlocking


      if (state == stopping && up > stoppedTh) {
        DEBUG_PRINT("%i", up);
        state = idle;
        DEBUG_PRINT("S\n");
      }

      // If the up multiranger is activated for the first time, prepare to be unlocked
      if (up < unlockThLow && state == idle && up > 0.001f) {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        state = lowUnlock;
      }
      
      // Unlock CF if hand above is removed, and if the positioningdeckand multiranger deck is initalized.
      if (up > unlockThHigh && state == lowUnlock && positioningInit && multirangerInit) {
        DEBUG_PRINT("Unlocked!\n");
        state = unlocked;
      }
      
      // Stop the crazyflie with idle or stopping state
      if (state == idle || state == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}


PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8, goLeft, &goLeft)
PARAM_ADD(PARAM_FLOAT, distanceWall, &distance_to_wall)
PARAM_ADD(PARAM_FLOAT, maxSpeed,  &max_speed)
PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT, vel_x_cmd, &vel_x_cmd)
LOG_ADD(LOG_FLOAT, vel_y_cmd, &vel_y_cmd)
LOG_ADD(LOG_FLOAT, vel_w_cmd, &vel_w_cmd)
LOG_ADD(LOG_UINT8, state_wf, &state_wf)
LOG_ADD(LOG_UINT8, state, &state)
LOG_GROUP_STOP(app)
