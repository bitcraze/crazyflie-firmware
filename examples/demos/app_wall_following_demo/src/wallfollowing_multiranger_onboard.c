/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire

The same wallfollowing strategy was used in the following paper:

 @article{mcguire2019minimal,
  title={Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment},
  author={McGuire, KN and De Wagter, Christophe and Tuyls, Karl and Kappen, HJ and de Croon, Guido CHE},
  journal={Science Robotics},
  volume={4},
  number={35},
  year={2019},
  publisher={Science Robotics}

 */

#include "wallfollowing_multiranger_onboard.h"
#include <math.h>
#include "usec_time.h"

// variables
static float refDistanceFromWall = 0.0f;
static float maxForwardSpeed = 0.2f;
static float maxTurnRate = 0.5f;
static float direction = 1.0f;
static float firstRun = false;
static float prevHeading = 0.0f;
static float wallAngle = 0.0f;
static bool aroundCornerBackTrack = false;
static float stateStartTime;

static StateWF stateWF = forward;

void wallFollowerInit(float refDistanceFromWallNew, float maxForwardSpeed_ref, StateWF initState)
{
  refDistanceFromWall = refDistanceFromWallNew;
  maxForwardSpeed = maxForwardSpeed_ref;
  firstRun = true;
  stateWF= initState;
}

// Static helper functions
static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
  if (real_value > checked_value - margin && real_value < checked_value + margin) {
    return true;
  } else {
    return false;
  }
}

static float wraptopi(float number)
{
  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return (number);
  }
}

// Static command functions
static void commandTurn(float *cmdVelX, float *cmdAngW, float ref_rate)
{
  *cmdVelX = 0.0f;
  *cmdAngW = direction * ref_rate;

}

static void commandAlignCorner(float *cmdVelY, float *cmdAngW, float ref_rate, float range,
                               float wanted_distance_from_corner)
{

  if (range > wanted_distance_from_corner + 0.3f) {
    *cmdAngW = direction * ref_rate;
    *cmdVelY = 0;

  } else {
    if (range > wanted_distance_from_corner) {
      *cmdVelY = direction * (-1.0f * maxForwardSpeed / 3.0f);
    } else {
      *cmdVelY = direction * (maxForwardSpeed / 3.9f);
    }
    *cmdAngW = 0;
  }
}

static void commandHover(float *cmdVelX, float *cmdVelY, float *cmdAngW)
{
  *cmdVelX = 0.0f;
  *cmdVelY = 0.0f;
  *cmdAngW = 0.0f;
}

static void commandForwardAlongWall(float *cmdVelX, float *cmdVelY, float range)
{
  *cmdVelX = maxForwardSpeed;
  bool check_distance_wall = logicIsCloseTo(refDistanceFromWall, range, 0.1f);
  *cmdVelY = 0;
  if (!check_distance_wall) {
    if (range > refDistanceFromWall) {
      *cmdVelY = direction * (-1.0f * maxForwardSpeed / 2.0f);
    } else {
      *cmdVelY = direction * (maxForwardSpeed / 2.0f);
    }
  }
}

static void commandTurnAroundCornerAndAdjust(float *cmdVelX, float *cmdVelY, float *cmdAngW, float radius, float range)
{
  *cmdVelX = maxForwardSpeed;
  *cmdAngW = direction * (-1 * (*cmdVelX) / radius);
  bool check_distance_to_wall = logicIsCloseTo(refDistanceFromWall, range, 0.1f);
  if (!check_distance_to_wall) {
    if (range > refDistanceFromWall) {
      *cmdVelY = direction * (-1.0f * maxForwardSpeed / 3.0f);
    }    else {
      *cmdVelY = direction * (maxForwardSpeed / 3.0f);
    }
  }
}

static void commandTurnAndAdjust(float *cmdVelY, float *cmdAngW, float rate, float range)
{
  *cmdAngW = direction * rate;
  *cmdVelY = 0.0f;

}

static StateWF transition(StateWF newState)
{
  float t =  usecTimestamp() / 1e6;
  stateStartTime = t;
  return newState;
}

void adjustDistanceWall(float distanceWallNew)
{
  refDistanceFromWall = distanceWallNew;
}

StateWF wallFollower(float *cmdVelX, float *cmdVelY, float *cmdAngW, float frontRange, float sideRange, float currentHeading,
                  int directionTurn)
{

  direction = directionTurn;

  float now = usecTimestamp() / 1e6;

  if (firstRun) {
    prevHeading = currentHeading;
    aroundCornerBackTrack = false;
    firstRun = false;
  }

  /***********************************************************
  * Handle state transitions
  ***********************************************************/
  switch (stateWF){
   
    case forward:
      if (frontRange < refDistanceFromWall + 0.2f) {
        stateWF= transition(turnToFindWall);
      }
      break;

    case hover:
      break;

    case turnToFindWall: ;
      // check if wall is found
      bool sideRangeCheck = sideRange < (refDistanceFromWall / (float)cos(0.78f) + 0.2f);
      bool frontRangeCheck = frontRange < (refDistanceFromWall / (float)cos(0.78f) + 0.2f);

      if (sideRangeCheck && frontRangeCheck) {
        prevHeading = currentHeading;
        wallAngle = direction * (1.57f - (float)atan(frontRange / sideRange) + 0.1f);
        stateWF= transition(turnToAlignToWall); 
      }
      if (sideRange < 1.0f && frontRange > 2.0f) {
        //  around_corner_first_turn = true;
        aroundCornerBackTrack = false;
        prevHeading = currentHeading;
        stateWF= transition(findCorner); 
      }
      break;

    case turnToAlignToWall: ;
      bool alignWallCheck = logicIsCloseTo(wraptopi(currentHeading - prevHeading), wallAngle, 0.1f);
      if (alignWallCheck) {
        // prev_sideRange = sideRange;
        stateWF= transition(forwardAlongWall);
      }
      break;

    case forwardAlongWall:
      // If side range is out of reach,
      //    end of the wall is reached
      if (sideRange > refDistanceFromWall + 0.3f) {
        //  around_corner_first_turn = true;
        stateWF= transition(findCorner);
      }
      // If front range is small
      //    then corner is reached
      if (frontRange < refDistanceFromWall + 0.2f) {
        prevHeading = currentHeading;
        stateWF= transition(rotateInCorner);
      }
      break;

    case rotateAroundWall:
      if (frontRange < refDistanceFromWall + 0.2f) {
        stateWF= transition(turnToFindWall);
      }
      break;

    case rotateInCorner: ;
      // Check if heading goes over 0.8 rad
      bool checkHeadingCorner = logicIsCloseTo(fabs(wraptopi(currentHeading - prevHeading)), 0.8f, 0.1f);
      if (checkHeadingCorner) {
        stateWF= transition(turnToFindWall);
      }
      break;

    case findCorner:
      if (sideRange <= refDistanceFromWall) {
        stateWF= transition(rotateAroundWall);
      }
      break;

    default:
      stateWF= transition(hover);

  }


  /***********************************************************
   * Handle state actions
   ***********************************************************/

  float cmdVelXTemp = 0.0f;
  float cmdVelYTemp = 0.0f;
  float cmdAngWTemp = 0.0f;

  switch(stateWF){
    case forward:
      cmdVelXTemp = maxForwardSpeed;
      cmdVelYTemp = 0.0f;
      cmdAngWTemp = 0.0f;
      break;

    case hover:
      commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);
      break;

    case turnToFindWall:
      commandTurn(&cmdVelXTemp, &cmdAngWTemp, maxTurnRate);
      cmdVelYTemp = 0.0f;
      break;

    case turnToAlignToWall:
      if (now - stateStartTime < 1.0f)
      {
        commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);
      } else { // then turn again
        commandTurn(&cmdVelXTemp, &cmdAngWTemp, maxTurnRate);
        cmdVelYTemp = 0.0f;
      }
      break;

    case forwardAlongWall:
      commandForwardAlongWall(&cmdVelXTemp, &cmdVelYTemp, sideRange);
      cmdAngWTemp = 0.0f;
      break;

    case rotateAroundWall:
      // If first time around corner
      //first try to find the corner again

      // if side range is larger than prefered distance from wall
      if (sideRange > refDistanceFromWall + 0.5f) {

        // check if scanning has already occured
        if (wraptopi(fabs(currentHeading - prevHeading)) > 0.8f) {
          aroundCornerBackTrack = true;
        }
        // turn and adjust distnace to corner from that point
        if (aroundCornerBackTrack) {
          // go back if it already went into one direction
          commandTurnAndAdjust(&cmdVelYTemp, &cmdAngWTemp, -1 * maxTurnRate, sideRange);
          cmdVelXTemp = 0.0f;
        } else {
          commandTurnAndAdjust(&cmdVelYTemp, &cmdAngWTemp, maxTurnRate, sideRange);
          cmdVelXTemp = 0.0f;
        }
      } else {
        // continue to turn around corner
        prevHeading = currentHeading;
        aroundCornerBackTrack = false;
        commandTurnAroundCornerAndAdjust(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp, refDistanceFromWall, sideRange);
      }
      break;

    case rotateInCorner:
      commandTurn(&cmdVelXTemp, &cmdAngWTemp, maxTurnRate);
      cmdVelYTemp = 0.0f;
      break;

    case findCorner:
      commandAlignCorner(&cmdVelYTemp, &cmdAngWTemp, -1 * maxTurnRate, sideRange, refDistanceFromWall);
      cmdVelXTemp = 0.0f;
      break;
  
    default:
      //State does not exist so hover!!
      commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);
  }
  
  *cmdVelX = cmdVelXTemp;
  *cmdVelY = cmdVelYTemp;
  *cmdAngW = cmdAngWTemp;

  return stateWF;

}
