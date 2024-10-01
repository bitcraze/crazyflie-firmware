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
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */

//#include "ext/Eigen/Eigen/Dense"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <vector>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "controller_pid.h"
#include "stabilizer.h"
#include "ergodic_controller.h"

#define DEBUG_MODULE "ERGODICCONTROLLER"
#include "debug.h"

static struct {
    // num imenstions
    uint8_t n;
    // array
    //Eigen::VectorXd map(n);
    std::vector vec(3);
    //
} target_distro; // target map

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
} //runs the task for this application

void controllerOutOfTreeInit() {
  controllerPidInit();
  ergodicControllerInit();
} //using the PID to test so we need to initilize it

void ergodicControllerInit() {
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  //create information map
  
  //get initial position

  //define drone dynamics
}

void updateMap() {

}

void getMap(target_distro& target_map) {
  //target_map *= 
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(control_t *control, 
                          const setpoint_t *setpoint, 
                          const sensorData_t *sensors, 
                          const state_t *state, 
                          const uint32_t tick) 
{
  controllerPid(control, setpoint, sensors, state, tick); //calling PID controller for now

  // TODO
  // Set mode
  // Get setpoint
  // Get state
  // Whichever mode we slect, we get the current motor torques or current x,y,z
  // Call ergodic controller to get next state
  // Update inforamtion map
}