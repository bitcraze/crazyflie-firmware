/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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

/**
 * @file   platform_cf2.c
 * @brief  Platform functionality for the CF2 platform
 *
 * The platform is the basic hardware/sensor configuration for a crazyflie 2.
 * Check out the type platformConfig_t to see the different platforms - from 
 * the Crazyflie 2 to Crazyflie bolt.
 *
 * @ingroup system
 *
 * @see platformInit()
 * @see platformConfig_t
 */

#define DEBUG_MODULE "PLATFORM"

#include <string.h>

#include "platform.h"
#include "exti.h"
#include "nvic.h"
#include "debug.h"

/**
 * @brief List of platform configurations of the Crazyflie 2 platform
 *
 * Contains information on sensor outfits, antenna positioning,
 * the motor map etc. 
 *
 * @see platformConfig_t
 */
static platformConfig_t configs[] = {
  {
    .deviceType = "CF20",
    .deviceTypeName = "Crazyflie 2.0",
    .sensorImplementation = SensorImplementation_mpu9250_lps25h,
    .physicalLayoutAntennasAreClose = true,
    .motorMap = motorMapDefaultBrushed,
  },
  {
    .deviceType = "CF21",
    .deviceTypeName = "Crazyflie 2.1",
    .sensorImplementation = SensorImplementation_bmi088_bmp388,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapDefaultBrushed,
  },
  {  // Old ID of Crzyflie Bolt
    .deviceType = "RZ10",
    .deviceTypeName = "Crazyflie Bolt",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp388,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapBoltBrushless,
  },
  {
    .deviceType = "CB10",
    .deviceTypeName = "Crazyflie Bolt",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp388,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapBoltBrushless,
  }
};

const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs) {
  *nrOfConfigs = sizeof(configs) / sizeof(platformConfig_t);
  return configs;
}

void platformInitHardware() {
  //Low level init: Clock and Interrupt controller
  nvicInit();

  //EXTI interrupts
  extiInit();
}


// Config functions ------------------------

const char* platformConfigGetPlatformName() {
  return "cf2";
}
