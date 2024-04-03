/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * platform_bolt.c - platform functionality for the Crazyflie Bolt
 */


#define DEBUG_MODULE "PLATFORM"

#include <string.h>

#include "platform.h"
#include "exti.h"
#include "nvic.h"
#include "debug.h"

static platformConfig_t configs[] = {
#ifdef CONFIG_SENSORS_BMI088_SPI
  {  // Old ID of Crazyflie Bolt
    .deviceType = "RZ10",
    .deviceTypeName = "Crazyflie Bolt",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp3xx,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapBoltBrushless,
  },
  {
    .deviceType = "CB10",
    .deviceTypeName = "Crazyflie Bolt",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp3xx,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapBoltBrushless,
  },
  {
    .deviceType = "CB11",
    .deviceTypeName = "Crazyflie Bolt 1.1",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp3xx,
    .physicalLayoutAntennasAreClose = false,
  #ifdef CONFIG_BOLT11_BRUSHED
    .motorMap = motorMapBolt11Brushed,
  #else
    .motorMap = motorMapBolt11Brushless,
  #endif
  }
#endif
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
  return "bolt";
}
