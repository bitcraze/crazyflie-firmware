/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * Platform functionality for the CF2 platform
 */

#define DEBUG_MODULE "PLATFORM"

#include <string.h>

#include "platform.h"
#include "exti.h"
#include "nvic.h"
#include "debug.h"
#include "radiolink.h"

// Define to decrease the nRF51 Tx power to reduce interference
#ifndef PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM
#define PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM (-12)
#endif

static int initPlatformConfiguration();
static void initHardware();


typedef struct {
  char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];
  char deviceTypeName[14];
  SensorImplementation_t sensorImplementation;
} config_t;

static config_t configs[] = {
  {
    .deviceType = "CF20",
    .deviceTypeName = "Crazyflie 2.0",
    .sensorImplementation = SensorImplementation_mpu9250_lps25h,
  },
  {
    .deviceType = "CF21",
    .deviceTypeName = "Crazyflie 2.1",
    .sensorImplementation = SensorImplementation_bmi088_bmp388,
  }
};

static config_t* active_config = 0;


int platformInit(void) {
  int err = initPlatformConfiguration();
  if (err != 0)
  {
    // This firmware is not compatible, abort init
    return 1;
  }

  initHardware();
  return 0;
}


void platformSetLowInterferenceRadioMode(void) {
  // Decrease the nRF51 Tx power to reduce interference
  radiolinkSetPowerDbm(PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM);
  DEBUG_PRINT("Low interference mode. NRF51 TX power offset by %ddb.\r\n", PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM);
}


static int initPlatformConfiguration() {
  char deviceTypeString[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];
  char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];

  platformGetDeviceTypeString(deviceTypeString);
  platformParseDeviceTypeString(deviceTypeString, deviceType);

  for (int i = 0; i < (sizeof(configs) / sizeof(config_t)); i++) {
    config_t* config = &configs[i];
    if (strcmp(config->deviceType, deviceType) == 0) {
      active_config = config;
      return 0;
    }
  }

  return 1;
}

static void initHardware() {
  //Low level init: Clock and Interrupt controller
  nvicInit();

  //EXTI interrupts
  extiInit();
}


// Config functions ------------------------

const char* platformConfigGetPlatformName() {
  return "cf2";
}

const char* platformConfigGetDeviceTypeName() {
  return active_config->deviceTypeName;
}

SensorImplementation_t platformConfigGetSensorImplementation() {
  return active_config->sensorImplementation;
}

