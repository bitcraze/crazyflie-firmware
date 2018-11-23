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
 * Generic platform functionality
 *
 */

#define DEBUG_MODULE "PLATFORM"

#include <string.h>
#include "platform.h"
#include "radiolink.h"
#include "debug.h"

// Define to decrease the nRF51 Tx power to reduce interference
#ifndef PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM
#define PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM (-12)
#endif

static const platformConfig_t* active_config = 0;
static int platformInitConfiguration(const platformConfig_t* configs, const int nrOfConfigs);

int platformInit(void) {
  int nrOfConfigs = 0;
  const platformConfig_t* configs = platformGetListOfConfigurations(&nrOfConfigs);

  int err = platformInitConfiguration(configs, nrOfConfigs);
  if (err != 0)
  {
    // This firmware is not compatible, abort init
    return 1;
  }

  platformInitHardware();
  return 0;
}

int platformParseDeviceTypeString(const char* deviceTypeString, char* deviceType) {
  if (deviceTypeString[0] != '0' || deviceTypeString[1] != ';') {
    return 1;
  }

  const int start = 2;
  const int last = start + PLATFORM_DEVICE_TYPE_MAX_LEN - 1;
  int end = 0;
  for (end = start; end <= last; end++) {
    if (deviceTypeString[end] == '\0' || deviceTypeString[end] == ';') {
      break;
    }
  }

  if (end > last) {
    return 1;
  }

  int length = end - start;
  memcpy(deviceType, &deviceTypeString[start], length);
  deviceType[length] = '\0';
  return 0;
}

static int platformInitConfiguration(const platformConfig_t* configs, const int nrOfConfigs) {
#ifndef DEVICE_TYPE_STRING_FORCE
  char deviceTypeString[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];
  char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];

  platformGetDeviceTypeString(deviceTypeString);
  platformParseDeviceTypeString(deviceTypeString, deviceType);
#else
  #define xstr(s) str(s)
  #define str(s) #s

  char* deviceType = xstr(DEVICE_TYPE_STRING_FORCE);
#endif

  for (int i = 0; i < nrOfConfigs; i++) {
    const platformConfig_t* config = &configs[i];
    if (strcmp(config->deviceType, deviceType) == 0) {
      active_config = config;
      return 0;
    }
  }

  return 1;
}

const char* platformConfigGetDeviceTypeName() {
  return active_config->deviceTypeName;
}

SensorImplementation_t platformConfigGetSensorImplementation() {
  return active_config->sensorImplementation;
}

bool platformConfigPhysicalLayoutAntennasAreClose() {
  return active_config->physicalLayoutAntennasAreClose;
}


void platformSetLowInterferenceRadioMode(void) {
  // Decrease the nRF51 Tx power to reduce interference
  radiolinkSetPowerDbm(PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM);
  DEBUG_PRINT("Low interference mode. NRF51 TX power offset by %ddb.\r\n", PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM);
}
