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
 *
 * Generic platform functionality
 *
 */

#include <string.h>
#include "platform.h"

static const platformConfig_t* active_config = 0;

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

int platformInitConfiguration(const platformConfig_t* configs, const int nrOfConfigs) {
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

const char* platformConfigGetDeviceType() {
  return active_config->deviceType;
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

const MotorPerifDef** platformConfigGetMotorMapping() {
  return active_config->motorMap;
}

