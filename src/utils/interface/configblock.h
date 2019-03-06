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
 * configblock.h - Simple static implementation of the config block
 */

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifndef __CONFIGBLOCK_H__
#define __CONFIGBLOCK_H__


/* Internal format of the config block */
#define CONFIGBLOCK_MAGIC 0x43427830
#define CONFIGBLOCK_VERSION 2
#define CONFIGBLOCK_HEADER_SIZE_BYTES 5 // magic + version
#define CONFIGBLOCK_OVERHEAD_SIZE_BYTES (CONFIGBLOCK_HEADER_SIZE_BYTES + 1) // + cksum

int configblockInit(void);
bool configblockTest(void);
bool configblockSave(void);
/* Static accessors */
int configblockGetRadioChannel(void);
int configblockGetRadioSpeed(void);
uint64_t configblockGetRadioAddress(void);

float configblockGetCalibPitch(void);
float configblockGetCalibRoll(void);

/* IMU EEPROM parameters Getters */
bool configblockGetGyroCalibrated(void);
float configblockGetGyroBiasX(void);
float configblockGetGyroBiasY(void);
float configblockGetGyroBiasZ(void);
bool configblockGetAccCalibrated(void);
float configblockGetAccScale(void);

/* IMU EEPROM parameters Setters */
void configblockSetGyroCalibrated(bool);
void configblockSetGyroBiasX(float);
void configblockSetGyroBiasY(float);
void configblockSetGyroBiasZ(float);
void configblockSetAccCalibrated(bool);
void configblockSetAccScale(float);

// Old versions
struct configblock_v0_s {
  /* header */
  uint32_t magic;
  uint8_t  version;
  /* Content */
  uint8_t radioChannel;
  uint8_t radioSpeed;
  float calibPitch;
  float calibRoll;
  /* Simple modulo 256 checksum */
  uint8_t cksum;
} __attribute__((__packed__));

// Master version
struct configblock_v1_s {
  /* header */
  uint32_t magic;
  uint8_t  version;
  /* Content */
  uint8_t radioChannel;
  uint8_t radioSpeed;
  float calibPitch;
  float calibRoll;
  uint8_t radioAddress_upper;
  uint32_t radioAddress_lower;
  /* Simple modulo 256 checksum */
  uint8_t cksum;
} __attribute__((__packed__));

// Dev version
struct configblock_v2_s {
  /* header */
  uint32_t magic;
  uint8_t  version;
  /* Content */
  uint8_t radioChannel;
  uint8_t radioSpeed;
  float calibPitch;
  float calibRoll;
  uint8_t radioAddress_upper;
  uint32_t radioAddress_lower;
  bool gyroCalibrated;
  float gyroBiasX;
  float gyroBiasY;
  float gyroBiasZ;
  bool accCalibrated;
  float accScale;
  /* Simple modulo 256 checksum */
  uint8_t cksum;
} __attribute__((__packed__));

// Set version 1 as current version
typedef struct configblock_v2_s configblock_t;

#endif //__CONFIGBLOCK_H__
