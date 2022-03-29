/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * param.h - Crazy parameter system header file.
 */

#pragma once

#include <stdint.h>

// Include param_logic.h for backwards compatibility in apps
#include "param_logic.h"

/* Basic parameter structure */
struct param_s {
  uint8_t type;
  uint8_t extended_type;
  char * name;
  void * address;
  void (*callback)(void);
  void * (*getter)(void);
};

typedef uint8_t * (*paramGetterUInt8)(void);
typedef uint16_t * (*paramGetterUInt16)(void);
typedef uint32_t * (*paramGetterUInt32)(void);
typedef uint64_t * (*paramGetterUInt64)(void);
typedef float * (*paramGetterFloat)(void);

#define PARAM_BYTES_MASK 0x03
#define PARAM_1BYTE  0x00
#define PARAM_2BYTES 0x01
#define PARAM_4BYTES 0x02
#define PARAM_8BYTES 0x03

#define PARAM_TYPE_MASK   0x0F
#define PARAM_TYPE_INT   (0x00<<2)
#define PARAM_TYPE_FLOAT (0x01<<2)

#define PARAM_SIGNED (0x00<<3)
#define PARAM_UNSIGNED (0x01<<3)

#define PARAM_VARIABLE (0x00<<7)
#define PARAM_GROUP    (0x01<<7)

#define PARAM_EXTENDED (1<<4)

#define PARAM_CORE (1<<5)

#define PARAM_RONLY (1<<6)

#define PARAM_START 1
#define PARAM_STOP  0

#define PARAM_SYNC 0x02

// Extended type bits
#define PARAM_PERSISTENT (1 << 8)

#define PARAM_PERSISTENT_STORED      1
#define PARAM_PERSISTENT_NOT_STORED  0

// User-friendly macros
#define PARAM_UINT8 (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT8  (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT16 (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT16  (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT32 (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT32  (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_SIGNED)

#define PARAM_FLOAT (PARAM_4BYTES | PARAM_TYPE_FLOAT | PARAM_SIGNED)

// CRTP
#define TOC_CH 0
#define READ_CH 1
#define WRITE_CH 2
#define MISC_CH 3
// CRTP Misc
#define MISC_SETBYNAME            0
#define MISC_VALUE_UPDATED        1
#define MISC_GET_EXTENDED_TYPE    2
#define MISC_PERSISTENT_STORE     3
#define MISC_PERSISTENT_GET_STATE 4
#define MISC_PERSISTENT_CLEAR     5
#define MISC_GET_DEFAULT_VALUE    6

/* Macros */

#define PARAM_ADD_FULL(TYPE, NAME, ADDRESS, CALLBACK, DEFAULT_GETTER) \
    { .type = ((TYPE) <= 0xFF) ? (TYPE) : (((TYPE) | PARAM_EXTENDED) & 0xFF), \
      .extended_type = (((TYPE) & 0xFF00) >> 8), \
      .name = #NAME, \
      .address = (void*)(ADDRESS), \
      .callback = (void *)CALLBACK, \
      .getter = (void *)DEFAULT_GETTER, },

#define PARAM_ADD(TYPE, NAME, ADDRESS) \
    PARAM_ADD_FULL(TYPE, NAME, ADDRESS, 0, 0)

// The callback notification function will run from the param task, it should not block and should run quickly.
#define PARAM_ADD_WITH_CALLBACK(TYPE, NAME, ADDRESS, CALLBACK) \
    PARAM_ADD_FULL(TYPE, NAME, ADDRESS, CALLBACK, 0)

#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)

#define PARAM_ADD_CORE_WITH_CALLBACK(TYPE, NAME, ADDRESS, CALLBACK) \
  PARAM_ADD_WITH_CALLBACK(TYPE | PARAM_CORE, NAME, ADDRESS, CALLBACK)

#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) \
  { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), .callback = 0, .getter = 0, },

#define PARAM_GROUP_STOP(NAME) \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x0) \
  };

#ifndef UNIT_TEST_MODE

#define PARAM_GROUP_START(NAME)  \
  static struct param_s __params_##NAME[] __attribute__((section(".param." #NAME), used)) = { \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)

#else // UNIT_TEST_MODE

  // Do not use a different data section when running unit tests
#define PARAM_GROUP_START(NAME)  \
  static const struct param_s __params_##NAME[] = { \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)

#endif // UNIT_TEST_MODE

// Do not remove! This definition is used by doxygen to generate parameter documentation.
/** @brief Core parameters
 *
 * The core parameters are considered part of the official API and are guaranteed
 * to be stable over time.
 *
 * @defgroup PARAM_CORE_GROUP */
