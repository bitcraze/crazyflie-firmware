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
 * cfassert.h - Assert macro
 */

#include "console.h"

#ifndef __CFASSERT_H__
#define __CFASSERT_H__

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

#define ASSERT_FAILED() assertFail( "", __FILE__, __LINE__ )

/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);
void printAssertSnapshotData();

#endif //__CFASSERT_H__
