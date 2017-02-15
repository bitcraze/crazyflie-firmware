/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * log.h - Dynamic log system
 */

#ifndef __MEM_H__
#define __MEM_H__

#include <stdbool.h>
#include <stdint.h>


#define MEM_SETTINGS_CH     0
#define MEM_READ_CH         1
#define MEM_WRITE_CH        2

#define MEM_CMD_GET_NBR     1
#define MEM_CMD_GET_INFO    2


/* Public functions */
void memInit(void);
bool memTest(void);

#endif /* __MEM_H__ */
