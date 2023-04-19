/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 * supervisor.h - Keep track of system state
 */

#pragma once

#include "stabilizer_types.h"

/**
 * @brief Update the supervisor. The supervisor will evaluate the current situation to determine if some action is
 * required.
 *
 * @param sensors  Sensor values
 * @return true   Motors are allowed to run
 * @return false  Motors must not run
 */
bool supervisorUpdate(const sensorData_t *sensors);

bool supervisorCanFly(void);
bool supervisorIsFlying(void);
bool supervisorIsTumbled(void);
