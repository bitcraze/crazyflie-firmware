/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 BitCraze AB
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
#pragma once

#include <stdbool.h>
#include "crtp.h"

/**
 * @brief Initialize the CRTP link for CPX
 * 
 */
void cpxlinkInit();

/**
 * @brief Run tests for CRTP link for CPX
 * 
 * @return true if tests pass
 * @return false if test do not pass
 */
bool cpxlinkTest();

/**
 * @brief Set the connection status of the underlying CPX link
 * 
 * This will effect message being pushed from CRTP modules.
 * 
 * @param isConnected true if connected otherwise false
 */
void cpxLinkSetConnected(bool isConnected);

/**
 * @brief Get the CPX link structure
 * 
 * This will get the function pointers that should be used by the CRTP
 * stack for using CPX.
 * 
 * @return struct crtpLinkOperations* function pointers for using link
 */
struct crtpLinkOperations * cpxlinkGetLink();
