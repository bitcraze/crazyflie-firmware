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
 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>

/**
 * @brief Throttles how much of the data from lighthouse base stations that is used. When multiple base stations
 * are received, pushing all the data to the estimator is nor necessary and it increases the risk of overloading
 * the system.
 *
 * This function tries to limit the rate of the samples used by randomly discarding samples when needed.
 *
 * @param now_ms The current time in ms
 * @return true   If the sample is to be used
 * @return false  If the sample should be discarded
 */
bool throttleLh2Samples(const uint32_t now_ms);
