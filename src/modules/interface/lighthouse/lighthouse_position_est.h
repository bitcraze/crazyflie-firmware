/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 *
 * lighthouse_position_est.h - position estimaton for the lighthouse system
 */

#pragma once

#include "pulse_processor.h"
#include "lighthouse_geometry.h"


extern baseStationGeometry_t lighthouseBaseStationsGeometry[PULSE_PROCESSOR_N_BASE_STATIONS];

/**
 * @brief Call when the lighthouseBaseStationsGeometry data has been updated
 * to recalculate cached data
 */
void lightHousePositionGeometryDataUpdated();

void lighthousePositionEstimatePoseCrossingBeams(pulseProcessorResult_t* angles, int baseStation);
void lighthousePositionEstimatePoseSweeps(pulseProcessorResult_t* angles, int baseStation);
