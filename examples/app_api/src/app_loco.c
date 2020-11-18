/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 * app_loco.c - LPS example calls
 *
 * This code is mainly intended to make sure code is compiled in CI
 */


#include "locodeck.h"
#include "app_loco.h"

void appLocoExampleCalls() {
  point_t position;
  locoDeckGetAnchorPosition(0, &position);

  uint8_t unorderedAnchorList[5];
  locoDeckGetAnchorIdList(unorderedAnchorList, 5);
  locoDeckGetActiveAnchorIdList(unorderedAnchorList, 5);
}
