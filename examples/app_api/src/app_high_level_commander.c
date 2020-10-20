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
 * app_high_level_commander.c - high level commander example
 */


#include "crtp_commander_high_level.h"
#include "param.h"

static int state = 0;
static const float duration = 2.0f;
static const bool relative = true;

void appEnableHighLevelCommander() {
  paramVarId_t paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
  paramSetInt(paramIdCommanderEnHighLevel, 1);
}

void appRunHighLevelCommanderSquare() {
  switch(state) {
    case 0:
      crtpCommanderHighLevelTakeoff(1.0f, duration);
      break;

    case 1:
      crtpCommanderHighLevelGoTo(1.0f, 0.0f, 0.0f, 0.0f, duration, relative);
      break;

    case 2:
      crtpCommanderHighLevelGoTo(0.0f, 1.0f, 0.0f, 0.0f, duration, relative);
      break;

    case 3:
      crtpCommanderHighLevelGoTo(-1.0f, 0.0f, 0.0f, 0.0f, duration, relative);
      break;

    case 4:
      crtpCommanderHighLevelGoTo(0.0f, -1.0f, 0.0f, 0.0f, duration, relative);
      break;

    case 5:
      crtpCommanderHighLevelLand(0.0f, duration);
      break;

    default:
      break;
  }

  state++;
  if (state > 5) {
    state = 0;
  }
}

void callAllFunctionsForCi() {
  crtpCommanderHighLevelTakeoff(1.0f, 1.0f);
  crtpCommanderHighLevelTakeoffYaw(1.0f, 1.0f, 1.0f);
  crtpCommanderHighLevelLand(0.0f, 1.0f);
  crtpCommanderHighLevelLandYaw(0.0f, 1.0f, 1.0f);
  crtpCommanderHighLevelStop();
  crtpCommanderHighLevelGoTo(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, false);
  crtpCommanderHighLevelStartTrajectory(3, 1.0f, true, false);
  crtpCommanderHighLevelDefineTrajectory(3, CRTP_CHL_TRAJECTORY_TYPE_POLY4D_COMPRESSED, 0, 17);
}
