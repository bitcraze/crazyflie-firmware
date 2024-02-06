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
 * api_app.c - App layer application that calls app API functions to make
 *             sure they are compiled in CI.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "ledseq.h"
#include "crtp_commander_high_level.h"
#include "locodeck.h"
#include "mem.h"
#include "log.h"
#include "param.h"
#include "pm.h"
#include "app_channel.h"
#include "system.h"


#define DEBUG_MODULE "APPAPI"

void appMain() {
  // Do not run this app
  ASSERT_FAILED();

  // LED sequencer
  {
    ledseqContext_t ledSeqContext;
    ledseqRegisterSequence(&ledSeqContext);
    ledseqRun(&ledSeqContext);
    ledseqRunBlocking(&ledSeqContext);
    ledseqStop(&ledSeqContext);
    ledseqStopBlocking(&ledSeqContext);
  }

  // High level commander
  {
    uint8_t dummyTrajectory[10];
    crtpCommanderHighLevelTakeoff(1.0f, 1.0f);
    crtpCommanderHighLevelTakeoffYaw(1.0f, 1.0f, 1.0f);
    crtpCommanderHighLevelLand(0.0f, 1.0f);
    crtpCommanderHighLevelLandYaw(0.0f, 1.0f, 1.0f);
    crtpCommanderHighLevelStop();
    crtpCommanderHighLevelGoTo(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, false);
    crtpCommanderHighLevelStartTrajectory(3, 1.0f, true, false);
    crtpCommanderHighLevelDefineTrajectory(3, CRTP_CHL_TRAJECTORY_TYPE_POLY4D_COMPRESSED, 0, 17);
    crtpCommanderHighLevelTrajectoryMemSize();
    crtpCommanderHighLevelWriteTrajectory(20, 10, dummyTrajectory);
    crtpCommanderHighLevelReadTrajectory(20, 10, dummyTrajectory);
    crtpCommanderHighLevelIsTrajectoryFinished();
    crtpCommanderHighLevelTakeoffWithVelocity(1.0f, 1.0f, true);
    crtpCommanderHighLevelLandWithVelocity(1.0f, 1.0f, true);
  }

  // LPS
  #ifdef CONFIG_DECK_LOCO
  {
    point_t position;
    uint8_t unorderedAnchorList[5];

    locoDeckGetAnchorPosition(0, &position);
    locoDeckGetAnchorIdList(unorderedAnchorList, 5);
    locoDeckGetActiveAnchorIdList(unorderedAnchorList, 5);
  }
  #endif

  // Memory sub system
  {
    static const MemoryHandlerDef_t memDef = {.type = MEM_TYPE_APP,};
    memoryRegisterHandler(&memDef);
  }

  // Log
  {
    logVarId_t id = logGetVarId("some", "log");
    logGetFloat(id);
    logGetInt(id);
    logGetUint(id);
  }

  // Param
  {
    paramVarId_t id = paramGetVarId("some", "param");
    paramGetFloat(id);
    paramGetInt(id);
    paramGetUint(id);
    paramSetInt(id, 0);
    paramSetFloat(id, 0.0f);
  }

  // Power management
  {
    pmIsBatteryLow();
    pmIsChargerConnected();
    pmIsCharging();
    pmIsDischarging();
  }

  // App-channel
  {
    char buffer[APPCHANNEL_MTU];
    appchannelSendPacket("hello", 5); // Deprecated (removed after August 2023)
    appchannelSendDataPacketBlock("hello", 5);
    appchannelSendDataPacket("hello", 5);
    appchannelReceivePacket(buffer, APPCHANNEL_MTU, APPCHANNEL_WAIT_FOREVER); // Deprecated (removed after August 2023)
    appchannelReceiveDataPacket(buffer, APPCHANNEL_MTU, APPCHANNEL_WAIT_FOREVER);
    appchannelHasOverflowOccured(); // Deprecated (removed after August 2023)
    appchannelHasOverflowOccurred();
  }

  // System
  {
    systemRequestShutdown();
  }
}
