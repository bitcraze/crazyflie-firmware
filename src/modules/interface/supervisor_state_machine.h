/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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

#include <stdint.h>

typedef enum {
    supervisorStatePreFlChecksNotPassed = 0,
    supervisorStatePreFlChecksPassed,
    supervisorStateReadyToFly,
    supervisorStateFlying,
    supervisorStateLanded,
    supervisorStateReset,
    supervisorStateWarningLevelOut,
    supervisorStateExceptFreeFall,
    supervisorStateLocked,
    supervisorState_NrOfStates,
} supervisorState_t;

// Conditions supported by the supervisor
enum {
  supervisorConditionArmed = 0,
  supervisorConditionChargerConnected,
  supervisorConditionIsFlying,
  supervisorConditionIsTumbled,
  supervisorConditionCommanderWdtWarning,
  supervisorConditionCommanderWdtTimeout,
  supervisorConditionEmergencyStop,
};

typedef uint32_t supervisorConditionBits_t;

// Condition bit definitions
#define SUPERVISOR_CB_NONE (0)
#define SUPERVISOR_CB_ARMED (1 << supervisorConditionArmed)
#define SUPERVISOR_CB_CHARGER_CONNECTED (1 << supervisorConditionChargerConnected)
#define SUPERVISOR_CB_IS_FLYING (1 << supervisorConditionIsFlying)
#define SUPERVISOR_CB_IS_TUMBLED (1 << supervisorConditionIsTumbled)
#define SUPERVISOR_CB_COMMANDER_WDT_WARNING (1 << supervisorConditionCommanderWdtWarning)
#define SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT (1 << supervisorConditionCommanderWdtTimeout)
#define SUPERVISOR_CB_EMERGENCY_STOP (1 << supervisorConditionEmergencyStop)


typedef struct {
  supervisorState_t newState;
  supervisorConditionBits_t mustBeSet;
  supervisorConditionBits_t mustNotBeSet;
} SupervisorStateTransition_t;

typedef struct {
  SupervisorStateTransition_t* transitionList;
  int length;
} SupervisorStateTransitionList_t;

// Macro used when defining SupervisorStateTransitionLists
#define SUPERVISOR_TRANSITION_ENTRY(TRANSITION_DEF) .transitionList=TRANSITION_DEF, .length=(sizeof(TRANSITION_DEF) / sizeof(SupervisorStateTransition_t))

supervisorState_t supervisorStateUpdate(const supervisorState_t currentState, const supervisorConditionBits_t conditions);
