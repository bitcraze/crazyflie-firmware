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
    supervisorStateNotInitialized = 0,
    supervisorStatePreFlChecksNotPassed,
    supervisorStatePreFlChecksPassed,
    supervisorStateReadyToFly,
    supervisorStateFlying,
    supervisorStateLanded,
    supervisorStateReset,
    supervisorStateWarningLevelOut,
    supervisorStateExceptFreeFall,
    supervisorStateLocked,
    supervisorStateCrashed,
    supervisorState_NrOfStates,
} supervisorState_t;

// Conditions supported by the supervisor
typedef enum {
  supervisorConditionArmed = 0,
  supervisorConditionIsFlying,
  supervisorConditionIsTumbled,
  supervisorConditionCommanderWdtWarning,
  supervisorConditionCommanderWdtTimeout,
  supervisorConditionEmergencyStop,
  supervisorConditionIsCrashed,
  supervisorConditionLandingTimeout,
  supervisorCondition_NrOfConditions,
} supervisorConditions_t;

typedef uint32_t supervisorConditionBits_t;

// Condition bit definitions
#define SUPERVISOR_CB_NONE (0)
#define SUPERVISOR_CB_ARMED (1 << supervisorConditionArmed)
#define SUPERVISOR_CB_IS_FLYING (1 << supervisorConditionIsFlying)
#define SUPERVISOR_CB_IS_TUMBLED (1 << supervisorConditionIsTumbled)
#define SUPERVISOR_CB_COMMANDER_WDT_WARNING (1 << supervisorConditionCommanderWdtWarning)
#define SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT (1 << supervisorConditionCommanderWdtTimeout)
#define SUPERVISOR_CB_EMERGENCY_STOP (1 << supervisorConditionEmergencyStop)
#define SUPERVISOR_CB_CRASHED (1 << supervisorConditionIsCrashed)
#define SUPERVISOR_CB_LANDING_TIMEOUT (1 << supervisorConditionLandingTimeout)


// Enum that is used to describe how to combine the bits in the required field
typedef enum {
  supervisorAll = 0, // AKA and
  supervisorAny,     // AKA or
  supervisorAlways,
  supervisorNever,
} SupervisorConditionCombiner_t;

// Describes the requirements for a state transition
typedef struct {
  supervisorState_t newState;

  supervisorConditionBits_t triggers;
  supervisorConditionBits_t negatedTriggers;
  SupervisorConditionCombiner_t triggerCombiner;

  supervisorConditionBits_t blockers;
  supervisorConditionBits_t negatedBlockers;
  SupervisorConditionCombiner_t blockerCombiner;
} SupervisorStateTransition_t;

typedef struct {
  SupervisorStateTransition_t* transitionList;
  int length;
} SupervisorStateTransitionList_t;

// Macro used when defining SupervisorStateTransitionLists
#define SUPERVISOR_TRANSITION_ENTRY(TRANSITION_DEF) .transitionList=TRANSITION_DEF, .length=(sizeof(TRANSITION_DEF) / sizeof(SupervisorStateTransition_t))

supervisorState_t supervisorStateUpdate(const supervisorState_t currentState, const supervisorConditionBits_t conditions);

const char* supervisorGetStateName(const supervisorState_t currentState);
const char* supervisorGetConditionName(const supervisorState_t condition);
