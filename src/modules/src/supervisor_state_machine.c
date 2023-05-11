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

#include <stdbool.h>
#include <assert.h>
#include "supervisor_state_machine.h"
#include "test_support.h"

// #define DEBUG_ME

#ifdef DEBUG_ME
#define DEBUG_MODULE "SUPST"
#include "debug.h"
#include "cfassert.h"

static const char* const stateNames[] = {
  "Pre-flight checks not passed",
  "Pre-flight checks passed",
  "Ready to fly",
  "Flying",
  "Landed",
  "Reset",
  "Warning, level out",
  "Exception, free fall",
  "Locked",
};
static_assert(sizeof(stateNames) / sizeof(stateNames[0]) == supervisorState_NrOfStates);
#endif


// State transition definitions
static SupervisorStateTransition_t transitionsPreFlChecksNotPassed[] = {
  {
    .newState = supervisorStatePreFlChecksPassed,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP
  }
};

static SupervisorStateTransition_t transitionsPreFlChecksPassed[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_IS_TUMBLED,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_EMERGENCY_STOP,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateReadyToFly,
    .mustBeSet = SUPERVISOR_CB_ARMED,
    .mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP
  },
};

static SupervisorStateTransition_t transitionsReadyToFly[] = {
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_EMERGENCY_STOP,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_ARMED
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_IS_TUMBLED,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateFlying,
    .mustBeSet = SUPERVISOR_CB_IS_FLYING,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  }
};

static SupervisorStateTransition_t transitionsFlying[] = {
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_IS_TUMBLED,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_EMERGENCY_STOP,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_ARMED
  },
  {
    .newState = supervisorStateWarningLevelOut,
    .mustBeSet = SUPERVISOR_CB_COMMANDER_WDT_WARNING,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateLanded,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_IS_FLYING
  }
};

static SupervisorStateTransition_t transitionsLanded[] = {
  {
    .newState = supervisorStateReset,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
};

static SupervisorStateTransition_t transitionsReset[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
};

static SupervisorStateTransition_t transitionsWarningLevelOut[] = {
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateExceptFreeFall,
    .mustBeSet = SUPERVISOR_CB_IS_TUMBLED,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
  {
    .newState = supervisorStateFlying,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT
  },
};

static SupervisorStateTransition_t transitionsExceptFreeFall[] = {
  {
    .newState = supervisorStateLocked,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
};

static SupervisorStateTransition_t transitionsLocked[] = {
  {
    .newState = supervisorStateLocked,
    .mustBeSet = SUPERVISOR_CB_NONE,
    .mustNotBeSet = SUPERVISOR_CB_NONE
  },
};

SupervisorStateTransitionList_t transitionLists[] = {
  {SUPERVISOR_TRANSITION_ENTRY(transitionsPreFlChecksNotPassed)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsPreFlChecksPassed)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsReadyToFly)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsFlying)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsLanded)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsReset)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsWarningLevelOut)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsExceptFreeFall)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsLocked)},
};
static_assert(sizeof(transitionLists) / sizeof(transitionLists[0]) == supervisorState_NrOfStates);


bool supervisorStateMachineInit() {
  if (sizeof(supervisorState_t) != sizeof(transitionLists)) {
    return false;
  }

  return true;
}

TESTABLE_STATIC supervisorState_t findTransition(const supervisorState_t currentState, const supervisorConditionBits_t conditions, const SupervisorStateTransitionList_t* transitions) {
  supervisorState_t newState = currentState;
  for (int i = 0; i < transitions->length; i++) {
    const SupervisorStateTransition_t* transitionDef = &transitions->transitionList[i];

    const supervisorConditionBits_t maskMustBeSet = transitionDef->mustBeSet;
    const supervisorConditionBits_t conditionsNotMetMustBeSet = (~conditions) & maskMustBeSet;

    const supervisorConditionBits_t maskMustNotBeSet = transitionDef->mustNotBeSet;
    const supervisorConditionBits_t conditionsNotMetMustNotBeSet = conditions & maskMustNotBeSet;

    const supervisorConditionBits_t conditionsNotMet = conditionsNotMetMustBeSet | conditionsNotMetMustNotBeSet;

    if (conditionsNotMet == 0) {
      newState = transitionDef->newState;
      break;
    }
  }
  return newState;
}

supervisorState_t supervisorStateUpdate(const supervisorState_t currentState, const supervisorConditionBits_t conditions) {
  #ifdef DEBUG_ME
  ASSERT(currentState < supervisorState_NrOfStates);
  #endif

  const SupervisorStateTransitionList_t* transitions = &transitionLists[currentState];
  const supervisorState_t newState = findTransition(currentState, conditions, transitions);

  #ifdef DEBUG_ME
  ASSERT(currentState < supervisorState_NrOfStates);
  ASSERT(newState < supervisorState_NrOfStates);

  if (newState != currentState) {
    DEBUG_PRINT("Enter %s -> %s\n", stateNames[currentState], stateNames[newState]);
  }
  #endif

  return newState;
}
