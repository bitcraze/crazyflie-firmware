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
#include "platform_defaults.h"
#include "test_support.h"

// #define DEBUG_ME

#ifdef DEBUG_ME
#define DEBUG_MODULE "SUPST"
#include "debug.h"
#include "cfassert.h"
#endif

static const char* const stateNames[] = {
  "Not initialized",
  "Pre-flight checks not passed",
  "Pre-flight checks passed",
  "Ready to fly",
  "Flying",
  "Landed",
  "Reset",
  "Warning, level out",
  "Exception, free fall",
  "Locked",
  "Crashed",
};
static_assert(sizeof(stateNames) / sizeof(stateNames[0]) == supervisorState_NrOfStates);

static const char* const conditionNames[] = {
  "armed",
  "isFlying",
  "isTumbled",
  "commanderWdtWarning",
  "commanderWdtTimeout",
  "emergencyStop",
  "isCrashed",
  "landingTimeout",
};
static_assert(sizeof(conditionNames) / sizeof(conditionNames[0]) == supervisorCondition_NrOfConditions);

#if SUPERVISOR_TUMBLE_CHECK_ENABLE
  #define SUPERVISOR_CB_CONF_IS_TUMBLED (SUPERVISOR_CB_IS_TUMBLED)
#else
  #define SUPERVISOR_CB_CONF_IS_TUMBLED (SUPERVISOR_CB_NONE)
#endif

// State transition definitions
static SupervisorStateTransition_t transitionsNotInitialized[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,
    .triggerCombiner = supervisorAlways,
    .blockerCombiner = supervisorNever,
  }
};

static SupervisorStateTransition_t transitionsPreFlChecksNotPassed[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStatePreFlChecksPassed,

    .triggerCombiner = supervisorAlways,

    .blockers = SUPERVISOR_CB_CONF_IS_TUMBLED,
    .negatedBlockers = SUPERVISOR_CB_NONE,
    .blockerCombiner = supervisorAny,
  }
};

static SupervisorStateTransition_t transitionsPreFlChecksPassed[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggers = SUPERVISOR_CB_CONF_IS_TUMBLED,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateReadyToFly,

    .triggers = SUPERVISOR_CB_ARMED,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockers = SUPERVISOR_CB_CONF_IS_TUMBLED,
    .negatedBlockers = SUPERVISOR_CB_NONE,
    .blockerCombiner = supervisorAny,
  },
};

static SupervisorStateTransition_t transitionsReadyToFly[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggers = SUPERVISOR_CB_CONF_IS_TUMBLED,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateFlying,

    .triggers = SUPERVISOR_CB_IS_FLYING,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  }
};

static SupervisorStateTransition_t transitionsFlying[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateCrashed,

    .triggers = SUPERVISOR_CB_CONF_IS_TUMBLED,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateWarningLevelOut,

    .triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateLanded,

    .triggers = SUPERVISOR_CB_NONE,
    .negatedTriggers = SUPERVISOR_CB_IS_FLYING,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  }
};

static SupervisorStateTransition_t transitionsLanded[] = {
  {
    .newState = supervisorStateReset,

    .triggers = SUPERVISOR_CB_LANDING_TIMEOUT,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateFlying,

    .triggers = SUPERVISOR_CB_IS_FLYING,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  }
};

static SupervisorStateTransition_t transitionsReset[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggerCombiner = supervisorAlways,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsWarningLevelOut[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_CONF_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateFlying,

    .triggers = SUPERVISOR_CB_NONE,
    .negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsExceptFreeFall[] = {
  {
    .newState = supervisorStateLocked,

    .triggerCombiner = supervisorAlways,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsLocked[] = {
  {
    .newState = supervisorStateLocked,

    .triggerCombiner = supervisorNever,

    .blockerCombiner = supervisorAlways,
  },
};


static SupervisorStateTransition_t transitionsTumbled[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggers = SUPERVISOR_CB_NONE,
    .negatedTriggers = SUPERVISOR_CB_CRASHED | SUPERVISOR_CB_IS_TUMBLED,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever
  },
  {
    .newState = supervisorStateLocked,

    .triggers = SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever
  },
};

SupervisorStateTransitionList_t transitionLists[] = {
  {SUPERVISOR_TRANSITION_ENTRY(transitionsNotInitialized)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsPreFlChecksNotPassed)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsPreFlChecksPassed)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsReadyToFly)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsFlying)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsLanded)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsReset)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsWarningLevelOut)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsExceptFreeFall)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsLocked)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsTumbled)},
};
static_assert(sizeof(transitionLists) / sizeof(transitionLists[0]) == supervisorState_NrOfStates);


bool supervisorStateMachineInit() {
  if (sizeof(supervisorState_t) != sizeof(transitionLists)) {
    return false;
  }

  return true;
}

static bool areAllSet(const supervisorConditionBits_t conditions, const supervisorConditionBits_t requirements) {
    return (~conditions & requirements) == 0;
}

static bool isAnySet(const supervisorConditionBits_t conditions, const supervisorConditionBits_t requirements) {
    return (conditions & requirements) != 0;
}

static bool areConditionsMet(const supervisorConditionBits_t conditions, const supervisorConditionBits_t requirements, const supervisorConditionBits_t negRequirements, const SupervisorConditionCombiner_t combiner) {
  bool result = false;

  switch(combiner) {
    case supervisorAll:
      result = areAllSet(conditions, requirements) && !isAnySet(conditions, negRequirements);
      break;
    case supervisorAny:
      result = isAnySet(conditions, requirements) || !areAllSet(conditions, negRequirements);
      break;
    case supervisorAlways:
      result = true;
      break;
    case supervisorNever:
      result = false;
      break;
    default:
      break;
  }

  return result;
}

TESTABLE_STATIC supervisorState_t findTransition(const supervisorState_t currentState, const supervisorConditionBits_t conditions, const SupervisorStateTransitionList_t* transitions) {
  supervisorState_t newState = currentState;
  for (int i = 0; i < transitions->length; i++) {
    const SupervisorStateTransition_t* transitionDef = &transitions->transitionList[i];

    const bool isTriggerMatch = areConditionsMet(conditions, transitionDef->triggers, transitionDef->negatedTriggers, transitionDef->triggerCombiner);
    const bool isBlockerMatch = areConditionsMet(conditions, transitionDef->blockers, transitionDef->negatedBlockers, transitionDef->blockerCombiner);

    const bool isStateTransitionValid = isTriggerMatch && !isBlockerMatch;
    if (isStateTransitionValid) {
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

const char* supervisorGetStateName(const supervisorState_t state) {
  return stateNames[state];
}

const char* supervisorGetConditionName(const supervisorState_t condition) {
  return conditionNames[condition];
}
