// File under test supervisor_state_machine.h
#include "supervisor_state_machine.h"

#include <stdlib.h>
#include <stdbool.h>

#include "unity.h"

// Function under test
supervisorState_t findTransition(const supervisorState_t currentState, const supervisorConditionBits_t triggerBitField, const SupervisorStateTransitionList_t* transitions);

// Helpers
static void assertStateTransition(supervisorConditionBits_t conditions,
  supervisorConditionBits_t triggers, supervisorConditionBits_t negatedTriggers, SupervisorConditionCombiner_t triggerCombiner,
  supervisorConditionBits_t blockers, supervisorConditionBits_t negatedBlockers, SupervisorConditionCombiner_t blockerCombiner);

static void assertNoStateTransition(supervisorConditionBits_t conditions,
  supervisorConditionBits_t triggers, supervisorConditionBits_t negatedTriggers, SupervisorConditionCombiner_t triggerCombiner,
  supervisorConditionBits_t blockers, supervisorConditionBits_t negatedBlockers, SupervisorConditionCombiner_t blockerCombiner);

void setUp(void) {
  // Empty
}

void tearDown(void) {
  // Empty
}

void testTransitionWithNoConditionsTriggerAlways(void) {
  // Fixture
  supervisorConditionBits_t conditions = 123;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionWithNoConditionsTriggerNever(void) {
  // Fixture
  supervisorConditionBits_t conditions = 123;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorNever;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionWithNoConditionsBlockAlways(void) {
  // Fixture
  supervisorConditionBits_t conditions = 123;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAlways;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneRequiredConditionMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneRequiredConditionNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneNegatedRequiredConditionNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneNegatedRequiredConditionMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredConditionsMetWithOtherBitsSet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED | SUPERVISOR_CB_EMERGENCY_STOP;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredConditionsOneMissing(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredConditionsOneMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAny;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredConditionsNoneMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAny;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiNegatedRequiredConditionOneNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiNegatedRequiredConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiMixedRequiredConditionsAllMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiMixedRequiredConditionsOnePositiveNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiMixedRequiredConditionsOneNegativeNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiMixedOnePositiveRequirementMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAny;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiMixedNoRequirementMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_FLYING | SUPERVISOR_CB_ARMED | SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_IS_FLYING | SUPERVISOR_CB_ARMED | SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAny;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiMixedOneNegativeRequirementMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_FLYING | SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_IS_FLYING | SUPERVISOR_CB_ARMED | SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAny;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredConditionsOneMissingButOtherBitsSet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorNever;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneProhibitedConditionMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneProhibitedConditionNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionOneProhibitedConditionNotMetWithOtherBitsSet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiProhibitedConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiProhibitedConditionsOneNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiProhibitedConditionsAllNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAll;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiProhibitedConditionsAllMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAll;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiNegativeProhibitedConditionsNoneMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiNegativeProhibitedConditionsOneMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiNegativeProhibitedConditionsOneNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAll;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiNegativeProhibitedConditionsAllMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAlways;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_IS_TUMBLED;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAll;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredAndProhibitedConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredAndProhibitedConditionsOneRequiredNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredAndProhibitedConditionsOneProhibitedNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testTransitionMultiRequiredAndProhibitedConditionsMultipleNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP;

  supervisorConditionBits_t triggers = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t negatedTriggers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t triggerCombiner = supervisorAll;

  supervisorConditionBits_t blockers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t negatedBlockers = SUPERVISOR_CB_NONE;
  SupervisorConditionCombiner_t blockerCombiner = supervisorAny;

  // Test
  // Assert
  assertNoStateTransition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner);
}

void testFirstValidTransitionIsChosen(void) {
    // Fixture
  const supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP;

  const supervisorState_t currentState = supervisorStateFlying;
  const supervisorState_t expected = supervisorStateExceptFreeFall;

  SupervisorStateTransition_t transitions[] = {
    {
      .newState = supervisorStateLanded,

      .triggers = SUPERVISOR_CB_IS_TUMBLED,
      .negatedTriggers = SUPERVISOR_CB_NONE,
      .triggerCombiner = supervisorAll,

      .blockers = SUPERVISOR_CB_EMERGENCY_STOP,
      .negatedBlockers = SUPERVISOR_CB_NONE,
      .blockerCombiner = supervisorAny,
    },
    { // We expect this state to be chosen
      .newState = expected,

      .triggers = SUPERVISOR_CB_IS_TUMBLED,
      .negatedTriggers = SUPERVISOR_CB_NONE,
      .triggerCombiner = supervisorAll,

      .blockerCombiner = supervisorNever,
    },
    {
      .newState = supervisorStatePreFlChecksNotPassed,

      .triggers = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
      .negatedTriggers = SUPERVISOR_CB_NONE,
      .triggerCombiner = supervisorAll,

      .blockerCombiner = supervisorNever,
    }
  };

  SupervisorStateTransitionList_t transitionsDef = {SUPERVISOR_TRANSITION_ENTRY(transitions)};

  // Test
  const supervisorState_t actual = findTransition(currentState, conditions, &transitionsDef);

  // ASSERT
  TEST_ASSERT_EQUAL(expected, actual);
}

// Helpers ////////////////////////////////////////////////

static bool check_state_transition(supervisorConditionBits_t conditions,
  supervisorConditionBits_t triggers, supervisorConditionBits_t negatedTriggers, SupervisorConditionCombiner_t triggerCombiner,
  supervisorConditionBits_t blockers, supervisorConditionBits_t negatedBlockers, SupervisorConditionCombiner_t blockerCombiner) {

  // Fixture
  const supervisorState_t currentState = supervisorStateFlying;

  SupervisorStateTransition_t transitions[] = {
    {
      .newState = supervisorStateLanded,

      .triggers = triggers,
      .negatedTriggers = negatedTriggers,
      .triggerCombiner = triggerCombiner,

      .blockers = blockers,
      .negatedBlockers = negatedBlockers,
      .blockerCombiner = blockerCombiner,
    }
  };

  SupervisorStateTransitionList_t transitionsDef = {SUPERVISOR_TRANSITION_ENTRY(transitions)};

  // Test
  const supervisorState_t newState = findTransition(currentState, conditions, &transitionsDef);

  // We did get a transition if the new state is different from the current state
  return newState != currentState;
}

static void assertStateTransition(supervisorConditionBits_t conditions,
  supervisorConditionBits_t triggers, supervisorConditionBits_t negatedTriggers, SupervisorConditionCombiner_t triggerCombiner,
  supervisorConditionBits_t blockers, supervisorConditionBits_t negatedBlockers, SupervisorConditionCombiner_t blockerCombiner) {
  TEST_ASSERT_TRUE(check_state_transition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner));
}

static void assertNoStateTransition(supervisorConditionBits_t conditions,
  supervisorConditionBits_t triggers, supervisorConditionBits_t negatedTriggers, SupervisorConditionCombiner_t triggerCombiner,
  supervisorConditionBits_t blockers, supervisorConditionBits_t negatedBlockers, SupervisorConditionCombiner_t blockerCombiner) {
  TEST_ASSERT_FALSE(check_state_transition(conditions, triggers, negatedTriggers, triggerCombiner, blockers, negatedBlockers, blockerCombiner));
}
