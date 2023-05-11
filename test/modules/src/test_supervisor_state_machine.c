// File under test supervisor_state_machine.h
#include "supervisor_state_machine.h"

#include <stdlib.h>
#include <stdbool.h>

#include "unity.h"

// Function under test
supervisorState_t findTransition(const supervisorState_t currentState, const supervisorConditionBits_t triggerBitField, const SupervisorStateTransitionList_t* transitions);

// Helpers
static void assertStateTransition(supervisorConditionBits_t conditions, supervisorConditionBits_t mustBeSet, supervisorConditionBits_t mustNotBeSet);
static void assertNoStateTransition(supervisorConditionBits_t conditions, supervisorConditionBits_t mustBeSet, supervisorConditionBits_t mustNotBeSet);

void setUp(void) {
  // Empty
}

void tearDown(void) {
  // Empty
}

void testTransitionWithNoConditions(void) {
  // Fixture
  supervisorConditionBits_t conditions = 123;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneRequiredConditionMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneRequiredConditionNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredConditionsMetWithOtherBitsSet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_ARMED | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredConditionsOneMissing(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredConditionsOneMissingButOtherBitsSet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_NONE;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneProhibitedConditionMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneProhibitedConditionNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneProhibitedConditionNotMetWithOtherBitsSet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiProhibitedConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = 0;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiProhibitedConditionsOneNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_NONE;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredAndProhibitedConditionsMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_EMERGENCY_STOP;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredAndProhibitedConditionsOneRequiredNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_EMERGENCY_STOP;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredAndProhibitedConditionsOneProhibitedNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_EMERGENCY_STOP;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiRequiredAndProhibitedConditionsMultipleNotMet(void) {
  // Fixture
  supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP;
  supervisorConditionBits_t mustBeSet = SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_TUMBLED;
  supervisorConditionBits_t mustNotBeSet = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_EMERGENCY_STOP;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testFirstValidTransitionIsChosen(void) {
    // Fixture
  const supervisorConditionBits_t conditions = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP;

  const supervisorState_t currentState = supervisorStateFlying;
  const supervisorState_t expected = supervisorStateExceptFreeFall;

  SupervisorStateTransition_t transitions[] = {
    {
      .newState = supervisorStateLanded,
      .mustBeSet = SUPERVISOR_CB_IS_TUMBLED,
      .mustNotBeSet = SUPERVISOR_CB_EMERGENCY_STOP
    },
    { // We expect this state to be chosen
      .newState = expected,
      .mustBeSet = SUPERVISOR_CB_IS_TUMBLED,
      .mustNotBeSet = SUPERVISOR_CB_NONE
    },
    {
      .newState = supervisorStatePreFlChecksNotPassed,
      .mustBeSet = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
      .mustNotBeSet = SUPERVISOR_CB_NONE
    }
  };

  SupervisorStateTransitionList_t transitionsDef = {SUPERVISOR_TRANSITION_ENTRY(transitions)};

  // Test
  const supervisorState_t actual = findTransition(currentState, conditions, &transitionsDef);

  // ASSERT
  TEST_ASSERT_EQUAL(expected, actual);
}

// Helpers ////////////////////////////////////////////////

static bool check_state_transition(supervisorConditionBits_t conditions, supervisorConditionBits_t mustBeSet, supervisorConditionBits_t mustNotBeSet) {
  // Fixture
  const supervisorState_t currentState = supervisorStateFlying;

  SupervisorStateTransition_t transitions[] = {
    {
      .newState = supervisorStateLanded,
      .mustBeSet = mustBeSet,
      .mustNotBeSet = mustNotBeSet
    }
  };

  SupervisorStateTransitionList_t transitionsDef = {SUPERVISOR_TRANSITION_ENTRY(transitions)};

  // Test
  const supervisorState_t newState = findTransition(currentState, conditions, &transitionsDef);

  // We did get a transition if the new state is different from the current state
  return newState != currentState;
}

static void assertStateTransition(supervisorConditionBits_t conditions, supervisorConditionBits_t mustBeSet, supervisorConditionBits_t mustNotBeSet) {
  TEST_ASSERT_TRUE(check_state_transition(conditions, mustBeSet, mustNotBeSet))
}

static void assertNoStateTransition(supervisorConditionBits_t conditions, supervisorConditionBits_t mustBeSet, supervisorConditionBits_t mustNotBeSet) {
  TEST_ASSERT_FALSE(check_state_transition(conditions, mustBeSet, mustNotBeSet))
}
