// File under test supervisor_state_machine.h
#include "supervisor_state_machine.h"

#include <stdlib.h>
#include <stdbool.h>

#include "unity.h"

// Function under test
supervisorState_t findTransition(const supervisorState_t currentState, const supervisorConditionBit_t triggerBitField, const SupervisorStateTransitionList_t* transitions);

// Helpers
static void assertStateTransition(supervisorConditionBit_t conditions, supervisorConditionBit_t mustBeSet, supervisorConditionBit_t mustNotBeSet);
static void assertNoStateTransition(supervisorConditionBit_t conditions, supervisorConditionBit_t mustBeSet, supervisorConditionBit_t mustNotBeSet);

void setUp(void) {
  // Empty
}

void tearDown(void) {
  // Empty
}

void testTransitionWithNoConditions(void) {
  // Fixture
  supervisorConditionBit_t conditions = 123;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_NONE;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOnePositiveConditionMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_CHARGER_CONNECTED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOnePositiveConditionNotMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_CHARGER_CONNECTED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveConditionsMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveConditionsMetWithOtherPositives(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_ARMED | SUPERVISOR_TB_EMERGENCY_STOP;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveConditionsOneMissing(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveConditionsOneMissingButOtherPositives(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_NONE;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneNegativeConditionMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = 0;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_NONE;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneNegativeConditionNotMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_NONE;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionOneNegativeConditionNotMetWithOtherPositives(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_NONE;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiNegativeConditionsMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = 0;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_NONE;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_IS_TUMBLED;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiNegativeConditionsOneNotMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_NONE;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_IS_TUMBLED;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveAndNegativeConditionsMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_ARMED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_ARMED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_EMERGENCY_STOP;

  // Test
  // Assert
  assertStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveAndNegativeConditionsOnePositiveNotMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_ARMED;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_ARMED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_EMERGENCY_STOP;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveAndNegativeConditionsOneNegativeNotMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_ARMED | SUPERVISOR_TB_IS_TUMBLED | SUPERVISOR_TB_EMERGENCY_STOP;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_ARMED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_EMERGENCY_STOP;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testTransitionMultiPositiveAndNegativeConditionsMultipleNotMet(void) {
  // Fixture
  supervisorConditionBit_t conditions = SUPERVISOR_TB_IS_TUMBLED | SUPERVISOR_TB_EMERGENCY_STOP;
  supervisorConditionBit_t mustBeSet = SUPERVISOR_TB_ARMED | SUPERVISOR_TB_IS_TUMBLED;
  supervisorConditionBit_t mustNotBeSet = SUPERVISOR_TB_CHARGER_CONNECTED | SUPERVISOR_TB_EMERGENCY_STOP;

  // Test
  // Assert
  assertNoStateTransition(conditions, mustBeSet, mustNotBeSet);
}

void testFirstValidTransitionIsChosen(void) {
    // Fixture
  const supervisorConditionBit_t conditions = SUPERVISOR_TB_IS_TUMBLED | SUPERVISOR_TB_EMERGENCY_STOP;

  const supervisorState_t currentState = supervisorStateFlying;
  const supervisorState_t expected = supervisorStateExceptFreeFall;

  SupervisorStateTransition_t transitions[] = {
    {
      .newState = supervisorStateLanded,
      .mustBeSet = SUPERVISOR_TB_IS_TUMBLED,
      .mustNotBeSet = SUPERVISOR_TB_EMERGENCY_STOP
    },
    { // We expect this state to be chosen
      .newState = expected,
      .mustBeSet = SUPERVISOR_TB_IS_TUMBLED,
      .mustNotBeSet = SUPERVISOR_TB_NONE
    },
    {
      .newState = supervisorStatePreFlChecksNotPassed,
      .mustBeSet = SUPERVISOR_TB_IS_TUMBLED | SUPERVISOR_TB_EMERGENCY_STOP,
      .mustNotBeSet = SUPERVISOR_TB_NONE
    }
  };

  SupervisorStateTransitionList_t transitionsDef = {SUPERVISOR_TRANSITION_ENTRY(transitions)};

  // Test
  const supervisorState_t actual = findTransition(currentState, conditions, &transitionsDef);

  // ASSERT
  TEST_ASSERT_EQUAL(expected, actual);
}

// Helpers ////////////////////////////////////////////////

static bool check_state_transition(supervisorConditionBit_t conditions, supervisorConditionBit_t mustBeSet, supervisorConditionBit_t mustNotBeSet) {
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

static void assertStateTransition(supervisorConditionBit_t conditions, supervisorConditionBit_t mustBeSet, supervisorConditionBit_t mustNotBeSet) {
  TEST_ASSERT_TRUE(check_state_transition(conditions, mustBeSet, mustNotBeSet))
}

static void assertNoStateTransition(supervisorConditionBit_t conditions, supervisorConditionBit_t mustBeSet, supervisorConditionBit_t mustNotBeSet) {
  TEST_ASSERT_FALSE(check_state_transition(conditions, mustBeSet, mustNotBeSet))
}
