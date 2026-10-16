/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * test_motors_sim.c - unit tests of the Actuator shim of the Simmyflie,
 * against a mocked Physics engine manager.
 */

// File under test. Named by its path under src/: src/drivers/src has no
// trailing slash in gcc.yml, so the test build does not find it from the
// header name.
// @MODULE "drivers/src/motors_sim.c"
#include "motors_sim.h"

#include "unity.h"
#include "mock_physics_engine_manager.h"

void setUp(void)
{
  motorsInit(0);
}

void tearDown(void)
{
}

void testThatSetRatioPassesIdAndRatioOnUnchanged(void)
{
  // Fixture
  physicsEngineManagerSetMotorRatio_Expect(0, 0);
  physicsEngineManagerSetMotorRatio_Expect(1, 1);
  physicsEngineManagerSetMotorRatio_Expect(2, 34200);
  physicsEngineManagerSetMotorRatio_Expect(3, 65535);

  // Test
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 1);
  motorsSetRatio(MOTOR_M3, 34200);
  motorsSetRatio(MOTOR_M4, 65535);

  // Assert
  // Verified by the mock: each call passed on once, in order
}

void testThatEveryCallToSetRatioIsPassedOn(void)
{
  // Fixture: the same value twice is passed on twice
  physicsEngineManagerSetMotorRatio_Expect(2, 1000);
  physicsEngineManagerSetMotorRatio_Expect(2, 1000);
  physicsEngineManagerSetMotorRatio_Expect(2, 2000);

  // Test
  motorsSetRatio(MOTOR_M3, 1000);
  motorsSetRatio(MOTOR_M3, 1000);
  motorsSetRatio(MOTOR_M3, 2000);

  // Assert
  // Verified by the mock
}

void testThatStopPassesZeroForEachMotor(void)
{
  // Fixture
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    physicsEngineManagerSetMotorRatio_Expect(id, 30000);
    motorsSetRatio(id, 30000);
  }

  physicsEngineManagerSetMotorRatio_Expect(0, 0);
  physicsEngineManagerSetMotorRatio_Expect(1, 0);
  physicsEngineManagerSetMotorRatio_Expect(2, 0);
  physicsEngineManagerSetMotorRatio_Expect(3, 0);

  // Test
  motorsStop();

  // Assert
  // Verified by the mock
}

void testThatStopStoresZeros(void)
{
  // Fixture
  physicsEngineManagerSetMotorRatio_Ignore();
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    motorsSetRatio(id, 30000);
  }

  // Test
  motorsStop();

  // Assert
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    TEST_ASSERT_EQUAL_UINT16(0, motorsGetRatio(id));
  }
}

void testThatAnIdOutsideTheMotorsIsNotPassedOn(void)
{
  // Fixture: no call is expected, the mock fails on any call

  // Test
  motorsSetRatio(NBR_OF_MOTORS, 1000);
  motorsSetRatio(UINT32_MAX, 1000);

  // Assert
  // Verified by the mock
}

void testThatAnIdOutsideTheMotorsStoresNothing(void)
{
  // Fixture
  physicsEngineManagerSetMotorRatio_Ignore();
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    motorsSetRatio(id, 1000 + id);
  }

  // Test
  motorsSetRatio(NBR_OF_MOTORS, 5000);

  // Assert
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    TEST_ASSERT_EQUAL_UINT16(1000 + id, motorsGetRatio(id));
  }
  TEST_ASSERT_EQUAL_UINT16(0, motorsGetRatio(NBR_OF_MOTORS));
}

void testThatGetRatioReturnsWhatWasStored(void)
{
  // Fixture
  physicsEngineManagerSetMotorRatio_Ignore();

  // Test
  motorsSetRatio(MOTOR_M1, 100);
  motorsSetRatio(MOTOR_M2, 200);
  motorsSetRatio(MOTOR_M3, 300);
  motorsSetRatio(MOTOR_M4, 65535);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(100, motorsGetRatio(MOTOR_M1));
  TEST_ASSERT_EQUAL_UINT16(200, motorsGetRatio(MOTOR_M2));
  TEST_ASSERT_EQUAL_UINT16(300, motorsGetRatio(MOTOR_M3));
  TEST_ASSERT_EQUAL_UINT16(65535, motorsGetRatio(MOTOR_M4));
}

void testThatGetRatioCallsNoManagerFunction(void)
{
  // Fixture: no call is expected, the mock fails on any call

  // Test
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    motorsGetRatio(id);
  }

  // Assert
  // Verified by the mock
}

void testThatGetRpmIsTheConstantWhateverTheRatio(void)
{
  // Fixture
  physicsEngineManagerSetMotorRatio_Ignore();
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 65535);

  // Test and assert
  for (uint32_t id = 0; id < NBR_OF_MOTORS; id++) {
    TEST_ASSERT_EQUAL_UINT16(MOTORS_SIM_RPM, motorsGetRPM(id));
  }
  TEST_ASSERT_EQUAL_UINT16(MOTORS_RPM_INVALID, motorsGetRPM(NBR_OF_MOTORS));
}
