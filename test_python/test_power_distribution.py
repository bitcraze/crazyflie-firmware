#!/usr/bin/env python

import cffirmware


def test_power_distribution_legacy_for_hover():
    # Fixture
    control = cffirmware.control_t()
    control.thrust = 10
    control.roll = 0
    control.pitch = 0
    control.yaw = 0
    control.controlMode = cffirmware.controlModeLegacy

    actual = cffirmware.motors_thrust_uncapped_t()

    # Test
    cffirmware.powerDistribution(control, actual)

    # Assert
    assert actual.motors.m1 == control.thrust
    assert actual.motors.m2 == control.thrust
    assert actual.motors.m3 == control.thrust
    assert actual.motors.m4 == control.thrust


def test_power_distribution_legacy_for_roll():
    # Fixture
    control = cffirmware.control_t()
    control.thrust = 0
    control.roll = 10
    control.pitch = 0
    control.yaw = 0
    control.controlMode = cffirmware.controlModeLegacy

    actual = cffirmware.motors_thrust_uncapped_t()

    # Test
    cffirmware.powerDistribution(control, actual)

    # Assert
    assert actual.motors.m1 == -control.roll / 2
    assert actual.motors.m2 == -control.roll / 2
    assert actual.motors.m3 == control.roll / 2
    assert actual.motors.m4 == control.roll / 2


def test_power_distribution_legacy_for_pitch():
    # Fixture
    control = cffirmware.control_t()
    control.thrust = 0
    control.roll = 0
    control.pitch = 12
    control.yaw = 0
    control.controlMode = cffirmware.controlModeLegacy

    actual = cffirmware.motors_thrust_uncapped_t()

    # Test
    cffirmware.powerDistribution(control, actual)

    # Assert
    assert actual.motors.m1 == control.pitch / 2
    assert actual.motors.m2 == -control.pitch / 2
    assert actual.motors.m3 == -control.pitch / 2
    assert actual.motors.m4 == control.pitch / 2


def test_power_distribution_legacy_for_yaw():
    # Fixture
    control = cffirmware.control_t()
    control.thrust = 0
    control.roll = 0
    control.pitch = 0
    control.yaw = 20
    control.controlMode = cffirmware.controlModeLegacy

    actual = cffirmware.motors_thrust_uncapped_t()

    # Test
    cffirmware.powerDistribution(control, actual)

    # Assert
    assert actual.motors.m1 == control.yaw
    assert actual.motors.m2 == -control.yaw
    assert actual.motors.m3 == control.yaw
    assert actual.motors.m4 == -control.yaw


def test_power_distribution_force_torque():
    # Fixture
    control = cffirmware.control_t()
    control.thrustSi = 0.1
    control.torqueX = 0
    control.torqueY = 0
    control.torqueZ = 0
    control.controlMode = cffirmware.controlModeForceTorque

    actual = cffirmware.motors_thrust_uncapped_t()

    # Test
    cffirmware.powerDistribution(control, actual)

    # Assert
    # For now only check that we get a non zero output
    assert actual.motors.m1 > 0
    assert actual.motors.m2 > 0
    assert actual.motors.m3 > 0
    assert actual.motors.m4 > 0


def test_power_distribution_cap_when_in_range():
    # Fixture
    input = cffirmware.motors_thrust_uncapped_t()
    input.motors.m1 = 1000
    input.motors.m2 = 2000
    input.motors.m3 = 3000
    input.motors.m4 = 4000

    actual = cffirmware.motors_thrust_pwm_t()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    # control.thrust will be at a (tuned) hover-state
    assert not isCapped
    assert actual.motors.m1 == input.motors.m1
    assert actual.motors.m2 == input.motors.m2
    assert actual.motors.m3 == input.motors.m3
    assert actual.motors.m4 == input.motors.m4


def test_power_distribution_cap_when_all_negative():
    # Fixture
    input = cffirmware.motors_thrust_uncapped_t()
    input.motors.m1 = -1000
    input.motors.m2 = -2000
    input.motors.m3 = -3000
    input.motors.m4 = -4000

    actual = cffirmware.motors_thrust_pwm_t()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    assert not isCapped
    assert actual.motors.m1 == 0
    assert actual.motors.m2 == 0
    assert actual.motors.m3 == 0
    assert actual.motors.m4 == 0


def test_power_distribution_cap_when_all_above_range():
    # Fixture
    input = cffirmware.motors_thrust_uncapped_t()
    input.motors.m1 = 0xffff + 1
    input.motors.m2 = 0xffff + 1
    input.motors.m3 = 0xffff + 1
    input.motors.m4 = 0xffff + 1

    actual = cffirmware.motors_thrust_pwm_t()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    assert isCapped
    assert actual.motors.m1 == 0xffff
    assert actual.motors.m2 == 0xffff
    assert actual.motors.m3 == 0xffff
    assert actual.motors.m4 == 0xffff


def test_power_distribution_cap_reduces_thrust_equally_much():
    # Fixture
    input = cffirmware.motors_thrust_uncapped_t()
    input.motors.m1 = 0xffff + 1
    input.motors.m2 = 0xffff + 5
    input.motors.m3 = 0xffff + 10
    input.motors.m4 = 0xffff + 15

    actual = cffirmware.motors_thrust_pwm_t()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    assert isCapped
    assert actual.motors.m1 == 0xffff - 14
    assert actual.motors.m2 == 0xffff - 10
    assert actual.motors.m3 == 0xffff - 5
    assert actual.motors.m4 == 0xffff - 0


def test_power_distribution_cap_reduces_thrust_equally_much_with_lower_cap():
    # Fixture
    input = cffirmware.motors_thrust_uncapped_t()
    input.motors.m1 = 0
    input.motors.m2 = 5
    input.motors.m3 = 1000
    input.motors.m4 = 0xffff + 10

    actual = cffirmware.motors_thrust_pwm_t()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    assert isCapped
    assert actual.motors.m1 == 0
    assert actual.motors.m2 == 0
    assert actual.motors.m3 == 1000 - 10
    assert actual.motors.m4 == 0xffff
