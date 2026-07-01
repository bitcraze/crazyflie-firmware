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
    idle_thrust = cffirmware.powerDistributionGetIdleThrust()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    # control.thrust will be at a (tuned) hover-state
    assert not isCapped
    assert actual.motors.m1 == max(input.motors.m1, idle_thrust)
    assert actual.motors.m2 == max(input.motors.m2, idle_thrust)
    assert actual.motors.m3 == max(input.motors.m3, idle_thrust)
    assert actual.motors.m4 == max(input.motors.m4, idle_thrust)


def test_power_distribution_cap_when_all_negative():
    # Fixture
    input = cffirmware.motors_thrust_uncapped_t()
    input.motors.m1 = -1000
    input.motors.m2 = -2000
    input.motors.m3 = -3000
    input.motors.m4 = -4000

    actual = cffirmware.motors_thrust_pwm_t()
    idle_thrust = cffirmware.powerDistributionGetIdleThrust()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    assert not isCapped
    assert actual.motors.m1 == max(0, idle_thrust)
    assert actual.motors.m2 == max(0, idle_thrust)
    assert actual.motors.m3 == max(0, idle_thrust)
    assert actual.motors.m4 == max(0, idle_thrust)


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
    idle_thrust = cffirmware.powerDistributionGetIdleThrust()

    # Test
    isCapped = cffirmware.powerDistributionCap(input, actual)

    # Assert
    assert isCapped
    assert actual.motors.m1 == max(0, idle_thrust)
    assert actual.motors.m2 == max(0, idle_thrust)
    assert actual.motors.m3 == max(1000 - 10, idle_thrust)
    assert actual.motors.m4 == max(0xffff, idle_thrust)


def test_power_distribution_force_clamps_out_of_range_normalized_forces():
    """controlModeForce must clamp normalized forces to [0, 1] before scaling.

    Out-of-range values (negative or > 1.0) must not produce a PWM-scale
    thrust outside [0, UINT16_MAX] — this is the per-path clamp that
    powerDistributionForceTorque deliberately does *not* apply (it leaves
    headroom for coordinated capping instead).
    """
    control = cffirmware.control_t()
    control.controlMode = cffirmware.controlModeForce
    # Intentionally out of the documented [0.0, 1.0] range on every motor.
    control.normalizedForces[0] = -0.5
    control.normalizedForces[1] = 1.5
    control.normalizedForces[2] = 2.0
    control.normalizedForces[3] = -10.0

    actual = cffirmware.motors_thrust_uncapped_t()
    cffirmware.powerDistribution(control, actual)

    assert actual.motors.m1 == 0
    assert actual.motors.m2 == 0xffff
    assert actual.motors.m3 == 0xffff
    assert actual.motors.m4 == 0


def test_power_distribution_force_torque_outrange_then_cap_clamps_pwm_duty():
    """Oversized force/torque must still yield a safe 16-bit PWM duty.

    powerDistributionForceTorque may leave per-motor thrust above UINT16_MAX
    so Cap can reduce all motors together. The final motor PWM duties written
    toward the hardware (motorsSetRatio) must still be saturated to
    [idle, 0xFFFF] — push a large collective thrust and confirm the clamp.
    """
    control = cffirmware.control_t()
    control.controlMode = cffirmware.controlModeForceTorque
    # Far beyond a single motor's nominal max force (THRUST_MAX is ~0.1 N).
    control.thrustSi = 50.0
    control.torqueX = 0.0
    control.torqueY = 0.0
    control.torqueZ = 0.0

    uncapped = cffirmware.motors_thrust_uncapped_t()
    cffirmware.powerDistribution(control, uncapped)

    # Allocation is allowed to request more than full-scale PWM so Cap can run.
    assert max(uncapped.motors.m1, uncapped.motors.m2,
               uncapped.motors.m3, uncapped.motors.m4) > 0xffff

    pwm = cffirmware.motors_thrust_pwm_t()
    idle_thrust = cffirmware.powerDistributionGetIdleThrust()
    is_capped = cffirmware.powerDistributionCap(uncapped, pwm)

    assert is_capped
    for duty in (pwm.motors.m1, pwm.motors.m2, pwm.motors.m3, pwm.motors.m4):
        assert idle_thrust <= duty <= 0xffff
    # Symmetric hover demand → all motors sit at full-scale after coordinated cap.
    assert pwm.motors.m1 == 0xffff
    assert pwm.motors.m2 == 0xffff
    assert pwm.motors.m3 == 0xffff
    assert pwm.motors.m4 == 0xffff


def test_power_distribution_cap_saturates_single_motor_above_uint16():
    """Even a single extreme duty is clamped to UINT16_MAX on the PWM output."""
    inp = cffirmware.motors_thrust_uncapped_t()
    inp.motors.m1 = 10**9
    inp.motors.m2 = 100
    inp.motors.m3 = 100
    inp.motors.m4 = 100

    actual = cffirmware.motors_thrust_pwm_t()
    idle_thrust = cffirmware.powerDistributionGetIdleThrust()
    is_capped = cffirmware.powerDistributionCap(inp, actual)

    assert is_capped
    assert actual.motors.m1 == 0xffff
    # Coordinated reduction pulls the other motors down; never below idle and
    # never above full-scale PWM.
    for duty in (actual.motors.m2, actual.motors.m3, actual.motors.m4):
        assert idle_thrust <= duty <= 0xffff
