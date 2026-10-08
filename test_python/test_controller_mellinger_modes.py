"""Regression tests for Mellinger controller mode selection.

Mixed-mode cases verify branch selection, not complete velocity control.
"""
import math

import cffirmware
import pytest


ABS = cffirmware.modeAbs
VELOCITY = cffirmware.modeVelocity
DISABLED = cffirmware.modeDisable


def _run_controller(x_mode, y_mode, z_mode, roll=0.0, pitch=0.0):
    controller = cffirmware.controllerMellinger_t()
    cffirmware.controllerMellingerInit(controller)

    control = cffirmware.control_t()
    setpoint = cffirmware.setpoint_t()
    setpoint.mode.x = x_mode
    setpoint.mode.y = y_mode
    setpoint.mode.z = z_mode
    setpoint.mode.roll = ABS
    setpoint.mode.pitch = ABS
    setpoint.mode.yaw = ABS
    setpoint.attitude.roll = roll
    setpoint.attitude.pitch = pitch
    setpoint.attitude.yaw = 0.0
    setpoint.thrust = 32000.0

    # Nonzero horizontal position errors make accidental entry into the
    # XYZ position-control branch observable. Z is at its target altitude.
    setpoint.position.x = 0.4
    setpoint.position.y = 0.3
    setpoint.position.z = 1.0

    state = cffirmware.state_t()
    state.position.z = 1.0
    state.attitudeQuaternion.w = 1.0
    sensors = cffirmware.sensorData_t()

    # The SWIG constructors zero-initialize other fields. Tick 100 executes
    # the attitude controller, as in the existing Mellinger Python test.
    cffirmware.controllerMellinger(
        controller, control, setpoint, sensors, state, 100
    )

    assert control.controlMode == cffirmware.controlModeLegacy
    values = (
        control.thrust, control.roll, control.pitch, control.yaw,
        controller.z_axis_desired.x,
        controller.z_axis_desired.y,
        controller.z_axis_desired.z,
    )
    assert all(math.isfinite(value) for value in values)
    assert control.thrust > 0.0
    return controller, control


def _assert_level(controller, control):
    assert (
        controller.z_axis_desired.x,
        controller.z_axis_desired.y,
        controller.z_axis_desired.z,
    ) == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
    assert control.roll == pytest.approx(0.0, abs=1e-6)
    assert control.pitch == pytest.approx(0.0, abs=1e-6)


def test_full_position_mode_responds_to_horizontal_error():
    controller, _ = _run_controller(ABS, ABS, ABS)
    assert controller.z_axis_desired.x > 0.0
    assert controller.z_axis_desired.y > 0.0


def test_manual_mode_preserves_direct_thrust_and_attitude_command():
    controller, control = _run_controller(
        DISABLED, DISABLED, DISABLED, roll=12.0, pitch=-7.0
    )
    assert control.thrust == pytest.approx(32000.0)
    assert abs(controller.z_axis_desired.x) > 0.01
    assert abs(controller.z_axis_desired.y) > 0.01


@pytest.mark.parametrize("xy_mode", [VELOCITY, DISABLED])
def test_existing_level_fallback_with_absolute_altitude(xy_mode):
    # Covers hover-assist-style input and a disabled-XY fallback input.
    # It does not exercise the commander's timeout logic itself.
    controller, control = _run_controller(xy_mode, xy_mode, ABS)
    _assert_level(controller, control)


@pytest.mark.parametrize("y_mode", [DISABLED, VELOCITY])
def test_nonabsolute_y_prevents_xyz_position_control(y_mode):
    controller, control = _run_controller(ABS, y_mode, ABS)
    _assert_level(controller, control)


def test_velocity_z_prevents_xyz_position_control():
    # Only the choice of horizontal fallback is asserted here. Handling a
    # Z-velocity command without a valid altitude target remains unresolved.
    controller, control = _run_controller(ABS, ABS, VELOCITY)
    _assert_level(controller, control)


def test_disabled_z_uses_manual_attitude_when_manual_conditions_match():
    reference, reference_control = _run_controller(
        DISABLED, DISABLED, DISABLED, roll=12.0, pitch=-7.0
    )
    controller, control = _run_controller(
        ABS, ABS, DISABLED, roll=12.0, pitch=-7.0
    )
    for field in ("x", "y", "z"):
        assert getattr(controller.z_axis_desired, field) == pytest.approx(
            getattr(reference.z_axis_desired, field), abs=1e-6
        )
    for field in ("thrust", "roll", "pitch", "yaw"):
        assert getattr(control, field) == pytest.approx(
            getattr(reference_control, field), abs=1e-6
        )
