#!/usr/bin/env python

import cffirmware

def test_basic_power_distribution():

    cffirmware.controllerMellingerInit()

    motorPower = cffirmware.motors_thrust_uncapped_t()
    control = cffirmware.control_t()
    control.thrust = 10
    control.roll = 0
    control.pitch = 0
    control.yaw = 0

    cffirmware.powerDistribution(control, motorPower)
    # control.thrust will be at a (tuned) hover-state
    assert motorPower.motors.m1 == control.thrust
    assert motorPower.motors.m2 == control.thrust
    assert motorPower.motors.m3 == control.thrust
    assert motorPower.motors.m4 == control.thrust
