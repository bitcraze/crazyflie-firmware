#!/usr/bin/env python

import cffirmware

def test_controller_mellinger():

    cffirmware.controllerMellingerInit()

    motorPower = cffirmware.motors_thrust_t()
    control = cffirmware.control_t()
    control.thrust = 10
    control.roll = 0
    control.pitch = 0
    control.yaw = 0

    cffirmware.powerDistribution(motorPower, control)
    # control.thrust will be at a (tuned) hover-state
    assert motorPower.m1 == control.thrust
    assert motorPower.m2 == control.thrust
    assert motorPower.m3 == control.thrust
    assert motorPower.m4 == control.thrust
