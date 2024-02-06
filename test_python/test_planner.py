#!/usr/bin/env python

import numpy as np
import cffirmware

def test_takeoff():
    # Fixture
    planner = cffirmware.planner()
    cffirmware.plan_init(planner)
    pos = cffirmware.mkvec(0, 0, 0)
    yaw = 0
    targetHeight = 1.0
    targetYaw = 0
    duration = 2

    # Test
    cffirmware.plan_takeoff(planner, pos, yaw, targetHeight, targetYaw, duration, 0)

    # Assert
    state = cffirmware.plan_current_goal(planner, duration)
    assert np.allclose(np.array([0, 0, targetHeight]), state.pos)
    assert np.allclose(np.array([0, 0, 0.0]), state.vel)
