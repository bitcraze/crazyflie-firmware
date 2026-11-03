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


def test_start_trajectory_relative_yaw_starts_at_start_position():
    # Fixture
    planner = cffirmware.planner()
    cffirmware.plan_init(planner)
    duration = 2.0
    pieces = cffirmware.poly4d_malloc(1)
    trajectory = cffirmware.piecewise_traj()
    trajectory.t_begin = 0
    trajectory.timescale = 1.0
    trajectory.n_pieces = 1
    trajectory.pieces = pieces
    piece = cffirmware.piecewise_get(trajectory, 0)
    piece.duration = duration
    for dim in range(4):
        for coef in range(cffirmware.PP_SIZE):
            cffirmware.poly4d_set(piece, dim, coef, 0)
    # Straight line from (1, 0, 0) to (2, 0, 0) with yaw 0, so it does not start at the origin
    cffirmware.poly4d_set(piece, 0, 0, 1.0)
    cffirmware.poly4d_set(piece, 0, 1, 0.5)
    start_from = cffirmware.mkvec(3, 4, 1)
    start_yaw = np.pi / 2

    # Test
    cffirmware.plan_start_trajectory(planner, trajectory, False, True, True, start_from, start_yaw)

    # Assert
    start = cffirmware.plan_current_goal(planner, 0)
    end = cffirmware.plan_current_goal(planner, duration)
    assert np.allclose(np.array([3, 4, 1]), start.pos)
    assert np.isclose(start_yaw, start.yaw)
    # The trajectory moves along +x, which is +y after rotating it to the start yaw
    assert np.allclose(np.array([3, 5, 1]), end.pos)

    cffirmware.poly4d_free(pieces)
