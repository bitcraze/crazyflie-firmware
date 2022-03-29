#!/usr/bin/env python

import numpy as np
import cffirmware

def test_that_traj_eval_zero_is_actually_zero():
    # Fixture

    # Test
    t = cffirmware.traj_eval_zero()
    valid = cffirmware.is_traj_eval_valid(t)

    # Assert
    assert valid
    assert np.allclose(np.zeros(3), t.pos)
    assert np.allclose(np.zeros(3), t.vel)
    assert np.allclose(np.zeros(3), t.acc)
    assert np.allclose(np.zeros(3), t.omega)
    assert t.yaw == 0


def test_that_traj_eval_invalid_is_actually_invalid():
    # Fixture

    # Test
    t = cffirmware.traj_eval_invalid()
    valid = cffirmware.is_traj_eval_valid(t)

    # Assert
    assert not valid
