#!/usr/bin/env python

import numpy as np
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.lighthouse_utils import load_lighthouse_calibration


def test_kalman_core_with_lighthouse_sweeps():
    """Run the Kalman estimator on recorded Lighthouse sweep-angle data through
    the bindings, exercising the in-EKF orientation estimation (the sweep-angle
    measurement model estimates full roll/pitch/yaw, not only position).

    No motion-capture ground truth is committed, so this is a smoke test: it
    verifies the estimator runs end to end and produces a finite, physically
    plausible trajectory (stays within the flight room and does not diverge).
    """
    # Fixture
    fixture_base = 'test_python/fixtures/kalman_core'
    bs_calib, bs_geo = load_lighthouse_calibration(fixture_base + '/geometry.yaml')
    runner = SdCardFileRunner(fixture_base + '/log30')
    emulator = EstimatorKalmanEmulator(
        basestation_calibration=bs_calib, basestation_poses=bs_geo)

    # Test
    actual = runner.run_estimator_loop(emulator)

    # Assert: a non-trivial, finite trajectory was produced.
    assert len(actual) > 1000
    states = np.array([state for _, state in actual])  # x, y, z, roll, pitch, yaw
    assert np.all(np.isfinite(states))

    # Position stays within a generous flight-room bound (no divergence).
    x, y, z = states[:, 0], states[:, 1], states[:, 2]
    assert np.all(np.abs(x) < 8.0)
    assert np.all(np.abs(y) < 8.0)
    assert np.all((z > -0.5) & (z < 4.0))

    # Attitude stays within valid Euler ranges.
    assert np.all(np.abs(states[:, 3:6]) <= 180.0 + 1e-3)
