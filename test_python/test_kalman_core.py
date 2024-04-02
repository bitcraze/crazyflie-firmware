#!/usr/bin/env python

import numpy as np
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.loco_utils import read_loco_anchor_positions
from bindings.util.lighthouse_utils import read_lh_basestation_positions_calibration

def test_kalman_core_with_tdoa3():
    # Fixture
    fixture_base = 'test_python/fixtures/kalman_core'
    anchor_positions = read_loco_anchor_positions(fixture_base + '/anchor_positions.yaml')
    runner = SdCardFileRunner(fixture_base + '/log05')
    emulator = EstimatorKalmanEmulator(anchor_positions)

    # Test
    actual = runner.run_estimator_loop(emulator)

    # Assert
    # Verify that the final position is close-ish to (0, 0, 0)
    actual_final_pos = np.array(actual[-1][1])
    assert np.linalg.norm(actual_final_pos - [0.0, 0.0, 0.0]) < 0.4


def test_kalman_core_with_sweep_angles():

    # Fixture
    fixture_base = 'test_python/fixtures/kalman_core'
    basestation_positions = read_lh_basestation_pose_calibration(fixture_base + '/basestation_positions_calibration.yaml')
