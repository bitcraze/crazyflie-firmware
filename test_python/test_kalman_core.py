#!/usr/bin/env python

import math
import pytest
import numpy as np
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.loco_utils import read_loco_anchor_positions

def test_kalman_core_with_tdoa3():
    # Test configuration
    TRUE_FINAL_POSITION = np.array([0.0, 0.0, 0.0])
    FINAL_POSITION_TOLERANCE = 0.4

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
    assert np.linalg.norm(actual_final_pos - TRUE_FINAL_POSITION) < FINAL_POSITION_TOLERANCE

def test_kalman_core_with_tdoa3_dead_reckoning():
    # Test configuration
    TRUE_FINAL_POSITION = np.array([0.0, 0.0, 0.0])
    FINAL_POSITION_TOLERANCE = 5

    # Fixture
    fixture_base = 'test_python/fixtures/kalman_core'
    anchor_positions = read_loco_anchor_positions(fixture_base + '/anchor_positions.yaml')
    runner = SdCardFileRunner(fixture_base + '/log05_no_pos_after_1s')
    emulator = EstimatorKalmanEmulator(anchor_positions)

    # Test
    actual = runner.run_estimator_loop(emulator)

    # Assert
    # Verify that the final position is close-ish to (0, 0, 0)
    actual_final_pos = np.array(actual[-1][1])
    assert np.linalg.norm(actual_final_pos - TRUE_FINAL_POSITION) < FINAL_POSITION_TOLERANCE

def test_robust_tdoa_update_is_exposed_and_runs():
    import cffirmware

    core = cffirmware.kalmanCoreData_t()
    params = cffirmware.kalmanCoreParams_t()
    cffirmware.kalmanCoreDefaultParams(params)
    cffirmware.kalmanCoreInit(core, params, 0)

    pos_a = cffirmware.vec3_s()
    pos_a.x, pos_a.y, pos_a.z = 0.0, 0.0, 0.0
    pos_b = cffirmware.vec3_s()
    pos_b.x, pos_b.y, pos_b.z = 4.0, 0.0, 0.0

    tdoa = cffirmware.tdoaMeasurement_t()
    tdoa.anchorIdA = 0
    tdoa.anchorIdB = 1
    tdoa.anchorPositionA = pos_a
    tdoa.anchorPositionB = pos_b
    tdoa.distanceDiff = 0.1
    tdoa.stdDev = 0.15

    outlier_filter_state = cffirmware.OutlierFilterTdoaState_t()
    cffirmware.outlierFilterTdoaReset(outlier_filter_state)

    # The robust update takes no now_ms (see mm_tdoa_robust.h)
    cffirmware.kalmanCoreRobustUpdateWithTdoa(core, tdoa, outlier_filter_state)


def test_emulator_rejects_unknown_tdoa_model():
    with pytest.raises(ValueError, match='huber'):
        EstimatorKalmanEmulator(anchor_positions={}, tdoa_model='huber')


def test_emulator_robust_model_processes_a_tdoa_sample():
    import cffirmware

    pos_a = cffirmware.vec3_s()
    pos_a.x, pos_a.y, pos_a.z = 0.0, 0.0, 0.0
    pos_b = cffirmware.vec3_s()
    pos_b.x, pos_b.y, pos_b.z = 4.0, 0.0, 0.0
    anchor_positions = {0: pos_a, 1: pos_b}

    emulator = EstimatorKalmanEmulator(anchor_positions, tdoa_model='robust')
    samples = [
        ('estAcceleration',
         {'timestamp': 1000.0, 'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0}),
        ('estGyroscope',
         {'timestamp': 1000.0, 'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0}),
        ('estTDOA',
         {'timestamp': 1001.0, 'idA': 0, 'idB': 1, 'distanceDiff': 0.1}),
    ]

    state = None
    while len(samples):
        _, state = emulator.run_one_1khz_iteration(samples)

    assert math.isfinite(state.position.x)
