"""Tests for the replay seam that do NOT require the cffirmware bindings.

The end-to-end replay (which does need the bindings) is covered by
test_tdoa_replay_smoke.py.
"""

import pytest

from bindings.util.tdoa_replay import (
    apply_policy,
    extract_imu_samples,
    extract_state_estimate,
    filter_known_anchors,
    merge_samples,
    seed_initial_position,
)
from bindings.util.tdoa_selection import build_candidate_groups, make_policy


def test_apply_policy_stamps_measurements_with_the_group_time():
    log = {'estTdoaCand': {
        'timestamp': [103.0, 101.0],
        'group': [7, 7],
        'idA': [1, 1],
        'idB': [2, 3],
        'distanceDiff': [0.5, 0.6],
        'isSelected': [1, 0],
    }}
    groups = build_candidate_groups(log)
    samples = apply_policy(make_policy('baseline'), groups)
    # ids in the estimator convention: idA = remote, idB = packet anchor
    assert len(samples) == 1
    log_type, sample = samples[0]
    assert log_type == 'estTDOA'
    assert sample['idA'] == 2
    assert sample['idB'] == 1
    assert sample['distanceDiff'] == 0.5
    assert sample['timestamp'] == 101.0
    # Per-packet candidate statistics for std models / filters: group median
    # is diffs[n // 2] = 0.6, so the MAD over {0.5, 0.6} is 0.1
    assert sample['n_cand'] == 2
    assert sample['group_spread'] == pytest.approx(0.1)


def test_merge_samples_orders_by_timestamp_and_is_stable():
    # F2: IMU and TDoA streams interleave into one correctly ordered stream;
    # equal timestamps keep their original relative order (stable sort)
    imu = [
        ('estAcceleration', {'timestamp': 100.0, 'acc.x': 0.0}),
        ('estGyroscope', {'timestamp': 100.0, 'gyro.x': 0.0}),
        ('estAcceleration', {'timestamp': 102.0, 'acc.x': 0.0}),
    ]
    tdoa = [
        ('estTDOA', {'timestamp': 101.0, 'idA': 1, 'idB': 2, 'distanceDiff': 0.5}),
        ('estTDOA', {'timestamp': 100.0, 'idA': 3, 'idB': 4, 'distanceDiff': 0.1}),
    ]
    merged = merge_samples(imu, tdoa)
    assert [s[1]['timestamp'] for s in merged] == [100.0, 100.0, 100.0, 101.0, 102.0]
    # Stability: within t=100.0 the IMU pair stays ahead of the TDoA sample
    assert [s[0] for s in merged[:3]] == ['estAcceleration', 'estGyroscope', 'estTDOA']


def test_extract_imu_samples_reads_both_imu_blocks():
    log = {
        'estAcceleration': {'timestamp': [1.0], 'acc.x': [0.1], 'acc.y': [0.2], 'acc.z': [1.0]},
        'estGyroscope': {'timestamp': [2.0], 'gyro.x': [0.0], 'gyro.y': [0.0], 'gyro.z': [0.0]},
    }
    samples = extract_imu_samples(log)
    assert {s[0] for s in samples} == {'estAcceleration', 'estGyroscope'}
    assert len(samples) == 2


def test_extract_state_estimate_reads_the_fixed_frequency_block():
    log = {'fixedFrequency': {
        'timestamp': [10.0, 20.0],
        'stateEstimate.x': [0.1, 0.2],
        'stateEstimate.y': [0.3, 0.4],
        'stateEstimate.z': [0.5, 0.6],
    }}
    assert extract_state_estimate(log) == [
        (10.0, (0.1, 0.3, 0.5)),
        (20.0, (0.2, 0.4, 0.6)),
    ]


def test_extract_state_estimate_handles_missing_block():
    assert extract_state_estimate({}) == []
    assert extract_state_estimate({'fixedFrequency': {'timestamp': [1.0]}}) == []


def test_filter_known_anchors_keeps_samples_with_both_ids_known():
    anchor_positions = {1: object(), 2: object()}
    samples = [
        ('estTDOA', {'idA': 1, 'idB': 2, 'distanceDiff': 0.5, 'timestamp': 100.0}),
    ]
    kept, n_skipped = filter_known_anchors(samples, anchor_positions)
    assert kept == samples
    assert n_skipped == 0


def test_filter_known_anchors_drops_samples_referencing_unknown_anchors():
    anchor_positions = {1: object(), 2: object()}
    samples = [
        ('estTDOA', {'idA': 1, 'idB': 2, 'distanceDiff': 0.5, 'timestamp': 100.0}),
        ('estTDOA', {'idA': 1, 'idB': 9, 'distanceDiff': 0.6, 'timestamp': 101.0}),
        ('estTDOA', {'idA': 9, 'idB': 2, 'distanceDiff': 0.7, 'timestamp': 102.0}),
    ]
    kept, n_skipped = filter_known_anchors(samples, anchor_positions)
    assert kept == [samples[0]]
    assert n_skipped == 2


def test_ground_truth_interpolation_and_scoring():
    # Interpolation and scoring live in the CLI module; importing it must
    # not require the cffirmware bindings.
    from tools.usdlog.replay_tdoa import _interp_ground_truth, score_trajectory

    gt = [(100.0, (0.0, 0.0, 0.0)), (200.0, (1.0, 0.0, 0.0))]

    assert _interp_ground_truth(gt, 150.0) == (0.5, 0.0, 0.0)
    assert _interp_ground_truth(gt, 100.0) == (0.0, 0.0, 0.0)
    assert _interp_ground_truth(gt, 99.0) is None
    assert _interp_ground_truth(gt, 201.0) is None

    # Trajectory exactly on the ground truth scores zero error
    trajectory = [(100.0, (0.0, 0.0, 0.0)), (150.0, (0.5, 0.0, 0.0))]
    metrics, errors = score_trajectory(trajectory, gt, flyaway_threshold_m=0.3)
    assert metrics['n'] == 2
    assert metrics['rms'] == 0.0
    assert metrics['flyaway_frac'] == 0.0

    # A 1 m error on one of two samples is reflected in max and flyaway_frac
    trajectory = [(100.0, (0.0, 0.0, 0.0)), (150.0, (0.5, 1.0, 0.0))]
    metrics, errors = score_trajectory(trajectory, gt, flyaway_threshold_m=0.3)
    assert metrics['max'] == 1.0
    assert metrics['flyaway_frac'] == 0.5


def test_extract_ground_truth_drops_origin_and_held_positions():
    from tools.usdlog.replay_tdoa import extract_ground_truth

    # The crossing-beam position is recomputed well below the rate the
    # fixedFrequency block is sampled at, so the log repeats the value held
    # since the last update. Only the first sample of each run is a real
    # observation. The leading all-zero sample is the firmware's "no crossing
    # beam" placeholder.
    log_data = {'fixedFrequency': {
        'timestamp': [10.0, 20.0, 30.0, 40.0, 50.0],
        'lighthouse.x': [0.0, 1.0, 1.0, 2.0, 2.0],
        'lighthouse.y': [0.0, 0.0, 0.0, 0.0, 0.0],
        'lighthouse.z': [0.0, 1.0, 1.0, 1.0, 1.0],
    }}

    assert extract_ground_truth(log_data) == [
        (20.0, (1.0, 0.0, 1.0)),
        (40.0, (2.0, 0.0, 1.0)),
    ]


def _log_with_state_estimate(xs, ys, zs, timestamps):
    return {'fixedFrequency': {
        'timestamp': timestamps,
        'stateEstimate.x': xs,
        'stateEstimate.y': ys,
        'stateEstimate.z': zs,
    }}


def test_seed_initial_position_medians_over_the_window():
    # Sample at t=300 is outside a 200 ms window, and the 16.0 spike inside it
    # is rejected by the median -- the point of not using a single sample.
    log = _log_with_state_estimate(
        xs=[1.0, 16.0, 1.2, 99.0],
        ys=[2.0, 2.1, 2.2, 99.0],
        zs=[3.0, 3.1, 3.2, 99.0],
        timestamps=[100.0, 200.0, 300.0, 5000.0])
    assert seed_initial_position(log, window_ms=150) == {
        'initialX': pytest.approx(8.5),   # even count -> mean of the middle two
        'initialY': pytest.approx(2.05),
        'initialZ': pytest.approx(3.05),
    }
    assert seed_initial_position(log, window_ms=250) == {
        'initialX': pytest.approx(1.2),   # median of {1.0, 16.0, 1.2}
        'initialY': pytest.approx(2.1),
        'initialZ': pytest.approx(3.1),
    }


def test_seed_initial_position_with_a_zero_window_uses_the_first_sample():
    log = _log_with_state_estimate(
        xs=[1.0, 16.0], ys=[2.0, 2.1], zs=[3.0, 3.1], timestamps=[100.0, 200.0])
    assert seed_initial_position(log, window_ms=0) == {
        'initialX': pytest.approx(1.0),
        'initialY': pytest.approx(2.0),
        'initialZ': pytest.approx(3.0),
    }


def test_seed_initial_position_without_a_state_estimate_block():
    # The lean capture config drops stateEstimate.*; callers keep the origin.
    assert seed_initial_position({}) is None
    assert seed_initial_position({'fixedFrequency': {'timestamp': [1.0]}}) is None
