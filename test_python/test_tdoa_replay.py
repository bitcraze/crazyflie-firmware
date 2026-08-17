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
