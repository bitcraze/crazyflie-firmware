"""Tests for the candidate-group seam (no cffirmware bindings required)."""

from bindings.util.tdoa_selection import (
    build_candidate_groups,
    make_policy,
    verify_baseline_reconstruction,
)


def _cand_log(events):
    """Build log_data with an estTdoaCand block from a list of tuples:
    (timestamp, group, idA, idB, distanceDiff, isSelected)."""
    keys = ('timestamp', 'group', 'idA', 'idB', 'distanceDiff', 'isSelected')
    return {'estTdoaCand': {k: [e[i] for e in events] for i, k in enumerate(keys)}}


def test_events_are_grouped_by_group_value_in_log_order():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 1),
        (100.0, 7, 1, 3, 0.6, 0),
        (105.0, 9, 4, 5, -0.1, 1),
    ])
    groups = build_candidate_groups(log)
    assert [g['group'] for g in groups] == [7, 9]
    assert [g['idA'] for g in groups] == [1, 4]
    assert [len(g['candidates']) for g in groups] == [2, 1]


def test_group_time_is_min_of_event_timestamps():
    # uSD stamps events at write time; the packet time is the group minimum
    log = _cand_log([
        (103.0, 7, 1, 2, 0.5, 1),
        (101.0, 7, 1, 3, 0.6, 0),
    ])
    groups = build_candidate_groups(log)
    assert groups[0]['t_ms'] == 101.0


def test_group_schema_is_frozen():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 1)])
    group = build_candidate_groups(log)[0]
    assert set(group.keys()) == {'group', 't_ms', 'idA', 'candidates'}
    assert set(group['candidates'][0].keys()) == {'idB', 'distanceDiff', 'isSelected'}
    assert isinstance(group['group'], int)
    assert isinstance(group['t_ms'], float)
    assert isinstance(group['idA'], int)
    assert isinstance(group['candidates'][0]['isSelected'], bool)


def test_missing_or_empty_candidate_block_gives_no_groups():
    assert build_candidate_groups({}) == []
    assert build_candidate_groups({'estTdoaCand': {}}) == []


def test_baseline_policy_picks_the_selected_candidate():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    measurements = make_policy('baseline').select(group)
    # Measurements are in the estimator (estTDOA) convention: idA = remote
    # candidate anchor, idB = the packet's transmitting anchor.
    assert measurements == [{'idA': 3, 'idB': 1, 'distanceDiff': 0.6}]


def test_baseline_policy_skips_groups_without_selection():
    # Legitimate case (spec C2 scope note): e.g. no matching algorithm ran
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 0)])
    group = build_candidate_groups(log)[0]
    assert make_policy('baseline').select(group) == []


def test_all_policy_returns_every_candidate():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    assert len(make_policy('all').select(group)) == 2


def test_round_robin_rotates_and_resets():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    policy = make_policy('round_robin')
    # The remote (rotating) anchor lands in idA under the estimator convention
    first = policy.select(group)[0]['idA']
    second = policy.select(group)[0]['idA']
    assert {first, second} == {2, 3}
    policy.reset()
    assert policy.select(group)[0]['idA'] == first


def test_verify_baseline_reconstruction_clean_log():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 1),
        (100.0, 7, 1, 3, 0.6, 0),
        (105.0, 8, 4, 5, -0.1, 1),
    ])
    log['estTDOA'] = {  # estimator convention: idA = remote, idB = packet anchor
        'timestamp': [100.0, 105.0],
        'idA': [2, 5],
        'idB': [1, 4],
        'distanceDiff': [0.5, -0.1],
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_groups'] == 2
    assert result['n_selectionless'] == 0
    assert result['n_baseline'] == 2
    assert result['n_est_tdoa'] == 2
    assert result['n_matched'] == 2
    assert result['n_unmatched_est'] == 0
    assert result['n_unmatched_baseline'] == 0
    assert result['first_unmatched_est'] is None


def test_reconstruction_matches_real_firmware_id_convention():
    """Regression test for the id-order convention, verified on hardware.

    The firmware logs the two events of one packet with OPPOSITE id order:
    - estTdoaCand (tdoaEngine.c): idA = anchor that transmitted the processed
      packet, idB = the remote (candidate) anchor.
    - estTDOA (estimator.c via enqueueTDOA): anchorIds[0]/idA = the REMOTE
      anchor, anchorIds[1]/idB = the packet's anchor, same distanceDiff value
      (= d(packet anchor) - d(remote)).

    Captured 2026-07-07: a desk log where every packet produced candidate
    (idA=an, idB=other, v) and estTDOA (idA=other, idB=an, v). The first
    replay implementation assumed equal id order and matched 7/83398.
    """
    log = _cand_log([
        # packet from anchor 3; remote candidates 5 (selected) and 7
        (100.0, 0, 3, 5, 0.25, 1),
        (100.0, 0, 3, 7, -0.75, 0),
        # packet from anchor 8; remote candidate 2 (selected)
        (110.0, 1, 8, 2, 1.5, 1),
    ])
    log['estTDOA'] = {  # as the real estimator logs it: (remote, packet, v)
        'timestamp': [100.0, 110.0],
        'idA': [5, 2],
        'idB': [3, 8],
        'distanceDiff': [0.25, 1.5],
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_matched'] == 2
    assert result['n_unmatched_est'] == 0
    assert result['first_unmatched_est'] is None


def test_verify_baseline_reconstruction_detects_mismatch():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 1)])
    log['estTDOA'] = {  # ids in estimator convention; the value differs
        'timestamp': [100.0],
        'idA': [2],
        'idB': [1],
        'distanceDiff': [0.7],  # differs from the flagged candidate
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_unmatched_est'] == 1
    assert result['first_unmatched_est'] == 0


def test_verify_baseline_reconstruction_tolerates_position_gated_gaps():
    # Three packets each with a selected candidate, but the 2nd measurement
    # was position-gated on the drone (one of its anchors had no valid
    # position) and never made it into the estTDOA stream.
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 1),
        (110.0, 8, 3, 4, 0.6, 1),
        (120.0, 9, 5, 6, -0.1, 1),
    ])
    log['estTDOA'] = {  # estimator convention: idA = remote, idB = packet anchor
        'timestamp': [100.0, 120.0],
        'idA': [2, 6],
        'idB': [1, 5],
        'distanceDiff': [0.5, -0.1],
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_matched'] == 2
    assert result['n_unmatched_est'] == 0
    assert result['n_unmatched_baseline'] == 1
    assert result['first_unmatched_est'] is None
