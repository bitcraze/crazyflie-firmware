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
    assert measurements == [{'idA': 1, 'idB': 3, 'distanceDiff': 0.6}]


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
    first = policy.select(group)[0]['idB']
    second = policy.select(group)[0]['idB']
    assert {first, second} == {2, 3}
    policy.reset()
    assert policy.select(group)[0]['idB'] == first


def test_verify_baseline_reconstruction_clean_log():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 1),
        (100.0, 7, 1, 3, 0.6, 0),
        (105.0, 8, 4, 5, -0.1, 1),
    ])
    log['estTDOA'] = {
        'timestamp': [100.0, 105.0],
        'idA': [1, 4],
        'idB': [2, 5],
        'distanceDiff': [0.5, -0.1],
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_groups'] == 2
    assert result['n_selectionless'] == 0
    assert result['n_baseline'] == 2
    assert result['n_est_tdoa'] == 2
    assert result['n_compared'] == 2
    assert result['n_mismatched'] == 0


def test_verify_baseline_reconstruction_detects_mismatch():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 1)])
    log['estTDOA'] = {
        'timestamp': [100.0],
        'idA': [1],
        'idB': [2],
        'distanceDiff': [0.7],  # differs from the flagged candidate
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_mismatched'] == 1
