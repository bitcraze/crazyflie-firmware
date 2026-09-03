"""Tests for the candidate-group seam (no cffirmware bindings required)."""

import pytest

from bindings.util.tdoa_selection import (
    GeometryPolicy,
    annotate_oracle_error,
    annotate_pair_geometry,
    annotate_remote_age,
    build_candidate_groups,
    make_policy,
    remote_age_blind_fraction,
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


def test_trimmed_all_keeps_in_tolerance_and_drops_outlier():
    # median of [0.5, 0.55, 0.6, 5.0] (sorted) at index 4//2=2 is 0.6.
    # 5.0 is far outside tol_m=0.2 and should be dropped; the rest kept.
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.55, 0),
        (100.0, 7, 1, 4, 0.6, 1),
        (100.0, 7, 1, 9, 5.0, 0),
    ])
    group = build_candidate_groups(log)[0]
    measurements = make_policy('trimmed_all', {'tol_m': 0.2}).select(group)
    ids = sorted(m['idA'] for m in measurements)
    assert ids == [2, 3, 4]
    assert 9 not in ids


def test_trimmed_all_with_huge_tolerance_matches_all():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
        (100.0, 7, 1, 9, 5.0, 0),
    ])
    group = build_candidate_groups(log)[0]
    trimmed = make_policy('trimmed_all', {'tol_m': 1e9}).select(group)
    everything = make_policy('all').select(group)
    assert trimmed == everything


def test_trimmed_all_with_zero_tolerance_returns_median_valued_candidates():
    # sorted distanceDiffs: [0.5, 0.6, 0.6] -> median (index 3//2=1) is 0.6
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
        (100.0, 7, 1, 4, 0.6, 0),
    ])
    group = build_candidate_groups(log)[0]
    measurements = make_policy('trimmed_all', {'tol_m': 0.0}).select(group)
    ids = sorted(m['idA'] for m in measurements)
    assert ids == [3, 4]


def test_trimmed_all_empty_candidates_returns_empty():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 0)])
    group = build_candidate_groups(log)[0]
    group['candidates'] = []
    assert make_policy('trimmed_all').select(group) == []


def test_top_k_with_k_at_least_group_size_matches_all():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
        (100.0, 7, 1, 9, 5.0, 0),
    ])
    group = build_candidate_groups(log)[0]
    top_k = make_policy('top_k', {'k': 10}).select(group)
    everything = make_policy('all').select(group)
    assert sorted(top_k, key=lambda m: m['idA']) == sorted(everything, key=lambda m: m['idA'])


def test_top_k_one_matches_median_choice():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
        (100.0, 7, 1, 9, 5.0, 0),
    ])
    group = build_candidate_groups(log)[0]
    assert make_policy('top_k', {'k': 1}).select(group) == make_policy('median').select(group)


def test_top_k_fewer_candidates_than_k_returns_all():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    assert len(make_policy('top_k', {'k': 5}).select(group)) == 2


def test_top_k_empty_candidates_returns_empty():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 0)])
    group = build_candidate_groups(log)[0]
    group['candidates'] = []
    assert make_policy('top_k').select(group) == []


def test_trimmed_all_measurement_conversion_uses_idA_idB_swap():
    # packet anchor 3 (idA), remote candidate 5 (idB), both within tolerance
    log = _cand_log([
        (100.0, 0, 3, 5, 0.25, 1),
        (100.0, 0, 3, 6, 0.30, 0),
    ])
    group = build_candidate_groups(log)[0]
    measurements = make_policy('trimmed_all', {'tol_m': 10.0}).select(group)
    assert {(m['idA'], m['idB'], m['distanceDiff']) for m in measurements} == {
        (5, 3, 0.25), (6, 3, 0.30),
    }


def test_top_k_measurement_conversion_uses_idA_idB_swap():
    log = _cand_log([
        (100.0, 0, 3, 5, 0.25, 1),
        (100.0, 0, 3, 6, 0.30, 0),
    ])
    group = build_candidate_groups(log)[0]
    measurements = make_policy('top_k', {'k': 2}).select(group)
    assert {(m['idA'], m['idB'], m['distanceDiff']) for m in measurements} == {
        (5, 3, 0.25), (6, 3, 0.30),
    }


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


def test_annotate_remote_age_measures_time_since_the_remote_last_transmitted():
    # Anchor 1 transmits at t=100. Anchor 2's packet at t=130 carries 1 as a
    # remote -> that data is 30 ms old; by t=500 the same pair is 400 ms stale.
    log = _cand_log([
        (100.0, 0, 1, 3, 0.0, 1),
        (130.0, 1, 2, 1, 0.0, 1),
        (500.0, 2, 2, 1, 0.0, 1),
    ])
    groups = annotate_remote_age(build_candidate_groups(log))
    assert groups[1]['candidates'][0]['age_ms'] == pytest.approx(30.0)
    assert groups[2]['candidates'][0]['age_ms'] == pytest.approx(400.0)


def test_annotate_remote_age_marks_never_seen_anchors_infinite():
    # Anchor 7 has not been heard transmitting yet, so its age is unknown and
    # any gate must reject it rather than treat it as fresh.
    log = _cand_log([(100.0, 0, 1, 7, 0.0, 1)])
    groups = annotate_remote_age(build_candidate_groups(log))
    assert groups[0]['candidates'][0]['age_ms'] == float('inf')


def test_freshness_policy_filters_stale_candidates_then_delegates():
    log = _cand_log([
        (100.0, 0, 1, 9, 0.0, 1),    # anchors 1 and 9 both transmit early
        (100.0, 1, 9, 1, 0.0, 1),
        (900.0, 2, 5, 1, 0.0, 1),    # remote 1 is 800 ms stale here
        (900.0, 3, 5, 9, 0.0, 1),    # remote 9 is 800 ms stale here
    ])
    groups = annotate_remote_age(build_candidate_groups(log))
    gate = make_policy('fresh', {'inner': 'all', 'max_age_ms': 40.0})
    gate.reset()
    assert gate.select(groups[2]) == []
    assert gate.select(groups[3]) == []
    # With a gate wide enough to admit them, the inner policy sees them again.
    wide = make_policy('fresh', {'inner': 'all', 'max_age_ms': 2000.0})
    assert len(wide.select(groups[2])) == 1


def test_freshness_policy_refuses_unannotated_groups():
    # Silently passing everything through would make the gate a no-op.
    log = _cand_log([(100.0, 0, 1, 9, 0.0, 1)])
    groups = build_candidate_groups(log)
    with pytest.raises(ValueError, match='annotate_remote_age'):
        make_policy('fresh').select(groups[0])


def test_remote_age_blind_fraction_counts_gaps_in_the_group_counter():
    # Groups 0 and 3 are present, 1 and 2 emitted no candidate at all.
    log = _cand_log([(100.0, 0, 1, 9, 0.0, 1), (200.0, 3, 1, 9, 0.0, 1)])
    assert remote_age_blind_fraction(log) == pytest.approx(0.5)
    assert remote_age_blind_fraction({}) is None


# Anchors 1 and 9 are 10 m apart on the x axis; 5 sits off the line.
_GEOMETRY_ANCHORS = {
    1: (-5.0, 0.0, 0.0),
    9: (5.0, 0.0, 0.0),
    5: (0.0, 4.0, 0.0),
}


def test_annotate_pair_geometry_is_the_ratio_to_the_anchor_separation():
    # 8.5 m of distance diff over a 10 m baseline is exactly the 0.85 limit.
    log = _cand_log([
        (100.0, 0, 1, 9, 8.5, 1),
        (100.0, 0, 1, 9, -2.0, 0),
    ])
    groups = annotate_pair_geometry(build_candidate_groups(log),
                                    _GEOMETRY_ANCHORS)
    ratios = [c['distance_ratio'] for c in groups[0]['candidates']]
    assert ratios == pytest.approx([0.85, 0.2])


def test_annotate_pair_geometry_accepts_dict_and_vec3_style_positions():
    class Vec3:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    log = _cand_log([(100.0, 0, 1, 9, 5.0, 1)])
    for anchors in ({1: {'x': -5.0, 'y': 0.0, 'z': 0.0},
                     9: {'x': 5.0, 'y': 0.0, 'z': 0.0}},
                    {1: Vec3(-5.0, 0.0, 0.0), 9: Vec3(5.0, 0.0, 0.0)}):
        groups = annotate_pair_geometry(build_candidate_groups(log), anchors)
        assert groups[0]['candidates'][0]['distance_ratio'] == pytest.approx(0.5)


def test_annotate_pair_geometry_marks_unknown_and_coincident_anchors_infinite():
    # The firmware rejects both cases in isGeometryGoodEnough (no position, and
    # a non-positive squared separation), so the ratio must fail any gate.
    log = _cand_log([
        (100.0, 0, 1, 7, 0.5, 1),    # anchor 7 is not in the anchors file
        (200.0, 1, 1, 2, 0.5, 1),    # anchors 1 and 2 share a position
    ])
    anchors = {1: (-5.0, 0.0, 0.0), 2: (-5.0, 0.0, 0.0)}
    groups = annotate_pair_geometry(build_candidate_groups(log), anchors)
    assert groups[0]['candidates'][0]['distance_ratio'] == float('inf')
    assert groups[1]['candidates'][0]['distance_ratio'] == float('inf')


def test_geometry_policy_rejects_at_the_limit_and_keeps_below_it():
    # The firmware test is ratioSquared < limit^2, so the limit itself rejects.
    log = _cand_log([
        (100.0, 0, 1, 9, 8.5, 1),    # ratio 0.85 -> rejected
        (200.0, 1, 1, 9, 8.4, 1),    # ratio 0.84 -> kept
    ])
    groups = annotate_pair_geometry(build_candidate_groups(log),
                                    _GEOMETRY_ANCHORS)
    gate = make_policy('geometry')
    gate.reset()
    assert gate.select(groups[0]) == []
    assert gate.select(groups[1]) == [{'idA': 9, 'idB': 1, 'distanceDiff': 8.4}]


def test_geometry_policy_lets_the_inner_policy_fall_back_to_a_good_candidate():
    # What the merged firmware does: reject the bad candidate, continue the
    # loop, and use the next one -- so the packet is not lost.
    log = _cand_log([
        (100.0, 0, 5, 1, 6.0, 1),    # ratio 6.0 / |5-1| = 0.937 -> rejected
        (100.0, 0, 5, 9, 1.0, 0),    # ratio 0.156 -> survives
    ])
    groups = annotate_pair_geometry(build_candidate_groups(log),
                                    _GEOMETRY_ANCHORS)
    # geometry(baseline) drops the packet: the live selection was the bad pair.
    assert make_policy('geometry').select(groups[0]) == []
    # geometry(round_robin) picks a survivor instead.
    fallback = make_policy('geometry', {'inner': 'round_robin'})
    fallback.reset()
    assert fallback.select(groups[0]) == [
        {'idA': 9, 'idB': 5, 'distanceDiff': 1.0}]


def test_geometry_policy_limit_is_tunable():
    log = _cand_log([(100.0, 0, 1, 9, 8.5, 1)])
    groups = annotate_pair_geometry(build_candidate_groups(log),
                                    _GEOMETRY_ANCHORS)
    assert make_policy('geometry', {'limit': 0.9}).select(groups[0]) != []
    assert make_policy('geometry', {'limit': 0.5}).select(groups[0]) == []


def test_geometry_policy_refuses_unannotated_groups():
    # Silently passing everything through would make the gate a no-op.
    log = _cand_log([(100.0, 0, 1, 9, 0.0, 1)])
    groups = build_candidate_groups(log)
    with pytest.raises(ValueError, match='annotate_pair_geometry'):
        make_policy('geometry').select(groups[0])


def test_geometry_and_freshness_gates_compose():
    log = _cand_log([
        (100.0, 0, 1, 9, 0.0, 1),
        (100.0, 1, 9, 1, 0.0, 1),
        (900.0, 2, 1, 9, 1.0, 1),    # good geometry, but remote 9 is stale
    ])
    groups = annotate_pair_geometry(
        annotate_remote_age(build_candidate_groups(log)), _GEOMETRY_ANCHORS)
    gate = make_policy('fresh', {'inner': GeometryPolicy(inner='all'),
                                 'max_age_ms': 40.0})
    gate.reset()
    assert gate.select(groups[2]) == []


def test_plain_policies_require_no_annotation():
    # A caller that unions `requires` over its policies must get an empty set
    # for every plain selector, so it does no annotation work at all.
    for name in ('baseline', 'all', 'median', 'round_robin', 'trimmed_all',
                 'top_k'):
        assert make_policy(name).requires == ()


def test_gate_policies_declare_what_they_need_and_propagate_it():
    assert set(make_policy('geometry').requires) == {'distance_ratio'}
    assert set(make_policy('fresh').requires) == {'age_ms'}
    # A wrapper around a wrapper asks for both.
    nested = make_policy('fresh', {'inner': GeometryPolicy(inner='baseline')})
    assert set(nested.requires) == {'age_ms', 'distance_ratio'}


def test_annotating_leaves_the_candidate_schema_alone_for_other_policies():
    # Adding the geometry gate to the menu must not change what a run that
    # does not use it sees: no annotate pass, no extra candidate keys.
    log = _cand_log([(100.0, 0, 1, 9, 0.5, 1)])
    groups = build_candidate_groups(log)
    assert set(groups[0]['candidates'][0]) == {'idB', 'distanceDiff', 'isSelected'}


# --- oracle ---------------------------------------------------------------
# Two anchors on the x axis 10 m apart, the tag between them: with the tag at
# (2, 0, 0) the true TDoA for the (idA=0, idB=1) estimator pair is
# |p - A1| - |p - A0| = 8 - 2 = 6.
_ORACLE_ANCHORS = {0: (0.0, 0.0, 0.0), 1: (10.0, 0.0, 0.0), 9: (0.0, 10.0, 0.0)}
# A stationary tag, sampled densely enough that the gap guard never fires.
_ORACLE_TRUTH = [(float(t), (2.0, 0.0, 0.0)) for t in range(0, 1001, 20)]


def _oracle_groups(events, truth=_ORACLE_TRUTH, **kwargs):
    groups = build_candidate_groups(_cand_log(events))
    return annotate_oracle_error(groups, _ORACLE_ANCHORS, truth, **kwargs)


def test_oracle_error_is_the_distance_from_the_truth_prediction():
    # build_candidate_groups logs (idA=packet anchor, idB=remote); the
    # estimator pair is the swap, so a candidate (idA=1, idB=0) is the
    # measurement (idA=0, idB=1) whose true distanceDiff is +6.
    groups = _oracle_groups([
        (100.0, 0, 1, 0, 6.0, 1),      # exact
        (100.0, 0, 1, 0, 6.25, 0),     # 25 cm off
        (100.0, 0, 1, 0, 5.0, 0),      # 1 m off
    ])
    errors = [c['oracle_error'] for c in groups[0]['candidates']]
    assert errors == pytest.approx([0.0, 0.25, 1.0])


def test_oracle_picks_the_candidate_closest_to_truth():
    groups = _oracle_groups([
        (100.0, 0, 1, 0, 5.0, 1),      # the flown choice, 1 m off
        (100.0, 0, 1, 0, 6.05, 0),     # the best one
    ])
    assert make_policy('oracle').select(groups[0]) == [
        {'idA': 0, 'idB': 1, 'distanceDiff': 6.05}]
    # ... which is emphatically not what baseline does.
    assert make_policy('baseline').select(groups[0])[0]['distanceDiff'] == 5.0


def test_oracle_k_and_max_error_bound_different_things():
    groups = _oracle_groups([
        (100.0, 0, 1, 0, 6.0, 1),
        (100.0, 0, 1, 0, 6.2, 0),
        (100.0, 0, 1, 0, 9.0, 0),
    ])
    # k=None feeds everything, best first.
    assert [m['distanceDiff'] for m in
            make_policy('oracle', {'k': None}).select(groups[0])] == [6.0, 6.2, 9.0]
    # k=2 is the best two.
    assert [m['distanceDiff'] for m in
            make_policy('oracle', {'k': 2}).select(groups[0])] == [6.0, 6.2]
    # A threshold turns it into a perfect outlier filter instead of a selector.
    assert [m['distanceDiff'] for m in
            make_policy('oracle', {'k': None, 'max_error_m': 0.5})
            .select(groups[0])] == [6.0, 6.2]


_SPARSE_TRUTH = [(0.0, (2.0, 0.0, 0.0)), (1000.0, (2.0, 0.0, 0.0))]


def test_oracle_refuses_to_score_across_a_truth_gap():
    # Truth only at t=0 and t=1000; a packet at t=500 sits in a 1 s hole, well
    # beyond max_gap_ms, so guessing its position is not allowed.
    groups = _oracle_groups([(500.0, 0, 1, 0, 6.0, 1)],
                            truth=_SPARSE_TRUTH, max_gap_ms=100.0)
    assert groups[0]['candidates'][0]['oracle_error'] == float('inf')


def test_oracle_falls_back_to_the_flown_selection_when_unscorable():
    groups = _oracle_groups([
        (500.0, 0, 1, 0, 5.0, 1),
        (500.0, 0, 1, 0, 6.0, 0),
    ], truth=_SPARSE_TRUTH, max_gap_ms=100.0)
    # Nothing scorable: the flown pair replays unchanged rather than vanishing.
    assert make_policy('oracle').select(groups[0])[0]['distanceDiff'] == 5.0
    # ... unless the caller asks for the packet to be dropped instead.
    assert make_policy('oracle', {'fallback': None}).select(groups[0]) == []


def test_oracle_never_selects_an_unknown_anchor():
    groups = _oracle_groups([
        (100.0, 0, 1, 0, 9.0, 1),      # scorable but bad
        (100.0, 0, 1, 77, 6.0, 0),     # would be perfect, but anchor 77 is unknown
    ])
    assert groups[0]['candidates'][1]['oracle_error'] == float('inf')
    assert make_policy('oracle').select(groups[0])[0]['distanceDiff'] == 9.0


def test_oracle_position_score_penalises_collinear_pairs():
    # Within a group idA is fixed (the packet's anchor) and only idB varies.
    # Pair (0, 1) puts the tag exactly on the segment between the anchors:
    # |h| = 2, its maximum, so position error = measurement error / 2. Pair
    # (9, 1) sees the tag off-axis, |h| < 2, so the same measurement error
    # displaces the estimate further.
    events = [(100.0, 0, 1, 0, 6.5, 1), (100.0, 0, 1, 9, 0.0, 0)]
    m = _oracle_groups(events, score='measurement')[0]['candidates']
    p = _oracle_groups(events, score='position')[0]['candidates']
    assert p[0]['oracle_error'] == pytest.approx(m[0]['oracle_error'] / 2.0)
    assert p[1]['oracle_error'] > m[1]['oracle_error'] / 2.0


def test_oracle_declares_what_it_needs():
    assert set(make_policy('oracle').requires) == {'oracle_error'}
    assert make_policy('oracle', {'fallback': None}).requires == ('oracle_error',)


def test_oracle_without_annotation_raises():
    groups = build_candidate_groups(_cand_log([(100.0, 0, 1, 0, 6.0, 1)]))
    with pytest.raises(ValueError, match='annotate_oracle_error'):
        make_policy('oracle').select(groups[0])


def test_annotate_oracle_error_rejects_an_unknown_score():
    with pytest.raises(ValueError, match='unknown oracle score'):
        _oracle_groups([(100.0, 0, 1, 0, 6.0, 1)], score='vibes')
