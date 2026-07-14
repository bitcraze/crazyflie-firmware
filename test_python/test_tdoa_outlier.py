"""Tests for the pluggable outlier filters (no cffirmware bindings required).

The 'integrator' filter wraps the real C filter and is exercised with the
bindings in test_tdoa_outlier_seam.py; here we only check its registration.
"""

import pytest

from bindings.util.tdoa_outlier import (
    FILTER_FACTORIES, AndFilter, OutlierFilter, make_outlier_filter)


class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Tdoa:
    """Stub with the tdoaMeasurement_t attributes the filters read."""

    def __init__(self, pa=(0.0, 0.0, 0.0), pb=(4.0, 0.0, 0.0), dd=0.0,
                 std=0.15, ids=(0, 1)):
        self.anchorPositionA = _Vec(*pa)
        self.anchorPositionB = _Vec(*pb)
        self.distanceDiff = dd
        self.stdDev = std
        self.anchorIdA, self.anchorIdB = ids


def test_all_spec_filters_are_registered():
    assert set(FILTER_FACTORIES) == {
        'integrator', 'none', 'sanity', 'mad_window', 'pair_integrator',
        'pair_hampel', 'sanity_mad', 'hampel_mad', 'anchor_quality'}


def test_unknown_name_raises_value_error_listing_available():
    with pytest.raises(ValueError, match='mad_window'):
        make_outlier_filter('nope')


def test_none_accepts_anything():
    f = make_outlier_filter('none')
    assert f.validate(_Tdoa(dd=99.0), error=99.0, now_ms=0)


def test_sanity_rejects_physically_impossible_distance_diff():
    f = make_outlier_filter('sanity')
    # anchors 4 m apart: |dd| must be < 4
    assert f.validate(_Tdoa(dd=3.9), error=0.0, now_ms=0)
    assert not f.validate(_Tdoa(dd=4.1), error=0.0, now_ms=0)
    # the innovation is irrelevant to this filter
    assert f.validate(_Tdoa(dd=1.0), error=100.0, now_ms=0)


def test_mad_window_accepts_everything_during_warmup():
    f = make_outlier_filter('mad_window')
    for i in range(20):
        assert f.validate(_Tdoa(), error=100.0 * i, now_ms=i)


def test_mad_window_rejects_far_from_median_after_warmup():
    f = make_outlier_filter('mad_window')
    for i in range(30):
        f.validate(_Tdoa(), error=0.0, now_ms=i)
    # MAD is floored at 0.5 * std = 0.075; gate = 5 * 0.075 = 0.375
    assert f.validate(_Tdoa(), error=0.3, now_ms=31)
    assert not f.validate(_Tdoa(), error=5.0, now_ms=32)


def test_mad_window_rejected_samples_still_enter_the_window():
    # A sustained level shift (real position jump) must eventually be accepted
    # because rejected innovations drag the median with them.
    f = make_outlier_filter('mad_window')
    for i in range(30):
        f.validate(_Tdoa(), error=0.0, now_ms=i)
    accepted = [f.validate(_Tdoa(), error=5.0, now_ms=100 + i) for i in range(60)]
    assert not accepted[0]
    assert accepted[-1]


def test_pair_integrator_isolates_anchor_pairs():
    f = make_outlier_filter('pair_integrator')
    # Converge pair (0, 1): good samples, dt capped at 30 ms each -> the
    # integrator reaches 300 (> resume level 270) after 10 samples and closes.
    for i in range(1, 11):
        assert f.validate(_Tdoa(ids=(0, 1)), error=0.0, now_ms=30 * i)
    # Pair (0, 1) is now closed: a large error is rejected...
    assert not f.validate(_Tdoa(ids=(0, 1)), error=2.0, now_ms=400)
    # ...but pair (0, 2) has fresh (open) state and lets it through.
    assert f.validate(_Tdoa(ids=(0, 2)), error=2.0, now_ms=400)


def test_pair_integrator_reset_reopens_all_pairs():
    f = make_outlier_filter('pair_integrator')
    for i in range(1, 11):
        f.validate(_Tdoa(ids=(0, 1)), error=0.0, now_ms=30 * i)
    assert not f.validate(_Tdoa(ids=(0, 1)), error=2.0, now_ms=400)
    f.reset()
    assert f.validate(_Tdoa(ids=(0, 1)), error=2.0, now_ms=500)


def test_pair_integrator_out_of_order_timestamp_clamps_like_c_uint32():
    # C computes dt in uint32_t: a negative delta wraps huge and is clamped
    # to +INTEGRATOR_SIZE/10. The port must not drive the integrator backwards.
    f = make_outlier_filter('pair_integrator')
    f.validate(_Tdoa(ids=(0, 1)), error=0.0, now_ms=100)
    f.validate(_Tdoa(ids=(0, 1)), error=0.0, now_ms=50)  # out of order
    s = f._pairs[(0, 1)]
    assert s['integrator'] == 60.0  # 30 + 30, never 30 - 50


def test_factories_return_fresh_instances():
    a = make_outlier_filter('mad_window')
    b = make_outlier_filter('mad_window')
    assert a is not b


# --- pair_hampel ---------------------------------------------------------


def test_pair_hampel_accepts_everything_during_warmup():
    f = make_outlier_filter('pair_hampel')
    # WARMUP = 4: the first 4 samples for a pair are accepted unconditionally,
    # even though 0.0 vs 1.0 would otherwise look like an outlier.
    assert f.validate(_Tdoa(dd=0.0), error=0.0, now_ms=0)
    assert f.validate(_Tdoa(dd=0.0), error=0.0, now_ms=1)
    assert f.validate(_Tdoa(dd=0.0), error=0.0, now_ms=2)
    assert f.validate(_Tdoa(dd=1.0), error=0.0, now_ms=3)


def test_pair_hampel_rejects_clear_outlier_after_warmup():
    f = make_outlier_filter('pair_hampel')
    for i in range(4):
        f.validate(_Tdoa(dd=0.0), error=0.0, now_ms=i)
    # med = 0, mad floored at 0.5 * std = 0.075, gate = 5 * 0.075 = 0.375
    assert f.validate(_Tdoa(dd=0.3), error=0.0, now_ms=10)
    assert not f.validate(_Tdoa(dd=3.0), error=0.0, now_ms=11)


def test_pair_hampel_pairs_are_independent():
    f = make_outlier_filter('pair_hampel')
    # Fill pair (0, 1) with a stable level and confirm it rejects outliers.
    for i in range(4):
        f.validate(_Tdoa(dd=0.0, ids=(0, 1)), now_ms=i, error=0.0)
    assert not f.validate(_Tdoa(dd=3.0, ids=(0, 1)), error=0.0, now_ms=10)
    # A fresh pair (0, 2) has no history: it is still in warmup and accepts
    # the very same value pair (0, 1) just rejected.
    assert f.validate(_Tdoa(dd=3.0, ids=(0, 2)), error=0.0, now_ms=10)


def test_pair_hampel_clears_stale_window_after_max_age():
    f = make_outlier_filter('pair_hampel')
    for i in range(4):
        f.validate(_Tdoa(dd=0.0, ids=(0, 1)), error=0.0, now_ms=i)
    # A gap longer than MAX_AGE_MS clears the pair's history, so the window
    # is back in warmup and a value that would otherwise be rejected passes.
    stale_ms = 4 + f.MAX_AGE_MS + 1
    assert f.validate(_Tdoa(dd=3.0, ids=(0, 1)), error=0.0, now_ms=stale_ms)
    s = f._pairs[(0, 1)]
    assert len(s['window']) == 1


def test_pair_hampel_rejects_physically_impossible_distance_diff():
    f = make_outlier_filter('pair_hampel')
    # anchors 4 m apart: |dd| must be < 4
    assert not f.validate(_Tdoa(dd=4.1), error=0.0, now_ms=0)
    # a rejected-for-impossibility sample must not be recorded into the
    # pair's window.
    assert (0, 1) not in f._pairs


def test_pair_hampel_recovers_from_a_sustained_level_shift():
    f = make_outlier_filter('pair_hampel')
    # Anchors 20 m apart so both the base level and the shifted level below
    # stay within the physically-possible range (|dd| < anchor separation).
    pb = (20.0, 0.0, 0.0)
    for i in range(f.WINDOW * 2):
        f.validate(_Tdoa(dd=0.0, pb=pb), error=0.0, now_ms=i)
    # A persistent step in distanceDiff: rejected at first, but every sample
    # (accepted or not) enters the window, so once the window is full of the
    # new level the shift is accepted again.
    accepted = [
        f.validate(_Tdoa(dd=5.0, pb=pb), error=0.0, now_ms=100 + i)
        for i in range(f.WINDOW * 2)
    ]
    assert not accepted[0]
    assert accepted[-1]


# --- AndFilter / sanity_mad ------------------------------------------------


class _RecordingFilter(OutlierFilter):
    """Stub that always accepts and records every call it sees."""
    name = 'recording'

    def __init__(self):
        self.calls = []

    def reset(self):
        self.calls = []

    def validate(self, tdoa, error, now_ms):
        self.calls.append((tdoa.distanceDiff, error, now_ms))
        return True


def test_and_filter_calls_every_child_on_every_sample():
    a = _RecordingFilter()
    b = _RecordingFilter()
    f = AndFilter([a, b])
    f.validate(_Tdoa(dd=1.0), error=0.5, now_ms=10)
    f.validate(_Tdoa(dd=2.0), error=0.6, now_ms=20)
    assert a.calls == [(1.0, 0.5, 10), (2.0, 0.6, 20)]
    assert b.calls == a.calls


def test_and_filter_does_not_short_circuit_a_rejecting_child():
    class _RejectingFilter(OutlierFilter):
        def validate(self, tdoa, error, now_ms):
            return False

    recorder = _RecordingFilter()
    f = AndFilter([_RejectingFilter(), recorder])
    assert not f.validate(_Tdoa(), error=0.0, now_ms=0)
    # The second child must still have been called despite the first
    # rejecting.
    assert recorder.calls == [(0.0, 0.0, 0)]


def test_and_filter_accepts_only_if_all_children_accept():
    class _AlwaysAccept(OutlierFilter):
        def validate(self, tdoa, error, now_ms):
            return True

    class _AlwaysReject(OutlierFilter):
        def validate(self, tdoa, error, now_ms):
            return False

    assert AndFilter([_AlwaysAccept(), _AlwaysAccept()]).validate(
        _Tdoa(), error=0.0, now_ms=0)
    assert not AndFilter([_AlwaysAccept(), _AlwaysReject()]).validate(
        _Tdoa(), error=0.0, now_ms=0)


def test_and_filter_reset_resets_all_children():
    a = _RecordingFilter()
    b = _RecordingFilter()
    f = AndFilter([a, b])
    f.validate(_Tdoa(), error=0.0, now_ms=0)
    assert a.calls and b.calls
    f.reset()
    assert a.calls == [] and b.calls == []


def test_sanity_mad_registered_and_combines_both_checks():
    f = make_outlier_filter('sanity_mad')
    assert isinstance(f, AndFilter)
    # Physically impossible: rejected regardless of the innovation.
    assert not f.validate(_Tdoa(dd=4.1), error=0.0, now_ms=0)
    # Physically possible and within warmup: accepted.
    for i in range(20):
        assert f.validate(_Tdoa(dd=0.0), error=0.0, now_ms=i)
    # After warmup a large innovation is rejected by the MAD gate even
    # though it is physically possible.
    assert not f.validate(_Tdoa(dd=0.0), error=5.0, now_ms=30)


def test_anchor_quality_suppresses_a_bursting_anchor_across_pairs():
    # ALPHA/Q_MAX tuned for a short test: two rejections push an anchor's
    # rejection EWMA to 0.51, and one subsequent acceptance only decays it
    # to 0.357 — still above Q_MAX, so suppression persists.
    f = make_outlier_filter('anchor_quality', {'ALPHA': 0.3, 'Q_MAX': 0.35})
    # Warm up two pairs that share anchor 9: (9, 1) and (9, 2).
    for i in range(4):
        assert f.validate(_Tdoa(dd=0.0, ids=(9, 1)), error=0.0, now_ms=i)
        assert f.validate(_Tdoa(dd=0.0, ids=(9, 2)), error=0.0, now_ms=i)
    # Anchor 9 goes bad: gross outliers on both its pairs. The detector
    # rejects them and anchor 9's rejection rate climbs past Q_MAX.
    assert not f.validate(_Tdoa(dd=3.0, ids=(9, 1)), error=0.0, now_ms=10)
    assert not f.validate(_Tdoa(dd=3.0, ids=(9, 2)), error=0.0, now_ms=11)
    # Now even an in-family measurement involving anchor 9 is suppressed
    # (the detector accepts it, the quality gate does not) ...
    assert not f.validate(_Tdoa(dd=0.0, ids=(9, 1)), error=0.0, now_ms=12)
    # ... while a pair not involving anchor 9 is unaffected.
    for i in range(4):
        assert f.validate(_Tdoa(dd=0.0, ids=(3, 4)), error=0.0, now_ms=13 + i)


def test_anchor_quality_readmits_an_anchor_when_its_data_recovers():
    f = make_outlier_filter('anchor_quality', {'ALPHA': 0.3, 'Q_MAX': 0.35})
    for i in range(4):
        f.validate(_Tdoa(dd=0.0, ids=(9, 1)), error=0.0, now_ms=i)
    assert not f.validate(_Tdoa(dd=3.0, ids=(9, 1)), error=0.0, now_ms=10)
    assert not f.validate(_Tdoa(dd=3.0, ids=(9, 1)), error=0.0, now_ms=11)
    # Consistent data keeps flowing: detector accepts, EWMA decays, and the
    # anchor is eventually readmitted.
    readmitted = False
    for i in range(10):
        if f.validate(_Tdoa(dd=0.0, ids=(9, 1)), error=0.0, now_ms=20 + i):
            readmitted = True
            break
    assert readmitted
