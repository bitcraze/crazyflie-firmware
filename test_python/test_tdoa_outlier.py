"""Tests for the pluggable outlier filters (no cffirmware bindings required).

The 'integrator' filter wraps the real C filter and is exercised with the
bindings in test_tdoa_outlier_seam.py; here we only check its registration.
"""

import pytest

from bindings.util.tdoa_outlier import FILTER_FACTORIES, make_outlier_filter


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
        'integrator', 'none', 'sanity', 'fixed', 'mad_window', 'pair_integrator'}


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


def test_fixed_gate_is_2p5_stddev():
    f = make_outlier_filter('fixed')
    t = _Tdoa(std=0.15)   # gate = 0.375
    assert f.validate(t, error=0.37, now_ms=0)
    assert not f.validate(t, error=0.38, now_ms=0)
    assert not f.validate(t, error=-0.38, now_ms=0)


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
