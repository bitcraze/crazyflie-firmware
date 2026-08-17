"""Parity + integration tests for the outlier-filter replay seam.

THE key test: replaying identical input through the legacy C path
(outlier filter hard-wired in kalmanCoreUpdateWithTdoa) and through the new
seam with the 'integrator' filter must produce EXACTLY equal trajectories —
this is the drift alarm for the mm_tdoa.c refactor.

Requires the SWIG bindings: make bindings_python, run with PYTHONPATH=build.
"""

import math
import random

import pytest

cffirmware = pytest.importorskip('cffirmware')

from bindings.util.tdoa_replay import replay

# Same synthetic scene as test_tdoa_replay_smoke.py: 8 anchors on a
# 4 x 4 x 3 m box, stationary tag — but with outliers injected so the
# outlier filter actually rejects samples (otherwise parity is trivial).
ANCHOR_COORDS = {
    0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0), 2: (0.0, 4.0, 0.0), 3: (4.0, 4.0, 0.0),
    4: (0.0, 0.0, 3.0), 5: (4.0, 0.0, 3.0), 6: (0.0, 4.0, 3.0), 7: (4.0, 4.0, 3.0),
}
TRUE_POS = (2.0, 2.0, 1.0)
DURATION_MS = 5000


def _anchor_positions():
    result = {}
    for anchor_id, (x, y, z) in ANCHOR_COORDS.items():
        p = cffirmware.vec3_s()
        p.x, p.y, p.z = x, y, z
        result[anchor_id] = p
    return result


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _synthetic_imu():
    samples = []
    for t in range(DURATION_MS):
        ts = float(t)
        samples.append(('estAcceleration',
                        {'timestamp': ts, 'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0}))
        samples.append(('estGyroscope',
                        {'timestamp': ts, 'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0}))
    return samples


def _synthetic_tdoa_with_outliers():
    # 100 Hz rotating pairs; every 10th sample corrupted by a large,
    # physically-possible error. Deterministic (seeded) so both replays see
    # byte-identical input.
    rng = random.Random(0)
    samples = []
    for k, t in enumerate(range(0, DURATION_MS, 10)):
        id_a = k % 8
        id_b = (k + 1) % 8
        dd = _dist(ANCHOR_COORDS[id_b], TRUE_POS) - _dist(ANCHOR_COORDS[id_a], TRUE_POS)
        dd += rng.gauss(0.0, 0.02)
        if k % 10 == 0:
            dd += rng.choice([-1.0, 1.0]) * 1.5
        samples.append(('estTDOA',
                        {'timestamp': float(t), 'idA': id_a, 'idB': id_b,
                         'distanceDiff': dd}))
    return samples


def test_integrator_via_seam_is_bit_identical_to_legacy_path():
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    legacy = replay(_anchor_positions(), list(imu), list(tdoa),
                    {'tdoa_std': 0.15})
    seam = replay(_anchor_positions(), list(imu), list(tdoa),
                  {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})
    assert seam == legacy


def test_none_filter_gives_a_different_trajectory_with_outliers_present():
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    gated = replay(_anchor_positions(), list(imu), list(tdoa),
                   {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})
    ungated = replay(_anchor_positions(), list(imu), list(tdoa),
                     {'tdoa_std': 0.15, 'outlier_filter': 'none'})
    assert ungated != gated


def test_every_registered_filter_replays_and_converges():
    from bindings.util.tdoa_outlier import FILTER_FACTORIES
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    for name in FILTER_FACTORIES:
        trajectory = replay(_anchor_positions(), list(imu), list(tdoa),
                            {'tdoa_std': 0.15, 'outlier_filter': name})
        assert len(trajectory) > 4000, name
        _, final_pos = trajectory[-1]
        # Outliers are injected, so allow a loose bound; 'none' is the worst.
        assert _dist(final_pos, TRUE_POS) < 1.5, name


def test_unknown_filter_name_fails_fast():
    with pytest.raises(ValueError, match='Unknown outlier filter'):
        replay(_anchor_positions(), _synthetic_imu()[:10], [],
               {'outlier_filter': 'nope'})


def test_robust_with_none_filter_equals_direct_robust_path():
    # The innovation call is pure and 'none' accepts everything, so the seam
    # must be a transparent wrapper around the robust update.
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()

    def robust_replay(extra_params):
        # mm_tdoa_robust.c carries M-estimation state across calls in a
        # static x_err buffer (see the replay docstring caveat). Prime with a
        # throwaway robust replay so both measured replays start from
        # identical static state.
        replay(_anchor_positions(), list(imu), list(tdoa),
               {'tdoa_std': 0.15, 'tdoa_model': 'robust'})
        return replay(_anchor_positions(), list(imu), list(tdoa),
                      {'tdoa_std': 0.15, 'tdoa_model': 'robust', **extra_params})

    direct = robust_replay({})
    seam = robust_replay({'outlier_filter': 'none'})
    assert seam == direct


def test_gated_robust_model_replays_and_converges():
    # Novel offline-only combination: outlier gate in front of the robust
    # update.
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    trajectory = replay(_anchor_positions(), list(imu), list(tdoa),
                        {'tdoa_std': 0.15, 'tdoa_model': 'robust',
                         'outlier_filter': 'mad_window'})
    assert len(trajectory) > 4000
    _, final_pos = trajectory[-1]
    assert _dist(final_pos, TRUE_POS) < 1.0
