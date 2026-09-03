"""Tests for the ground-truth/anchor-frame alignment fit (needs numpy only)."""

import math

import pytest

from bindings.util.tdoa_align import ALIGNMENT_MODELS, fit_alignment


def _traj(points, step_ms=20.0):
    return [(i * step_ms, p) for i, p in enumerate(points)]


# A path with extent on all three axes, so rotation and scale are observable.
_PATH = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.2), (1.0, 1.0, 0.4), (0.0, 1.0, 0.6),
         (0.0, 0.0, 0.8), (2.0, 1.0, 1.0), (2.0, -1.0, 1.2), (-1.0, 0.5, 1.4)]


def _apply(points, R, t, s=1.0):
    out = []
    for x, y, z in points:
        out.append((s * (R[0][0]*x + R[0][1]*y + R[0][2]*z) + t[0],
                    s * (R[1][0]*x + R[1][1]*y + R[1][2]*z) + t[1],
                    s * (R[2][0]*x + R[2][1]*y + R[2][2]*z) + t[2]))
    return out


_I = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def test_recovers_a_pure_translation():
    offset = (0.086, 0.011, 0.106)      # the shape of the real lab offset
    a = fit_alignment(_traj(_PATH), _traj(_apply(_PATH, _I, offset)),
                      model='translation')
    assert a.t == pytest.approx(list(offset), abs=1e-9)
    assert a.rotation_deg == pytest.approx(0.0, abs=1e-9)
    assert a.scale == 1.0
    assert a.rms_after == pytest.approx(0.0, abs=1e-9)
    assert a.rms_before == pytest.approx(math.sqrt(sum(c*c for c in offset)),
                                         abs=1e-9)


def test_recovers_a_rotation_and_translation():
    R, t = _rot_z(12.0), (0.5, -0.25, 0.1)
    a = fit_alignment(_traj(_PATH), _traj(_apply(_PATH, R, t)), model='rigid')
    assert a.rotation_deg == pytest.approx(12.0, abs=1e-6)
    assert a.t == pytest.approx(list(t), abs=1e-9)
    assert a.rms_after == pytest.approx(0.0, abs=1e-9)


def test_recovers_a_scale_only_when_the_model_allows_it():
    R, t, s = _rot_z(5.0), (0.1, 0.2, 0.3), 0.97
    src, dst = _traj(_PATH), _traj(_apply(_PATH, R, t, s))
    loose = fit_alignment(src, dst, model='similarity')
    assert loose.scale == pytest.approx(s, abs=1e-9)
    assert loose.rms_after == pytest.approx(0.0, abs=1e-9)
    # The rigid model cannot represent the scale, so it must leave a residual
    # rather than silently absorbing it somewhere else.
    tight = fit_alignment(src, dst, model='rigid')
    assert tight.scale == 1.0
    assert tight.rms_after > 0.01


def test_extra_degrees_of_freedom_never_raise_the_residual():
    # The nested models must be genuinely nested, which is precisely why "the
    # residual fell" is worthless as evidence and reproducibility across
    # flights is the real test. Perturb the target so no model fits exactly,
    # otherwise all three land at machine precision and the ordering is noise.
    moved = _apply(_PATH, _rot_z(8.0), (0.2, 0.0, 0.1))
    wobble = [(x + 0.03 * (i % 3), y - 0.02 * (i % 2), z + 0.01 * i)
              for i, (x, y, z) in enumerate(moved)]
    src, dst = _traj(_PATH), _traj(wobble)
    loose, mid, tight = (fit_alignment(src, dst, model=m).rms_after
                         for m in ('translation', 'rigid', 'similarity'))
    assert tight > 0.0                      # nothing fits this exactly
    assert loose >= mid >= tight


def test_alignment_maps_the_trajectory_into_the_target_frame():
    offset = (0.086, 0.011, 0.106)
    src = _traj(_PATH)
    a = fit_alignment(src, _traj(_apply(_PATH, _I, offset)), model='translation')
    moved = a.apply_trajectory(src)
    assert [t for t, _ in moved] == [t for t, _ in src]
    assert moved[3][1] == pytest.approx(
        tuple(_PATH[3][k] + offset[k] for k in range(3)), abs=1e-9)


def test_a_reflection_is_never_returned():
    # Mirrored data has a lower-residual improper solution; the fit must keep
    # det(R) = +1, because two surveys of one room cannot be mirror images.
    mirrored = [(-x, y, z) for x, y, z in _PATH]
    a = fit_alignment(_traj(_PATH), _traj(mirrored), model='rigid')
    R = a.R
    det = (R[0][0]*(R[1][1]*R[2][2] - R[1][2]*R[2][1])
           - R[0][1]*(R[1][0]*R[2][2] - R[1][2]*R[2][0])
           + R[0][2]*(R[1][0]*R[2][1] - R[1][1]*R[2][0]))
    assert det == pytest.approx(1.0, abs=1e-9)


def test_target_gaps_are_not_bridged():
    src = _traj(_PATH, step_ms=20.0)
    # Target holds only the endpoints, 140 ms apart: every interior source
    # sample would need interpolation across that gap.
    target = [(0.0, (0.0, 0.0, 0.0)), (140.0, (0.0, 0.0, 0.0))]
    assert fit_alignment(src, target, max_gap_ms=50.0) is None
    assert fit_alignment(src, target, max_gap_ms=200.0) is not None


def test_too_few_paired_samples_returns_none():
    assert fit_alignment([], _traj(_PATH)) is None
    assert fit_alignment(_traj(_PATH[:2]), _traj(_PATH[:2])) is None


def test_unknown_model_raises():
    with pytest.raises(ValueError, match='unknown alignment model'):
        fit_alignment(_traj(_PATH), _traj(_PATH), model='vibes')
    assert set(ALIGNMENT_MODELS) == {'translation', 'rigid', 'similarity'}


def test_with_scale_keyword_still_selects_a_model():
    src, dst = _traj(_PATH), _traj(_apply(_PATH, _I, (0.1, 0.0, 0.0), 0.95))
    assert fit_alignment(src, dst, with_scale=True).scale == pytest.approx(0.95)
    assert fit_alignment(src, dst, with_scale=False).scale == 1.0
