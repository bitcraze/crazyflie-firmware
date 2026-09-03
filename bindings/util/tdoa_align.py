"""
Rigid alignment between the ground-truth frame and the Loco anchor frame.

Lighthouse and Loco are surveyed independently, so their frames agree only as
well as the two surveys do. On the lab logs measured here the disagreement is
a systematic ~13 cm -- larger than the estimator's own scatter -- and it
contaminates anything scored against Lighthouse:

* the accuracy report reads it as estimator error, when no estimator could
  remove it;
* ``tdoa_selection.annotate_oracle_error`` reads it as measurement error, so an
  oracle spends its selection budget cancelling a survey offset instead of
  finding genuinely clean measurements, and the headroom it reports is not the
  headroom a real policy could reach.

Fitting the transform separates the two. What is left after alignment is the
part an estimator (or a selection policy) can actually act on, and the fitted
transform itself is the diagnostic: a pure translation is an origin offset, a
rotation is a mis-levelled or mis-yawed survey, and a scale departure from 1 is
a clock or speed-of-light calibration error rather than a geometric one.

The fit is Kabsch/Umeyama -- the closed-form least-squares rigid transform --
on the samples the two trajectories have in common.
"""

import math


def _paired_samples(source, target, max_gap_ms):
    """Sample pairs at the source's timestamps, interpolating the target.

    Returns (P, Q) as lists of (x, y, z), where P[i] and Q[i] are the two
    frames' view of the same instant. Source samples outside the target's span,
    or falling in a target gap wider than ``max_gap_ms``, are dropped rather
    than extrapolated.
    """
    import bisect
    if not source or not target:
        return [], []
    times = [t for t, _ in target]
    points = [p for _, p in target]
    P, Q = [], []
    for t_ms, p in source:
        if t_ms < times[0] or t_ms > times[-1]:
            continue
        i = bisect.bisect_left(times, t_ms)
        if i == 0:
            q = points[0]
        else:
            t0, t1 = times[i - 1], times[i]
            if t1 - t0 > max_gap_ms:
                continue
            w = 0.0 if t1 == t0 else (t_ms - t0) / (t1 - t0)
            a, b = points[i - 1], points[i]
            q = (a[0] + w * (b[0] - a[0]),
                 a[1] + w * (b[1] - a[1]),
                 a[2] + w * (b[2] - a[2]))
        P.append(tuple(p))
        Q.append(q)
    return P, Q


class Alignment:
    """A fitted similarity transform ``q = scale * R @ p + t`` and its report."""

    def __init__(self, R, t, scale, n, rms_before, rms_after):
        self.R = R
        self.t = t
        self.scale = scale
        self.n = n
        self.rms_before = rms_before
        self.rms_after = rms_after

    def apply(self, point):
        R, t, s = self.R, self.t, self.scale
        x, y, z = point
        return (s * (R[0][0] * x + R[0][1] * y + R[0][2] * z) + t[0],
                s * (R[1][0] * x + R[1][1] * y + R[1][2] * z) + t[1],
                s * (R[2][0] * x + R[2][1] * y + R[2][2] * z) + t[2])

    def apply_trajectory(self, trajectory):
        """Map [(t_ms, (x, y, z))] into the target frame."""
        return [(t_ms, self.apply(p)) for t_ms, p in trajectory]

    @property
    def rotation_deg(self):
        """Rotation magnitude as a single angle [deg] (axis-angle form).

        One number is the right summary here: the question is whether the two
        surveys are rotated relative to each other at all, not about which axis.
        """
        trace = self.R[0][0] + self.R[1][1] + self.R[2][2]
        return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))

    @property
    def translation_norm(self):
        return math.sqrt(sum(c * c for c in self.t))

    def describe(self):
        lines = [
            f'  fitted on {self.n} paired samples',
            f'  translation  ({self.t[0]:+.4f}, {self.t[1]:+.4f}, '
            f'{self.t[2]:+.4f}) m   |t| = {self.translation_norm:.4f} m',
            f'  rotation     {self.rotation_deg:.3f} deg',
            f'  scale        {self.scale:.6f}'
            + ('' if abs(self.scale - 1.0) < 1e-9
               else f'   ({(self.scale - 1.0) * 1e6:+.0f} ppm)'),
            f'  residual rms {self.rms_before:.4f} m -> {self.rms_after:.4f} m',
        ]
        return '\n'.join(lines)


ALIGNMENT_MODELS = ('translation', 'rigid', 'similarity')


def fit_alignment(source, target, max_gap_ms=100.0, model='rigid',
                  with_scale=None):
    """Least-squares transform mapping source onto target.

    Args:
        source: [(t_ms, (x, y, z))] trajectory to be moved, e.g. the Lighthouse
            ground truth.
        target: [(t_ms, (x, y, z))] trajectory defining the destination frame,
            e.g. the live onboard estimate, which lives in the Loco frame.
        max_gap_ms: do not interpolate the target across gaps wider than this.
        model: how many degrees of freedom to grant the fit.
            ``'translation'`` (3 dof) solves only the origin offset;
            ``'rigid'`` (6 dof) adds rotation; ``'similarity'`` (7 dof) adds a
            uniform scale.
        with_scale: deprecated spelling of ``model='similarity'``.

    Choose the model by what the data supports, not by what fits best -- every
    extra degree of freedom lowers the residual whether or not it corresponds
    to anything real. The test is reproducibility across flights: on the lab
    logs measured here the translation repeats to within 4 mm over four
    flights, while the fitted rotation swings between 0.5 and 5.2 degrees and
    the scale between -1.9% and -4.9%. A frame relationship is a property of
    the two surveys and cannot change between flights, so those two terms are
    absorbing noise -- unsurprisingly, given a flight volume only a few metres
    across offers little leverage on either. Translation-only is the honest
    model there; rigid and similarity are worth fitting to *check* whether
    rotation and scale reproduce, which is exactly how that was established.

    Fitting on the *live* trajectory rather than a replayed one keeps the
    transform independent of whatever policy or filter a replay is testing, so
    the same alignment can be applied when comparing replays against each other.
    Note that the transform is still fitted to data: absolute accuracy after
    alignment is optimistic by however much the fit absorbed, while comparisons
    between runs sharing one alignment stay fair.

    Returns an Alignment, or None when fewer than 3 samples pair up.
    """
    import numpy as np

    if with_scale is not None:
        model = 'similarity' if with_scale else 'rigid'
    if model not in ALIGNMENT_MODELS:
        raise ValueError(f"unknown alignment model '{model}', "
                         f"expected one of {ALIGNMENT_MODELS}")

    P, Q = _paired_samples(source, target, max_gap_ms)
    if len(P) < 3:
        return None
    A = np.asarray(P, dtype=float)
    B = np.asarray(Q, dtype=float)
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    A0, B0 = A - ca, B - cb

    if model == 'translation':
        R, S, d = np.eye(3), np.zeros(3), 1.0
    else:
        H = A0.T @ B0
        U, S, Vt = np.linalg.svd(H)
        # Reflections are valid solutions of the unconstrained problem but not
        # physical: flip the least-significant axis to keep det(R) = +1.
        d = 1.0 if np.linalg.det(Vt.T @ U.T) > 0 else -1.0
        R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T

    scale = 1.0
    if model == 'similarity':
        varA = (A0 ** 2).sum()
        if varA > 0:
            scale = float((S * np.array([1.0, 1.0, d])).sum() / varA)
    t = cb - scale * (R @ ca)

    rms_before = float(np.sqrt(((A - B) ** 2).sum(axis=1).mean()))
    moved = (scale * (R @ A.T).T) + t
    rms_after = float(np.sqrt(((moved - B) ** 2).sum(axis=1).mean()))
    return Alignment([list(map(float, row)) for row in R],
                     [float(c) for c in t], scale, len(P),
                     rms_before, rms_after)
