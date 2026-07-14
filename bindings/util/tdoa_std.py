"""
Per-update TDoA measurement-noise (std) models for offline replay.

Third seam next to selection policies (tdoa_selection.py) and outlier filters
(tdoa_outlier.py): instead of the constant TDOA_ENGINE_MEASUREMENT_NOISE_STD,
a std model computes the stdDev for every measurement before the Kalman
update. Soft outlier handling — a doubtful measurement is de-weighted rather
than rejected.

``stddev(tdoa, sample, error, now_ms)`` receives the tdoaMeasurement_t (with
``stdDev`` preset to the base value), the raw sample dict (which carries the
per-packet candidate statistics ``n_cand`` and ``group_spread`` attached by
``apply_policy``), the innovation ``error`` [m] against the current state
(may be NaN for degenerate geometry — return the base std then), and the
sample time [ms]. It returns the std to use.

All models must stay firmware-implementable: causal, O(1) state, cheap math.
"""

import math


class StdModel:
    """Base class. ``stddev`` returns the measurement std to use [m]."""
    name = 'base'

    def reset(self):
        """Reset internal state (called once before a replay run)."""

    def stddev(self, tdoa, sample, error, now_ms):
        raise NotImplementedError


class HuberStdModel(StdModel):
    """Huber-style soft gate: inflate std linearly once the innovation exceeds
    DELTA_STD innovation-sigmas, so far-off measurements still pull the state
    (allowing recovery after a divergence) but with bounded influence.

    Equivalent in spirit to the mm_tdoa_robust M-estimator but a single
    multiply on top of the standard update — trivially portable to C.

    MAX_INFLATION caps the de-weighting: without it, a diverged state
    inflates every measurement's std proportionally to its innovation and
    nothing can ever pull the state back (observed as replay divergence).
    With the cap, a far-off measurement retains at least
    1/MAX_INFLATION**2 of its nominal information.
    """
    name = 'huber'
    DELTA_STD = 3.0
    MAX_INFLATION = 4.0

    def stddev(self, tdoa, sample, error, now_ms):
        base = tdoa.stdDev
        if math.isnan(error):
            return base
        delta = self.DELTA_STD * base
        if abs(error) <= delta:
            return base
        return base * min(abs(error) / delta, self.MAX_INFLATION)


class SpreadStdModel(StdModel):
    """Scale std with the candidate group's spread (MAD of the packet's
    distanceDiff candidates): a packet whose candidates disagree is likely
    corrupted by multipath/NLOS, so every measurement drawn from it is
    de-weighted. State-independent — cannot self-confirm a diverged state.
    """
    name = 'spread'
    GAIN = 1.0

    def stddev(self, tdoa, sample, error, now_ms):
        base = tdoa.stdDev
        spread = float(sample.get('group_spread', 0.0))
        return math.sqrt(base * base + (self.GAIN * spread) ** 2)


STD_MODEL_FACTORIES = {
    'huber': HuberStdModel,
    'spread': SpreadStdModel,
}


def make_std_model(name, params=None):
    try:
        m = STD_MODEL_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown std model '{name}'. "
            f"Available: {', '.join(sorted(STD_MODEL_FACTORIES))}")
    for key, value in (params or {}).items():
        if not hasattr(m, key):
            raise ValueError(f"Std model '{name}' has no tuning constant '{key}'")
        setattr(m, key, value)
    m.reset()
    return m
