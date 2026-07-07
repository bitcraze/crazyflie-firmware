"""
Pluggable TDoA outlier filters for offline replay.

Companion of ``tdoa_selection.py`` (selection policies): where a policy decides
*which* candidate measurements to feed to the estimator, an outlier filter
decides — per measurement, given the innovation against the current Kalman
state — whether the update is applied at all. The live firmware hard-codes the
integrator filter inside ``kalmanCoreUpdateWithTdoa`` (mm_tdoa.c); the replay
seam externalizes that decision so alternative filter designs can be A/B tested
on identical logged data.

``validate(tdoa, error, now_ms)`` receives a ``tdoaMeasurement_t``-shaped
object (anchor ids/positions, distanceDiff, stdDev), the innovation ``error``
[m] computed by the firmware measurement model, and the sample time [ms].
Only ``IntegratorFilter`` needs the cffirmware bindings (imported lazily);
everything else is pure Python and unit-testable without them.
"""

from collections import deque


def _anchor_distance_sq(tdoa):
    a, b = tdoa.anchorPositionA, tdoa.anchorPositionB
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2


def _is_physically_possible(tdoa):
    # |distance difference| must be smaller than the anchor separation
    # (same check as isDistanceDiffSmallerThanDistanceBetweenAnchors in
    # outlierFilterTdoa.c).
    return tdoa.distanceDiff ** 2 < _anchor_distance_sq(tdoa)


class OutlierFilter:
    """Base class. ``validate`` returns True if the update should be applied."""
    name = 'base'

    def reset(self):
        """Reset internal state (called once before a replay run)."""

    def validate(self, tdoa, error, now_ms):
        raise NotImplementedError


class IntegratorFilter(OutlierFilter):
    """The real firmware filter (outlierFilterTdoaValidateIntegrator) via the
    bindings — the firmware-parity baseline. Requires ``tdoa`` to be an actual
    ``cffirmware.tdoaMeasurement_t``."""
    name = 'integrator'

    def __init__(self):
        import cffirmware  # lazy: only this filter needs the bindings
        self._cffirmware = cffirmware
        self._state = cffirmware.OutlierFilterTdoaState_t()
        self.reset()

    def reset(self):
        self._cffirmware.outlierFilterTdoaReset(self._state)

    def validate(self, tdoa, error, now_ms):
        return bool(self._cffirmware.outlierFilterTdoaValidateIntegrator(
            self._state, tdoa, error, int(now_ms)))


class NoneFilter(OutlierFilter):
    """Accept everything — the control group."""
    name = 'none'

    def validate(self, tdoa, error, now_ms):
        return True


class SanityFilter(OutlierFilter):
    """Only the physically-impossible check; no innovation gating."""
    name = 'sanity'

    def validate(self, tdoa, error, now_ms):
        return _is_physically_possible(tdoa)


class FixedGateFilter(OutlierFilter):
    """Static innovation gate at the integrator's closed-state acceptance
    level (2.5 sigma), with no adaptivity: a probe for whether the
    integrator's open/close machinery earns its keep."""
    name = 'fixed'
    GATE_STD_MULTIPLIER = 2.5

    def validate(self, tdoa, error, now_ms):
        return abs(error) < self.GATE_STD_MULTIPLIER * tdoa.stdDev


class MadWindowFilter(OutlierFilter):
    """Robust adaptive gate: reject innovations far from the running median,
    scaled by the median absolute deviation (MAD) of a sliding window.

    All innovations — accepted and rejected — enter the window, so a sustained
    level shift (a real position jump the estimator must follow) drags the
    median and reopens the gate; that is this filter's equivalent of the
    integrator's force-open recovery. The MAD is floored so a quiet window
    cannot collapse the gate, and everything is accepted until the window
    holds WARMUP samples.
    """
    name = 'mad_window'
    WINDOW = 50
    K = 5.0
    WARMUP = 20
    MAD_FLOOR_STD_MULTIPLIER = 0.5

    def __init__(self):
        self.reset()

    def reset(self):
        self._window = deque(maxlen=self.WINDOW)

    @staticmethod
    def _median(values):
        vals = sorted(values)
        n = len(vals)
        mid = n // 2
        if n % 2:
            return vals[mid]
        return 0.5 * (vals[mid - 1] + vals[mid])

    def validate(self, tdoa, error, now_ms):
        if len(self._window) < self.WARMUP:
            accepted = True
        else:
            med = self._median(self._window)
            mad = self._median(abs(e - med) for e in self._window)
            mad = max(mad, self.MAD_FLOOR_STD_MULTIPLIER * tdoa.stdDev)
            accepted = abs(error - med) <= self.K * mad
        self._window.append(error)
        return accepted


class PairIntegratorFilter(OutlierFilter):
    """Python port of the firmware integrator with one state per anchor pair,
    so a single bad anchor cannot force the global filter open. Same constants
    and update logic as outlierFilterTdoa.c, keyed by (idA, idB)."""
    name = 'pair_integrator'
    INTEGRATOR_SIZE = 300.0
    FORCE_OPEN_LEVEL = INTEGRATOR_SIZE * 0.1
    RESUME_ACTION_LEVEL = INTEGRATOR_SIZE * 0.9
    ACCEPT_STD_MULTIPLIER = 2.5
    TRIGGER_STD_MULTIPLIER = 2.0

    def __init__(self):
        self.reset()

    def reset(self):
        self._pairs = {}

    def validate(self, tdoa, error, now_ms):
        if not _is_physically_possible(tdoa):
            return False
        key = (int(tdoa.anchorIdA), int(tdoa.anchorIdB))
        s = self._pairs.setdefault(
            key, {'integrator': 0.0, 'is_open': True, 'latest_ms': 0})

        accepted_distance = tdoa.stdDev * self.ACCEPT_STD_MULTIPLIER
        trigger_distance = tdoa.stdDev * self.TRIGGER_STD_MULTIPLIER

        dt_ms = min(now_ms - s['latest_ms'], self.INTEGRATOR_SIZE / 10.0)
        if abs(error) < trigger_distance:
            s['integrator'] = min(s['integrator'] + dt_ms, self.INTEGRATOR_SIZE)
        else:
            s['integrator'] = max(s['integrator'] - dt_ms, 0.0)

        if s['is_open']:
            sample_is_good = True
            if s['integrator'] > self.RESUME_ACTION_LEVEL:
                s['is_open'] = False
        else:
            sample_is_good = abs(error) < accepted_distance
            if s['integrator'] < self.FORCE_OPEN_LEVEL:
                s['is_open'] = True

        s['latest_ms'] = now_ms
        return sample_is_good


# Factories (not instances) so each replay run gets fresh filter state.
FILTER_FACTORIES = {
    'integrator': IntegratorFilter,
    'none': NoneFilter,
    'sanity': SanityFilter,
    'fixed': FixedGateFilter,
    'mad_window': MadWindowFilter,
    'pair_integrator': PairIntegratorFilter,
}


def make_outlier_filter(name):
    try:
        return FILTER_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown outlier filter '{name}'. "
            f"Available: {', '.join(sorted(FILTER_FACTORIES))}")
