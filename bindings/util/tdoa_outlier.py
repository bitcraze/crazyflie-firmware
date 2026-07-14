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


def _median(values):
    vals = sorted(values)
    n = len(vals)
    mid = n // 2
    if n % 2:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


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
        return _median(values)

    def validate(self, tdoa, error, now_ms):
        if len(self._window) < self.WARMUP:
            accepted = True
        else:
            med = _median(self._window)
            mad = _median(abs(e - med) for e in self._window)
            mad = max(mad, self.MAD_FLOOR_STD_MULTIPLIER * tdoa.stdDev)
            accepted = abs(error - med) < self.K * mad
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

        dt_ms = now_ms - s['latest_ms']
        if dt_ms < 0:
            # The C original computes this in uint32_t: a negative delta wraps
            # to a huge value and the fminf clamp reduces it to the cap.
            dt_ms = self.INTEGRATOR_SIZE / 10.0
        else:
            dt_ms = min(dt_ms, self.INTEGRATOR_SIZE / 10.0)
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


class PairHampelFilter(OutlierFilter):
    """Measurement-domain (not innovation-domain) Hampel gate, per anchor pair.

    Unlike every other filter here, this one never looks at the Kalman
    innovation: it keeps, per (idA, idB) anchor pair, a sliding window of raw
    ``distanceDiff`` values and rejects a new sample that deviates from the
    pair's median by more than K times the pair's MAD (median absolute
    deviation), floored so a quiet window cannot collapse the gate. Because
    the decision is purely a function of the measurement stream, a diverged
    Kalman state can never make this filter "agree" with it — it cannot
    self-confirm the way an innovation-gated filter can.

    As with MadWindowFilter, every sample — accepted or rejected — enters the
    window, so a sustained level shift (a real, persistent change in the
    pair's distanceDiff) eventually drags the median along and reopens the
    gate. A window is cleared if it goes stale (no sample for MAX_AGE_MS):
    stale history must not gate fresh data after a coverage gap.
    """
    name = 'pair_hampel'
    WINDOW = 8
    WARMUP = 4
    K = 5.0
    MAD_FLOOR_STD_MULTIPLIER = 0.5
    MAX_AGE_MS = 2000

    def __init__(self):
        self.reset()

    def reset(self):
        self._pairs = {}

    def validate(self, tdoa, error, now_ms):
        if not _is_physically_possible(tdoa):
            return False

        key = (int(tdoa.anchorIdA), int(tdoa.anchorIdB))
        s = self._pairs.setdefault(
            key, {'window': deque(maxlen=self.WINDOW), 'last_seen_ms': None})

        if s['last_seen_ms'] is not None \
                and now_ms - s['last_seen_ms'] > self.MAX_AGE_MS:
            s['window'].clear()

        window = s['window']
        if len(window) < self.WARMUP:
            accepted = True
        else:
            med = _median(window)
            mad = _median(abs(v - med) for v in window)
            mad = max(mad, self.MAD_FLOOR_STD_MULTIPLIER * tdoa.stdDev)
            accepted = abs(tdoa.distanceDiff - med) < self.K * mad

        window.append(tdoa.distanceDiff)
        s['last_seen_ms'] = now_ms
        return accepted


class AndFilter(OutlierFilter):
    """Composition: accept only if every child filter accepts.

    Every child's ``validate`` is called on every sample, with no
    short-circuiting — stateful children (e.g. MadWindowFilter,
    PairIntegratorFilter) must observe every sample to keep their internal
    state consistent, even once the combined result is already known to be
    False.

    Note: ``make_outlier_filter`` applies ``params`` to a filter via
    ``setattr``/``hasattr`` on the top-level instance, so tuning-constant
    overrides are not routed to children of a composite; construct children
    with their desired constants already set (e.g. via a factory closure)
    instead of relying on the ``params`` mechanism for filters built from
    this class.
    """
    name = 'and'

    def __init__(self, filters):
        self._filters = list(filters)

    def reset(self):
        for f in self._filters:
            f.reset()

    def validate(self, tdoa, error, now_ms):
        results = [f.validate(tdoa, error, now_ms) for f in self._filters]
        return all(results)


class AnchorQualityFilter(OutlierFilter):
    """Per-anchor health tracking on top of the pair-Hampel detector.

    Motivation (seen in ground-truth diagnostics of real logs): outlier bursts
    are often anchor-specific — one anchor goes bad for a few seconds (NLOS,
    interference) and pollutes *every* pair it participates in. A per-pair
    gate rejects many of those samples one by one, but each pair's window has
    to discover the problem separately. This filter aggregates: it keeps one
    exponentially-weighted rejection rate per anchor id, fed by the pair-Hampel
    verdicts of every sample the anchor participates in, and suppresses all
    measurements of an anchor whose recent rejection rate exceeds Q_MAX.

    Recovery is data-driven: a suppressed anchor's samples keep flowing
    through the detector (windows and rates keep updating), so when its
    measurements become consistent again the rejection rate decays below
    Q_MAX and the anchor is readmitted.

    ALPHA is a per-sample EWMA weight, so its effective time constant scales
    with the anchor's sample rate (~1/(ALPHA * rate)); at TDoA3 rates each
    anchor participates in a few hundred samples/s.

    State: the pair windows of PairHampelFilter plus one float per anchor id
    — comfortably firmware-sized.
    """
    name = 'anchor_quality'
    ALPHA = 0.005
    Q_MAX = 0.35

    def __init__(self):
        self._detector = PairHampelFilter()
        self.reset()

    def reset(self):
        self._detector.reset()
        self._quality = {}

    def validate(self, tdoa, error, now_ms):
        detector_ok = self._detector.validate(tdoa, error, now_ms)
        reject = 0.0 if detector_ok else 1.0
        ids = (int(tdoa.anchorIdA), int(tdoa.anchorIdB))
        for anchor_id in ids:
            q = self._quality.get(anchor_id, 0.0)
            self._quality[anchor_id] = q + self.ALPHA * (reject - q)
        return detector_ok and all(
            self._quality[anchor_id] <= self.Q_MAX for anchor_id in ids)


def _make_sanity_mad_filter():
    return AndFilter([SanityFilter(), MadWindowFilter()])


def _make_hampel_mad_filter():
    return AndFilter([PairHampelFilter(), MadWindowFilter()])


# Factories (not instances) so each replay run gets fresh filter state.
FILTER_FACTORIES = {
    'integrator': IntegratorFilter,
    'none': NoneFilter,
    'sanity': SanityFilter,
    'mad_window': MadWindowFilter,
    'pair_integrator': PairIntegratorFilter,
    'pair_hampel': PairHampelFilter,
    'sanity_mad': _make_sanity_mad_filter,
    'hampel_mad': _make_hampel_mad_filter,
    'anchor_quality': AnchorQualityFilter,
}


def make_outlier_filter(name, params=None):
    try:
        f = FILTER_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown outlier filter '{name}'. "
            f"Available: {', '.join(sorted(FILTER_FACTORIES))}")
    # Tuning-constant overrides (class attrs like K / WINDOW); reset afterwards
    # so state sized from a constant (e.g. a deque maxlen) picks it up.
    for key, value in (params or {}).items():
        if not hasattr(f, key):
            raise ValueError(f"Filter '{name}' has no tuning constant '{key}'")
        setattr(f, key, value)
    f.reset()
    return f
