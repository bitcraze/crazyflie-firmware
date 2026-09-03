"""
Anchor-pair selection policies for offline replay of TDoA candidate logs.

The firmware (with param ``tdoaEngine.logCand > 0``) logs an ``estTdoaCand``
event for every valid anchor-pair candidate of a received packet, *before* the
matching algorithm collapses the candidates to a single pair. All candidates of
one packet share the same ``group`` value.

This module groups those events back into per-packet candidate sets and provides
pluggable selection policies. A policy maps one candidate group to a list of
TDoA measurements to feed to the estimator (0, 1 or many), which lets us A/B
different selection strategies -- including feeding *all* candidates instead of
discarding all but one -- on identical logged data.
"""


def build_candidate_groups(log_data):
    """Group ``estTdoaCand`` events by packet.

    Args:
        log_data: dict returned by ``cfusdlog.decode``.

    Returns:
        List of groups in log order. Each group is a dict::

            {
                'group': int,          # per-packet counter from the firmware
                't_ms': float,         # packet time (min timestamp of the group)
                'idA': int,            # transmitting anchor
                'candidates': [ {'idB': int,
                                 'distanceDiff': float,
                                 'isSelected': bool}, ... ],
            }
    """
    cand = log_data.get('estTdoaCand')
    if not cand or 'timestamp' not in cand:
        return []

    groups = {}
    order = []
    for i in range(len(cand['timestamp'])):
        g = int(cand['group'][i])
        if g not in groups:
            groups[g] = {
                'group': g,
                't_ms': float(cand['timestamp'][i]),
                'idA': int(cand['idA'][i]),
                'candidates': [],
            }
            order.append(g)
        grp = groups[g]
        grp['t_ms'] = min(grp['t_ms'], float(cand['timestamp'][i]))
        grp['candidates'].append({
            'idB': int(cand['idB'][i]),
            'distanceDiff': float(cand['distanceDiff'][i]),
            'isSelected': bool(cand['isSelected'][i]),
        })

    return [groups[g] for g in order]


# Default staleness gate [ms]. The firmware holds anchor-to-anchor data to
# REMOTE_DATA_VALIDITY_PERIOD = 30 ms (tdoaStorage.c) but holds the tag's own
# reception of the remote anchor to ANCHOR_ACTIVE_VALIDITY_PERIOD = 2000 ms,
# and calcTDoA() combines the two. 40 ms keeps the 30 ms window plus a margin
# for the jitter in the proxy below.
DEFAULT_MAX_REMOTE_AGE_MS = 40.0


def annotate_remote_age(groups):
    """Attach ``age_ms`` to every candidate: how stale the remote anchor's data is.

    ``calcTDoA`` (tdoaEngine.c) combines two views of the *same* transmission by
    the remote anchor: when the packet anchor heard it (held to 30 ms) and when
    the tag heard it (held to 2000 ms). When the tag's view is the old one the
    two terms can describe different transmissions, and the measurement is off
    by tens to hundreds of metres. Gating on this age removes most of them.

    ``age_ms`` is measured as the time since that anchor last transmitted a
    packet *of its own* in the log, which is a proxy for the tag's reception
    time the firmware actually uses. It is blind only to receptions that
    produced no candidate at all -- ~1% of packets on the logs measured so far,
    countable as gaps in the ``group`` counter (see ``remote_age_blind_fraction``)
    -- so it can only ever over-estimate the age.

    A candidate whose remote anchor has not been seen transmitting yet gets
    ``float('inf')``, which any gate rejects. This affects only the opening
    moments of a log.

    Mutates the candidates in place and returns ``groups`` for chaining.
    """
    last_tx_ms = {}
    for group in groups:
        t_ms = group['t_ms']
        for candidate in group['candidates']:
            last = last_tx_ms.get(candidate['idB'])
            candidate['age_ms'] = float('inf') if last is None else t_ms - last
        # After the group's own candidates: an anchor is never its own remote,
        # but this keeps the update strictly once per processed packet.
        last_tx_ms[group['idA']] = t_ms
    return groups


def remote_age_blind_fraction(log_data):
    """Fraction of received packets that annotate_remote_age cannot see.

    The firmware bumps ``candidateLogGroup`` once per processed packet, whether
    or not any candidate survives its validity checks, so gaps in the logged
    ``group`` sequence count the packets that emitted nothing. Those receptions
    still refreshed the tag's clock in the firmware but are invisible here, and
    are the only source of over-estimated ``age_ms``.

    Returns None when there is no candidate data.
    """
    cand = log_data.get('estTdoaCand')
    if not cand or 'timestamp' not in cand:
        return None
    seen = {int(g) for g in cand['group']}
    if not seen:
        return None
    span = max(seen) - min(seen) + 1
    return (span - len(seen)) / span


# Default anchor-pair geometry gate, from firmware PR #1650
# (TDOA_ENGINE_DEFAULT_DISTANCE_RATIO_LIMIT in tdoaEngine.c).
DEFAULT_DISTANCE_RATIO_LIMIT = 0.85


def _anchor_xyz(position):
    """Anchor position as (x, y, z), from a vec3_s, a dict or a sequence."""
    if hasattr(position, 'x'):
        return float(position.x), float(position.y), float(position.z)
    if isinstance(position, dict):
        return float(position['x']), float(position['y']), float(position['z'])
    x, y, z = position
    return float(x), float(y), float(z)


def _anchor_separation(anchor_positions, idA, idB):
    """Distance between two anchors [m], or 0.0 when either is unknown.

    0.0 is also what genuinely coincident anchors give, and both cases are
    rejected by the gate below -- matching the firmware, which bails out of
    isGeometryGoodEnough() on an unknown position and on a non-positive
    squared separation alike.
    """
    a = anchor_positions.get(idA)
    b = anchor_positions.get(idB)
    if a is None or b is None:
        return 0.0
    ax, ay, az = _anchor_xyz(a)
    bx, by, bz = _anchor_xyz(b)
    return ((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2) ** 0.5


def annotate_pair_geometry(groups, anchor_positions):
    """Attach ``distance_ratio`` to every candidate: |distanceDiff| / anchor separation.

    This is the quantity firmware PR #1650 ("Improve Loco TDoA3 pairing") gates
    on in ``isGeometryGoodEnough`` (tdoaEngine.c). The measurement is
    h(p) = |p - A1| - |p - A2|, whose gradient wrt the tag position is
    u1 - u2 -- the difference of the two lines of sight, with magnitude
    2*sin(theta/2). The triangle inequality bounds |distanceDiff| by the
    anchor separation, so a physically consistent measurement has a ratio in
    [0, 1]; it approaches 1 exactly when the tag is near the line through the
    two anchors, where theta -> 0 and the pair carries almost no positional
    information while still carrying full measurement noise.

    A ratio *above* 1 is not poor geometry but an impossible measurement, and
    those dominate what the gate rejects in practice: on the field logs
    measured here roughly three quarters of the rejected candidates are
    ratio > 1, and virtually all of those carry stale remote-anchor data (see
    ``annotate_remote_age``). The gate is therefore a gross-outlier reject
    first and a conditioning test second.

    Candidates whose anchor positions are unknown (or coincident) get
    ``float('inf')``, which any gate rejects -- the firmware rejects those
    cases too.

    Args:
        groups: from ``build_candidate_groups``.
        anchor_positions: dict anchor id -> position, as ``vec3_s`` (from
            ``loco_utils.read_loco_anchor_positions``), ``{'x','y','z'}`` or
            ``(x, y, z)``.

    Mutates the candidates in place and returns ``groups`` for chaining.
    """
    separations = {}
    for group in groups:
        for candidate in group['candidates']:
            pair = (group['idA'], candidate['idB'])
            if pair not in separations:
                separations[pair] = _anchor_separation(anchor_positions, *pair)
            separation = separations[pair]
            candidate['distance_ratio'] = (
                float('inf') if separation <= 0.0
                else abs(candidate['distanceDiff']) / separation)
    return groups


DEFAULT_ORACLE_MAX_GAP_MS = 100.0
ORACLE_SCORES = ('measurement', 'position')


def annotate_oracle_error(groups, anchor_positions, truth,
                          max_gap_ms=DEFAULT_ORACLE_MAX_GAP_MS,
                          score='measurement'):
    """Attach ``oracle_error`` to every candidate: how wrong it is against truth.

    This is the annotation behind ``OraclePolicy``, and the reason an oracle
    over selection is cheap at all. The obvious formulation -- "pick the
    candidate that leaves the estimator closest to the truth" -- is a
    sequential decision problem: every choice moves the Kalman state, which
    changes the innovations of every later packet, so the optimum is a search
    over ``candidates ** packets``. Scoring against the *known true position*
    instead removes the coupling entirely::

        error = distanceDiff - (|g - B| - |g - A|)

    depends only on the measurement and two anchor positions, never on the
    filter. So the whole oracle is one O(candidates) pass before the replay
    starts, and the replay itself runs at baseline speed.

    What that yields is the least-corrupted-measurement bound, not the
    trajectory optimum: picking the best measurement at every packet is greedy
    in measurement space, and the mapping from measurement error to position
    error runs through the measurement Jacobian, which differs per pair. The
    ``score='position'`` mode compensates by dividing by ``|h|``, the magnitude
    of that Jacobian (``2*sin(theta/2)`` for the angle theta subtended by the
    two anchors at the tag), giving a position-equivalent error: a 10 cm
    measurement error on a nearly-collinear pair displaces the estimate much
    further than the same error on a well-conditioned one.

    Candidates get ``float('inf')`` -- which ``OraclePolicy`` treats as
    unusable -- when either anchor position is unknown, or when the nearest
    truth samples are more than ``max_gap_ms`` apart (so the position is
    interpolated across a real gap rather than measured). Refusing to score is
    deliberate: an oracle that quietly guesses is no longer an upper bound.

    Args:
        groups: from ``build_candidate_groups``.
        anchor_positions: dict anchor id -> position (``vec3_s``, ``{'x','y','z'}``
            or ``(x, y, z)``).
        truth: [(t_ms, (x, y, z))] ground truth, e.g. from
            ``replay_tdoa.extract_ground_truth`` (Lighthouse). Must be sorted.
        max_gap_ms: refuse to interpolate truth across a gap wider than this.
        score: ``'measurement'`` for metres of TDoA error, ``'position'`` for
            metres of equivalent position error.

    Mutates the candidates in place and returns ``groups`` for chaining.
    """
    if score not in ORACLE_SCORES:
        raise ValueError(f"unknown oracle score '{score}', "
                         f"expected one of {ORACLE_SCORES}")
    at = _truth_interpolator(truth, max_gap_ms)
    positions = {}
    for group in groups:
        g = at(group['t_ms'])
        for candidate in group['candidates']:
            m = _measurement(group, candidate)
            a = anchor_positions.get(m['idA'])
            b = anchor_positions.get(m['idB'])
            if g is None or a is None or b is None:
                candidate['oracle_error'] = float('inf')
                continue
            if m['idA'] not in positions:
                positions[m['idA']] = _anchor_xyz(a)
            if m['idB'] not in positions:
                positions[m['idB']] = _anchor_xyz(b)
            ax, ay, az = positions[m['idA']]
            bx, by, bz = positions[m['idB']]
            d0 = ((g[0] - ax) ** 2 + (g[1] - ay) ** 2 + (g[2] - az) ** 2) ** 0.5
            d1 = ((g[0] - bx) ** 2 + (g[1] - by) ** 2 + (g[2] - bz) ** 2) ** 0.5
            if d0 <= 0.0 or d1 <= 0.0:
                candidate['oracle_error'] = float('inf')
                continue
            error = abs(m['distanceDiff'] - (d1 - d0))
            if score == 'position':
                # |h| = |u1 - u0|, the gradient of the measurement wrt the tag
                # position -- the same h that mm_tdoa.c builds.
                hx = (g[0] - bx) / d1 - (g[0] - ax) / d0
                hy = (g[1] - by) / d1 - (g[1] - ay) / d0
                hz = (g[2] - bz) / d1 - (g[2] - az) / d0
                h = (hx * hx + hy * hy + hz * hz) ** 0.5
                error = float('inf') if h <= 0.0 else error / h
            candidate['oracle_error'] = error
    return groups


def _truth_interpolator(truth, max_gap_ms):
    """Linear interpolation over a truth trajectory, refusing to bridge gaps.

    Returns a callable t_ms -> (x, y, z) or None. None is returned outside the
    trajectory's own time span and whenever the two bracketing samples are more
    than ``max_gap_ms`` apart: Lighthouse only produces a crossing-beam fix when
    all four sensors see both base stations, so its samples are irregular and a
    wide gap means the position was never observed there.
    """
    import bisect
    if not truth:
        return lambda t_ms: None
    times = [t for t, _ in truth]
    points = [p for _, p in truth]

    def at(t_ms):
        if t_ms < times[0] or t_ms > times[-1]:
            return None
        i = bisect.bisect_left(times, t_ms)
        if i == 0:
            return points[0]
        t0, t1 = times[i - 1], times[i]
        if t1 - t0 > max_gap_ms:
            return None
        if t1 == t0:
            return points[i]
        w = (t_ms - t0) / (t1 - t0)
        a, b = points[i - 1], points[i]
        return (a[0] + w * (b[0] - a[0]),
                a[1] + w * (b[1] - a[1]),
                a[2] + w * (b[2] - a[2]))
    return at


def _measurement(group, candidate):
    # Convert from the candidate-log convention to the estimator (estTDOA)
    # convention, verified on hardware: the candidate event logs
    # (idA = packet's anchor, idB = remote), while the live estimator
    # (enqueueTDOA in tdoaEngine.c) enqueues (anchorIds[0] = remote,
    # anchorIds[1] = packet's anchor) with the same distanceDiff value
    # (= d(packet anchor) - d(remote) = d(idB) - d(idA) in this convention).
    # Everything downstream (baseline verifier, Kalman emulator) consumes
    # the estimator convention.
    return {
        'idA': candidate['idB'],
        'idB': group['idA'],
        'distanceDiff': candidate['distanceDiff'],
    }


class SelectionPolicy:
    """Base class. ``select`` returns a list of measurement dicts."""
    name = 'base'

    # Candidate keys this policy needs an annotate_* pass to have attached.
    # Empty for every plain selector: they read only what
    # build_candidate_groups produces. A caller can union this over the
    # policies it is about to run and do no annotation work at all when the
    # result is empty (see tools/usdlog/tdoa_experiment.load_context).
    requires = ()

    def reset(self):
        """Reset any internal state (called once before a replay run)."""

    def select(self, group):
        raise NotImplementedError


class BaselinePolicy(SelectionPolicy):
    """Reproduce what the live firmware did: use the flagged selected candidate.

    This is the on-drone baseline and should match the logged ``estTDOA`` /
    ``stateEstimate`` trajectory (up to estimator-config differences).
    """
    name = 'baseline'

    def select(self, group):
        for c in group['candidates']:
            if c['isSelected']:
                return [_measurement(group, c)]
        return []


class AllCandidatesPolicy(SelectionPolicy):
    """Feed every candidate to the estimator instead of discarding all but one.

    The Kalman TDoA outlier filter still gates each update, so this tests whether
    using the discarded candidates (with outlier rejection) is better than the
    current one-pair-per-packet scheme.
    """
    name = 'all'

    def select(self, group):
        return [_measurement(group, c) for c in group['candidates']]


class MedianPolicy(SelectionPolicy):
    """Pick the single candidate whose distanceDiff is closest to the group median.

    A cheap consensus/outlier-resistant selector: an outlier candidate far from
    the rest of the group is unlikely to be chosen.
    """
    name = 'median'

    def select(self, group):
        cs = group['candidates']
        if not cs:
            return []
        vals = sorted(c['distanceDiff'] for c in cs)
        median = vals[len(vals) // 2]
        best = min(cs, key=lambda c: abs(c['distanceDiff'] - median))
        return [_measurement(group, best)]


class TrimmedAllPolicy(SelectionPolicy):
    """Feed every candidate within ``tol_m`` of the group median distanceDiff.

    Middle ground between 'all' (feed everything) and 'median' (feed a single
    candidate): keeps the redundancy of 'all' but trims candidates whose
    distanceDiff is far from the group consensus, which is cheap to compute
    and causal (per-group, no history).
    """
    name = 'trimmed_all'

    def __init__(self, tol_m=0.75):
        self.tol_m = tol_m

    def select(self, group):
        cs = group['candidates']
        if not cs:
            return []
        vals = sorted(c['distanceDiff'] for c in cs)
        median = vals[len(vals) // 2]
        kept = [c for c in cs if abs(c['distanceDiff'] - median) <= self.tol_m]
        if not kept:
            kept = [min(cs, key=lambda c: abs(c['distanceDiff'] - median))]
        return [_measurement(group, c) for c in kept]


class TopKPolicy(SelectionPolicy):
    """Feed the ``k`` candidates closest to the group median distanceDiff.

    Another middle ground between 'all' and 'median': a fixed-size, cheap
    (bounded sort) subset centered on the group consensus.
    """
    name = 'top_k'

    def __init__(self, k=3):
        self.k = k

    def select(self, group):
        cs = group['candidates']
        if not cs:
            return []
        vals = sorted(c['distanceDiff'] for c in cs)
        median = vals[len(vals) // 2]
        ranked = sorted(cs, key=lambda c: abs(c['distanceDiff'] - median))
        kept = ranked[:self.k]
        return [_measurement(group, c) for c in kept]


class RoundRobinPolicy(SelectionPolicy):
    """Pick one candidate per packet, rotating through the group indices.

    Mimics the spirit of the firmware's rotating ("Random") matcher, but replayed
    deterministically so results are reproducible.
    """
    name = 'round_robin'

    def __init__(self):
        self._k = 0

    def reset(self):
        self._k = 0

    def select(self, group):
        cs = group['candidates']
        if not cs:
            return []
        c = cs[self._k % len(cs)]
        self._k += 1
        return [_measurement(group, c)]


class FreshnessPolicy(SelectionPolicy):
    """Drop candidates whose remote-anchor data is stale, then delegate.

    This is a *wrapper*, not a selector: it filters the group and hands the
    survivors to an inner policy, so it composes with any of the others.
    ``fresh(baseline)`` isolates the effect of dropping stale candidates;
    ``fresh(median)`` additionally lets the packet fall back to a fresh
    alternative it would otherwise have wasted.

    Requires ``annotate_remote_age`` to have been run on the groups -- it
    raises rather than silently passing everything through.
    """
    name = 'fresh'

    def __init__(self, inner='baseline', max_age_ms=DEFAULT_MAX_REMOTE_AGE_MS,
                 inner_params=None):
        self.max_age_ms = float(max_age_ms)
        self.inner = (make_policy(inner, inner_params)
                      if isinstance(inner, str) else inner)

    @property
    def requires(self):
        return ('age_ms',) + tuple(self.inner.requires)

    def reset(self):
        self.inner.reset()

    def select(self, group):
        candidates = group['candidates']
        if candidates and 'age_ms' not in candidates[0]:
            raise ValueError(
                "FreshnessPolicy needs candidate 'age_ms'; call "
                "tdoa_selection.annotate_remote_age(groups) before replaying")
        fresh = [c for c in candidates if c['age_ms'] <= self.max_age_ms]
        if not fresh:
            return []
        return self.inner.select({**group, 'candidates': fresh})


class GeometryPolicy(SelectionPolicy):
    """Drop candidates with bad pair geometry, then delegate (firmware PR #1650).

    The A/B counterpart of ``matchRandomAnchor``'s geometry filter: a candidate
    is rejected when ``|distanceDiff| / |A1 - A2| >= limit``, i.e. when the tag
    sits close to the line through the two anchors and the pair is nearly blind
    to its position.

    Like ``FreshnessPolicy`` this is a *wrapper*, not a selector, and the choice
    of inner policy decides which half of PR #1650 is being measured:

    * ``geometry(baseline)`` -- keep the pair the live firmware actually chose,
      unless its geometry is bad, in which case the packet is dropped. This
      isolates the *cost* of the filter: how much data #1650 throws away, and
      whether the discarded measurements were hurting.
    * ``geometry(round_robin)`` -- the closer analogue of the merged firmware,
      which rejects a bad candidate and ``continue``s to the next one in the
      rotated candidate list, so a packet is only lost when *every* candidate
      fails. This measures the filter as shipped.

    Note that the firmware applies the gate in the Random matcher only; the
    Youngest matcher is untouched by #1650.

    Requires ``annotate_pair_geometry`` to have been run on the groups -- it
    raises rather than silently passing everything through.
    """
    name = 'geometry'

    def __init__(self, inner='baseline', limit=DEFAULT_DISTANCE_RATIO_LIMIT,
                 inner_params=None):
        self.limit = float(limit)
        self.inner = (make_policy(inner, inner_params)
                      if isinstance(inner, str) else inner)

    @property
    def requires(self):
        return ('distance_ratio',) + tuple(self.inner.requires)

    def reset(self):
        self.inner.reset()

    def select(self, group):
        candidates = group['candidates']
        if candidates and 'distance_ratio' not in candidates[0]:
            raise ValueError(
                "GeometryPolicy needs candidate 'distance_ratio'; call "
                "tdoa_selection.annotate_pair_geometry(groups, anchor_positions) "
                "before replaying")
        good = [c for c in candidates if c['distance_ratio'] < self.limit]
        if not good:
            return []
        return self.inner.select({**group, 'candidates': good})


class OraclePolicy(SelectionPolicy):
    """Upper bound on selection: feed the candidate that ground truth says is best.

    Not implementable on a drone -- it reads the answer. Its job is to bound
    what *any* selection policy could achieve on a given log, so that a real
    policy can be judged against the headroom that actually exists rather than
    against the baseline alone. If the oracle barely beats ``baseline``, the
    per-packet choice is not what limits the estimate and the effort belongs
    elsewhere (measurement noise scaling, the outlier filter, anchor survey).

    ``k=1`` (the default) keeps the one-measurement-per-packet shape of the
    firmware, so the comparison against ``baseline`` isolates *which* candidate
    is chosen from *how many* are used. ``k=None`` feeds every usable candidate
    in oracle order, and ``max_error_m`` additionally drops candidates worse
    than a threshold -- ``OraclePolicy(k=None, max_error_m=0.3)`` is then a
    perfect outlier filter rather than a selector, which bounds a different
    thing.

    Candidates that could not be scored (``oracle_error`` is infinite: unknown
    anchor, or no ground truth near this packet) are never selected. When a
    packet has no scorable candidate at all the policy defers to ``fallback``
    -- ``baseline`` by default, so the stretches a Lighthouse log does not
    cover replay exactly as the firmware flew them instead of silently
    dropping out and making the trajectories incomparable. Pass
    ``fallback=None`` to drop those packets instead.

    Requires ``annotate_oracle_error`` to have been run on the groups.
    """
    name = 'oracle'

    def __init__(self, k=1, max_error_m=None, fallback='baseline',
                 fallback_params=None):
        self.k = None if k is None else int(k)
        self.max_error_m = None if max_error_m is None else float(max_error_m)
        if fallback is None:
            self.fallback = None
        else:
            self.fallback = (make_policy(fallback, fallback_params)
                             if isinstance(fallback, str) else fallback)

    @property
    def requires(self):
        inner = () if self.fallback is None else tuple(self.fallback.requires)
        return ('oracle_error',) + inner

    def reset(self):
        if self.fallback is not None:
            self.fallback.reset()

    def select(self, group):
        candidates = group['candidates']
        if candidates and 'oracle_error' not in candidates[0]:
            raise ValueError(
                "OraclePolicy needs candidate 'oracle_error'; call "
                "tdoa_selection.annotate_oracle_error(groups, anchor_positions, "
                "truth) before replaying")
        usable = [c for c in candidates if c['oracle_error'] != float('inf')]
        if not usable:
            return [] if self.fallback is None else self.fallback.select(group)
        usable.sort(key=lambda c: c['oracle_error'])
        if self.max_error_m is not None:
            usable = [c for c in usable if c['oracle_error'] <= self.max_error_m]
            if not usable:
                return []
        if self.k is not None:
            usable = usable[:self.k]
        return [_measurement(group, c) for c in usable]


# Factories (not instances) so each replay run gets fresh policy state.
POLICY_FACTORIES = {
    'baseline': BaselinePolicy,
    'all': AllCandidatesPolicy,
    'median': MedianPolicy,
    'round_robin': RoundRobinPolicy,
    'trimmed_all': TrimmedAllPolicy,
    'top_k': TopKPolicy,
    'fresh': FreshnessPolicy,
    'geometry': GeometryPolicy,
    'oracle': OraclePolicy,
}


def make_policy(name, params=None):
    try:
        factory = POLICY_FACTORIES[name]
    except KeyError:
        raise ValueError(
            f"Unknown selection policy '{name}'. "
            f"Available: {', '.join(sorted(POLICY_FACTORIES))}")
    return factory(**(params or {}))


def verify_baseline_reconstruction(log_data):
    """F1 measurement-level check: baseline reconstruction covers logged estTDOA.

    The candidate flagged ``isSelected`` and the ``estTDOA`` event of the same
    packet both originate from the same ``calcDistanceDiff`` double in the
    firmware and are stored as float32, so a matching pair must still be
    *exactly* equal.

    The two streams are not expected to be a strict 1:1 zip, though: candidate
    logging is gated only on clock correction, while the live estimator
    measurement additionally requires both anchor positions to be valid (and
    the loco deck enabled). The baseline (selected-candidate) stream is
    therefore a superset of the ``estTDOA`` stream -- every ``estTDOA`` entry
    must appear among the selected candidates, in order, but a selected
    candidate lacking a matching ``estTDOA`` entry is expected
    (position-gated) and not a fault.

    This matches each ``estTDOA`` entry against the baseline stream as an
    in-order subsequence (estTDOA is the reference). A healthy, drop-free log
    has ``n_unmatched_est == 0``; ``n_unmatched_baseline > 0`` is normal on
    real logs. ``first_unmatched_est`` (index into the estTDOA stream, or
    None if everything matched) is the entry point for debugging a genuine
    capture/reconstruction fault.

    Verify losslessness (the usd.eventsRequested / usd.eventsAccepted check)
    first -- this function does not account for dropped uSD events.
    """
    groups = build_candidate_groups(log_data)
    policy = BaselinePolicy()
    baseline = []
    n_selectionless = 0
    for group in groups:
        measurements = policy.select(group)
        if measurements:
            baseline.extend(measurements)
        else:
            n_selectionless += 1

    est = log_data.get('estTDOA') or {}
    n_est_tdoa = len(est.get('timestamp', []))

    # The baseline stream is a superset of the estTDOA stream: the firmware
    # additionally gates estimator measurements on valid anchor positions, so
    # selected candidates without a matching estTDOA entry are expected
    # (position-gated). Match estTDOA entries against the baseline stream in
    # order; an estTDOA entry that cannot be matched indicates a real
    # capture/reconstruction fault.
    bi = 0
    n_matched = 0
    first_unmatched_est = None
    for i in range(n_est_tdoa):
        target = (int(est['idA'][i]), int(est['idB'][i]), float(est['distanceDiff'][i]))
        j = bi
        while j < len(baseline) and (
                (baseline[j]['idA'], baseline[j]['idB'], baseline[j]['distanceDiff']) != target):
            j += 1
        if j < len(baseline):
            n_matched += 1
            bi = j + 1
        elif first_unmatched_est is None:
            first_unmatched_est = i

    return {
        'n_groups': len(groups),
        'n_selectionless': n_selectionless,
        'n_baseline': len(baseline),
        'n_est_tdoa': n_est_tdoa,
        'n_matched': n_matched,
        'n_unmatched_est': n_est_tdoa - n_matched,
        'n_unmatched_baseline': len(baseline) - n_matched,
        'first_unmatched_est': first_unmatched_est,
    }
