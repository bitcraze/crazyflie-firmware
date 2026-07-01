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


def _measurement(group, candidate):
    return {
        'idA': group['idA'],
        'idB': candidate['idB'],
        'distanceDiff': candidate['distanceDiff'],
    }


class SelectionPolicy:
    """Base class. ``select`` returns a list of measurement dicts."""
    name = 'base'

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


# Factories (not instances) so each replay run gets fresh policy state.
POLICY_FACTORIES = {
    'baseline': BaselinePolicy,
    'all': AllCandidatesPolicy,
    'median': MedianPolicy,
    'round_robin': RoundRobinPolicy,
}


def make_policy(name):
    try:
        return POLICY_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown selection policy '{name}'. "
            f"Available: {', '.join(sorted(POLICY_FACTORIES))}")
