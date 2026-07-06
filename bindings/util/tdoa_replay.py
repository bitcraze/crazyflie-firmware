"""
Replay seam: uSD log -> sample streams -> the real firmware Kalman core.

This is the stable, documented interface that analysis scripts build on::

    log_data = cfusdlog.decode(path)
    groups   = tdoa_selection.build_candidate_groups(log_data)
    imu      = tdoa_replay.extract_imu_samples(log_data)
    tdoa     = tdoa_replay.apply_policy(policy, groups)
    traj     = tdoa_replay.replay(anchor_positions, imu, tdoa,
                                  {'tdoa_std': 0.15})

``replay`` imports cffirmware lazily, so everything else in this module can be
used (and unit tested) without the SWIG bindings being built.

Sample format (shared with EstimatorKalmanEmulator): ``(log_type, sample_dict)``
where ``sample_dict`` always carries a ``'timestamp'`` in milliseconds.
"""

DEFAULT_TDOA_STD = 0.15


def extract_imu_samples(log_data):
    """Return IMU samples in the (log_type, sample_dict) form the emulator wants."""
    samples = []
    for log_type in ('estAcceleration', 'estGyroscope'):
        data = log_data.get(log_type)
        if not data or 'timestamp' not in data:
            continue
        n = len(data['timestamp'])
        for i in range(n):
            sample = {name: values[i] for name, values in data.items()}
            samples.append((log_type, sample))
    return samples


def extract_state_estimate(log_data):
    """The live on-drone estimate [(t_ms, (x, y, z))] from the fixedFrequency block.

    Used to quantify replay-vs-live divergence (spec F1). Requires
    stateEstimate.x/y/z in the uSD fixedFrequency config.
    """
    ff = log_data.get('fixedFrequency')
    if not ff or 'timestamp' not in ff:
        return []
    needed = ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z')
    if not all(k in ff for k in needed):
        return []
    return [
        (float(ff['timestamp'][i]),
         (float(ff['stateEstimate.x'][i]),
          float(ff['stateEstimate.y'][i]),
          float(ff['stateEstimate.z'][i])))
        for i in range(len(ff['timestamp']))
    ]


def apply_policy(policy, groups):
    """Map candidate groups to 'estTDOA' samples using a selection policy.

    The policy is reset first, so one policy instance per replay run.
    All measurements of a group get the group's packet time (min event
    timestamp, see build_candidate_groups).
    """
    policy.reset()
    samples = []
    for group in groups:
        for m in policy.select(group):
            samples.append(('estTDOA', {
                'idA': m['idA'],
                'idB': m['idB'],
                'distanceDiff': m['distanceDiff'],
                'timestamp': group['t_ms'],
            }))
    return samples


def filter_known_anchors(tdoa_samples, anchor_positions):
    """Split TDoA samples into (kept, n_skipped) by anchor-position availability.

    The live firmware only feeds the estimator measurements whose anchor
    positions are known and valid; replay mirrors that by skipping samples
    that reference an anchor id missing from anchors.yaml, instead of
    crashing on it.
    """
    kept = [s for s in tdoa_samples
            if s[1]['idA'] in anchor_positions and s[1]['idB'] in anchor_positions]
    return kept, len(tdoa_samples) - len(kept)


def merge_samples(*sample_lists):
    """Merge sample streams into one, ordered by timestamp.

    The sort is stable: samples with equal timestamps keep the relative order
    of their input streams (spec F2).
    """
    merged = [s for lst in sample_lists for s in lst]
    merged.sort(key=lambda s: s[1]['timestamp'])
    return merged


def replay(anchor_positions, imu_samples, tdoa_samples, params=None):
    """Run samples through the real firmware Kalman core.

    Args:
        anchor_positions: dict anchor id -> cffirmware.vec3_s (see
            loco_utils.read_loco_anchor_positions).
        imu_samples: from extract_imu_samples.
        tdoa_samples: from apply_policy (or synthetic, same shape).
        params: optional dict of tuning parameters. Supported:
            'tdoa_std' (float, default 0.15): TDoA measurement std dev [m].

    Returns:
        [(t_ms, (x, y, z))] trajectory, one entry per 1 kHz iteration.
    """
    from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator

    params = params or {}
    tdoa_samples, n_skipped = filter_known_anchors(tdoa_samples, anchor_positions)
    if n_skipped:
        print(f'WARNING: skipped {n_skipped} TDoA samples referencing anchors '
              f'not in the anchors file')
    samples = merge_samples(imu_samples, tdoa_samples)
    if not samples:
        return []

    emulator = EstimatorKalmanEmulator(anchor_positions)
    emulator.TDOA_ENGINE_MEASUREMENT_NOISE_STD = params.get('tdoa_std', DEFAULT_TDOA_STD)

    trajectory = []
    while len(samples):
        now_ms, state = emulator.run_one_1khz_iteration(samples)
        trajectory.append((now_ms, (state.position.x, state.position.y, state.position.z)))
    return trajectory
