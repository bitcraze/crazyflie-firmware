"""
Replay seam: uSD log -> sample streams -> the real firmware Kalman core.

This is the stable, documented interface that analysis scripts build on::

    log_data = cfusdlog.decode(path)
    groups   = tdoa_selection.build_candidate_groups(log_data)
    imu      = tdoa_replay.extract_imu_samples(log_data)
    tdoa     = tdoa_replay.apply_policy(policy, groups)
    traj     = tdoa_replay.replay(anchor_positions, imu, tdoa,
                                  {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})

``replay`` imports cffirmware lazily, so everything else in this module can be
used (and unit tested) without the SWIG bindings being built.

Sample format (shared with EstimatorKalmanEmulator): ``(log_type, sample_dict)``
where ``sample_dict`` always carries a ``'timestamp'`` in milliseconds.
"""

import math

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


def extract_sweep_samples(log_data):
    """Return Lighthouse sweep-angle samples in (log_type, sample_dict) form.

    Reads the ``estSweepAngle`` event block written by estimator.c when the
    Lighthouse deck feeds the estimator (estimationMethod 1, i.e. a build
    without DECK_LIGHTHOUSE_AS_GROUNDTRUTH). Empty on a ground-truth build,
    where the deck pushes a pre-computed crossing-beam position instead.
    """
    data = log_data.get('estSweepAngle')
    if not data or 'timestamp' not in data:
        return []
    return [
        ('estSweepAngle', {name: values[i] for name, values in data.items()})
        for i in range(len(data['timestamp']))
    ]


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


DEFAULT_INIT_WINDOW_MS = 2000.0


def seed_initial_position(log_data, window_ms=DEFAULT_INIT_WINDOW_MS):
    """Kalman initial position taken from the live estimate at the log start.

    ``kalmanCoreInit`` seeds the state from ``initialX/Y/Z``, which default to
    the origin. That is fine when the anchor frame is centred on the takeoff
    spot, but in a frame whose origin is metres away (a show grid, say) the
    replay starts far from where the drone actually was, and the TDoA
    measurement Jacobians spend the first seconds linearized about the wrong
    point. Seeding from the drone's own estimate removes that transient.

    The median over ``window_ms`` is used rather than the single first sample:
    while the drone is still on the ground the live estimate jitters by a few
    centimetres, and the median is immune to one bad sample at the log start.
    ``window_ms=0`` selects the first sample alone.

    Returns a dict suitable for merging into the ``kalman_params`` replay
    parameter, or None when the log has no ``stateEstimate`` block (the lean
    capture config drops it) -- in which case the caller keeps the default
    origin init.
    """
    live = extract_state_estimate(log_data)
    if not live:
        return None

    t_start = live[0][0]
    window = [pos for t_ms, pos in live if t_ms - t_start <= window_ms]
    if not window:
        window = [live[0][1]]

    return {
        'initialX': _median(sorted(p[0] for p in window)),
        'initialY': _median(sorted(p[1] for p in window)),
        'initialZ': _median(sorted(p[2] for p in window)),
    }


def _median(sorted_values):
    """Median of an already sorted sequence (mean of the middle two if even)."""
    n = len(sorted_values)
    mid = n // 2
    if n % 2:
        return sorted_values[mid]
    return 0.5 * (sorted_values[mid - 1] + sorted_values[mid])


def apply_policy(policy, groups):
    """Map candidate groups to 'estTDOA' samples using a selection policy.

    The policy is reset first, so one policy instance per replay run.
    All measurements of a group get the group's packet time (min event
    timestamp, see build_candidate_groups).
    """
    policy.reset()
    samples = []
    for group in groups:
        selected = policy.select(group)
        if not selected:
            continue
        # Per-packet candidate statistics, available to std models / filters.
        # Causal and firmware-implementable: the firmware holds the same
        # candidate set when it processes the packet.
        diffs = sorted(c['distanceDiff'] for c in group['candidates'])
        n = len(diffs)
        median = diffs[n // 2]
        spread = sorted(abs(d - median) for d in diffs)[n // 2]  # MAD
        for m in selected:
            samples.append(('estTDOA', {
                'idA': m['idA'],
                'idB': m['idB'],
                'distanceDiff': m['distanceDiff'],
                'timestamp': group['t_ms'],
                'n_cand': n,
                'group_spread': spread,
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


def replay(anchor_positions, imu_samples, tdoa_samples, params=None,
           sweep_samples=None):
    """Run samples through the real firmware Kalman core.

    Args:
        anchor_positions: dict anchor id -> cffirmware.vec3_s (see
            loco_utils.read_loco_anchor_positions). May be None/empty for a
            Lighthouse-only replay.
        imu_samples: from extract_imu_samples.
        tdoa_samples: from apply_policy (or synthetic, same shape).
        sweep_samples: optional, from extract_sweep_samples. Merged into the
            same stream, so the Kalman core sees Lighthouse and TDoA updates
            interleaved on the logged clock -- which is the point when
            replaying a LH+TDoA -> TDoA-only handover.
        params: optional dict of tuning parameters. Supported:
            'tdoa_std' (float, default 0.15): TDoA measurement std dev [m].
            'tdoa_model' (str, default 'standard'): TDoA measurement model.
                'standard' is kalmanCoreUpdateWithTdoa; 'robust' is the
                M-estimation kalmanCoreRobustUpdateWithTdoa, which bypasses
                the outlier filter (firmware behavior with kalman.robustTdoa=1).
                Caveat: mm_tdoa_robust.c keeps M-estimation state in a static
                buffer (x_err), shared process-wide, so back-to-back 'robust'
                replays in one process are not strictly independent; run one
                robust replay per process for exact reproducibility.
            'outlier_filter' (str, optional): outlier filter name (see
                bindings/util/tdoa_outlier.py). Absent -> the firmware's
                built-in behavior (standard model: C integrator filter;
                robust model: ungated).
            'outlier_filter_params' (dict, optional): tuning-constant
                overrides for the outlier filter (e.g. {'K': 4.0}).
            'std_model' (str, optional): per-update measurement-noise model
                (see bindings/util/tdoa_std.py). Absent -> constant tdoa_std.
            'std_model_params' (dict, optional): tuning-constant overrides
                for the std model.
            'kalman_params' (dict, optional): kalmanCoreParams_t field
                overrides applied on top of kalmanCoreDefaultParams before
                kalmanCoreInit, e.g. the initialX/Y/Z from
                seed_initial_position, or {'procNoiseVel': 0.3}.
            'basestation_poses', 'basestation_calibration' (dict, optional):
                Lighthouse geometry and calibration, both as returned by
                lighthouse_utils.load_lighthouse_calibration(). Required when
                sweep_samples is non-empty.
            'sweep_std' (float, optional): Lighthouse sweep-angle measurement
                std dev [rad]; default is the firmware's lighthouse.sweepStd2.

    Returns:
        [(t_ms, (x, y, z))] trajectory, one entry per 1 kHz iteration.
    """
    from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator

    params = params or {}
    outlier_filter = None
    filter_name = params.get('outlier_filter')
    if filter_name is not None:
        from bindings.util.tdoa_outlier import make_outlier_filter
        outlier_filter = make_outlier_filter(
            filter_name, params.get('outlier_filter_params'))

    std_model = None
    std_model_name = params.get('std_model')
    if std_model_name is not None:
        from bindings.util.tdoa_std import make_std_model
        std_model = make_std_model(std_model_name,
                                   params.get('std_model_params'))

    emulator_kwargs = {}
    for key in ('basestation_poses', 'basestation_calibration', 'sweep_std'):
        if params.get(key) is not None:
            emulator_kwargs[key] = params[key]

    emulator = EstimatorKalmanEmulator(
        anchor_positions, tdoa_model=params.get('tdoa_model', 'standard'),
        outlier_filter=outlier_filter, std_model=std_model,
        kalman_params=params.get('kalman_params'), **emulator_kwargs)
    emulator.TDOA_ENGINE_MEASUREMENT_NOISE_STD = params.get('tdoa_std', DEFAULT_TDOA_STD)

    n_before = len(tdoa_samples)
    known = anchor_positions or {}
    missing = sorted({i for _, m in tdoa_samples
                      for i in (m['idA'], m['idB']) if i not in known})
    tdoa_samples, n_skipped = filter_known_anchors(tdoa_samples, known)
    if n_skipped:
        print(f'WARNING: skipped {n_skipped}/{n_before} TDoA samples '
              f'({n_skipped / n_before * 100.0:.1f}%) referencing anchor id(s) '
              f'{missing}, which are not in the anchors file. The drone ranged '
              f'against them, so the replay is working from less data than the '
              f'live estimate did.')
    samples = merge_samples(imu_samples, tdoa_samples, sweep_samples or [])
    if not samples:
        return []

    trajectory = []
    while len(samples):
        now_ms, state = emulator.run_one_1khz_iteration(samples)
        pos = (state.position.x, state.position.y, state.position.z)
        if not all(math.isfinite(v) for v in pos):
            print(f'ERROR: the Kalman state went non-finite at '
                  f'{now_ms / 1000.0:.1f} s; truncating the replay after '
                  f'{len(trajectory)} iterations. The filter diverged -- the '
                  f'trajectory up to here is still valid, everything after it '
                  f'would have been NaN.')
            break
        trajectory.append((now_ms, pos))
    return trajectory
