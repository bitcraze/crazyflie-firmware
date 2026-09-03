"""
Plot x/y/z trajectories from a Crazyflie uSD log: the replayed estimate from the
real firmware Kalman core (cffirmware bindings), the live on-drone estimate
(stateEstimate), and the Lighthouse crossing-beam ground truth.

Run from this folder with uv (creates the venv and installs matplotlib etc.)::

    uv run plot_tdoa.py ../../run_validation.bin --anchors ../../anchors.yaml

or from the repository root::

    uv run --project tools/tdoa_plot tools/tdoa_plot/plot_tdoa.py \
        run_validation.bin --anchors anchors.yaml

If the cffirmware SWIG bindings are not importable for the running interpreter,
they are built automatically via ``make bindings_python`` (needs swig and a C
compiler).
"""

import argparse
import importlib
import math
import statistics
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
# Repo modules (tools.usdlog, bindings.util) and the bindings build output.
sys.path.insert(0, str(REPO_ROOT / 'build'))
sys.path.insert(0, str(REPO_ROOT))


def ensure_bindings(rebuild=False):
    """Import cffirmware, building it with make bindings_python if needed.

    The build is done with the *running* interpreter (PYTHON=sys.executable) so
    the extension matches the venv uv created, not whatever python3 is on PATH.
    """
    if not rebuild:
        try:
            import cffirmware  # noqa: F401
            return
        except ImportError as e:
            print(f'cffirmware bindings not importable ({e}); '
                  f'building with "make bindings_python" ...')
    subprocess.run(
        ['make', 'bindings_python', f'PYTHON={sys.executable}'],
        cwd=REPO_ROOT, check=True)
    importlib.invalidate_caches()
    import cffirmware  # noqa: F401



def load_series(args):
    """Decode the log and return (replayed, live, ground_truth) trajectories.

    Each is a list of (t_ms, (x, y, z)); replayed is None when the log has no
    candidate data to replay.
    """
    import tools.usdlog.cfusdlog as cfusdlog
    from bindings.util.loco_utils import read_loco_anchor_positions
    from bindings.util.tdoa_replay import (
        apply_policy, extract_imu_samples, extract_state_estimate, replay,
        seed_initial_position)
    from bindings.util.tdoa_selection import (
        FreshnessPolicy, GeometryPolicy, annotate_oracle_error,
        annotate_pair_geometry, annotate_remote_age, build_candidate_groups,
        make_policy, remote_age_blind_fraction)
    from tools.usdlog.replay_tdoa import extract_ground_truth

    log_data = cfusdlog.decode(args.logfile)
    live = extract_state_estimate(log_data)
    gt = extract_ground_truth(log_data)
    if args.align_truth != 'none' and gt and live:
        from bindings.util.tdoa_align import fit_alignment
        reference = {'live': live}[args.align_truth]
        alignment = fit_alignment(gt, reference, model=args.align_model)
        if alignment is None:
            print('Frame alignment: too few paired samples, skipping')
        else:
            gt = alignment.apply_trajectory(gt)
            print(f'Frame alignment: mapped Lighthouse into the Loco frame '
                  f'with a {args.align_model} fit against the '
                  f'{args.align_truth} estimate')
            print(alignment.describe())

    groups = build_candidate_groups(log_data)
    if not groups:
        print('No estTdoaCand data found; plotting live estimate and '
              'ground truth only.')
        return None, live, gt

    anchor_positions = read_loco_anchor_positions(args.anchors)
    imu_samples = extract_imu_samples(log_data)
    policy_params = parse_params(args.policy_param)
    filter_params = parse_params(args.filter_param)
    policy = make_policy(args.selection_policy, policy_params)
    if 'oracle_error' in policy.requires:
        if args.oracle_reference == 'live':
            # Scoring against the drone's OWN estimate instead of ground truth
            # makes this a realizable policy rather than a bound: the firmware
            # can compute each candidate's innovation before choosing, since it
            # is two distances and a subtraction, no Kalman update needed. It
            # approximates in-loop selection with a previous pass's trajectory,
            # so it does not reproduce the self-confirming feedback that a true
            # in-loop version would have -- treat it as optimistic until that
            # is built.
            if not live:
                raise SystemExit('--oracle-reference live needs stateEstimate '
                                 'in the log.')
            reference, ref_name = live, "the drone's own live estimate"
        else:
            reference, ref_name = gt, 'Lighthouse ground truth'
        if not reference:
            raise SystemExit(
                'The oracle selection policy scores candidates against ground '
                'truth, and this log has no Lighthouse data (lighthouse.x/y/z '
                'in the uSD fixedFrequency config, with base stations visible). '
                'Without it there is nothing to be an oracle about.')
        annotate_oracle_error(groups, anchor_positions, reference,
                              max_gap_ms=args.oracle_max_gap_ms,
                              score=args.oracle_score)
        n_cand = sum(len(g['candidates']) for g in groups)
        n_scored = sum(1 for g in groups for c in g['candidates']
                       if c['oracle_error'] != float('inf'))
        n_packets = sum(1 for g in groups
                        if any(c['oracle_error'] != float('inf')
                               for c in g['candidates']))
        print(f'Oracle: scoring candidates against {len(reference)} samples of '
              f'{ref_name} ({args.oracle_score} error, gaps wider than '
              f'{args.oracle_max_gap_ms:.0f} ms left unscored)')
        print(f'  scored {n_scored}/{n_cand} candidates '
              f'({n_scored / max(n_cand, 1) * 100:.1f}%); '
              f'{n_packets}/{len(groups)} packets '
              f'({n_packets / max(len(groups), 1) * 100:.1f}%) have a scorable '
              f'candidate, the rest fall back to the flown selection')
    if args.distance_ratio_limit is not None:
        annotate_pair_geometry(groups, anchor_positions)
        policy = GeometryPolicy(inner=policy, limit=args.distance_ratio_limit)
        n_cand = sum(len(g['candidates']) for g in groups)
        n_good = sum(1 for g in groups for c in g['candidates']
                     if c['distance_ratio'] < args.distance_ratio_limit)
        n_packets_kept = sum(1 for g in groups
                             if any(c['distance_ratio'] < args.distance_ratio_limit
                                    for c in g['candidates']))
        print(f'Geometry gate (firmware PR #1650): rejecting candidates whose '
              f'|distanceDiff| is at or above {args.distance_ratio_limit:.2f} '
              f'x the anchor-pair separation')
        print(f'  kept {n_good}/{n_cand} candidates '
              f'({n_good / max(n_cand, 1) * 100:.1f}%); '
              f'{n_packets_kept}/{len(groups)} packets '
              f'({n_packets_kept / max(len(groups), 1) * 100:.1f}%) still have '
              f'a well-conditioned candidate')
    if args.max_remote_age_ms is not None:
        annotate_remote_age(groups)
        policy = FreshnessPolicy(inner=policy, max_age_ms=args.max_remote_age_ms)
        blind = remote_age_blind_fraction(log_data)
        print(f'Freshness gate: dropping candidates whose remote anchor was '
              f'last heard more than {args.max_remote_age_ms:.0f} ms ago'
              + (f' (age is a proxy, blind to {blind * 100:.2f}% of packets, '
                 f'so it only over-estimates)' if blind is not None else ''))
    tdoa_samples = apply_policy(policy, groups)
    if args.max_remote_age_ms is not None:
        n_cand = sum(len(g['candidates']) for g in groups)
        n_fresh = sum(1 for g in groups for c in g['candidates']
                      if c['age_ms'] <= args.max_remote_age_ms)
        n_packets_kept = sum(1 for g in groups
                             if any(c['age_ms'] <= args.max_remote_age_ms
                                    for c in g['candidates']))
        print(f'  kept {n_fresh}/{n_cand} candidates ({n_fresh / max(n_cand, 1) * 100:.1f}%); '
              f'{n_packets_kept}/{len(groups)} packets '
              f'({n_packets_kept / max(len(groups), 1) * 100:.1f}%) still have a fresh candidate')
    print(f'Replaying {len(tdoa_samples)} TDoA + {len(imu_samples)} IMU samples '
          f'through the firmware Kalman core '
          f'(selection policy: {args.selection_policy}'
          + (f' {policy_params}' if policy_params else '')
          + f', measurement model: {args.tdoa_model}, '
          f'outlier filter: {args.outlier_filter}'
          + (f' {filter_params}' if filter_params else '') + ') ...')
    kalman_params = resolve_initial_position(args, log_data,
                                             seed_initial_position)
    replayed = replay(anchor_positions, imu_samples, tdoa_samples,
                      {'tdoa_std': args.tdoa_std,
                       'tdoa_model': args.tdoa_model,
                       'outlier_filter': args.outlier_filter,
                       'outlier_filter_params': filter_params,
                       'kalman_params': kalman_params})
    return replayed, live, gt


def resolve_initial_position(args, log_data, seed_initial_position):
    """Turn --init-from/--init-window-ms into kalmanCoreParams_t overrides.

    Returns None (default origin init) for --init-from origin, or when the log
    has no stateEstimate to seed from.
    """
    if args.init_from == 'origin':
        return None
    seed = seed_initial_position(log_data, window_ms=args.init_window_ms)
    if seed is None:
        print('WARNING: --init-from log, but this log has no stateEstimate.* '
              '(lean capture config?); replay starts at the origin.')
        return None
    window = ('first sample' if args.init_window_ms <= 0
              else f'median of first {args.init_window_ms:.0f} ms')
    print(f"Seeding the replay at ({seed['initialX']:+.3f}, "
          f"{seed['initialY']:+.3f}, {seed['initialZ']:+.3f}) m "
          f'from the live estimate ({window}).')
    return seed


def parse_params(pairs):
    """Parse repeated KEY=VALUE args into a dict, as int where possible.

    Tuning constants are numeric; sizes like WINDOW must stay int (they end
    up as a deque maxlen). ``KEY=none`` yields None, which several policies
    take as "no limit" -- OraclePolicy's k=none feeds every candidate, and its
    fallback=none drops unscorable packets instead of replaying the flown pair.
    """
    params = {}
    for pair in pairs or []:
        key, sep, value = pair.partition('=')
        if not sep:
            sys.exit(f"--*-param expects KEY=VALUE, got '{pair}'")
        if value.lower() == 'none':
            params[key] = None
            continue
        try:
            params[key] = int(value)
        except ValueError:
            try:
                params[key] = float(value)
            except ValueError:
                # Not every tuning constant is numeric: FreshnessPolicy takes
                # inner=<policy name>, for instance.
                params[key] = value
    return params


def print_stats(replayed, live, gt):
    """Print accuracy stats of each estimate against the lighthouse ground truth.

    Errors are estimate - ground truth, with the ground truth linearly
    interpolated at each estimate timestamp (samples outside the ground-truth
    time range are skipped).
    """
    from tools.usdlog.replay_tdoa import _interp_ground_truth

    if not gt:
        print('No Lighthouse ground truth in this log; skipping accuracy stats.')
        return

    for name, traj in [('live estimate (onboard)', live),
                       ('bindings estimate (replay)', replayed)]:
        if not traj:
            continue
        per_axis = ([], [], [])   # signed errors per axis [m]
        err_3d = []               # Euclidean error [m]
        n_non_finite = 0
        first_non_finite_ms = None
        for t_ms, pos in traj:
            if not all(math.isfinite(c) for c in pos):
                n_non_finite += 1
                if first_non_finite_ms is None:
                    first_non_finite_ms = t_ms
                continue
            gt_pos = _interp_ground_truth(gt, t_ms)
            if gt_pos is None:
                continue
            e = [pos[k] - gt_pos[k] for k in range(3)]
            for k in range(3):
                per_axis[k].append(e[k])
            err_3d.append(math.sqrt(sum(c * c for c in e)))

        print(f'--- accuracy vs ground truth: {name} ---')
        if n_non_finite:
            print(f'  WARNING: estimator diverged, {n_non_finite} non-finite '
                  f'samples excluded (first at t={first_non_finite_ms / 1000.0:.1f} s)')
        if not err_3d:
            print('  no samples overlap the ground-truth time range')
            continue
        print(f'  samples: {len(err_3d)} of {len(traj)} '
              f'(within ground-truth time range)')
        for k, axis in enumerate('xyz'):
            errs = per_axis[k]
            bias = statistics.mean(errs)
            std = statistics.pstdev(errs)
            rms = math.sqrt(sum(e * e for e in errs) / len(errs))
            print(f'  {axis}:  bias {bias:+.3f} m   std {std:.3f} m   '
                  f'var {std * std:.4f} m2   rms {rms:.3f} m')
        vals = sorted(err_3d)
        n = len(vals)
        mean = statistics.mean(vals)
        std = statistics.pstdev(vals)
        rms = math.sqrt(sum(e * e for e in vals) / n)
        p95 = vals[min(n - 1, int(0.95 * n))]
        print(f'  3D: mean {mean:.3f} m   std {std:.3f} m   '
              f'var {std * std:.4f} m2   rms {rms:.3f} m')
        print(f'      median {vals[n // 2]:.3f} m   p95 {p95:.3f} m   '
              f'max {vals[-1]:.3f} m')


def plot(replayed, live, gt, title, save_path=None):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    series = [
        (gt, 'ground truth (lighthouse)', dict(color='black', alpha=0.5, lw=1.0)),
        (live, 'live estimate (onboard)', dict(color='tab:orange', lw=1.0)),
        (replayed, 'bindings estimate (replay)', dict(color='tab:blue', lw=1.0)),
    ]
    for dim, (ax, label) in enumerate(zip(axes, 'xyz')):
        for traj, name, style in series:
            if not traj:
                continue
            t = [s[0] / 1000.0 for s in traj]
            v = [s[1][dim] for s in traj]
            ax.plot(t, v, label=name, **style)
        ax.set_ylabel(f'{label} [m]')
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    axes[0].set_title(title)
    axes[-1].set_xlabel('time [s]')
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')
    else:
        plt.show()


def main():
    # Safe to import eagerly: tdoa_replay only imports cffirmware inside
    # replay(), so this does not front-run ensure_bindings().
    from bindings.util.tdoa_replay import DEFAULT_INIT_WINDOW_MS
    from bindings.util.tdoa_selection import (
        DEFAULT_DISTANCE_RATIO_LIMIT, DEFAULT_MAX_REMOTE_AGE_MS)

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logfile',
                        help='uSD log file (binary) recorded on the Crazyflie')
    parser.add_argument('--anchors', required=True,
                        help='YAML file mapping anchor id -> {x, y, z}')
    parser.add_argument('--selection-policy', default='baseline',
                        help='Anchor-pair selection policy for the replay '
                             '(default: baseline)')
    parser.add_argument('--tdoa-model', default='standard',
                        choices=('standard', 'robust'),
                        help='TDoA measurement model for the replay: standard '
                             '(kalmanCoreUpdateWithTdoa) or robust '
                             '(M-estimation, kalman.robustTdoa=1 equivalent) '
                             '(default: standard)')
    parser.add_argument('--outlier-filter', default='integrator',
                        help='TDoA outlier filter for the replay (see '
                             'bindings/util/tdoa_outlier.py; default: '
                             'integrator, the firmware filter)')
    parser.add_argument('--policy-param', action='append', metavar='KEY=VALUE',
                        help='Selection-policy constructor kwarg, repeatable '
                             '(e.g. --policy-param k=3 for top_k)')
    parser.add_argument('--filter-param', action='append', metavar='KEY=VALUE',
                        help='Outlier-filter tuning-constant override, '
                             'repeatable (e.g. --filter-param WINDOW=16 '
                             '--filter-param WARMUP=6 for pair_hampel)')
    parser.add_argument('--align-truth', default='none',
                        choices=('none', 'live'),
                        help='Fit a rigid transform mapping the Lighthouse '
                             'ground truth into the Loco anchor frame before '
                             'anything is scored against it, removing the '
                             'survey disagreement between the two systems. '
                             "'live' fits against the onboard estimate, which "
                             'is independent of whatever the replay is testing, '
                             'so several policies can be compared under one '
                             'alignment (default: none)')
    parser.add_argument('--align-model', default='translation',
                        choices=('translation', 'rigid', 'similarity'),
                        help='Degrees of freedom for --align-truth: '
                             "'translation' solves the origin offset only, "
                             "'rigid' adds rotation, 'similarity' adds a "
                             'uniform scale. Default is translation, because '
                             'on these logs it is the only term that '
                             'reproduces between flights - fitted rotation and '
                             'scale vary flight to flight, which a real frame '
                             'relationship cannot. Fit the wider models to '
                             'check that, not to lower the residual')
    parser.add_argument('--oracle-reference', default='truth',
                        choices=('truth', 'live'),
                        help="What --selection-policy oracle scores against. "
                             "'truth' is Lighthouse, giving an upper bound no "
                             "policy can reach. 'live' is the drone's own "
                             'estimate, which the firmware has at selection '
                             'time, turning the same machinery into a '
                             'realizable minimum-innovation selector '
                             '(default: truth)')
    parser.add_argument('--oracle-score', default='measurement',
                        choices=('measurement', 'position'),
                        help="Objective for --selection-policy oracle: "
                             "'measurement' minimises TDoA error against "
                             "ground truth [m]; 'position' divides by the "
                             "measurement Jacobian magnitude |h| = 2 sin(0.5 "
                             "theta) so nearly-collinear anchor pairs, where a "
                             "given measurement error displaces the estimate "
                             "further, are penalised accordingly "
                             "(default: measurement)")
    parser.add_argument('--oracle-max-gap-ms', type=float, default=100.0,
                        help='Refuse to interpolate ground truth across gaps '
                             'wider than MS when scoring oracle candidates; '
                             'those candidates stay unscored rather than being '
                             'guessed at (default: 100)')
    parser.add_argument('--tdoa-std', type=float, default=0.15,
                        help='TDoA measurement std dev [m] used in the Kalman '
                             'update (default: 0.15)')
    parser.add_argument('--max-remote-age-ms', type=float, default=None,
                        metavar='MS',
                        help='Drop candidates whose remote anchor was last '
                             'heard more than MS ago, before the selection '
                             'policy runs. Stale remote data is the dominant '
                             'source of gross TDoA outliers (see '
                             'bindings/util/tdoa_selection.annotate_remote_age); '
                             f'{DEFAULT_MAX_REMOTE_AGE_MS:.0f} is a good '
                             'starting point. Omitted: no freshness gate')
    parser.add_argument('--distance-ratio-limit', type=float, default=None,
                        metavar='RATIO',
                        help='Reject candidates whose |distanceDiff| is at or '
                             'above RATIO times the separation between the two '
                             'anchors, before the selection policy runs. This '
                             'is the anchor-pair geometry filter of firmware '
                             'PR #1650 (matchRandomAnchor); the ratio nears 1 '
                             'when the tag is close to the line through the '
                             'pair, where the measurement carries little '
                             'position information. The firmware default is '
                             f'{DEFAULT_DISTANCE_RATIO_LIMIT}. Omitted: no '
                             'geometry gate')
    parser.add_argument('--init-from', default='log',
                        choices=('log', 'origin'),
                        help='Where the replay starts: log seeds the Kalman '
                             'initial position from the live stateEstimate at '
                             'the start of the log, origin keeps the firmware '
                             'default of (0, 0, 0). Use log whenever the '
                             'anchor frame is not centred on the takeoff spot '
                             '(default: log)')
    parser.add_argument('--init-window-ms', type=float, default=None,
                        metavar='MS',
                        help='Length of the window at the log start that '
                             '--init-from log takes the median over. 0 uses '
                             'the single first sample. Keep it inside the '
                             'stationary period before takeoff '
                             f'(default: {DEFAULT_INIT_WINDOW_MS:.0f})')
    parser.add_argument('--save', default=None, metavar='PNG',
                        help='Save the figure to this file instead of showing it')
    parser.add_argument('--rebuild-bindings', action='store_true',
                        help='Force a rebuild of the cffirmware bindings')
    args = parser.parse_args()
    if args.init_window_ms is None:
        args.init_window_ms = DEFAULT_INIT_WINDOW_MS

    ensure_bindings(rebuild=args.rebuild_bindings)
    replayed, live, gt = load_series(args)
    if not (replayed or live or gt):
        sys.exit('Nothing to plot: no replayable candidates, no stateEstimate '
                 'and no Lighthouse data in this log.')
    print_stats(replayed, live, gt)
    title = (f'{Path(args.logfile).name}  '
             f'(selection policy: {args.selection_policy}, '
             f'model: {args.tdoa_model}, filter: {args.outlier_filter})')
    plot(replayed, live, gt, title, save_path=args.save)


if __name__ == '__main__':
    main()
