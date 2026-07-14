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
        apply_policy, extract_imu_samples, extract_state_estimate, replay)
    from bindings.util.tdoa_selection import build_candidate_groups, make_policy
    from tools.usdlog.replay_tdoa import extract_ground_truth

    log_data = cfusdlog.decode(args.logfile)
    live = extract_state_estimate(log_data)
    gt = extract_ground_truth(log_data)

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
    tdoa_samples = apply_policy(policy, groups)
    print(f'Replaying {len(tdoa_samples)} TDoA + {len(imu_samples)} IMU samples '
          f'through the firmware Kalman core '
          f'(selection policy: {args.selection_policy}'
          + (f' {policy_params}' if policy_params else '')
          + f', measurement model: {args.tdoa_model}, '
          f'outlier filter: {args.outlier_filter}'
          + (f' {filter_params}' if filter_params else '') + ') ...')
    replayed = replay(anchor_positions, imu_samples, tdoa_samples,
                      {'tdoa_std': args.tdoa_std,
                       'tdoa_model': args.tdoa_model,
                       'outlier_filter': args.outlier_filter,
                       'outlier_filter_params': filter_params})
    return replayed, live, gt


def parse_params(pairs):
    """Parse repeated KEY=VALUE args into a dict, as int where possible.

    Tuning constants are numeric; sizes like WINDOW must stay int (they end
    up as a deque maxlen).
    """
    params = {}
    for pair in pairs or []:
        key, sep, value = pair.partition('=')
        if not sep:
            sys.exit(f"--*-param expects KEY=VALUE, got '{pair}'")
        try:
            params[key] = int(value)
        except ValueError:
            params[key] = float(value)
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
    parser.add_argument('--tdoa-std', type=float, default=0.15,
                        help='TDoA measurement std dev [m] used in the Kalman '
                             'update (default: 0.15)')
    parser.add_argument('--save', default=None, metavar='PNG',
                        help='Save the figure to this file instead of showing it')
    parser.add_argument('--rebuild-bindings', action='store_true',
                        help='Force a rebuild of the cffirmware bindings')
    args = parser.parse_args()

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
