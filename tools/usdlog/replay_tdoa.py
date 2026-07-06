"""
Offline replay + A/B testing of TDoA anchor-pair selection policies.

Feeds a uSD log recorded with ``config_tdoa_candidates.txt`` back through the
*real* firmware Kalman core (via the ``cffirmware`` SWIG bindings) under several
anchor-pair selection policies, and scores each resulting trajectory against the
Lighthouse crossing-beam ground truth logged on the same clock.

Use it to (a) locate fly-away events (position error spikes vs ground truth) and
(b) compare selection policies on identical logged candidate data.

Run from the repository root::

    python3 -m tools.usdlog.replay_tdoa run01.bin --anchors anchors.yaml
    python3 -m tools.usdlog.replay_tdoa run01.bin --anchors anchors.yaml \
        --policies baseline all median --csv-dir replay_out

Requires the bindings to be built first (see bindings/README / build_python.sh)
so that ``import cffirmware`` works.
"""

import argparse
import math
import os

import tools.usdlog.cfusdlog as cfusdlog
from bindings.util.tdoa_replay import (
    apply_policy,
    extract_imu_samples,
    extract_state_estimate,
    replay,
)
from bindings.util.tdoa_selection import (
    build_candidate_groups,
    make_policy,
    verify_baseline_reconstruction,
)


def extract_ground_truth(log_data):
    """Extract Lighthouse crossing-beam ground truth from the fixedFrequency block.

    Returns a list of (t_ms, (x, y, z)) sorted by time, skipping invalid samples
    (position exactly at the origin, which the firmware writes when no crossing
    beam is available).
    """
    ff = log_data.get('fixedFrequency')
    if not ff or 'timestamp' not in ff:
        return []
    needed = ('lighthouse.x', 'lighthouse.y', 'lighthouse.z')
    if not all(k in ff for k in needed):
        return []

    gt = []
    for i in range(len(ff['timestamp'])):
        x = float(ff['lighthouse.x'][i])
        y = float(ff['lighthouse.y'][i])
        z = float(ff['lighthouse.z'][i])
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        gt.append((float(ff['timestamp'][i]), (x, y, z)))
    gt.sort(key=lambda s: s[0])
    return gt


def run_policy(policy, anchor_positions, imu_samples, groups, tdoa_std):
    """Run the Kalman replay for one selection policy. Returns [(t_ms, (x,y,z))]."""
    tdoa_samples = apply_policy(policy, groups)
    return replay(anchor_positions, imu_samples, tdoa_samples, {'tdoa_std': tdoa_std})


def _interp_ground_truth(gt, t_ms):
    """Linearly interpolate ground-truth position at t_ms. None if out of range."""
    if not gt or t_ms < gt[0][0] or t_ms > gt[-1][0]:
        return None
    # Binary search for the bracketing samples.
    lo, hi = 0, len(gt) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if gt[mid][0] <= t_ms:
            lo = mid
        else:
            hi = mid
    t0, p0 = gt[lo]
    t1, p1 = gt[hi]
    if t1 == t0:
        return p0
    a = (t_ms - t0) / (t1 - t0)
    return tuple(p0[k] + a * (p1[k] - p0[k]) for k in range(3))


def score_trajectory(trajectory, gt, flyaway_threshold_m):
    """Compare a trajectory to ground truth. Returns a metrics dict + error series."""
    errors = []          # (t_ms, err_m)
    for t_ms, pos in trajectory:
        gt_pos = _interp_ground_truth(gt, t_ms)
        if gt_pos is None:
            continue
        err = math.sqrt(sum((pos[k] - gt_pos[k]) ** 2 for k in range(3)))
        errors.append((t_ms, err))

    if not errors:
        return {'n': 0}, errors

    vals = sorted(e for _, e in errors)
    n = len(vals)
    rms = math.sqrt(sum(e * e for e in vals) / n)
    p95 = vals[min(n - 1, int(0.95 * n))]
    over = sum(1 for e in vals if e > flyaway_threshold_m)
    peak_t = max(errors, key=lambda e: e[1])
    return {
        'n': n,
        'rms': rms,
        'p95': p95,
        'max': vals[-1],
        'flyaway_frac': over / n,
        'flyaway_t_ms': peak_t[0],
    }, errors


def write_csv(path, trajectory, gt):
    with open(path, 'w') as f:
        f.write('t_ms,x,y,z,gt_x,gt_y,gt_z,err_m\n')
        for t_ms, pos in trajectory:
            gt_pos = _interp_ground_truth(gt, t_ms)
            if gt_pos is None:
                f.write(f'{t_ms},{pos[0]},{pos[1]},{pos[2]},,,,\n')
            else:
                err = math.sqrt(sum((pos[k] - gt_pos[k]) ** 2 for k in range(3)))
                f.write(f'{t_ms},{pos[0]},{pos[1]},{pos[2]},'
                        f'{gt_pos[0]},{gt_pos[1]},{gt_pos[2]},{err}\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logfile', help='uSD log file (binary) recorded on the Crazyflie')
    parser.add_argument('--anchors', required=True,
                        help='YAML file mapping anchor id -> {x, y, z}')
    parser.add_argument('--policies', nargs='+',
                        default=['baseline', 'all', 'median', 'round_robin'],
                        help='Selection policies to evaluate')
    parser.add_argument('--tdoa-std', type=float, default=0.15,
                        help='TDoA measurement std dev [m] used in the Kalman update '
                             '(firmware default is 0.15, or 0.30 with DECK_LOCO_LONGER_RANGE)')
    parser.add_argument('--flyaway-threshold', type=float, default=0.3,
                        help='Position error [m] above which a sample counts as a fly-away')
    parser.add_argument('--csv-dir', default=None,
                        help='If set, write a per-policy trajectory+error CSV here')
    args = parser.parse_args()

    log_data = cfusdlog.decode(args.logfile)
    # Imported here: loco_utils needs the cffirmware bindings, which the pure
    # helpers in this module (tested without bindings) must not depend on
    from bindings.util.loco_utils import read_loco_anchor_positions
    anchor_positions = read_loco_anchor_positions(args.anchors)

    groups = build_candidate_groups(log_data)
    imu_samples = extract_imu_samples(log_data)
    gt = extract_ground_truth(log_data)
    live = extract_state_estimate(log_data)

    n_candidates = sum(len(g['candidates']) for g in groups)
    print(f'Log: {args.logfile}')
    print(f'  candidate packets (groups): {len(groups)}')
    print(f'  candidate pairs total:      {n_candidates}'
          + (f'  (avg {n_candidates / len(groups):.1f}/packet)' if groups else ''))
    print(f'  IMU samples:                {len(imu_samples)}')
    print(f'  ground-truth samples:       {len(gt)}')
    check = verify_baseline_reconstruction(log_data)
    if check['n_est_tdoa']:
        status = 'OK' if (check['n_mismatched'] == 0
                          and check['n_baseline'] == check['n_est_tdoa']) else 'MISMATCH'
        print(f"  baseline reconstruction:    {status} "
              f"({check['n_compared']} compared, {check['n_mismatched']} mismatched, "
              f"{check['n_selectionless']} groups without selection)")
    if not groups:
        print('No estTdoaCand data found. Was tdoaEngine.logCand > 0 during logging?')
        return
    if not gt:
        print('WARNING: no Lighthouse ground truth found; reporting trajectories only.')

    if args.csv_dir:
        os.makedirs(args.csv_dir, exist_ok=True)

    print()
    header = f'{"policy":<12} {"n":>7} {"rms[m]":>8} {"p95[m]":>8} {"max[m]":>8} {"fly>thr":>8}'
    print(header)
    print('-' * len(header))
    for name in args.policies:
        policy = make_policy(name)
        trajectory = run_policy(policy, anchor_positions, imu_samples, groups, args.tdoa_std)
        metrics, _ = score_trajectory(trajectory, gt, args.flyaway_threshold)
        if name == 'baseline' and live:
            live_metrics, _ = score_trajectory(trajectory, live, args.flyaway_threshold)
            if live_metrics['n']:
                print(f'{"":12} baseline vs live stateEstimate: '
                      f'rms {live_metrics["rms"]:.3f} m, max {live_metrics["max"]:.3f} m '
                      f'(replay-vs-onboard divergence, spec F1)')
        if args.csv_dir:
            write_csv(os.path.join(args.csv_dir, f'replay_{name}.csv'), trajectory, gt)
        if metrics['n'] == 0:
            print(f'{name:<12} {"-":>7} {"(no overlap with ground truth)":>0}')
        else:
            print(f'{name:<12} {metrics["n"]:>7} {metrics["rms"]:>8.3f} '
                  f'{metrics["p95"]:>8.3f} {metrics["max"]:>8.3f} '
                  f'{metrics["flyaway_frac"]:>7.1%}'
                  f'   peak@{metrics["flyaway_t_ms"] / 1000.0:.1f}s')

    print()
    print('Lower rms / p95 / max / fly>thr is better. "peak@" marks the worst fly-away.')


if __name__ == '__main__':
    main()
