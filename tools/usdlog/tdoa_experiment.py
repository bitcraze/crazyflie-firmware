"""
Experiment harness for TDoA replay tuning with anti-overfitting time split.

Wraps the replay seam (bindings/util/tdoa_replay.py) to evaluate many
(policy, outlier filter, tdoa model, std) combinations on one captured log,
scoring each trajectory against Lighthouse ground truth separately on a
TRAIN segment (first part of the ground-truth overlap) and a VALIDATE
segment (the rest). Tune on train only; report/rank on validate.

Run from the repository root::

    pixi run python -m tools.usdlog.tdoa_experiment run_validation.bin \
        --anchors anchors.yaml --grid grid.yaml --out results.csv

The grid file is a YAML list of config dicts, e.g.::

    - {policy: baseline, filter: integrator, model: standard, std: 0.15}
    - {policy: median,   filter: mad_window, model: standard, std: 0.30}

Each replay runs in a fresh forked worker process (the robust model keeps
M-estimation state in a process-wide static buffer, so runs must not share a
process; see bindings/util/tdoa_replay.py).
"""

import argparse
import csv
import math
import multiprocessing as mp
import sys

import yaml

import tools.usdlog.cfusdlog as cfusdlog
from tools.usdlog.replay_tdoa import extract_ground_truth, _interp_ground_truth
from bindings.util.tdoa_replay import (
    apply_policy,
    extract_imu_samples,
    replay,
)
from bindings.util.tdoa_selection import build_candidate_groups, make_policy

FLYAWAY_THRESHOLD_M = 0.3
TRAIN_FRACTION = 0.6
# Skip the estimator's cold-start convergence transient at the very beginning
# of the log; it is identical noise for every config and would dilute the
# train-segment contrast between configs.
WARMUP_S = 5.0

# Loaded once in the parent; inherited by forked workers.
_CTX = {}


def segment_metrics(errors):
    """Tails-first metrics for a list of (t_ms, err_m).

    A NaN error means the estimator diverged to NaN state — score it as
    infinite error so diverged runs rank last instead of silently passing
    every comparison.
    """
    if not errors:
        return {'n': 0, 'rms': float('nan'), 'p95': float('nan'),
                'max': float('nan'), 'flyaway_frac': float('nan')}
    vals = sorted(float('inf') if e != e else e for _, e in errors)
    n = len(vals)
    return {
        'n': n,
        'rms': math.sqrt(sum(e * e for e in vals) / n),
        'p95': vals[min(n - 1, int(0.95 * n))],
        'max': vals[-1],
        'flyaway_frac': sum(1 for e in vals if e > FLYAWAY_THRESHOLD_M) / n,
    }


def score_split(trajectory, gt, t_split_ms, t_warmup_end_ms):
    """Error series vs gt, split into train/validate at t_split_ms."""
    train, val = [], []
    for t_ms, pos in trajectory:
        if t_ms < t_warmup_end_ms:
            continue
        gt_pos = _interp_ground_truth(gt, t_ms)
        if gt_pos is None:
            continue
        err = math.sqrt(sum((pos[k] - gt_pos[k]) ** 2 for k in range(3)))
        (train if t_ms < t_split_ms else val).append((t_ms, err))
    return segment_metrics(train), segment_metrics(val)


def run_config(config):
    """Worker: one replay + scoring. Uses the parent-loaded _CTX via fork."""
    groups = _CTX['groups']
    policy = make_policy(config['policy'], config.get('policy_params'))
    tdoa_samples = apply_policy(policy, groups)
    params = {
        'tdoa_std': float(config.get('std', 0.15)),
        'tdoa_model': config.get('model', 'standard'),
    }
    if config.get('filter'):
        params['outlier_filter'] = config['filter']
    if config.get('filter_params'):
        params['outlier_filter_params'] = config['filter_params']
    if config.get('std_model'):
        params['std_model'] = config['std_model']
    if config.get('std_model_params'):
        params['std_model_params'] = config['std_model_params']
    if config.get('kalman_params'):
        params['kalman_params'] = config['kalman_params']

    trajectory = replay(_CTX['anchor_positions'], list(_CTX['imu_samples']),
                        tdoa_samples, params)
    train, val = score_split(trajectory, _CTX['gt'],
                             _CTX['t_split_ms'], _CTX['t_warmup_end_ms'])
    return config, train, val


def config_label(config):
    parts = [config['policy'], config.get('filter') or 'builtin',
             config.get('model', 'standard'), f"std={config.get('std', 0.15)}"]
    if config.get('std_model'):
        parts.append(f"stdmod={config['std_model']}")
    for key in ('policy_params', 'filter_params', 'std_model_params',
                'kalman_params'):
        if config.get(key):
            parts.append(str(config[key]))
    return '/'.join(parts)


def load_context(logfile, anchors_path, train_fraction=TRAIN_FRACTION):
    from bindings.util.loco_utils import read_loco_anchor_positions
    log_data = cfusdlog.decode(logfile)
    _CTX['anchor_positions'] = read_loco_anchor_positions(anchors_path)
    _CTX['groups'] = build_candidate_groups(log_data)
    _CTX['imu_samples'] = extract_imu_samples(log_data)
    gt = extract_ground_truth(log_data)
    _CTX['gt'] = gt
    if not gt:
        sys.exit('No ground truth in log; the experiment harness needs it.')
    t0, t1 = gt[0][0], gt[-1][0]
    _CTX['t_warmup_end_ms'] = t0 + WARMUP_S * 1000.0
    _CTX['t_split_ms'] = t0 + train_fraction * (t1 - t0)
    return log_data


def print_results(results, sort_segment):
    """Sorted table; sort_segment is 'train' or 'val'."""
    idx = 1 if sort_segment == 'train' else 2

    def sort_key(row):
        m = row[idx]
        if not m['n']:
            return (float('inf'),) * 3
        return (m['flyaway_frac'], m['p95'], m['rms'])

    header = (f'{"config":<58} '
              f'{"tr_fly":>7} {"tr_p95":>7} {"tr_rms":>7} '
              f'{"va_fly":>7} {"va_p95":>7} {"va_max":>7} {"va_rms":>7}')
    print(header)
    print('-' * len(header))
    for config, train, val in sorted(results, key=sort_key):
        label = config_label(config)[:58]
        def fmt(m, key, pct=False):
            v = m[key]
            if not m['n'] or v != v:
                return '      -'
            return f'{v:>6.1%} ' if pct else f'{v:>7.3f}'
        print(f'{label:<58} '
              f'{fmt(train, "flyaway_frac", True)}{fmt(train, "p95")} {fmt(train, "rms")} '
              f'{fmt(val, "flyaway_frac", True)}{fmt(val, "p95")} {fmt(val, "max")} {fmt(val, "rms")}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logfile')
    parser.add_argument('--anchors', required=True)
    parser.add_argument('--grid', required=True,
                        help='YAML list of config dicts (policy/filter/model/std/...)')
    parser.add_argument('--out', default=None, help='CSV output path')
    parser.add_argument('--jobs', type=int, default=max(1, mp.cpu_count() - 2))
    parser.add_argument('--sort', choices=['train', 'val'], default='train',
                        help='Segment to sort the table by (tune on train!)')
    parser.add_argument('--train-fraction', type=float, default=TRAIN_FRACTION,
                        help='Fraction of the ground-truth span used as the '
                             'train segment (rest is validate). Use alternate '
                             'values to check ranking stability of a shortlist.')
    args = parser.parse_args()

    with open(args.grid) as f:
        configs = yaml.safe_load(f)
    if not isinstance(configs, list):
        sys.exit('Grid file must be a YAML list of config dicts.')

    load_context(args.logfile, args.anchors, args.train_fraction)
    t0 = _CTX['gt'][0][0] / 1000.0
    print(f'{len(configs)} configs; gt span {t0:.1f}-{_CTX["gt"][-1][0]/1000.0:.1f}s, '
          f'warmup ends {_CTX["t_warmup_end_ms"]/1000.0:.1f}s, '
          f'train/val split at {_CTX["t_split_ms"]/1000.0:.1f}s')

    # fork start method: workers inherit _CTX. maxtasksperchild=1 gives every
    # replay a pristine process (robust-model static state).
    ctx = mp.get_context('fork')
    with ctx.Pool(processes=args.jobs, maxtasksperchild=1) as pool:
        results = pool.map(run_config, configs, chunksize=1)

    print()
    print_results(results, args.sort)

    if args.out:
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['config', 'segment', 'n', 'rms', 'p95', 'max', 'flyaway_frac'])
            for config, train, val in results:
                for seg_name, m in (('train', train), ('val', val)):
                    w.writerow([config_label(config), seg_name, m['n'],
                                m['rms'], m['p95'], m['max'], m['flyaway_frac']])
        print(f'\nWrote {args.out}')


if __name__ == '__main__':
    main()
