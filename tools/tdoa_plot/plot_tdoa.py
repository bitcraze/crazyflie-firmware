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
    """Decode the log and return (replayed, replayed_full, live, ground_truth,
    filter_window, tdoa_innovations).

    replayed is [(t_ms, (x, y, z))]; replayed_full is
    [(t_ms, state_dict)] (see tdoa_replay.replay_full_state), or None when
    the log has no candidate data to replay. filter_window is
    [(t_ms, is_open)], the onboard TDoA outlier filter's state, or []
    when the log has no outlierf.tdoaWin. tdoa_innovations is
    tdoa_replay.replay_full_state's per-measurement innovation trace
    (idA/idB/error/std_dev/accepted), or [] when there is nothing to replay.
    """
    import tools.usdlog.cfusdlog as cfusdlog
    from bindings.util.loco_utils import read_loco_anchor_positions
    from bindings.util.tdoa_replay import (
        apply_policy, extract_imu_samples, extract_outlier_filter_window,
        extract_state_estimate, replay_full_state, seed_initial_position)
    from bindings.util.tdoa_selection import build_candidate_groups, make_policy
    from tools.usdlog.replay_tdoa import extract_ground_truth

    log_data = cfusdlog.decode(args.logfile)
    live = extract_state_estimate(log_data)
    gt = extract_ground_truth(log_data)
    filter_window = extract_outlier_filter_window(log_data)

    groups = build_candidate_groups(log_data)
    if not groups:
        print('No estTdoaCand data found; plotting live estimate and '
              'ground truth only.')
        return None, None, live, gt, filter_window, []

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
    kalman_params = resolve_initial_position(args, log_data,
                                             seed_initial_position)
    replayed_full, tdoa_innovations = replay_full_state(
        anchor_positions, imu_samples, tdoa_samples,
        {'tdoa_std': args.tdoa_std,
         'tdoa_model': args.tdoa_model,
         'outlier_filter': args.outlier_filter,
         'outlier_filter_params': filter_params,
         'kalman_params': kalman_params})
    replayed = [(t_ms, s['position']) for t_ms, s in replayed_full]
    return replayed, replayed_full, live, gt, filter_window, tdoa_innovations


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


def _clip_to_time_range(series, start_s, end_s, t_of=lambda item: item[0]):
    """Keep only samples with t_of(sample)/1000 in [start_s, end_s].

    start_s/end_s may be None to leave that side unbounded. t_of extracts the
    t_ms from a sample; series items are usually (t_ms, ...) tuples, but
    tdoa_innovations entries are dicts keyed by 'timestamp' instead.
    """
    if start_s is None and end_s is None:
        return series
    lo_ms = -math.inf if start_s is None else start_s * 1000.0
    hi_ms = math.inf if end_s is None else end_s * 1000.0
    return [item for item in series if lo_ms <= t_of(item) <= hi_ms]


def clip_series_to_time_range(replayed, replayed_full, live, gt, filter_window,
                              tdoa_innovations, start_s, end_s):
    """Restrict every loaded series to [start_s, end_s] seconds (log-clock time).

    Applied after load_series/replay, so the Kalman replay itself always runs
    over the full log (it needs the true history); only what gets plotted and
    scored is narrowed to the requested window.
    """
    if start_s is None and end_s is None:
        return replayed, replayed_full, live, gt, filter_window, tdoa_innovations
    return (
        _clip_to_time_range(replayed, start_s, end_s),
        _clip_to_time_range(replayed_full, start_s, end_s),
        _clip_to_time_range(live, start_s, end_s),
        _clip_to_time_range(gt, start_s, end_s),
        _clip_to_time_range(filter_window, start_s, end_s),
        _clip_to_time_range(tdoa_innovations, start_s, end_s,
                            t_of=lambda e: e['timestamp']),
    )


def _time_filter_open(series):
    """(seconds open, seconds spanned) for a [(t_ms, is_open)] series.

    Each sample's is_open is held constant until the next sample (left hold),
    so the last sample doesn't contribute any duration on its own.
    """
    if len(series) < 2:
        return 0.0, 0.0
    open_s = sum((t1 - t0) / 1000.0
                for (t0, is_open), (t1, _) in zip(series, series[1:])
                if is_open)
    span_s = (series[-1][0] - series[0][0]) / 1000.0
    return open_s, span_s


def print_filter_window_stats(filter_window, replayed_full):
    """Print how long the TDoA outlier filter window was open.

    Always generated (unlike the accuracy stats below), since it needs no
    ground truth -- just the filter's own open/closed state.
    """
    replayed_filter_window = [
        (t_ms, s['filter_open']) for t_ms, s in (replayed_full or [])
        if s['filter_open'] is not None]

    print('--- outlier filter window (open = large TDoA residuals rejected) ---')
    found_any = False
    for name, series in [('live estimate (onboard)', filter_window or []),
                         ('bindings estimate (replay)', replayed_filter_window)]:
        if not series:
            continue
        found_any = True
        open_s, span_s = _time_filter_open(series)
        pct = 100.0 * open_s / span_s if span_s > 0 else 0.0
        print(f'  {name}: open {open_s:.2f} s of {span_s:.2f} s ({pct:.1f}%)')
    if not found_any:
        print('  no outlier-filter-window data in this log')


def print_innovation_stats(tdoa_innovations):
    """Print TDoA innovation-error stats overall and per anchor pair.

    Innovation error is measured - predicted distanceDiff [m]
    (kalmanCoreTdoaInnovation), evaluated against the replayed Kalman core
    state at the time each measurement was applied -- so, unlike the
    accuracy stats below, this needs no Lighthouse ground truth.
    """
    print('--- TDoA innovation error (measured - predicted distanceDiff) ---')
    if not tdoa_innovations:
        print('  no TDoA candidate data in this log')
        return

    finite = [e for e in tdoa_innovations if math.isfinite(e['error'])]
    n_nan = len(tdoa_innovations) - len(finite)
    n_rejected = sum(1 for e in finite if e['accepted'] is False)

    print(f'  {len(tdoa_innovations)} measurements'
          + (f', {n_nan} skipped (degenerate geometry)' if n_nan else '')
          + (f', {n_rejected} rejected by outlier filter' if n_rejected else ''))
    if not finite:
        return

    errs = [e['error'] for e in finite]
    bias = statistics.mean(errs)
    median_abs = statistics.median(abs(e) for e in errs)
    std = statistics.pstdev(errs)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    print(f'  overall: bias {bias:+.3f} m   median|err| {median_abs:.3f} m   '
          f'std {std:.3f} m   rms {rms:.3f} m')

    print('  per anchor pair (idA-idB, idA<idB): n, bias, median|err|, std, rms [m]')
    # idA/idB order depends on which anchor transmitted the packet, not on id
    # magnitude, so the same physical pair can appear as (12, 13) or (13, 12).
    # Normalize to the smaller id first, flipping the error sign to match
    # (distanceDiff negates when the anchor order is swapped).
    normalized = []
    for e in finite:
        a, b, err = e['idA'], e['idB'], e['error']
        if a > b:
            a, b, err = b, a, -err
        normalized.append((a, b, err))
    pairs = sorted({(a, b) for a, b, _ in normalized})
    rows = []
    for pair in pairs:
        pair_errs = [err for a, b, err in normalized if (a, b) == pair]
        rows.append((pair, len(pair_errs), statistics.mean(pair_errs),
                    statistics.median(abs(e) for e in pair_errs),
                    statistics.pstdev(pair_errs),
                    math.sqrt(sum(e * e for e in pair_errs) / len(pair_errs))))
    rows.sort(key=lambda r: -r[1])
    for (a, b), n, p_bias, p_median_abs, p_std, p_rms in rows:
        print(f'    {a:3d}-{b:<3d}  n={n:5d}   bias {p_bias:+.3f}   '
              f'median|err| {p_median_abs:.3f}   std {p_std:.3f}   rms {p_rms:.3f}')


def print_stats(replayed, live, gt, filter_window=None, replayed_full=None,
                tdoa_innovations=None):
    """Print accuracy stats of each estimate against the lighthouse ground truth.

    Errors are estimate - ground truth, with the ground truth linearly
    interpolated at each estimate timestamp (samples outside the ground-truth
    time range are skipped).
    """
    from tools.usdlog.replay_tdoa import _interp_ground_truth

    print_filter_window_stats(filter_window, replayed_full)
    print_innovation_stats(tdoa_innovations or [])

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


def plot(replayed, live, gt, title, save_path=None, replayed_full=None,
        filter_window=None):
    import matplotlib.pyplot as plt

    replayed_filter_window = None
    if replayed_full:
        replayed_filter_window = [
            (t_ms, s['filter_open']) for t_ms, s in replayed_full
            if s['filter_open'] is not None]

    has_filter_row = bool(filter_window) or bool(replayed_filter_window)
    n_rows = 4 if has_filter_row else 3
    fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(12, 8 if has_filter_row else 6))
    pos_axes = axes[:3] if has_filter_row else axes
    series = [
        (gt, 'ground truth (lighthouse)', dict(color='black', alpha=0.5, lw=1.0)),
        (live, 'live estimate (onboard)', dict(color='tab:orange', lw=1.0)),
        (replayed, 'bindings estimate (replay)', dict(color='tab:blue', lw=1.0)),
    ]
    for dim, (ax, label) in enumerate(zip(pos_axes, 'xyz')):
        for traj, name, style in series:
            if not traj:
                continue
            t = [s[0] / 1000.0 for s in traj]
            v = [s[1][dim] for s in traj]
            ax.plot(t, v, label=name, **style)
        if replayed_full:
            t = [s[0] / 1000.0 for s in replayed_full]
            v = [s[1]['position'][dim] for s in replayed_full]
            sd = [s[1]['std_position'][dim] for s in replayed_full]
            ax.fill_between(t, [vi - sdi for vi, sdi in zip(v, sd)],
                            [vi + sdi for vi, sdi in zip(v, sd)],
                            color='tab:blue', alpha=0.2, lw=0,
                            label='+/-1 std dev')
        ax.set_ylabel(f'{label} [m]')
        ax.grid(True, alpha=0.3)
    pos_axes[0].legend(loc='upper right')
    pos_axes[0].set_title(title)

    if has_filter_row:
        ax = axes[3]
        if filter_window:
            t = [s[0] / 1000.0 for s in filter_window]
            v = [int(s[1]) for s in filter_window]
            ax.step(t, v, where='post', label='live estimate (onboard)',
                    color='tab:orange', lw=1.0)
        if replayed_filter_window:
            t = [s[0] / 1000.0 for s in replayed_filter_window]
            v = [int(s[1]) for s in replayed_filter_window]
            ax.step(t, v, where='post', label='bindings estimate (replay)',
                    color='tab:blue', lw=1.0, linestyle='--')
        ax.set_ylabel('outlier filter')
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['closed', 'open'])
        ax.set_ylim(-0.2, 1.2)
        ax.grid(True, alpha=0.3)
        if filter_window and replayed_filter_window:
            ax.legend(loc='upper right')

    axes[-1].set_xlabel('time [s]')
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')

    return axes[0]


def plot_state_with_std(replayed_full, field, std_field, axis_labels, ylabel,
                        title, save_path=None, sharex=None):
    """Plot one state field (replayed only) as 3 subplots, one per axis.

    Each subplot shows the replayed value with a shaded +/-1 std dev band
    from the Kalman core covariance (std_field). replayed_full must be
    non-empty.

    sharex: an Axes from another (already-created) time-based figure to link
    x-axis zoom/pan with, or None to not link.
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    if sharex is not None:
        axes[0].sharex(sharex)
    t = [s[0] / 1000.0 for s in replayed_full]
    for dim, (ax, label) in enumerate(zip(axes, axis_labels)):
        v = [s[1][field][dim] for s in replayed_full]
        sd = [s[1][std_field][dim] for s in replayed_full]
        lo = [vi - sdi for vi, sdi in zip(v, sd)]
        hi = [vi + sdi for vi, sdi in zip(v, sd)]
        ax.plot(t, v, label='bindings estimate (replay)', color='tab:blue', lw=1.0)
        ax.fill_between(t, lo, hi, color='tab:blue', alpha=0.2, lw=0,
                        label='+/-1 std dev')
        ax.set_ylabel(f'{label} [{ylabel}]')
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    axes[0].set_title(title)
    axes[-1].set_xlabel('time [s]')
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')

    return axes[0]


def plot_3d(anchor_positions, replayed, live, gt, title, save_path=None):
    """Interactive 3D view of the full trajectory together with the Loco
    anchors (numbered), for rotating/zooming around the flight volume.

    anchor_positions: dict[int, cffirmware.vec3_s], as returned by
    loco_utils.read_loco_anchor_positions. replayed/live/gt are
    [(t_ms, (x, y, z))] as elsewhere in this file; any of them may be empty.
    """
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(projection='3d')

    # Keep the toolbar's Pan/Zoom tool from hijacking left-drag: with it
    # engaged, drag pans the view instead of the native 3D rotate. Disabling
    # it here means left-drag always rotates, middle-drag pans, and
    # right-drag zooms.
    ax.can_pan = lambda: False
    ax.can_zoom = lambda: False

    all_x, all_y, all_z = [], [], []
    series = [
        (gt, 'ground truth (lighthouse)', dict(color='black', alpha=0.5, lw=1.0)),
        (live, 'live estimate (onboard)', dict(color='tab:orange', lw=1.0)),
        (replayed, 'bindings estimate (replay)', dict(color='tab:blue', lw=1.0)),
    ]
    for traj, name, style in series:
        if not traj:
            continue
        xs = [p[0] for _, p in traj]
        ys = [p[1] for _, p in traj]
        zs = [p[2] for _, p in traj]
        ax.plot(xs, ys, zs, label=name, **style)
        all_x += xs
        all_y += ys
        all_z += zs

    anchor_ids = sorted(anchor_positions)
    anchor_x = [anchor_positions[i].x for i in anchor_ids]
    anchor_y = [anchor_positions[i].y for i in anchor_ids]
    anchor_z = [anchor_positions[i].z for i in anchor_ids]
    ax.scatter(anchor_x, anchor_y, anchor_z, s=70, c='black', marker='o',
              depthshade=True, label='anchors')
    for anchor_id, x, y, z in zip(anchor_ids, anchor_x, anchor_y, anchor_z):
        ax.text(x, y, z, f'  {anchor_id}', fontsize=10, color='black', weight='bold')
    all_x += anchor_x
    all_y += anchor_y
    all_z += anchor_z

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title(title)
    ax.legend(loc='upper right')

    if all_x:
        # Equal aspect ratio so distances/shape aren't distorted, spanning
        # both the anchors and every plotted trajectory.
        max_range = max(max(all_x) - min(all_x), max(all_y) - min(all_y),
                        max(all_z) - min(all_z)) / 2.0
        mid_x = (max(all_x) + min(all_x)) / 2.0
        mid_y = (max(all_y) + min(all_y)) / 2.0
        mid_z = (max(all_z) + min(all_z)) / 2.0
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')


def plot_innovation(tdoa_innovations, title, save_path=None, clip_view=True,
                    sharex=None):
    """Scatter the TDoA innovation error over time, colored by anchor A id.

    tdoa_innovations: from tdoa_replay.replay_full_state (see
    print_innovation_stats for the field meanings). Points with a
    non-finite error (degenerate anchor geometry) are dropped. Samples the
    outlier filter rejected (accepted is False) are drawn hollow; accepted
    samples, and samples where acceptance isn't known (accepted is None --
    no outlier_filter configured), are drawn filled.

    clip_view: clip the y axis to the bulk of the data (see below) and call
    out off-scale points; pass False (e.g. for --accepted-only, where
    outliers have already been filtered out upstream) to let matplotlib
    auto-scale to the actual data instead.

    sharex: an Axes from another (already-created) time-based figure to link
    x-axis zoom/pan with, or None to not link.

    idB isn't encoded visually (too many distinct pairs for a legend to
    stay readable); it is still in the returned tdoa_innovations for
    per-pair inspection (see print_innovation_stats's per-pair table).
    """
    import matplotlib.pyplot as plt

    finite = [e for e in tdoa_innovations if math.isfinite(e['error'])]
    fig, ax = plt.subplots(figsize=(12, 4))
    if sharex is not None:
        ax.sharex(sharex)
    if not finite:
        print('No finite TDoA innovations to plot.')
    else:
        if clip_view:
            # A diverged replay can produce a handful of huge-residual
            # updates (metres to kilometres) that would otherwise squash the
            # axis flat; clip the view to the bulk of the data and call out
            # what's hidden.
            abs_errs = sorted(abs(e['error']) for e in finite)
            limit = max(abs_errs[int(0.99 * (len(abs_errs) - 1))] * 1.3, 0.05)
            n_off_scale = sum(1 for e in abs_errs if e > limit)
            ax.set_ylim(-limit, limit)
            if n_off_scale:
                ax.text(0.99, 0.02,
                        f'{n_off_scale} point(s) off-scale (up to '
                        f'{abs_errs[-1]:.3g} m; see per-pair stats)',
                        transform=ax.transAxes, ha='right', va='bottom',
                        fontsize='small', color='dimgray')

        anchor_as = sorted({e['idA'] for e in finite})
        cmap = plt.get_cmap('tab10')
        color_of = {a: cmap(i % 10) for i, a in enumerate(anchor_as)}

        for a in anchor_as:
            pts = [e for e in finite if e['idA'] == a]
            accepted = [e for e in pts if e['accepted'] is not False]
            rejected = [e for e in pts if e['accepted'] is False]
            color = color_of[a]
            if accepted:
                ax.scatter([e['timestamp'] / 1000.0 for e in accepted],
                          [e['error'] for e in accepted],
                          s=8, color=color, alpha=0.6, label=f'idA={a}')
            if rejected:
                ax.scatter([e['timestamp'] / 1000.0 for e in rejected],
                          [e['error'] for e in rejected],
                          s=14, facecolors='none', edgecolors=color, linewidths=0.8)
        ax.legend(loc='upper right', ncol=min(len(anchor_as), 6),
                  fontsize='small',
                  title='color = anchor A id (hollow = rejected)')

    ax.axhline(0.0, color='black', lw=0.8, alpha=0.5)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('innovation error [m]\n(measured - predicted distanceDiff)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')


def plot_innovation_matrix(tdoa_innovations, title, save_path=None, color_max=1.2):
    """Heatmap of median |innovation error| per anchor pair.

    Anchor A id on the x axis, anchor B id on the y axis, color = median of
    |error| [m] across every measurement seen for that pair. Uses the same
    idA<idB normalization as print_innovation_stats (the same physical pair
    can appear as (idA, idB) or (idB, idA) depending on which anchor
    transmitted), then mirrors each pair's value into both (a, b) and (b, a)
    so a row/column reads as "this anchor vs everyone else". Pairs never
    seen together (including the diagonal) are left blank.

    color_max: value [m] at which the color scale saturates at red; any
    value at or above it is drawn fully red.
    """
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import LinearSegmentedColormap

    finite = [e for e in tdoa_innovations if math.isfinite(e['error'])]
    fig, ax = plt.subplots(figsize=(7, 6))
    if not finite:
        print('No finite TDoA innovations to plot (anchor pair matrix).')
        return

    normalized = []
    for e in finite:
        a, b, err = e['idA'], e['idB'], e['error']
        if a > b:
            a, b, err = b, a, -err
        normalized.append((a, b, err))

    anchors = sorted({a for a, b, _ in normalized} | {b for a, b, _ in normalized})
    index = {anchor_id: i for i, anchor_id in enumerate(anchors)}
    n = len(anchors)
    fig.set_size_inches(max(6, n * 0.6 + 2), max(5, n * 0.6 + 1))

    pair_errs = {}
    for a, b, err in normalized:
        pair_errs.setdefault((a, b), []).append(err)

    matrix = np.full((n, n), np.nan)
    for (a, b), errs in pair_errs.items():
        value = statistics.median(abs(e) for e in errs)
        i, j = index[a], index[b]
        matrix[i, j] = value
        matrix[j, i] = value

    # Semantic heat: green (best/near zero) via yellow to red (worst).
    cmap = LinearSegmentedColormap.from_list(
        'green_yellow_red', ['#0ca30c', '#fab219', '#d03b3b'])
    cmap.set_bad('#e1e0d9')

    im = ax.imshow(np.ma.masked_invalid(matrix), cmap=cmap, origin='lower',
                    vmin=0, vmax=color_max)

    ax.set_xticks(range(n))
    ax.set_xticklabels(anchors)
    ax.set_yticks(range(n))
    ax.set_yticklabels(anchors)
    ax.set_xlabel('anchor A id')
    ax.set_ylabel('anchor B id')
    ax.set_title(title)

    for i in range(n):
        for j in range(n):
            value = matrix[i, j]
            if np.isnan(value):
                continue
            # Green and red are both mid-lightness and yellow is light, so
            # pick text color from the actual cell color's luminance rather
            # than assuming darker == larger value.
            r, g, b, _ = im.cmap(im.norm(value))
            luminance = 0.299 * r + 0.587 * g + 0.114 * b
            color = 'white' if luminance < 0.5 else 'black'
            ax.text(j, i, f'{value:.3f}', ha='center', va='center',
                    fontsize='small', color=color)

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label('median |innovation error| [m]')
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')


def main():
    # Safe to import eagerly: tdoa_replay only imports cffirmware inside
    # replay(), so this does not front-run ensure_bindings().
    from bindings.util.tdoa_replay import DEFAULT_INIT_WINDOW_MS

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
    parser.add_argument('--start-time', type=float, default=None, metavar='S',
                        help='Only plot/score samples at or after this time '
                             '[s], matching the plots\' x-axis (log-clock '
                             'time, not relative to log start). The Kalman '
                             'replay itself still runs over the full log; '
                             'only the displayed window is narrowed '
                             '(default: from the start of the log)')
    parser.add_argument('--end-time', type=float, default=None, metavar='S',
                        help='Only plot/score samples at or before this time '
                             '[s] (see --start-time; default: to the end of '
                             'the log)')
    parser.add_argument('--accepted-only', action='store_true',
                        help='Only use TDoA samples accepted by the outlier '
                             'filter (accepted is not False) in the '
                             'innovation plots and statistics; rejected '
                             'samples are dropped instead of drawn hollow '
                             '(default: include rejected samples too)')
    parser.add_argument('--innovation-matrix-max', type=float, default=1.2,
                        metavar='M',
                        help='Anchor pair matrix: |innovation error| [m] at '
                             'which the color scale saturates at red; '
                             'values at or above this are fully red '
                             '(default: 1.2)')
    parser.add_argument('--save', default=None, metavar='PNG',
                        help='Save the figures instead of showing them, one '
                             'file per plot window with this path\'s stem '
                             'suffixed (e.g. out.png -> out_position.png, '
                             'out_velocity.png, out_angles.png)')
    parser.add_argument('--rebuild-bindings', action='store_true',
                        help='Force a rebuild of the cffirmware bindings')
    args = parser.parse_args()
    if args.init_window_ms is None:
        args.init_window_ms = DEFAULT_INIT_WINDOW_MS

    ensure_bindings(rebuild=args.rebuild_bindings)
    from bindings.util.loco_utils import read_loco_anchor_positions
    anchor_positions = read_loco_anchor_positions(args.anchors)

    replayed, replayed_full, live, gt, filter_window, tdoa_innovations = load_series(args)
    if not (replayed or live or gt):
        sys.exit('Nothing to plot: no replayable candidates, no stateEstimate '
                 'and no Lighthouse data in this log.')
    replayed, replayed_full, live, gt, filter_window, tdoa_innovations = (
        clip_series_to_time_range(replayed, replayed_full, live, gt,
                                  filter_window, tdoa_innovations,
                                  args.start_time, args.end_time))
    if not (replayed or live or gt):
        sys.exit('Nothing to plot: --start-time/--end-time excludes every '
                 'sample in this log.')
    if args.accepted_only:
        n_before = len(tdoa_innovations)
        tdoa_innovations = [e for e in tdoa_innovations if e['accepted'] is not False]
        print(f'--accepted-only: dropped {n_before - len(tdoa_innovations)} '
              f'of {n_before} TDoA samples rejected by the outlier filter')
    print_stats(replayed, live, gt, filter_window, replayed_full, tdoa_innovations)
    title = (f'{Path(args.logfile).name}  '
             f'(selection policy: {args.selection_policy}, '
             f'model: {args.tdoa_model}, filter: {args.outlier_filter})')
    if args.start_time is not None or args.end_time is not None:
        lo = 'start' if args.start_time is None else f'{args.start_time:.1f}s'
        hi = 'end' if args.end_time is None else f'{args.end_time:.1f}s'
        title += f'  [{lo}-{hi}]'

    def save_path_for(suffix):
        if not args.save:
            return None
        p = Path(args.save)
        return str(p.with_name(f'{p.stem}_{suffix}{p.suffix}'))

    # All time-x-axis figures share their x-axis (via Axes.sharex) with this
    # one so zooming/panning any of them moves the others in lockstep.
    time_axis = plot(replayed, live, gt, title, save_path=save_path_for('position'),
        replayed_full=replayed_full, filter_window=filter_window)
    plot_3d(anchor_positions, replayed, live, gt, f'{title}  -  3D trajectory',
           save_path=save_path_for('3d'))
    if replayed_full:
        plot_state_with_std(replayed_full, 'velocity', 'std_velocity',
                            ('vx (body)', 'vy (body)', 'vz (body)'), 'm/s',
                            f'{title}  -  velocity',
                            save_path=save_path_for('velocity'), sharex=time_axis)
        plot_state_with_std(replayed_full, 'attitude', 'std_attitude',
                            ('roll', 'pitch', 'yaw'), 'deg',
                            f'{title}  -  angles',
                            save_path=save_path_for('angles'), sharex=time_axis)
        plot_innovation(tdoa_innovations, f'{title}  -  TDoA innovation',
                        save_path=save_path_for('innovation'),
                        clip_view=not args.accepted_only, sharex=time_axis)
        plot_innovation_matrix(tdoa_innovations,
                              f'{title}  -  anchor pair matrix',
                              save_path=save_path_for('innovation_matrix'),
                              color_max=args.innovation_matrix_max)
    else:
        print('No replay data; skipping velocity, angle and innovation plots '
              '(they need estTdoaCand data to replay).')

    if not args.save:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == '__main__':
    main()
