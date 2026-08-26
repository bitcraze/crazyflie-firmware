"""Tests for tools/tdoa_plot/plot_tdoa.py helpers (no bindings needed)."""

from tools.tdoa_plot.plot_tdoa import print_stats


def _traj(points):
    return [(float(i * 100), p) for i, p in enumerate(points)]


def test_print_stats_tolerates_non_finite_replay_samples(capsys):
    # A diverged replay (e.g. the robust model on a hard log) contains NaN;
    # stats must skip those samples and report the divergence, not crash.
    gt = _traj([(0.0, 0.0, 1.0)] * 10)
    live = _traj([(0.0, 0.0, 1.0)] * 10)
    replayed = _traj([(0.1, 0.0, 1.0)] * 8
                     + [(float('nan'),) * 3, (float('inf'),) * 3])

    print_stats(replayed, live, gt)

    out = capsys.readouterr().out
    assert '2 non-finite samples' in out
    assert 'bindings estimate (replay)' in out


def test_print_stats_all_non_finite_still_reports(capsys):
    gt = _traj([(0.0, 0.0, 1.0)] * 3)
    replayed = _traj([(float('nan'),) * 3] * 3)

    print_stats(replayed, [], gt)

    out = capsys.readouterr().out
    assert '3 non-finite samples' in out
