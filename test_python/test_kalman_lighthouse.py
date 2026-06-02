#!/usr/bin/env python
import os
import matplotlib
# Select a non-interactive backend when there is no display, so figure creation
# (inside Plotter) does not try to open a window on headless machines.
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')

from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.lighthouse_utils import load_lighthouse_calibration
from scipy.spatial.transform import Rotation
from plot import Plotter, Evaluator
import logging
import csv
import sys
import numpy as np


class EstimatorSource:
    def __init__(self, name, cutoff_time=None):
        self.name = name
        self.estimates = []
        self.cutoff_time = cutoff_time

    def extract_estimates(self):
        raise NotImplementedError

    def align_timestamps(self, reference_timestamps):
        pass  # Optional to override

    def apply_cutoff(self):
        if self.cutoff_time is not None:
            self.estimates = [
                (t, pos) for t, pos in self.estimates if t <= self.cutoff_time
            ]

    def get_estimates(self):
        return self.estimates

    def save_estimates(self, directory='output'):
        if not os.path.exists(directory):
            os.makedirs(directory)

        file_path = os.path.join(directory, f'{self.name}_estimates.csv')
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
            for timestamp, (x, y, z, roll, pitch, yaw) in self.estimates:
                writer.writerow([timestamp, x, y, z, roll, pitch, yaw])

        print(f'Estimates saved to {file_path}')


class OnboardSource(EstimatorSource):
    def __init__(self, samples, cutoff_time=None):
        super().__init__('onboard', cutoff_time)
        self.samples = samples or []  # Handle missing samples
        self.extract_estimates()
        self.apply_cutoff()  # Apply the cutoff after estimates are extracted

    def extract_estimates(self):
        for entry_type, data in self.samples:
            if entry_type == 'fixedFrequency':
                timestamp = float(data['timestamp'])
                x = float(data.get('stateEstimate.x', 0.0))
                y = float(data.get('stateEstimate.y', 0.0))
                z = float(data.get('stateEstimate.z', 0.0))
                roll = float(data.get('stateEstimate.roll', 0.0))
                pitch = float(data.get('stateEstimate.pitch', 0.0))
                yaw = float(data.get('stateEstimate.yaw', 0.0))
                self.estimates.append((timestamp, (x, y, z, roll, pitch, yaw)))


class OffboardSource(EstimatorSource):
    def __init__(self, runner, emulator, name='offboard', cutoff_time=None):
        super().__init__(name, cutoff_time)
        self.runner = runner
        self.emulator = emulator
        self.estimates = []
        if runner and emulator:
            self.extract_estimates()
            self.apply_cutoff()  # Apply the cutoff after estimates are extracted
        else:
            logging.warning('Runner or Emulator not provided for OffboardSource')

    def extract_estimates(self):
        self.estimates = self.runner.run_estimator_loop(self.emulator)


class MocapSource(EstimatorSource):
    def __init__(self, filepath, cutoff_time=None):
        super().__init__('mocap', cutoff_time)
        self.filepath = filepath
        self.timestamps = []
        self.data_points = []
        self.extract_estimates()
        self.apply_cutoff()

    def extract_estimates(self):
        try:
            with open(self.filepath, 'r') as file:
                reader = csv.reader(file)
                header = next(reader)

                for row in reader:
                    timestamp, x, y, z, q0, q1, q2, q3 = map(float, row)
                    roll, pitch, yaw = Rotation.from_quat([q0, q1, q2, q3]).as_euler(
                        'xyz', degrees=True
                    )
                    self.timestamps.append(timestamp)
                    self.data_points.append((x, y, z, roll, -pitch, yaw))

        except (FileNotFoundError, ValueError) as e:
            logging.warning(f"Error reading Mocap file '{self.filepath}': {e}")

    def align_timestamps(self, reference_estimates):
        """Align the mocap trajectory onto the estimator (Crazyflie) clock.

        The mocap PC and the Crazyflie do not share a clock, so their timebases
        differ by both an offset and a small rate error (~1% on these logs). We
        recover a linear map ``t_cf = scale * (t_mocap - t_mocap[0]) + offset``
        by minimising the mismatch in vertical position ``z`` against a reference
        estimate. ``z`` is gravity-aligned and therefore invariant to any
        horizontal rotation/translation between the mocap and lighthouse frames,
        so the fit reflects timing only, not a spatial frame difference.

        This replaces an earlier heuristic that assumed a fixed clock ratio and a
        manually inserted ~1 s gap near the start of the recording; neither holds
        across these logs.

        Args:
            reference_estimates: list of ``(t_ms, (x, y, z, roll, pitch, yaw))``
                from an offboard run (Crazyflie clock).
        """
        if not self.timestamps or not reference_estimates:
            logging.warning('Cannot align mocap: missing mocap or reference data.')
            return

        # Mocap timestamps -> milliseconds, auto-detecting the recorded unit from
        # the sample period (recordings have used s, ms and us).
        t_raw = np.asarray(self.timestamps, dtype=float)
        median_dt = np.median(np.diff(t_raw))
        if median_dt < 1e-2:        # seconds
            tm = t_raw * 1000.0
        elif median_dt > 100.0:     # microseconds
            tm = t_raw / 1000.0
        else:                       # already milliseconds
            tm = t_raw.copy()
        tm -= tm[0]
        zm = np.asarray([p[2] for p in self.data_points], dtype=float)

        tr = np.asarray([t for t, _ in reference_estimates], dtype=float)
        zr = np.asarray([p[2] for _, p in reference_estimates], dtype=float)

        # Downsample the mocap z signal for the fit; coarse structure is enough.
        step = max(1, len(tm) // 1000)
        tm_f, zm_f = tm[::step], zm[::step]

        def z_rmse(scale, offset):
            t_mapped = scale * tm_f + offset
            mask = (t_mapped >= tr[0]) & (t_mapped <= tr[-1])
            if mask.sum() < 200:
                return np.inf
            d = np.interp(t_mapped[mask], tr, zr) - zm_f[mask]
            d -= d.mean()           # ignore any constant z frame offset
            return np.sqrt(np.mean(d ** 2))

        def search(scales, offsets):
            best = (np.inf, 1.0, 0.0)
            for s in scales:
                for o in offsets:
                    r = z_rmse(s, o)
                    if r < best[0]:
                        best = (r, s, o)
            return best

        # Coarse grid over (scale, offset), then refine around the best cell.
        _, s0, o0 = search(np.arange(0.97, 1.031, 0.002),
                           np.arange(tr[0] - 25000.0, tr[0] + 15000.0, 200.0))
        rmse, scale, offset = search(np.arange(s0 - 0.002, s0 + 0.00201, 0.0002),
                                     np.arange(o0 - 300.0, o0 + 301.0, 20.0))
        logging.info('Mocap alignment: scale=%.4f offset=%.0f ms (z-RMSE=%.3f m)',
                     scale, offset, rmse)
        if rmse > 0.10:
            logging.warning('Mocap alignment z-RMSE is high (%.3f m); check the '
                            'reference trajectory and mocap data.', rmse)

        self.estimates = [
            (scale * t + offset, pos) for t, pos in zip(tm, self.data_points)
        ]
        self.apply_cutoff()


logging.basicConfig(level=logging.INFO)

fixture_base = 'test_python/fixtures/kalman_core'
# Optional CLI args select the fixture pair, e.g. `log30 pose_data30.csv`.
# Defaults to the committed log30/pose_data30.csv fixtures.
log_name = sys.argv[1] if len(sys.argv) > 1 else 'log30'
mocap_name = sys.argv[2] if len(sys.argv) > 2 else 'pose_data30.csv'
mocap_data = f'{fixture_base}/{mocap_name}'
cutoff_time = 750000  # in milliseconds
bs_calib, bs_geo = load_lighthouse_calibration(fixture_base + '/geometry.yaml')

# Both baselines are produced offboard, by replaying the *same* recorded log
# through the same emulator. This keeps the timing, prediction cadence and init
# identical, so the only difference is the measurement model itself:
#   - master (legacy): position-only sweep update + replayed crossing-beams yaw
#     (the deprecated method, removed from the firmware in this branch, whose
#      output was logged as estYawError on the recording firmware).
#   - this PR: orientation (roll/pitch/yaw) estimated directly in the EKF from
#     the sweep angles.
# A separate runner is used per pass because run_estimator_loop consumes samples.
runner_master = SdCardFileRunner(f'{fixture_base}/{log_name}')
runner_pr = SdCardFileRunner(f'{fixture_base}/{log_name}')

emulator_master = EstimatorKalmanEmulator(
    basestation_calibration=bs_calib, basestation_poses=bs_geo,
    estimate_orientation_from_sweep=False, replay_yaw_error=True,
)
emulator_pr = EstimatorKalmanEmulator(
    basestation_calibration=bs_calib, basestation_poses=bs_geo,
    estimate_orientation_from_sweep=True, replay_yaw_error=False,
)

# Recorded onboard estimate, if the log carries it (some logs only contain raw
# measurements). Built before the PR run consumes runner_pr.samples.
onboard_source = OnboardSource(runner_pr.samples, cutoff_time=cutoff_time)

# Runs execute here (in the constructors), sequentially: master then PR.
master_source = OffboardSource(runner_master, emulator_master, name='master', cutoff_time=cutoff_time)
pr_source = OffboardSource(runner_pr, emulator_pr, name='offboard', cutoff_time=cutoff_time)
mocap_source = MocapSource(mocap_data, cutoff_time=cutoff_time)
mocap_source.align_timestamps(pr_source.get_estimates())

master_source.save_estimates()
pr_source.save_estimates()
mocap_source.save_estimates()


def print_error_table():
    """Per-dimension error of each estimator against the aligned mocap ground truth.

    Orientation carries a constant bias from how the lighthouse deck and the mocap
    markers are mounted (two separate decks), so the mean error (bias) is reported
    next to RMSE; the std (RMSE with the bias removed) reflects tracking quality.
    """
    master_stats = Evaluator.calculate_statistics(
        'output/master_estimates.csv', 'output/mocap_estimates.csv')
    pr_stats = Evaluator.calculate_statistics(
        'output/offboard_estimates.csv', 'output/mocap_estimates.csv')
    if not master_stats or not pr_stats:
        logging.warning('Could not compute error table (missing data).')
        return

    units = {'X': 'm', 'Y': 'm', 'Z': 'm', 'Roll': 'deg', 'Pitch': 'deg', 'Yaw': 'deg'}
    print('\nError vs mocap  (master = legacy, PR = this branch)')
    print(f"{'dim':<6}{'master RMSE':>13}{'PR RMSE':>12}"
          f"{'master bias':>14}{'PR bias':>12}{'master std':>13}{'PR std':>11}")
    print('-' * 81)
    for dim in ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']:
        m, p, u = master_stats[dim], pr_stats[dim], units[dim]
        print(f"{dim:<6}{m['RMSE']:>10.3f} {u:<2}{p['RMSE']:>9.3f} {u:<2}"
              f"{m['Mean Error']:>11.3f} {u:<2}{p['Mean Error']:>9.3f} {u:<2}"
              f"{m['STDDEV Error']:>10.3f} {u:<2}{p['STDDEV Error']:>8.3f} {u:<2}")


print_error_table()

file_paths = [
    'output/master_estimates.csv',
    'output/offboard_estimates.csv',
    'output/mocap_estimates.csv',
]
file_labels = ['master (legacy)', 'this PR', 'mocap']

# Include the recorded onboard estimate as an extra trace only when present.
if onboard_source.get_estimates():
    onboard_source.save_estimates()
    file_paths.append('output/onboard_estimates.csv')
    file_labels.append('onboard (recorded)')

fig_time = Plotter.plot_time_series(file_paths, file_labels)
if os.environ.get('DISPLAY'):
    fig_time.show()
    input('Press Enter to close plots...')
else:
    out = 'output/time_series.png'
    fig_time.savefig(out)
    print(f'No display detected; saved plot to {out}')
