#!/usr/bin/env python
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.lighthouse_utils import load_lighthouse_calibration
from scipy.spatial.transform import Rotation
from plot import Plotter
import logging
import csv
import os


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
    def __init__(self, runner, emulator, cutoff_time=None):
        super().__init__('offboard', cutoff_time)
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

    def align_timestamps(self, reference_timestamps):
        if not self.timestamps:
            logging.warning('No timestamps available in MocapSource for alignment.')
            return

        # Convert all timestamps from microseconds to milliseconds.
        ms_timestamps = [t / 1010.0 for t in self.timestamps]

        # In milliseconds, a 1-second gap is 1000.
        expected_gap = 1000
        tolerance = 10  # Roughly equivalent to 0.01 seconds.

        min_diff = float('inf')
        gap_index = 0

        # Step 1: Find the index where a large gap (~1 second) occurs.
        for i in range(1, int(len(ms_timestamps) / 4)):
            diff = abs(ms_timestamps[i] - ms_timestamps[i - 1] - expected_gap)
            if diff < min_diff:
                min_diff = diff
                gap_index = i

        logging_delay = 750
        if gap_index:
            if min_diff > tolerance:
                logging.warning('Time aligning likely failed, large gap detected.')

            # Now, we want to find the timestamp closest to exactly 1 second after the detected gap, plus delay for mecap
            target_time = ms_timestamps[gap_index] + logging_delay
            closest_index = gap_index
            closest_diff = float('inf')

            for i in range(gap_index + 1, len(ms_timestamps)):
                current_diff = abs(ms_timestamps[i] - target_time)
                if current_diff < closest_diff:
                    closest_diff = current_diff
                    closest_index = i

                # Early exit if the diff is almost perfect
                if current_diff < 1e-3:
                    break

            # Now align from the closest index instead of the gap_index
            time_shift = reference_timestamps[0] - ms_timestamps[closest_index]
            self.estimates = [
                (t + time_shift, pos)
                for t, pos in zip(
                    ms_timestamps[closest_index:], self.data_points[closest_index:]
                )
            ]
        self.apply_cutoff()


logging.basicConfig(level=logging.INFO)

fixture_base = 'test_python/fixtures/kalman_core'
runner = SdCardFileRunner(fixture_base + '/logc')
mocap_data = fixture_base + '/pose_datac.csv'
cutoff_time = 750000  # in milliseconds
bs_calib, bs_geo = load_lighthouse_calibration(fixture_base + '/geometry.yaml')
emulator = EstimatorKalmanEmulator(
    basestation_calibration=bs_calib, basestation_poses=bs_geo
)

onboard_source = OnboardSource(runner.samples, cutoff_time=cutoff_time)
offboard_source = OffboardSource(runner, emulator, cutoff_time=cutoff_time)
mocap_source = MocapSource(mocap_data, cutoff_time=cutoff_time)
mocap_source.align_timestamps([t for t, _ in offboard_source.get_estimates()])

onboard_source.save_estimates()
offboard_source.save_estimates()
mocap_source.save_estimates()

file_paths = [
    'output/onboard_estimates.csv',
    'output/offboard_estimates.csv',
    'output/mocap_estimates.csv',
]
file_labels = ['Onboard', 'Offboard', 'Mocap']

fig_time = Plotter.plot_time_series(file_paths, file_labels)
fig_time.show()
input('Press Enter to close plots...')
