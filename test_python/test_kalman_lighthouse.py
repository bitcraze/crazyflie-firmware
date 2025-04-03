#!/usr/bin/env python
import matplotlib.pyplot as plt
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.lighthouse_utils import load_lighthouse_calibration
from scipy.spatial.transform import Rotation
import logging
import csv
import numpy as np


class EstimatorSource:
    def __init__(self, name):
        self.name = name
        self.estimates = []

    def extract_estimates(self):
        raise NotImplementedError

    def align_timestamps(self, reference_timestamps):
        pass  # Optional to override

    def get_estimates(self):
        return self.estimates


class OnboardSource(EstimatorSource):
    def __init__(self, samples):
        super().__init__('Onboard')
        self.samples = samples or []  # Handle missing samples
        self.extract_estimates()

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
    def __init__(self, runner, emulator):
        super().__init__('Offboard')
        self.runner = runner
        self.emulator = emulator
        self.estimates = []
        if runner and emulator:
            self.extract_estimates()
        else:
            logging.warning("Runner or Emulator not provided for OffboardSource")

    def extract_estimates(self):
        self.estimates = self.runner.run_estimator_loop(self.emulator)


class MocapSource(EstimatorSource):
    def __init__(self, filepath):
        super().__init__('Mocap')
        self.filepath = filepath
        self.timestamps = []
        self.data_points = []
        self.extract_estimates()

    def extract_estimates(self):
        try:
            with open(self.filepath, 'r') as file:
                reader = csv.reader(file)
                header = next(reader)

                for row in reader:
                    timestamp, x, y, z, q0, q1, q2, q3 = map(float, row)
                    roll, pitch, yaw = Rotation.from_quat([q0, q1, q2, q3]).as_euler('xyz', degrees=True)
                    self.timestamps.append(timestamp)
                    self.data_points.append((x, y, z, roll, -pitch, yaw))

        except (FileNotFoundError, ValueError) as e:
            logging.warning(f"Error reading Mocap file '{self.filepath}': {e}")

    def align_timestamps(self, reference_timestamps):
        if not self.timestamps:
            logging.warning("No timestamps available in MocapSource for alignment.")
            return

        min_diff = float('inf')
        gap_index = 0

        for i in range(1, len(self.timestamps)):
            diff = abs(self.timestamps[i] - self.timestamps[i - 1] - 1.0)
            if diff < min_diff:
                min_diff = diff
                gap_index = i+190


        if gap_index:
            if min_diff > 0.01:
                logging.warning("Time aligning likely failed, large gap detected.")
            time_shift = reference_timestamps[0] - (self.timestamps[gap_index] * 1000)
            self.estimates = [((t * 1000) + time_shift, pos) for t, pos in zip(self.timestamps[gap_index:], self.data_points[gap_index:])]


class Plotter:
    @staticmethod
    def plot(data_sources):
        fig, axs = plt.subplots(6, 1, sharex=True, figsize=(10, 10))
        labels = ['X Position', 'Y Position', 'Z Position', 'Roll', 'Pitch', 'Yaw']

        for label_index, label in enumerate(labels):
            for source in data_sources:
                data = source.get_estimates()
                if data:
                    timestamps = np.array([t for t, _ in data])
                    values = np.array([pos[label_index] for _, pos in data])

                    axs[label_index].plot(timestamps, values, label=source.name)

            axs[label_index].set_ylabel(label)
            axs[label_index].grid(True)
            axs[label_index].legend()

        axs[-1].set_xlabel('Time (ms)')
        plt.tight_layout()
        plt.show()


class Evaluator:
    @staticmethod
    def calculate_mse(source, reference):
        reference_data = {t: pos for t, pos in reference.get_estimates()}
        source_times = np.array([t for t, _ in source.get_estimates()])
        reference_times = np.array([t for t, _ in reference.get_estimates()])

        if not len(reference_times) or not len(source_times):
            logging.warning(f"No data available for MSE calculation between {source.name} and {reference.name}.")
            return

        interpolated_reference_values = []
        labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']

        for i in range(6):
            ref_values = np.array([pos[i] for _, pos in reference.get_estimates()])
            interpolated_values = np.interp(source_times, reference_times, ref_values)
            interpolated_reference_values.append(interpolated_values)

        interpolated_reference_values = np.array(interpolated_reference_values).T

        source_values = np.array([pos for _, pos in source.get_estimates()])
        mse = np.mean((source_values - interpolated_reference_values) ** 2, axis=0)

        print(f"MSE between {source.name} and {reference.name}:")
        for i, label in enumerate(labels):
            print(f"{label}: {mse[i]}")



logging.basicConfig(level=logging.INFO)

fixture_base = 'test_python/fixtures/kalman_core'
runner = SdCardFileRunner(fixture_base + '/log19')
mocap_data = fixture_base + '/pose_data11.csv'
bs_calib, bs_geo = load_lighthouse_calibration(fixture_base + '/geometry.yaml')
emulator = EstimatorKalmanEmulator(basestation_calibration=bs_calib, basestation_poses=bs_geo)

onboard_source = OnboardSource(runner.samples)
offboard_source = OffboardSource(runner, emulator)
mocap_source = MocapSource(mocap_data)
mocap_source.align_timestamps([t for t, _ in offboard_source.get_estimates()])

Plotter.plot([onboard_source, offboard_source, mocap_source])
Evaluator.calculate_mse(onboard_source, mocap_source)
Evaluator.calculate_mse(offboard_source, mocap_source)