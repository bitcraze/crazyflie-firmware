#!/usr/bin/env python
import matplotlib.pyplot as plt
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
from bindings.util.lighthouse_utils import load_lighthouse_calibration
import logging

def test_kalman_core_with_sweeps():
    # Fixture
    fixture_base = 'test_python/fixtures/kalman_core'
    runner = SdCardFileRunner(fixture_base + '/log18')
    bs_calib, bs_geo = load_lighthouse_calibration(fixture_base + '/geometry.yaml')
    emulator = EstimatorKalmanEmulator(basestation_calibration=bs_calib, basestation_poses=bs_geo)

    # Test
    onboard_estimates = extract_onboard_estimates(runner.samples)
    actual = runner.run_estimator_loop(emulator)
    plot_positions(actual, onboard_estimates)

def extract_onboard_estimates(samples):
    estimates = []
    for entry_type, data in samples:
        if entry_type == 'fixedFrequency':
            timestamp = float(data['timestamp'])
            x = float(data.get('stateEstimate.x', 0.0))
            y = float(data.get('stateEstimate.y', 0.0))
            z = float(data.get('stateEstimate.z', 0.0))
            roll = float(data.get('stateEstimate.roll', 0.0))
            pitch = float(data.get('stateEstimate.pitch', 0.0))
            yaw = float(data.get('stateEstimate.yaw', 0.0))
            estimates.append((timestamp, (x, y, z, roll, pitch, yaw)))
    return estimates


def plot_positions(actual, onboard=None):
    timestamps = [t for t, _ in actual]
    xs = [pos[0] for _, pos in actual]
    ys = [pos[1] for _, pos in actual]
    zs = [pos[2] for _, pos in actual]
    rolls = [pos[3] for _, pos in actual]
    pitchs = [pos[4] for _, pos in actual]
    yaws = [pos[5] for _, pos in actual]

    fig, axs = plt.subplots(6, 1, sharex=True, figsize=(10, 10))

    axs[0].plot(timestamps, xs, label='Emulator')
    axs[0].set_ylabel('X Position')
    axs[0].grid(True)

    axs[1].plot(timestamps, ys, label='Emulator')
    axs[1].set_ylabel('Y Position')
    axs[1].grid(True)

    axs[2].plot(timestamps, zs, label='Emulator')
    axs[2].set_ylabel('Z Position')
    axs[2].grid(True)

    axs[3].plot(timestamps, rolls, label='Emulator')
    axs[3].set_ylabel('Roll')
    axs[3].grid(True)

    axs[4].plot(timestamps, pitchs, label='Emulator')
    axs[4].set_ylabel('Pitch')
    axs[4].grid(True)

    axs[5].plot(timestamps, yaws, label='Emulator')
    axs[5].set_ylabel('Yaw')
    axs[5].set_xlabel('Time (ms)')
    axs[5].grid(True)

    if onboard:
        onboard_t = [t for t, _ in onboard]
        onboard_x = [pos[0] for _, pos in onboard]
        onboard_y = [pos[1] for _, pos in onboard]
        onboard_z = [pos[2] for _, pos in onboard]
        onboard_roll = [pos[3] for _, pos in onboard]
        onboard_pitch = [pos[4] for _, pos in onboard]
        onboard_yaw = [pos[5] for _, pos in onboard]

        axs[0].plot(onboard_t, onboard_x, label='Onboard', linestyle='--')
        axs[1].plot(onboard_t, onboard_y, label='Onboard', linestyle='--')
        axs[2].plot(onboard_t, onboard_z, label='Onboard', linestyle='--')
        axs[3].plot(onboard_t, onboard_roll, label='Onboard', linestyle='--')
        axs[4].plot(onboard_t, onboard_pitch, label='Onboard', linestyle='--')
        axs[5].plot(onboard_t, onboard_yaw, label='Onboard', linestyle='--')

    for ax in axs:
        ax.legend()

    plt.tight_layout()
    plt.show()

logging.basicConfig(level=logging.INFO)
test_kalman_core_with_sweeps()