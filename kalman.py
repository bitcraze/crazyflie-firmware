#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import minimize

import tools.usdlog.cfusdlog as cfusdlog
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner

DATA_PATH = "./kalman_data/log06"

def set_parameters(param_array, core_params):
    core_params.stdDevInitialPosition_z = param_array[0]
    core_params.stdDevInitialTerrainHeight = param_array[1]
    core_params.procNoiseTerrain = param_array[2]
    core_params.procNoiseAcc_z = param_array[3]
    core_params.measNoiseBaro = param_array[4]
    print(f"Set parameters: {param_array}")

def interpolate_desired(timestamps_actual, timestamps_desired, desired_z):
    """Interpolates the desired z-values to match the actual timestamps."""
    interpolator = interp1d(
        timestamps_desired,
        desired_z,
        kind="linear",
        bounds_error=False,
        fill_value="extrapolate",
    )
    return interpolator(timestamps_actual)

def find_take_off_index(data):
    timestamps = data["fixedFrequency"]["timestamp"]
    z_values = data["fixedFrequency"]["stateEstimate.z"]
    z_derivative = np.gradient(z_values, timestamps)
    for i, dz in enumerate(z_derivative):
        if dz > 0.001:
            print(f"Take-off index found at: {i}, Time: {timestamps[i]}, Z-derivative: {dz}")
            return i
    raise ValueError("Take-off index not found.")

def objective_function(params, data, take_off_index):
    """Objective function to minimize: computes the error between desired and actual z."""
    runner = SdCardFileRunner(DATA_PATH)
    emulator = EstimatorKalmanEmulator()
    emulator._fetch_core_params()
    set_parameters(params, emulator.coreParams)

    actual = runner.run_estimator_loop(emulator, take_off_index)
    timestamps_actual = [entry[0] for entry in actual]
    actual_z = [entry[1][2] for entry in actual]

    timestamps_desired = data["fixedFrequency"]["timestamp"]
    desired_z = data["fixedFrequency"]["stateEstimate.z"]

    desired_z_interp = interpolate_desired(timestamps_actual, timestamps_desired, desired_z)
    error = np.mean((np.array(actual_z) - desired_z_interp) ** 2)
    print(f"Error: {error}")
    return error

def optimize(data, take_off_index):
    initial_params = np.array([1.0e-03, 1.0e+00, 1.26860063e+00, 8.52703449e-01, 5.26013486e+00])
    bounds = [(0.001, 1000.0)] * 5

    result = minimize(
        objective_function,
        initial_params,
        args=(data, take_off_index),
        bounds=bounds,
        method="L-BFGS-B",
        options={"disp": True, "eps": 0.0001},
    )

    print("Optimized Parameters:", result.x)
    print("Optimization Success:", result.success)
    return result

def calculate_final_results(data, params, take_off_index):
    runner = SdCardFileRunner(DATA_PATH)
    emulator = EstimatorKalmanEmulator()
    emulator._fetch_core_params()
    set_parameters(params, emulator.coreParams)
    return runner.run_estimator_loop(emulator, take_off_index)

def fetch_desired_results(data, final_timestamps):
    timestamps = data["fixedFrequency"]["timestamp"]
    x_values = data["fixedFrequency"]["stateEstimate.x"]
    y_values = data["fixedFrequency"]["stateEstimate.y"]
    z_values = data["fixedFrequency"]["stateEstimate.z"]
    desired_results_x = interpolate_desired(final_timestamps, timestamps, x_values)
    desired_results_y = interpolate_desired(final_timestamps, timestamps, y_values)
    desired_results_z = interpolate_desired(final_timestamps, timestamps, z_values)
    return list(zip(final_timestamps, zip(desired_results_x, desired_results_y, desired_results_z)))

def plot_optimization_results(actual, desired):
    timestamps = [entry[0] for entry in actual]
    actual_x = [entry[1][0] for entry in actual]
    actual_y = [entry[1][1] for entry in actual]
    actual_z = [entry[1][2] for entry in actual]
    desired_x = [entry[1][0] for entry in desired]
    desired_y = [entry[1][1] for entry in desired]
    desired_z = [entry[1][2] for entry in desired]

    plt.figure(figsize=(10, 6))
    plt.scatter(desired_z, actual_z, c="blue", label="Actual vs Desired Z", alpha=0.7)
    plt.plot(
        [min(desired_z), max(desired_z)],
        [min(desired_z), max(desired_z)],
        "r--",
        label="Ideal Line",
    )
    plt.xlabel("Desired Z")
    plt.ylabel("Actual Z")
    plt.title("Actual vs Desired Z")
    plt.legend()
    plt.grid(True)
    plt.show()

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(actual_x, actual_y, actual_z, c="blue", label="Actual", alpha=0.7)
    ax.scatter(desired_x, desired_y, desired_z, c="green", label="Desired", alpha=0.7)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Plot: Actual vs Desired")
    ax.legend()
    plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, [entry[1][2] for entry in desired], label="Desired Z", color="green", linestyle="--")
    plt.plot(timestamps, [entry[1][2] for entry in actual], label="Actual Z", color="blue")
    plt.xlabel("Time")
    plt.ylabel("Z Value")
    plt.title("Desired vs Actual Z Over Time")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    data = cfusdlog.decode(DATA_PATH)
    take_off_index = find_take_off_index(data)
    result = optimize(data, take_off_index)
    actual_estimate = calculate_final_results(data, result.x, take_off_index)
    desired_estimate = fetch_desired_results(data, [entry[0] for entry in actual_estimate])
    plot_optimization_results(actual_estimate, desired_estimate)
