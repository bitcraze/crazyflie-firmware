#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.sd_card_file_runner import SdCardFileRunner
import tools.usdlog.cfusdlog as cfusdlog

DATA_PATH = './log07'

def interpolate_desired(timestamps_actual, timestamps_desired, desired_z):
    """Interpolates the desired z-values to match the actual timestamps."""
    interpolator = interp1d(timestamps_desired, desired_z, kind='linear', bounds_error=False, fill_value='extrapolate')
    return interpolator(timestamps_actual)

def objective_function(params, data_path, timestamps_desired, desired_z):
    """Objective function to minimize: computes the error between desired and actual z."""
    runner = SdCardFileRunner(data_path)
    emulator = EstimatorKalmanEmulator()
    emulator.set_parameters(params)  # Assume this method exists to update Kalman parameters

    # Run the emulator
    actual = runner.run_estimator_loop(emulator)

    # Unpack timestamps and positions
    timestamps_actual = [entry[0] for entry in actual]  # Extract timestamps
    actual_z = [entry[1][2] for entry in actual]       # Extract z-values from positions

    # Interpolate desired z to match actual timestamps
    desired_z_interp = interpolate_desired(timestamps_actual, timestamps_desired, desired_z)
    
    # Compute mean squared error
    error = np.mean((np.array(actual_z) - desired_z_interp) ** 2)
    print(error)
    return error

def do_it():
    data = cfusdlog.decode(DATA_PATH)
    
    timestamps_desired = data['fixedFrequency']['timestamp']
    desired_z = data['fixedFrequency']['stateEstimate.z']

    # Initial parameters (e.g., process noise, measurement noise, initial uncertainties)
    initial_params = np.array([1.0, 1.0, 1.0])  # Example initial guesses

    # Define bounds for parameters, if applicable
    bounds = [(0.1, 10.0), (0.1, 10.0), (0.1, 10.0)]  # Example bounds

    # Optimize Kalman filter parameters
    result = minimize(
        objective_function,
        initial_params,
        args=(DATA_PATH, timestamps_desired, desired_z),
        # bounds=bounds,
        method='L-BFGS-B',
        options={'disp': True}
    )

    # Output optimized parameters
    print("Optimized Parameters:", result.x)
    print("Optimization Success:", result.success)

if __name__ == '__main__':
    do_it()
