from __future__ import annotations
import cffirmware
import yaml
import ctypes


def read_lh_basestation_positions_calibration(file_name: str) -> dict[int, cffirmware.vec3_s]:
    """
    Read anchor position data from a file exported from the client.

    Args:
        file_name (str): The name of the file

    Returns:
        dict[int, cffirmware.vec3_s]: A dictionary from anchor id to a 3D-vector
    """
    result = {}

    with open(file_name, 'r') as file:
        data = yaml.safe_load(file)
        data_calib = data['calibs']
        results_calib = {}
        for id, vals in data_calib.items():

            lhCalibration = cffirmware.lighthouseCalibration_t()
            ptr = lhCalibration.sweep
            sweep = cffirmware.sweepAngleMeasurement_t()
            lhCalibration.sweep =
            print(vals['sweeps'][0])
            print(lhCalibration.sweep)# = vals['sweeps'][0]['curve']
            #results_calib[id] = lhCalbiration

            #point.x = vals['x']
            #point.y = vals['y']
            #point.z = vals['z']
            #result[id] = point

    return result