from __future__ import annotations
import cffirmware
import yaml
import ctypes


def read_lh_basestation_pose_calibration(file_name: str) -> dict[int, cffirmware.vec3_s]:
    """
    Read basestation calibration and position data from a file exported from the client.

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

            for i in range(0, 2):
                data_calib_sweep = vals['sweeps'][i]
                lhSweep = cffirmware.lighthouseCalibrationSweep_t()
                lhSweep.phase = data_calib_sweep['phase']
                lhSweep.tilt = data_calib_sweep['tilt']
                lhSweep.curve = data_calib_sweep['curve']
                lhSweep.gibmag = data_calib_sweep['gibmag']
                lhSweep.gibphase = data_calib_sweep['gibphase']
                lhSweep.ogeemag = data_calib_sweep['ogeemag']
                lhSweep.ogeephase = data_calib_sweep['ogeephase']
                cffirmware.set_sweep(lhCalibration, lhSweep, i)

            lhCalibration.uid = vals['uid']

            cffirmware.print_sweeps(lhCalibration)


    return result