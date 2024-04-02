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

            results_calib[id] = lhCalibration

            cffirmware.print_sweeps(lhCalibration)

        print(results_calib)

        data_geo = data['geos']
        results_geo = {}
        for id, vals in data_geo.items():
            basestation_geo = cffirmware.baseStationGeometry_t()
            origin = cffirmware.vec3_s()
            origin.x = vals['origin'][0]
            origin.y = vals['origin'][1]
            origin.z = vals['origin'][2]
            mat1 = cffirmware.vec3_s()
            mat1.x = vals['rotation'][0][0]
            mat1.y = vals['rotation'][0][1]
            mat1.z = vals['rotation'][0][2]
            mat2 = cffirmware.vec3_s()
            mat2.x = vals['rotation'][1][0]
            mat2.y = vals['rotation'][1][1]
            mat2.z = vals['rotation'][1][2]
            mat3 = cffirmware.vec3_s()
            mat3.x = vals['rotation'][2][0]
            mat3.y = vals['rotation'][2][1]
            mat3.z = vals['rotation'][2][2]

            cffirmware.set_origin_mat(basestation_geo, origin, mat1, mat2, mat3)


    return result