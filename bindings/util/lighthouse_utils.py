# from __future__ import annotations
import cffirmware
import yaml


def load_lighthouse_calibration(file_path: str) -> tuple[dict[int, cffirmware.vec3_s], dict[int, dict[str, cffirmware.mat33]]]:
    """
    Load and parse lighthouse basestation calibration and geometry data from a YAML file.

    Args:
        file_path (str): Path to the YAML file containing calibration data.

    Returns:
        tuple: A tuple containing:
            - calibration_data (dict[int, dict[int, cffirmware.lighthouseCalibrationSweep_t]]):
                Calibration data for each basestation, mapped by ID and sweep index.
            - geometry_data (dict[int, dict[str, Union[cffirmware.vec3_s, cffirmware.mat33]]]):
                Geometry data for each basestation, mapped by ID, containing origin (vec3_s) and rotation matrix (mat33).
    """

    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

        calibration_data = {}
        for base_id, values in yaml_data['calibs'].items():
            sweeps = {}

            for index in range(2):
                sweep_data = values['sweeps'][index]
                sweep = cffirmware.lighthouseCalibrationSweep_t()

                sweep.phase = sweep_data['phase']
                sweep.tilt = sweep_data['tilt']
                sweep.curve = sweep_data['curve']
                sweep.gibmag = sweep_data['gibmag']
                sweep.gibphase = sweep_data['gibphase']
                sweep.ogeemag = sweep_data['ogeemag']
                sweep.ogeephase = sweep_data['ogeephase']

                sweeps[index] = sweep

            calibration_data[base_id] = sweeps

        geometry_data = {}
        for base_id, values in yaml_data['geos'].items():
            origin = cffirmware.vec3_s()
            origin.x, origin.y, origin.z = values['origin']

            rotation_matrix = cffirmware.mat33()
            rotation_values = values['rotation']
            rotation_matrix.i11, rotation_matrix.i12, rotation_matrix.i13 = rotation_values[0]
            rotation_matrix.i21, rotation_matrix.i22, rotation_matrix.i23 = rotation_values[1]
            rotation_matrix.i31, rotation_matrix.i32, rotation_matrix.i33 = rotation_values[2]

            geometry_data[base_id] = {'origin': origin, 'rotation_matrix': rotation_matrix}

    return calibration_data, geometry_data
