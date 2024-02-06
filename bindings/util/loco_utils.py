import cffirmware
import yaml

def read_loco_anchor_positions(file_name: str) -> dict[int, cffirmware.vec3_s]:
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
        for id, vals in data.items():
            point = cffirmware.vec3_s()
            point.x = vals['x']
            point.y = vals['y']
            point.z = vals['z']
            result[id] = point

    return result
