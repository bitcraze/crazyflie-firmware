"""Helper functions for plotting"""

import numpy as np


def loadFile(filename: str) -> dict:
    rawdata = np.loadtxt(filename, delimiter=",", skiprows=1, ndmin=2)
    data = {}
    time = rawdata[:, 0]
    data["time"] = time - time[0]  # make time start at 0
    data["thrust"] = rawdata[:, 1]  # load cell force in [N]
    data["pwm"] = rawdata[:, 2]
    data["vbat"] = rawdata[:, 3]  # in [V]
    data["rpm1"] = rawdata[:, 4]
    data["rpm2"] = rawdata[:, 5]
    data["rpm3"] = rawdata[:, 6]
    data["rpm4"] = rawdata[:, 7]
    data["rpm_avg"] = (data["rpm1"] + data["rpm2"] + data["rpm3"] + data["rpm4"]) / 4
    data["v"] = rawdata[:, 8]  # in [V]
    data["i"] = rawdata[:, 9]  # in [A]
    data["p"] = rawdata[:, 10]  # in [W]
    data["cmd"] = rawdata[:, 11]  # in [PWM]
    try:
        data["torque_x"] = rawdata[:, 12]  # in [N]
        data["torque_y"] = rawdata[:, 13]  # in [N]
        data["torque_z"] = rawdata[:, 14]  # in [N]
    except IndexError:
        print(f"Warning: {filename} does not contain torque data. Setting it to 0")
        data["torque_x"] = 0.0 * rawdata[:, 1]
        data["torque_y"] = 0.0 * rawdata[:, 1]
        data["torque_z"] = 0.0 * rawdata[:, 1]
    return data


def loadFiles(filenames: list) -> dict:
    data = loadFile(filenames[0])
    for i in range(1, len(filenames)):
        data_new = loadFile(filenames[i])
        # append data
        for k, v in data_new.items():
            data[k] = np.append(data[k], v)
    return data


def cutData(data: dict, tStart: float | None = None, tEnd: float | None = None):
    if tEnd is not None:
        tEnd = data["time"][-1] + tEnd if tEnd < 0 else tEnd
        end_idx = np.searchsorted(data["time"], tEnd, side="left") - 1
    else:
        end_idx = -1

    if tStart is not None:
        start_idx = np.searchsorted(data["time"], tStart, side="right")
        data["time"] = data["time"] - tStart
    else:
        start_idx = 0

    for k, v in data.items():
        data[k] = v[start_idx:end_idx]

    return data


def poly(x, p, order):
    y = 0
    for i in range(order + 1):
        y += p[i] * x**i
    return y


def inversepoly(y, param, order):
    # index of param = order, i.e. y = p[i] * x**i
    assert len(param) == order + 1
    if order == 2:
        return (-param[1] + np.sqrt(param[1] ** 2 - 4 * param[2] * (param[0] - y))) / (
            2 * param[2]
        )
    elif order == 3:
        # # https://math.vanderbilt.edu/schectex/courses/cubic/
        # # a = p[3], b = p[2], c = p[1], d = p[0]-thrust
        # p = -param[2] / (3 * param[3])
        # q = p**3 + (param[2] * param[1] - 3 * param[3] * (param[0] - y)) / (
        #     6 * param[3] ** 2
        # )
        # r = param[1] / (3 * param[3])

        # qrp = np.sqrt(q**2 + (r - p**2) ** 3)
        # return np.cbrt(q + qrp) + np.cbrt(q - qrp) + p
        sol = []
        y = np.atleast_1d(y)
        for yi in y:
            # Cubic solved via numpy.roots
            coeffs = [param[3], param[2], param[1], param[0] - yi]
            roots = np.roots(coeffs)
            real_roots = roots[np.isreal(roots)].real
            # for our fits, the solution is always the largest root
            sol.append(float(np.max(real_roots)))
        return np.array(sol)
    else:
        raise NotImplementedError(
            f"Inverted polynomial of order {order} not supported."
        )


def convert_parameters(obj):
    """Recursively convert:
    - NumPy arrays -> lists
    - NumPy floats/ints -> Python floats/ints
    Works for nested dicts and lists.
    """
    if isinstance(obj, dict):
        return {k: convert_parameters(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_parameters(x) for x in obj]
    elif isinstance(obj, np.ndarray):
        return convert_parameters(obj.tolist())
    elif isinstance(obj, (np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, (np.int32, np.int64)):
        return int(obj)
    else:
        return obj
