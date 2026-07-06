"""End-to-end smoke test of the replay seam through the real cffirmware
bindings: a synthetic, noise-free scenario must converge to the true position.

Requires the SWIG bindings: make bindings_python, then run with
PYTHONPATH=build.
"""

import math

import pytest

cffirmware = pytest.importorskip('cffirmware')

from bindings.util.tdoa_replay import replay

# 8 anchors on the corners of a 4 x 4 x 3 m box; tag stationary inside it
ANCHOR_COORDS = {
    0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0), 2: (0.0, 4.0, 0.0), 3: (4.0, 4.0, 0.0),
    4: (0.0, 0.0, 3.0), 5: (4.0, 0.0, 3.0), 6: (0.0, 4.0, 3.0), 7: (4.0, 4.0, 3.0),
}
TRUE_POS = (2.0, 2.0, 1.0)
DURATION_MS = 5000
POSITION_TOLERANCE_M = 0.5


def _anchor_positions():
    result = {}
    for anchor_id, (x, y, z) in ANCHOR_COORDS.items():
        p = cffirmware.vec3_s()
        p.x, p.y, p.z = x, y, z
        result[anchor_id] = p
    return result


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _synthetic_imu():
    # Stationary tag: 1 g on the z axis (acc is logged in g), zero rotation
    samples = []
    for t in range(DURATION_MS):
        ts = float(t)
        samples.append(('estAcceleration',
                        {'timestamp': ts, 'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0}))
        samples.append(('estGyroscope',
                        {'timestamp': ts, 'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0}))
    return samples


def _synthetic_tdoa():
    # 100 Hz, rotating over anchor pairs. Firmware sign convention
    # (see mm_tdoa.c): distanceDiff = d(idB) - d(idA).
    samples = []
    for k, t in enumerate(range(0, DURATION_MS, 10)):
        id_a = k % 8
        id_b = (k + 1) % 8
        dd = _dist(ANCHOR_COORDS[id_b], TRUE_POS) - _dist(ANCHOR_COORDS[id_a], TRUE_POS)
        samples.append(('estTDOA',
                        {'timestamp': float(t), 'idA': id_a, 'idB': id_b,
                         'distanceDiff': dd}))
    return samples


def test_replay_seam_converges_to_the_true_position():
    trajectory = replay(_anchor_positions(), _synthetic_imu(), _synthetic_tdoa(),
                        {'tdoa_std': 0.15})

    assert len(trajectory) > 4000
    times = [t for t, _ in trajectory]
    assert times == sorted(times)

    _, final_pos = trajectory[-1]
    assert _dist(final_pos, TRUE_POS) < POSITION_TOLERANCE_M
