"""Tests for the mm_tdoa bindings seam: innovation + unfiltered update.

Requires the SWIG bindings: make bindings_python, run with PYTHONPATH=build.
"""

import math

import pytest

cffirmware = pytest.importorskip('cffirmware')


def _point(x, y, z):
    # point_t is a typedef of struct vec3_s; vec3_s is the name the bindings
    # expose (same pattern as test_tdoa_replay_smoke.py / loco_utils.py).
    p = cffirmware.vec3_s()
    p.x, p.y, p.z = x, y, z
    return p


def _tdoa(pa, pb, dd, std=0.15):
    t = cffirmware.tdoaMeasurement_t()
    t.anchorIdA = 0
    t.anchorIdB = 1
    t.anchorPositionA = _point(*pa)
    t.anchorPositionB = _point(*pb)
    t.distanceDiff = dd
    t.stdDev = std
    return t


def _init_core():
    core = cffirmware.kalmanCoreData_t()
    params = cffirmware.kalmanCoreParams_t()
    cffirmware.kalmanCoreDefaultParams(params)
    cffirmware.kalmanCoreInit(core, params, 0)
    return core


def _position(core):
    state = cffirmware.state_t()
    acc = cffirmware.Axis3f()
    cffirmware.kalmanCoreExternalizeState(core, state, acc)
    return (state.position.x, state.position.y, state.position.z)


def test_innovation_is_measured_minus_predicted():
    # Tag at origin (default init), anchors symmetric at +-1 m on x:
    # dA == dB, so predicted == 0 and the innovation equals distanceDiff.
    core = _init_core()
    tdoa = _tdoa(pa=(1.0, 0.0, 0.0), pb=(-1.0, 0.0, 0.0), dd=0.25)
    error = cffirmware.kalmanCoreTdoaInnovation(core, tdoa)
    assert error == pytest.approx(0.25, abs=1e-6)


def test_innovation_is_nan_when_tag_is_at_an_anchor():
    # Anchor A at the origin == the initial tag position, so dA == 0
    # (degenerate). Firmware skips these samples; the seam reports NaN.
    core = _init_core()
    tdoa = _tdoa(pa=(0.0, 0.0, 0.0), pb=(2.0, 0.0, 0.0), dd=0.5)
    assert math.isnan(cffirmware.kalmanCoreTdoaInnovation(core, tdoa))


def test_unfiltered_update_moves_the_state():
    core = _init_core()
    before = _position(core)
    tdoa = _tdoa(pa=(1.0, 0.0, 0.0), pb=(-1.0, 0.0, 0.0), dd=0.25)
    cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(core, tdoa)
    assert _position(core) != before


def test_unfiltered_update_is_a_noop_when_degenerate():
    core = _init_core()
    before = _position(core)
    tdoa = _tdoa(pa=(0.0, 0.0, 0.0), pb=(2.0, 0.0, 0.0), dd=0.5)
    cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(core, tdoa)
    assert _position(core) == before
