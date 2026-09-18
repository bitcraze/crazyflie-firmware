#!/usr/bin/env python3
"""Replay terrain logs through the firmware Kalman core, with different ways of handling terrain.

The logs are recorded with the rik/terrain-logging firmware: ToF and flow are logged but not
fused, so the Lighthouse estimate in the log is the ground truth. The replay runs the Kalman core
as a flow deck only drone (IMU + ToF + flow, no Lighthouse) and scores each method against it.

The bindings must be built with the terrain state (bindings/setup.py defines KALMAN_TERRAIN_STATE):
    cd <repo> && uv run --project tools/usdlog --with setuptools python bindings/setup.py build_ext --inplace
Run from tools/usdlog:
    PYTHONPATH=../../build uv run replay_terrain.py terrain_runs/20260918-141016_line.bin
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import tyro

import cffirmware as c
import cfusdlog

DEG_TO_RAD = math.pi / 180.0
GRAVITY = 9.81
PREDICT_STEP_MS = 10  # 100 Hz, as PREDICT_RATE in estimator_kalman.c

# Flow model constants, as in mm_flow.c
NPIX = 35.0
THETAPIX = 0.71674
FLOW_RESOLUTION = 0.10

Z, PX, PY, T = c.KC_STATE_Z, c.KC_STATE_PX, c.KC_STATE_PY, c.KC_STATE_T


# ---------------------------------------------------------------------------------------------
# Measurement models in Python, so the terrain state can be included


def tof_cos_angle(core) -> Optional[float]:
    """cos of the ToF angle as in mm_tof.c, None when the measurement is not used"""
    r22 = c.kcGetR(core, 2, 2)
    if not (abs(r22) > 0.1 and r22 > 0):
        return None
    angle = max(abs(math.acos(min(r22, 1.0))) - DEG_TO_RAD * 7.5, 0.0)
    return math.cos(angle)


def tof_update_terrain(core, distance: float, std: float, cos_a: float, terrain: bool = True) -> None:
    """ToF model of mm_tof.c with the distance measured to the terrain: d = (z - t) / cos"""
    if terrain:
        error = distance - (c.kcGetS(core, Z) - c.kcGetS(core, T)) / cos_a
        c.kcScalarUpdate(core, Z, 1.0 / cos_a, T, -1.0 / cos_a, -1, 0.0, error, std)
    else:
        error = distance - c.kcGetS(core, Z) / cos_a
        c.kcScalarUpdate(core, Z, 1.0 / cos_a, -1, 0.0, -1, 0.0, error, std)


def innovation_variance(core, hz: float, ht: float, std: float) -> float:
    p_zz, p_zt, p_tt = c.kcGetP(core, Z, Z), c.kcGetP(core, Z, T), c.kcGetP(core, T, T)
    return hz * hz * p_zz + 2 * hz * ht * p_zt + ht * ht * p_tt + std * std


def flow_update(core, m: dict, gyro, terrain: bool, use_hz: bool) -> None:
    """Flow model of mm_flow.c (flow deck at the centre of mass), height above terrain if terrain is set"""
    omega_x = gyro.x * DEG_TO_RAD
    omega_y = gyro.y * DEG_TO_RAD
    dx_b, dy_b = c.kcGetS(core, PX), c.kcGetS(core, PY)
    z_g = c.kcGetS(core, Z) - (c.kcGetS(core, T) if terrain else 0.0)
    z_g = max(z_g, 0.1)
    r22 = c.kcGetR(core, 2, 2)
    k = m["dt"] * NPIX / THETAPIX

    for v_b, omega, sign, idx, pix, std in ((dx_b, omega_y, -1.0, PX, m["dpixelx"], m["stdDevX"]),
                                             (dy_b, omega_x, +1.0, PY, m["dpixely"], m["stdDevY"])):
        predicted = k * (v_b * r22 / z_g + sign * omega)
        measured = pix * FLOW_RESOLUTION
        h_v = k * r22 / z_g
        h_z = k * r22 * v_b / (-z_g * z_g) if use_hz else 0.0
        h_t = -h_z if terrain else 0.0
        c.kcScalarUpdate(core, idx, h_v, Z if use_hz else -1, h_z, T if (terrain and use_hz) else -1, h_t,
                         measured - predicted, std * FLOW_RESOLUTION)


# ---------------------------------------------------------------------------------------------
# Terrain handling methods


class Method:
    name = "?"

    def __init__(self):
        self.resets: list[float] = []  # times of terrain resets/jumps [ms]

    def init(self, core) -> None:
        pass

    def step(self, core, dt_s: float, now_ms: int) -> None:
        """Called every loop iteration, after the process noise"""

    def tof(self, core, m: dict, now_ms: int) -> None:
        raise NotImplementedError

    def flow(self, core, m: dict, gyro, now_ms: int) -> None:
        raise NotImplementedError


class Stock(Method):
    """Firmware as it is: height is height above whatever the ToF sees"""
    name = "stock"

    def tof(self, core, m, now_ms):
        tof = c.tofMeasurement_t()
        tof.distance, tof.stdDev = m["distance"], m["stdDev"]
        c.kalmanCoreUpdateWithTof(core, tof)

    def flow(self, core, m, gyro, now_ms):
        f = c.flowMeasurement_t()
        f.dt, f.dpixelx, f.dpixely, f.stdDevX, f.stdDevY = m["dt"], m["dpixelx"], m["dpixely"], m["stdDevX"], m["stdDevY"]
        c.kalmanCoreUpdateWithFlow(core, f, gyro)


class StockPython(Stock):
    """Python copy of the stock models, with the terrain fixed at 0. Must match Stock."""
    name = "stock-py"

    def tof(self, core, m, now_ms):
        cos_a = tof_cos_angle(core)
        if cos_a is not None:
            tof_update_terrain(core, m["distance"], m["stdDev"], cos_a, terrain=False)

    def flow(self, core, m, gyro, now_ms):
        flow_update(core, m, gyro, terrain=False, use_hz=True)


class PR1431(Stock):
    """#1431: terrain offset outside the filter, jump when |predicted - measured elevation| > 4 cm"""
    name = "pr1431"

    def __init__(self, threshold: float = 0.04):
        super().__init__()
        self.threshold_after_first = threshold
        self.threshold = math.inf
        self.terrain = 0.0

    def tof(self, core, m, now_ms):
        cos_a = tof_cos_angle(core)
        if cos_a is None:
            return
        predicted_elevation = c.kcGetS(core, Z)
        measured_elevation = m["distance"] * cos_a + self.terrain
        if abs(predicted_elevation - measured_elevation) > self.threshold:
            self.terrain += predicted_elevation - measured_elevation
            self.resets.append(now_ms)
        self.threshold = self.threshold_after_first
        residual = m["distance"] - predicted_elevation / cos_a + self.terrain
        c.kcScalarUpdate(core, Z, 1.0 / cos_a, -1, 0.0, -1, 0.0, residual, m["stdDev"])
    # flow: stock, uses Z as height above the surface


class Fork(Method):
    """finnBsch fork: floor state F without process noise, reset when |error| > 10 * stdDev"""
    name = "fork"

    def __init__(self, factor: float = 10.0, variance_after_detection: float = 50.0, sensor_sigma: bool = False):
        super().__init__()
        self.factor = factor
        self.variance_after_detection = variance_after_detection
        self.sensor_sigma = sensor_sigma
        if sensor_sigma:
            self.name = "fork+sigma"

    def tof(self, core, m, now_ms):
        r22 = c.kcGetR(core, 2, 2)
        if not (abs(r22) > 0.1 and r22 > 0):
            return
        std = m["zranger2.sigma"] / 1000.0 if self.sensor_sigma else m["stdDev"]
        error = m["distance"] - (c.kcGetS(core, Z) - c.kcGetS(core, T)) / r22
        if error * error > (self.factor * std) ** 2:
            c.kcSetP(core, T, T, self.variance_after_detection)
            c.kcSetS(core, T, c.kcGetS(core, Z) - m["distance"] * r22)
            error = 0.0
            self.resets.append(now_ms)
        c.kcScalarUpdate(core, Z, 1.0 / r22, T, -1.0 / r22, -1, 0.0, error, std)

    def flow(self, core, m, gyro, now_ms):
        flow_update(core, m, gyro, terrain=True, use_hz=False)


class RandomWalk(Method):
    """Terrain state as a random walk in time (rik/kalman_terrain, with the noise scaled by dt)"""

    def __init__(self, q: float = 0.1):
        super().__init__()
        self.q = q  # [m/sqrt(s)]
        self.name = f"randwalk q={q:g}"

    def init(self, core):
        c.kcSetP(core, T, T, 0.01 ** 2)

    def step(self, core, dt_s, now_ms):
        c.kcSetP(core, T, T, c.kcGetP(core, T, T) + self.q * self.q * dt_s)

    def tof(self, core, m, now_ms):
        cos_a = tof_cos_angle(core)
        if cos_a is not None:
            tof_update_terrain(core, m["distance"], m["stdDev"], cos_a)

    def flow(self, core, m, gyro, now_ms):
        flow_update(core, m, gyro, terrain=True, use_hz=True)


class DistanceGate(Method):
    """Terrain noise per metre flown (slopes) plus a jump reset on a proper innovation gate (steps)"""

    def __init__(self, slope_std: float = 0.1, gate_sigma: float = 5.0, sensor_sigma: bool = False,
                 reset_variance: float = 1.0, confirm: int = 1, skip_bad_status: bool = False):
        super().__init__()
        self.slope_std = slope_std  # expected terrain slope [m/m]
        self.gate_sigma = gate_sigma
        self.sensor_sigma = sensor_sigma
        self.reset_variance = reset_variance
        self.confirm = confirm  # consecutive readings outside the gate before a reset
        self.skip_bad_status = skip_bad_status
        self.outside = 0
        self.name = (f"dist s={slope_std:g} gate={gate_sigma:g}" + ("+sigma" if sensor_sigma else "")
                     + (f" conf={confirm}" if confirm > 1 else "") + (" skip7" if skip_bad_status else ""))

    def init(self, core):
        c.kcSetP(core, T, T, 0.01 ** 2)

    def step(self, core, dt_s, now_ms):
        speed = math.hypot(c.kcGetS(core, PX), c.kcGetS(core, PY))
        c.kcSetP(core, T, T, c.kcGetP(core, T, T) + (self.slope_std * speed * dt_s) ** 2)

    def tof(self, core, m, now_ms):
        cos_a = tof_cos_angle(core)
        if cos_a is None:
            return
        if self.skip_bad_status and m["zranger2.status"] != 0:
            return  # e.g. status 7 (wrap target fail): the cone covers an edge, the range is in between
        std = max(m["zranger2.sigma"] / 1000.0, 0.001) if self.sensor_sigma else m["stdDev"]
        error = m["distance"] - (c.kcGetS(core, Z) - c.kcGetS(core, T)) / cos_a
        s = innovation_variance(core, 1.0 / cos_a, -1.0 / cos_a, std)
        if error * error > self.gate_sigma ** 2 * s:
            self.outside += 1
            if self.outside < self.confirm:
                return  # could be a single edge reading, wait for the next one
            # A step in the terrain: move it all into T, and let the next measurements refine it
            c.kcSetS(core, T, c.kcGetS(core, Z) - m["distance"] * cos_a)
            c.kcSetP(core, T, T, self.reset_variance)
            self.resets.append(now_ms)
        self.outside = 0
        tof_update_terrain(core, m["distance"], std, cos_a)

    def flow(self, core, m, gyro, now_ms):
        flow_update(core, m, gyro, terrain=True, use_hz=True)


# ---------------------------------------------------------------------------------------------
# Replay


@dataclass
class Truth:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    flying: np.ndarray
    good: np.ndarray  # Lighthouse estimate trustworthy


def load(path: str):
    log = cfusdlog.decode(path)
    ff = log["fixedFrequency"]
    truth = Truth(
        t=ff["timestamp"], x=ff["stateEstimate.x"], y=ff["stateEstimate.y"], z=ff["stateEstimate.z"],
        vx=ff["stateEstimate.vx"], vy=ff["stateEstimate.vy"],
        flying=(ff["supervisor.info"].astype(int) & 0x10) != 0,
        good=ff["kalman.varZ"] < 1e-3,
    )
    events = []
    for name in ("estAcceleration", "estGyroscope", "estTOF", "estFlow", "estBarometer", "fixedFrequency"):
        data = log[name]
        keys = list(data.keys())
        for i in range(len(data["timestamp"])):
            events.append((data["timestamp"][i], name, {k: float(data[k][i]) for k in keys}))
    events.sort(key=lambda e: e[0])
    q = [ff[f"stateEstimate.q{a}"][0] for a in "xyzw"]
    yaw0 = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
    return truth, events, (truth.x[0], truth.y[0], truth.z[0], yaw0)


def replay(method: Method, events, start, flow_std_scale: float = 1.0, use_baro: bool = False,
           param_overrides: Optional[dict] = None) -> dict:
    params = c.kalmanCoreParams_t()
    c.kalmanCoreDefaultParams(params)
    for key, value in (param_overrides or {}).items():
        setattr(params, key, value)
    params.attitudeReversion = 0.0  # the flow deck turns it off, see estimatorKalmanInit()
    params.initialX, params.initialY, params.initialZ, params.initialYaw = start

    core = c.kalmanCoreData_t()
    acc_sub, gyro_sub = c.Axis3fSubSampler_t(), c.Axis3fSubSampler_t()
    c.axis3fSubSamplerInit(acc_sub, GRAVITY)
    c.axis3fSubSamplerInit(gyro_sub, DEG_TO_RAD)
    gyro_latest = c.Axis3f()
    acc_latest = c.Axis3f()

    now = int(events[0][0])
    c.kalmanCoreInit(core, params, now)
    method.init(core)
    next_predict = now + PREDICT_STEP_MS
    flying = False
    i = 0
    out_t, out = [], []
    external = c.state_t()

    while i < len(events):
        if now >= next_predict:
            c.axis3fSubSamplerFinalize(acc_sub)
            c.axis3fSubSamplerFinalize(gyro_sub)
            c.kalmanCorePredict(core, params, acc_sub.subSample, gyro_sub.subSample, now, flying)
            next_predict += PREDICT_STEP_MS
        c.kalmanCoreAddProcessNoise(core, params, now)
        method.step(core, 0.001, now)

        while i < len(events) and events[i][0] <= now:
            _, name, d = events[i]
            i += 1
            if name == "estAcceleration":
                acc_latest.x, acc_latest.y, acc_latest.z = d["acc.x"], d["acc.y"], d["acc.z"]
                c.axis3fSubSamplerAccumulate(acc_sub, acc_latest)
            elif name == "estGyroscope":
                gyro_latest.x, gyro_latest.y, gyro_latest.z = d["gyro.x"], d["gyro.y"], d["gyro.z"]
                c.axis3fSubSamplerAccumulate(gyro_sub, gyro_latest)
            elif name == "estTOF":
                method.tof(core, d, now)
            elif name == "estFlow":
                if flow_std_scale != 1.0:
                    d = dict(d, stdDevX=d["stdDevX"] * flow_std_scale, stdDevY=d["stdDevY"] * flow_std_scale)
                method.flow(core, d, gyro_latest, now)
            elif name == "estBarometer":
                if use_baro:
                    c.kalmanCoreUpdateWithBaro(core, params, d["baro.asl"], flying)
            elif name == "fixedFrequency":
                flying = (int(d["supervisor.info"]) & 0x10) != 0

        c.kalmanCoreFinalize(core)
        if now % 10 == 0:
            c.kalmanCoreExternalizeState(core, external, acc_latest)
            out_t.append(now)
            out.append((external.position.x, external.position.y, external.position.z,
                        external.velocity.x, external.velocity.y, c.kcGetS(core, T)))
        now += 1

    a = np.array(out)
    return {"t": np.array(out_t), "x": a[:, 0], "y": a[:, 1], "z": a[:, 2], "vx": a[:, 3], "vy": a[:, 4],
            "terrain": a[:, 5], "resets": list(method.resets)}


def score(est: dict, truth: Truth) -> dict:
    m = truth.flying & truth.good
    t = truth.t[m]
    ez = np.interp(t, est["t"], est["z"]) - truth.z[m]
    evx = np.interp(t, est["t"], est["vx"]) - truth.vx[m]
    evy = np.interp(t, est["t"], est["vy"]) - truth.vy[m]
    exy = np.hypot(np.interp(t, est["t"], est["x"]) - truth.x[m], np.interp(t, est["t"], est["y"]) - truth.y[m])
    flying_t = truth.t[truth.flying]
    resets_in_flight = [r for r in est["resets"] if flying_t.size and flying_t[0] <= r <= flying_t[-1]]
    return {
        "z_rms": float(np.sqrt(np.mean(ez ** 2))), "z_max": float(np.max(np.abs(ez))),
        "v_rms": float(np.sqrt(np.mean(evx ** 2 + evy ** 2))),
        "xy_end": float(exy[-1]), "resets": len(resets_in_flight),
    }


def default_methods() -> list[Method]:
    return [
        Stock(), StockPython(), PR1431(), Fork(), Fork(sensor_sigma=True),
        RandomWalk(0.05), RandomWalk(0.2),
        DistanceGate(0.1, 5.0), DistanceGate(0.1, 5.0, sensor_sigma=True), DistanceGate(0.3, 5.0, sensor_sigma=True),
    ]


@dataclass
class Args:
    logs: tyro.conf.Positional[list[str]]
    """Log files to replay"""
    save: Optional[str] = None
    """Save all estimates to this .npz file for plotting"""


def main() -> None:
    args = tyro.cli(Args)
    saved = {}
    for path in args.logs:
        truth, events, start = load(path)
        print(f"\n{path}  ({(truth.t[-1] - truth.t[0]) / 1000:.0f} s, flying {truth.flying.mean() * 100:.0f}%)")
        print(f"  {'method':<28} {'z rms':>7} {'z max':>7} {'v rms':>7} {'xy end':>7} {'resets':>7}")
        for method in default_methods():
            est = replay(method, events, start)
            s = score(est, truth)
            print(f"  {method.name:<28} {s['z_rms'] * 100:6.1f}c {s['z_max'] * 100:6.1f}c "
                  f"{s['v_rms'] * 100:6.1f}c {s['xy_end'] * 100:6.0f}c {s['resets']:7d}")
            saved[f"{path}|{method.name}"] = est
    if args.save:
        np.savez_compressed(args.save, **{k.replace("/", "_"): np.array([v["t"], v["x"], v["y"], v["z"], v["vx"], v["vy"], v["terrain"]])
                                          for k, v in saved.items()})
        print(f"\nSaved estimates to {args.save}")


if __name__ == "__main__":
    main()
