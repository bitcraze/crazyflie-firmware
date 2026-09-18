#!/usr/bin/env python3
"""Grid search for the distance + gate terrain method, tuned and validated on separate runs.

Run from tools/usdlog:
    PYTHONPATH=../../build uv run tune_terrain.py
"""

import itertools

import numpy as np

import replay_terrain as r

RUNS = "terrain_runs/20260918-{}.bin"
TUNE = {"flat climb": "140246_climb", "boxes 0.8": "141016_line", "box climb A": "144848_climb"}
VALIDATE = {"boxes 1.6": "143219_line", "box climb B": "145542_climb"}
SLOPE = {"slope": "143938_line"}

# Flow noise measured against Lighthouse: 0.47 px residual, the firmware assumes 2 * 0.1 = 0.2 px
FLOW_SCALES = (1.0, 2.35)


def load_all(runs: dict) -> dict:
    return {name: r.load(RUNS.format(run)) for name, run in runs.items()}


def evaluate(make_method, data: dict, flow_scale: float) -> dict:
    out = {}
    for name, (truth, events, start) in data.items():
        out[name] = r.score(r.replay(make_method(), events, start, flow_scale), truth)
    return out


def summary(scores: dict) -> tuple[float, float]:
    z = np.mean([s["z_rms"] for s in scores.values()]) * 100
    v = np.mean([s["v_rms"] for s in scores.values()]) * 100
    return z, v


def fmt(scores: dict) -> str:
    return "  ".join(f"{n}: z {s['z_rms'] * 100:4.1f} v {s['v_rms'] * 100:4.1f} r {s['resets']:3d}" for n, s in scores.items())


def main() -> None:
    tune, validate, slope = load_all(TUNE), load_all(VALIDATE), load_all(SLOPE)

    print("Reference methods, per flow noise scale (z and v RMS in cm, r = resets in flight)")
    refs = {"stock": r.Stock, "pr1431": r.PR1431, "fork": r.Fork, "fork+sigma": lambda: r.Fork(sensor_sigma=True)}
    for (name, make), fs in itertools.product(refs.items(), FLOW_SCALES):
        sc = evaluate(make, {**tune, **validate, **slope}, fs)
        print(f"  {name:<11} flow x{fs:<4g} {fmt(sc)}")

    grid = list(itertools.product(
        (0.1, 0.3, 0.5, 0.8),   # slope_std [m/m]
        (3.0, 5.0, 8.0, 12.0),  # gate [sigma]
        (1, 2, 3),              # confirm
        (False, True),          # sensor sigma
        FLOW_SCALES,
    ))
    print(f"\nGrid search on the tuning runs ({len(grid)} settings, skip7 on)...")
    results = []
    for s, gate, conf, sig, fs in grid:
        make = lambda: r.DistanceGate(s, gate, sig, confirm=conf, skip_bad_status=True)
        sc = evaluate(make, tune, fs)
        z, v = summary(sc)
        results.append((z + 0.25 * v, z, v, (s, gate, conf, sig, fs), sc))
    results.sort(key=lambda x: x[0])

    print("\nBest 8 on the tuning runs (objective = mean z RMS + 0.25 * mean v RMS), then scored on slope and held-out runs:")
    for obj, z, v, (s, gate, conf, sig, fs), sc in results[:8]:
        make = lambda: r.DistanceGate(s, gate, sig, confirm=conf, skip_bad_status=True)
        vz, vv = summary(evaluate(make, validate, fs))
        sz, sv = summary(evaluate(make, slope, fs))
        print(f"  s={s:<4g} gate={gate:<4g} conf={conf} sigma={str(sig):<5} flow x{fs:<4g} | "
              f"tune z {z:4.1f} v {v:4.1f} | slope z {sz:5.1f} v {sv:4.1f} | VALIDATE z {vz:4.1f} v {vv:4.1f}")
        print(f"      {fmt(sc)}")


if __name__ == "__main__":
    main()
