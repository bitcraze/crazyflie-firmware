# TDoA replay tuning experiments — findings

Offline exploration (branch `tdoa-experiments`) of selection policies, outlier
filters and measurement-noise settings on captured candidate data, using the
replay seam and scored against Lighthouse ground truth. Everything below comes
from **one** hand-moved ~64 s log (`run_validation.bin`, 16121 packets,
84398 candidate pairs, avg 5.2 candidates/packet) — treat the numbers as one
data point and the *directions* as the result. Re-run on more logs (and real
flights) before adopting anything in firmware.

## Method

`tools/usdlog/tdoa_experiment.py` replays a grid of
(policy, filter, model, std) configs through the real firmware Kalman core and
scores each trajectory against ground truth **separately on a train segment
(first 60 % of the ground-truth span) and a validation segment (the rest)**.
All tuning decisions were made on the train columns; the validation columns
decided the final ranking. The shortlist was additionally re-checked with the
split at 40 % and 75 % to confirm ranking stability. Metrics are tails-first
(fly-away fraction above 0.3 m, p95, max) with rms as secondary. The first
5 s after ground truth starts are excluded (cold-start transient, identical
for every config).

Reproduce (repo root):

```
pixi run python -m tools.usdlog.tdoa_experiment run_validation.bin \
    --anchors anchors.yaml --grid grid.yaml --out results.csv
```

with `grid.yaml` a YAML list like
`- {policy: all, filter: pair_hampel, std: 0.04, filter_params: {WINDOW: 16, WARMUP: 6}}`.

## Ground-truth diagnostics of the raw candidate stream

(Script: offline residuals of every candidate vs Lighthouse-implied
distanceDiff.)

- Typical candidate noise: median |residual| ≈ 0.15 m, MAD ≈ 0.09 m.
- 8.7 % of candidates are outliers (>0.5 m); 68 % of those are near-misses
  (0.5–1 m), 14 % are gross (≥5 m).
- Outlier bursts are **anchor-specific**: one 5 s stretch (68.8–73.8 s) has
  27 % outliers, driven by anchors 0 (46 % bad) and 4 (38 % bad).
- Only 0.7 % of packets are entirely bad — in 99.3 % of packets at least one
  candidate is good, and 29 % of packets are a good/bad mix.

## Results (train 60 % / validate 40 %)

| config | tr fly>0.3 | tr p95 | va fly>0.3 | va p95 | va max | va rms |
|---|---|---|---|---|---|---|
| baseline/integrator, std 0.15 (= live firmware) | 55.1 % | 1.42 | 55.5 % | 1.39 | 1.70 | 0.72 |
| all/integrator, std 0.15 | 45.2 % | 1.00 | 47.5 % | 1.61 | 3.58 | 0.70 |
| all/mad_window, std 0.04 | 18.1 % | 0.44 | 23.6 % | 0.67 | 1.26 | 0.34 |
| all/pair_hampel, std 0.04 | 21.0 % | 0.52 | 21.2 % | 0.60 | 1.13 | 0.31 |
| **all/pair_hampel W16/WU6, std 0.04** | 19.0 % | 0.46 | **18.7 %** | **0.59** | **1.14** | **0.31** |

The winner also ranks first (or statistically tied first) with the split at
40 % and 75 %, and beats the live baseline by ~3× on every validation metric
in every split.

## What worked

1. **Feed all candidates (`all` policy) instead of selecting one.** The single
   biggest win. Redundancy (5.2 measurements/packet) lets the EKF average out
   near-miss outliers; every subset-selection variant tried (`median`,
   `top_k`, `trimmed_all`, `round_robin`) was worse on validation.
2. **A measurement-domain gate: `pair_hampel`.** Per anchor pair, reject a
   distanceDiff further than K·MAD from the pair's recent median (window 16,
   warmup 6, K 5, MAD floored at 0.5·std, windows expire after 2 s).
   It generalized better than the innovation-domain `mad_window` (which had
   better *train* scores — the classic overfit signature) and, because it
   never looks at the Kalman state, it cannot self-confirm a diverged
   estimate. Firmware cost: N_pairs × (16 floats + timestamp).
3. **Much lower measurement std: ≈0.04 m instead of 0.15 m** (with the `all`
   policy). Validation improved monotonically down to ~0.03–0.05 and
   degraded below that. Caveat: this is ~4× more confident than the true
   per-candidate noise (~0.15 m) and partly compensates the correlated-
   measurement double-counting of feeding 5 same-packet updates — it is the
   most likely setting to *not* transfer to other conditions. Validate on
   more data before trusting it.

## What did not work (all worse on validation, some catastrophically)

- **The robust M-estimator model (`kalman.robustTdoa=1` path)**: diverged
  (>100 m) on this log under every policy.
- **Soft de-weighting in general** (Huber-style std inflation, capped or not;
  group-spread-scaled std): once the state drifts, everything that disagrees
  gets de-weighted and nothing pulls the state back. Hard gates with a
  data-driven reopen mechanism (Hampel window refill, integrator force-open)
  were strictly better.
- **The global C integrator filter with the `all` policy**: unstable
  (validation max 3.6 m; diverges at std ≥ 0.3).
- **Candidate trimming** (`trimmed_all`, `top_k`): always worse than `all` —
  the EKF handles near-misses better than a consensus trim does.
- **`anchor_quality`** (per-anchor rejection-rate EWMA on top of
  `pair_hampel`, targeting the anchor-specific bursts): only marginal gains
  (val max 1.13→1.11 m) — the per-pair gate already absorbs most of the
  anchor-0/4 burst. Kept in the roster; may matter more on logs with harder
  anchor failures.

## Is the low std really underestimated process noise?

Follow-up experiment (grid over `kalman_params` overrides, same
train/validate protocol): what the low std mostly buys is **gain**, i.e. it
compensates a process noise that understates how bad the prediction is while
the drone is hand-carried. Evidence:

- Raising the accelerometer process noise ~10× (`procNoiseAcc_xy` 0.5→5,
  `procNoiseAcc_z` 1→10) at the honest std 0.15 recovers most of the low-std
  gain: validation fly-away 40.1 % → 22.3 % (vs 18.7 % for std 0.04), p95
  0.895 → 0.622 m (vs 0.590), rms 0.446 → 0.319 (vs 0.310). Stable across
  alternate splits.
- Q and R are substitutes, not complements: raising Q *on top of* std 0.04
  makes everything worse (the filter starts chasing measurement noise).
- Scaling each measurement's std by sqrt(n_cand) (packet-correlation
  compensation, `std_model: ncand`) at base std 0.02 (effective ≈ 0.046)
  performs the same as plain std 0.04 — consistent with "total per-packet
  weight is what matters", not the per-measurement value.

Interpretation: std 0.04 is not a better noise estimate (true per-candidate
noise is ~0.15 m) — it is a cheap way of restoring tracking bandwidth. The
boosted-Q variant at std 0.15 is only marginally worse in raw error and keeps
R honest, which should make the covariance (and everything that reads it:
innovation gates, supervisor) far less overconfident. For *flight* (better
prediction than hand-carry) the right Q is probably lower; both knobs need
retuning on flight logs.

## Firmware implications (if confirmed on more data)

Replace "select one pair per packet + global innovation integrator" with
"enqueue **all** valid candidate pairs + per-pair Hampel gate on
distanceDiff", and drop TDOA_ENGINE_MEASUREMENT_NOISE_STD to ~0.05.
Everything is causal, O(anchors²) small state, and integer/float-cheap. Open
questions: CPU load of ~5× more Kalman updates (each TDoA update is a scalar
update — likely fine), and whether the low std holds up when the drone flies
(IMU quality, vibration) rather than being hand-carried.
