# tdoa_plot

Plots x/y/z trajectories from a Crazyflie uSD log recorded with
`tools/usdlog/config_tdoa_candidates.txt`, comparing three sources per axis:

- **bindings estimate**: the log replayed through the real firmware Kalman core
  (cffirmware SWIG bindings), like `tools/usdlog/replay_tdoa.py`
- **live estimate**: the onboard `stateEstimate` logged during the flight
- **ground truth**: the Lighthouse crossing-beam position

## Usage

From this folder (uv sets up the venv and dependencies automatically):

```
uv run plot_tdoa.py path/to/log.bin --anchors path/to/anchors.yaml
```

If the cffirmware bindings are missing (or built for a different Python), they
are built automatically via `make bindings_python` — this needs `swig` and a C
compiler. Force a rebuild with `--rebuild-bindings`.

Options: `--selection-policy` selects the anchor-pair selection policy for the
replay (default `baseline`), `--tdoa-model` the Kalman measurement model
(`standard` or `robust`, the M-estimation model the firmware uses with
`kalman.robustTdoa=1`), `--tdoa-std` the measurement noise, and
`--save out.png` writes the figure to a file instead of opening a window.
`--policy-param KEY=VALUE` / `--filter-param KEY=VALUE` (repeatable) pass
tuning constants to parametrized policies/filters, e.g. the best-known config
from `tools/usdlog/README_tdoa_experiments.md`:

```
uv run plot_tdoa.py path/to/log.bin --anchors path/to/anchors.yaml \
    --selection-policy all --outlier-filter pair_hampel \
    --filter-param WINDOW=16 --filter-param WARMUP=6 --tdoa-std 0.04
```
