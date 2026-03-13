"""
loco_geometry_estimation.py
===========================
Automatic geometry estimation for Crazyflie LPS TDoA3 anchor systems.

Requires NO custom firmware.  Uses only existing log/param variables:

    tdoaEngine.tof        – inter-anchor ToF in UWB ticks  (selected pair)
    tdoaEngine.tdoa       – d(CF,Ai) – d(CF,Aj) in metres  (selected pair)
    tdoaEngine.cc         – clock correction for selected anchor (used for discovery)
    tdoaEngine.logId      – param: primary anchor ID
    tdoaEngine.logOthrId  – param: secondary anchor ID

PROCESS
-------
1. ANCHOR DISCOVERY
   discover_anchors() probes IDs in SCAN_ID_RANGE.  For each candidate ID
   it checks whether the firmware has a live clock-correction value (cc ≈ 1).
   Takes ~2 s per candidate ID — runs automatically at startup.

   Adjust SCAN_ID_RANGE at the top of this file if your anchors use IDs
   outside 0–31 (e.g. change to range(32, 64)).

2. TOF MATRIX  (build_tof_matrix)
   The inter-anchor ToF comes from the anchors' OWN broadcast packets —
   the CF is a passive listener.  Anchor A embeds its measured ToF to
   anchor B in every packet it sends; the CF logs whichever pair is
   currently selected via logId/logOthrId.

   STRATEGY IN LARGE SPACES:
   • You do NOT need to be between two anchors — just within receive range
     of either one.
   • In large systems (not all anchors visible from one spot), call
     build_tof_matrix() in streaming mode: move the CF slowly through the
     space while the script keeps refreshing the matrix.  Call
     build_tof_matrix(roam=True) and press Enter when you have visited all
     anchor clusters.  The script merges all readings it has collected.
   • Anchor pairs that are out of each other's UWB range will stay NaN.
     MDS handles missing entries by imputation — you need roughly 60–70 %
     of pairs to have data for a good result.

3. TDOA SNAPSHOTS  (capture_tdoa_snapshot)
   Place the CF STATIONARY at a known position (at least origin + one other
   point on the X-axis).  The script reads d(CF,Ai)–d(CF,Aj) for each pair.
   In large spaces use positions with good multi-anchor visibility.

4. estimate() → anchor positions in world frame.

5. upload_positions() → push back to CF (forwards to anchors via LPP).

DEPENDENCIES
------------
    pip install cflib numpy scipy

USAGE AS SCRIPT
---------------
    python loco_geometry_estimation.py --uri radio://0/80/2M/E7E7E7E7E7
    python3 tools/loco_geometry/loco_geometry_estimation.py --load-file 11_anchors_v2.json 
    python loco_geometry_estimation.py --uri radio://0/48/2M/D91F700159 --anchor-ids 1 2 3 5 6 7 8 9 10 11 --save-file my_lab_session.json

USAGE AS MODULE
---------------
    from loco_geometry_estimation import LocoGeometryEstimator
"""

import argparse
import json
import struct
import time
import threading
import logging
import warnings
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s  %(levelname)-7s  %(message)s')
log = logging.getLogger(__name__)

# ── Constants ──────────────────────────────────────────────────────────────────

LOCODECK_TS_FREQ     = 499.2e6 * 128    # UWB clock ticks per second (~63.9 GHz)
SPEED_OF_LIGHT       = 299_792_458.0    # m/s
# The raw tof values logged by tdoaEngine.tof are UNCORRECTED — they include the
# antenna delay offset that the firmware subtracts before computing real distances
# (see lpsTdoa3Tag.c: SPEED_OF_LIGHT * (tof_T - LOCODECK_ANTENNA_DELAY) / LOCODECK_TS_FREQ).
# LOCODECK_ANTENNA_OFFSET is defined in locodeck.h as 154.6 metres.
LOCODECK_ANTENNA_OFFSET = 154.6         # metres, baked into raw anchor tof broadcasts

# ── CONFIGURE THIS to match your anchor ID range ───────────────────────────
# IDs to scan during anchor discovery.  Takes ~2 s per candidate.
# Example: range(0, 8) for IDs 0–7, range(0, 16) for IDs 0–15, etc.
SCAN_ID_RANGE    = range(0, 16)
# ──────────────────────────────────────────────────────────────────────────

# Valid clock-correction range: active TDoA3 anchors have cc ≈ 1.0
CC_VALID_MIN     = 0.85
CC_VALID_MAX     = 1.15

# Two-phase TDoA pair reading:
#
# The firmware's tdoaStats resets stats->tdoa to 0 within STATS_INTERVAL=500ms of a
# pair change (tdoaStats.c), then a fresh natural computation sets the real value.
# Reading too early gives a stale value from the PREVIOUS pair (not the requested one).
#
# Phase 1 – wait for the stats reset (tdoa → 0):
PAIR_RESET_WAIT_S  = 0.65  # > STATS_INTERVAL (500ms) to guarantee the reset fires

# Phase 2 – wait for a fresh non-zero computation after the reset:
# Pair (i,j) is computed at roughly 1–5 Hz by the firmware, so ≤ 2 s covers 99 %.
PAIR_FRESH_WAIT_S  = 3.0   # max time to wait for a non-zero fresh measurement

DISCOVERY_SETTLE_S = 2.0   # long enough for edge-of-range anchors (~20 packets)

# Symmetry quality filter: if |tof(A→B) - tof(B→A)| exceeds this (metres), the
# pair's TWR measurement is unreliable (stale timestamps, clock-sync error, etc.)
# and BOTH directions are replaced with NaN so MDS can impute from good neighbours.
SYMMETRY_THRESHOLD_M = 1.5

# Minimum fraction of undirected pairs that must pass the symmetry check for the
# geometry estimate to be considered trustworthy.
MIN_QUALITY_FRACTION = 0.55

# Number of log samples to average per pair.
SAMPLES_PER_PAIR = 5
LOG_PERIOD_MS    = 50    # 20 Hz log rate

# TDoA outlier filtering.
# The firmware returns 0.0 when a pair has not been recently observed
# (uninitialized log variable).  Some clock-sync faults produce values
# far outside the physical maximum (|tdoa| ≤ distance between the two anchors).
TDOA_MIN_M        = 0.01   # |tdoa| below this is treated as 'not observed' → skip
TDOA_MAX_M        = 15.0   # |tdoa| above this is physically impossible → skip

# Number of random restarts for the direct TDoA optimiser (in addition to the
# MDS-seeded start).  More restarts improve robustness at the cost of compute time.
N_DIRECT_RESTARTS = 5


# ── Estimator ─────────────────────────────────────────────────────────────────

class LocoGeometryEstimator:
    """
    Drives the anchor geometry estimation process using only existing
    firmware log variables — no custom firmware required.

    Parameters
    ----------
    scf : SyncCrazyflie
        An already-open SyncCrazyflie context.
    """

    def __init__(self, scf: SyncCrazyflie):
        self._cf   = scf.cf
        self._lock = threading.Lock()
        self._log_samples: List[Tuple[int, float, float]] = []  # (tof, tdoa, cc)
        self._snapshots: Dict[str, Tuple[np.ndarray, List[float]]] = {}

        lg = LogConfig(name='locoGeo', period_in_ms=LOG_PERIOD_MS)
        lg.add_variable('tdoaEngine.tof',  'uint16_t')
        lg.add_variable('tdoaEngine.tdoa', 'float')
        lg.add_variable('tdoaEngine.cc',   'float')
        self._cf.log.add_config(lg)
        lg.data_received_cb.add_callback(self._on_log)
        lg.start()
        self._lg = lg
        log.info("Log config started (tof + tdoa + cc @ %d ms).", LOG_PERIOD_MS)

        # Optional: log CF position from the state estimator.
        # Only useful after anchor geometry has been uploaded and LPS is running.
        # Uses a separate LogConfig to avoid overflowing the 26-byte packet limit.
        self._cf_pos_current: List[float] = [0.0, 0.0, 0.0]
        self._has_cf_pos = False
        self._lg_pos = None
        try:
            lg_pos = LogConfig(name='locoGeoPos', period_in_ms=LOG_PERIOD_MS)
            lg_pos.add_variable('stateEstimate.x', 'float')
            lg_pos.add_variable('stateEstimate.y', 'float')
            lg_pos.add_variable('stateEstimate.z', 'float')
            self._cf.log.add_config(lg_pos)
            lg_pos.data_received_cb.add_callback(self._on_pos_log)
            lg_pos.start()
            self._lg_pos = lg_pos
            self._has_cf_pos = True
            log.info("CF position logging enabled (stateEstimate.x/y/z).")
        except Exception as exc:
            log.info("CF position log not available (%s) — sweep mode disabled.", exc)

    def stop(self):
        """Stop the log configs.  Call when done."""
        self._lg.stop()
        if self._lg_pos is not None:
            self._lg_pos.stop()

    # ── Low-level pair reading ────────────────────────────────────────────────

    def _on_log(self, _timestamp, data, _logconf):
        with self._lock:
            self._log_samples.append((
                data.get('tdoaEngine.tof',  0),
                data.get('tdoaEngine.tdoa', float('nan')),
                data.get('tdoaEngine.cc',   0.0),
            ))

    def _on_pos_log(self, _timestamp, data, _logconf):
        with self._lock:
            self._cf_pos_current = [
                data.get('stateEstimate.x', 0.0),
                data.get('stateEstimate.y', 0.0),
                data.get('stateEstimate.z', 0.0),
            ]

    def _get_cf_pos(self) -> List[float]:
        """Return the latest LPS-estimated CF position (requires geometry upload)."""
        with self._lock:
            return list(self._cf_pos_current)

    def _select_pair(self, id_a: int, id_b: int):
        """Switch the firmware's logging pair and flush the sample buffer."""
        self._cf.param.set_value('tdoaEngine.logId',    str(id_a))
        self._cf.param.set_value('tdoaEngine.logOthrId', str(id_b))
        with self._lock:
            self._log_samples.clear()

    def _read_pair(self, id_a: int, id_b: int) -> Tuple[float, float]:
        """
        Switch to pair (id_a, id_b) and return a FRESH (mean_tof, mean_tdoa).

        Two-phase protocol (mirrors firmware behaviour in tdoaStats.c):

        Phase 1 – stats reset:
          The firmware's tdoaStatsUpdate() resets stats->tdoa = 0 within
          STATS_INTERVAL (500 ms) of a pair-param change.  Before that reset
          fires, the log reads the OLD pair's stale value — which is wrong.
          We wait PAIR_RESET_WAIT_S (> 500 ms) to guarantee the reset has fired.

        Phase 2 – fresh computation:
          After the reset, stats->tdoa = 0.  We wait until the firmware
          naturally computes a TDoA for (id_a, id_b) and writes a non-zero
          value (typically < 1 s at the LPS packet rate).

        Returns (0.0, nan) if no fresh value arrives within PAIR_FRESH_WAIT_S.
        """
        self._select_pair(id_a, id_b)

        # Phase 1: wait for the firmware stats reset (tdoa → 0)
        time.sleep(PAIR_RESET_WAIT_S)

        # Re-clear the buffer so phase-2 only sees post-reset samples
        with self._lock:
            self._log_samples.clear()

        # Phase 2: wait for first non-zero (= fresh) tdoa value
        deadline = time.time() + PAIR_FRESH_WAIT_S
        fresh_tdoas: List[float] = []
        fresh_tofs:  List[float] = []
        while time.time() < deadline:
            time.sleep(0.025)
            with self._lock:
                for s in self._log_samples:
                    tof, tdoa, _ = s
                    if np.isfinite(tdoa) and abs(tdoa) >= TDOA_MIN_M:
                        fresh_tdoas.append(tdoa)
                        if tof > 0:
                            fresh_tofs.append(float(tof))
            if fresh_tdoas:
                break   # got at least one fresh sample — stop polling

        if not fresh_tdoas:
            return 0.0, float('nan')

        return (float(np.mean(fresh_tofs)) if fresh_tofs else 0.0,
                float(np.mean(fresh_tdoas)))

    # ── Anchor discovery ──────────────────────────────────────────────────────

    def discover_anchors(self,
                         scan_range=SCAN_ID_RANGE) -> List[int]:
        """
        Discover active anchor IDs by detecting whether cc (clock correction)
        actually changes after switching logId to each candidate.

        The firmware only writes stats.clockCorrection when it processes a packet
        from the anchor whose ID matches logId.  For a non-existent anchor,
        cc is NEVER updated — it stays frozen at whatever value it had before.
        We detect this by recording the cc value BEFORE the logId switch
        (stale_cc) and checking whether any sample after the switch differs.

        Returns a sorted list of active anchor IDs.
        """
        log.info("Discovering anchors in ID range %s…", list(scan_range))
        active: List[int] = []

        # Seed stale_cc from the very first sample before any probe
        with self._lock:
            stale_cc = self._log_samples[-1][2] if self._log_samples else 0.0

        for candidate in scan_range:
            dummy = (candidate + 1) % 256
            self._cf.param.set_value('tdoaEngine.logId',     str(candidate))
            self._cf.param.set_value('tdoaEngine.logOthrId', str(dummy))
            with self._lock:
                self._log_samples.clear()
            time.sleep(DISCOVERY_SETTLE_S)

            with self._lock:
                recent = list(self._log_samples)

            if not recent:
                log.debug("  ID %d: no log data received", candidate)
                # stale_cc unchanged — keep it for next iteration
                continue

            ccs     = [s[2] for s in recent]
            mean_cc = float(np.mean(ccs))

            # Key check: did cc change from the pre-switch stale value?
            # A non-existent anchor never updates cc → all samples == stale_cc.
            # A real anchor updates cc within a packet or two → at least one sample differs.
            updated = any(abs(cc - stale_cc) > 1e-9 for cc in ccs)

            if updated and CC_VALID_MIN <= mean_cc <= CC_VALID_MAX:
                active.append(candidate)
                log.info("  Found anchor %d  (cc=%.6f, was %.6f)",
                         candidate, mean_cc, stale_cc)
            else:
                log.debug("  ID %d: cc=%.6f stale=%.6f updated=%s",
                          candidate, mean_cc, stale_cc, updated)

            # The last sample is the new stale_cc baseline for the next candidate
            stale_cc = ccs[-1]

        log.info("Discovery complete.  Active anchors: %s", active)
        return sorted(active)

    # ── Phase 1: ToF matrix ───────────────────────────────────────────────────

    def build_tof_matrix(self,
                          active_ids: List[int]) -> Tuple[List[int], np.ndarray]:
        """
        Probe all pairs among active_ids and build the inter-anchor distance
        matrix.  Pass the list returned by discover_anchors().

        In large spaces where not all anchors are visible simultaneously,
        call this once while you move the CF through the space — it merges
        all readings it receives.  The inter-anchor ToF comes from the
        anchors' own broadcast packets (passive listen), so you just need to
        be within receive range of each anchor at some point.

        Returns
        -------
        active_ids : same list (unchanged)
        dist_m     : ndarray shape (N, N), metres, NaN where no data
        """
        ids = list(active_ids)
        n   = len(ids)
        tof_raw = np.zeros((n, n))

        log.info("Building ToF matrix (%d IDs × %d IDs = %d pairs)…",
                 n, n, n * (n - 1))

        for i, id_i in enumerate(ids):
            for j, id_j in enumerate(ids):
                if i == j:
                    continue
                tof, _ = self._read_pair(id_i, id_j)
                tof_raw[i, j] = tof
                if tof > 0:
                    dist_raw = tof * SPEED_OF_LIGHT / LOCODECK_TS_FREQ
                    dist_corr = max(0.0, dist_raw - LOCODECK_ANTENNA_OFFSET)
                    log.info("  tof(%d,%d) = %.0f ticks → %.3f m (raw) → %.3f m (corrected)",
                             id_i, id_j, tof, dist_raw, dist_corr)

        # Subtract the antenna delay offset baked into the raw tof values.
        # Raw tof from anchors = actual_propagation_ticks + antenna_delay_ticks.
        # antenna_delay_ticks = LOCODECK_ANTENNA_OFFSET * LOCODECK_TS_FREQ / SPEED_OF_LIGHT
        dist_m = np.where(tof_raw > 0,
                          np.maximum(0.0,
                                     tof_raw * SPEED_OF_LIGHT / LOCODECK_TS_FREQ
                                     - LOCODECK_ANTENNA_OFFSET),
                          np.nan)
        np.fill_diagonal(dist_m, 0.0)

        # ── Symmetry quality filter ──────────────────────────────────────────
        # tof(A→B) and tof(B→A) are independent TWR measurements of the same
        # distance and should agree.  Large asymmetry means the anchor's TWR
        # result was stale or clock-sync failed — average them would be worse
        # than NaN + imputation from good neighbours.
        n_both   = 0   # pairs where BOTH directions were measured
        n_sym    = 0   # pairs that passed the symmetry check
        bad_anchors: Dict[int, int] = {}  # id → count of bad pairs involving it

        for i in range(n):
            for j in range(i + 1, n):
                d_fwd = dist_m[i, j]
                d_rev = dist_m[j, i]
                both_finite = np.isfinite(d_fwd) and np.isfinite(d_rev)
                if both_finite:
                    n_both += 1
                    asym = abs(d_fwd - d_rev)
                    if asym > SYMMETRY_THRESHOLD_M:
                        log.warning(
                            "  Pair (%d,%d): asymmetry %.2f m > %.1f m threshold "
                            "(fwd=%.2f, rev=%.2f) → NaN (will be imputed)",
                            ids[i], ids[j], asym, SYMMETRY_THRESHOLD_M, d_fwd, d_rev)
                        dist_m[i, j] = np.nan
                        dist_m[j, i] = np.nan
                        for aid in (ids[i], ids[j]):
                            bad_anchors[aid] = bad_anchors.get(aid, 0) + 1
                    else:
                        n_sym += 1

        quality = n_sym / n_both if n_both > 0 else 0.0
        valid_after = int(np.sum(np.isfinite(dist_m)) - n)
        log.info("ToF quality: %d / %d symmetric pairs (%.0f%%)  "
                 "[threshold %.1f m]",
                 n_sym, n_both, 100 * quality, SYMMETRY_THRESHOLD_M)
        log.info("Valid pairs after quality filter: %d / %d directed",
                 valid_after, n * (n - 1))

        if bad_anchors:
            ranked = sorted(bad_anchors.items(), key=lambda kv: -kv[1])
            log.warning(
                "Anchors involved in the most asymmetric pairs (possible "
                "calibration / clock-sync issues): %s",
                ", ".join(f"#{aid}({cnt})" for aid, cnt in ranked))

        if quality < MIN_QUALITY_FRACTION:
            log.warning(
                "Only %.0f%% of pairs passed the symmetry check (minimum %.0f%%). "
                "Geometry estimate may be unreliable — consider re-collecting data "
                "after verifying anchor calibration and ensuring the CF can hear all "
                "anchor pairs from a central position.",
                100 * quality, 100 * MIN_QUALITY_FRACTION)

        return ids, dist_m, quality

    # ── Phase 2: TDoA snapshot ────────────────────────────────────────────────

    def capture_tdoa_snapshot(self, cf_position: List[float],
                               active_ids: List[int],
                               label: Optional[str] = None) -> np.ndarray:
        """
        Collect TDoA measurements (d(CF,Ai) – d(CF,Aj)) at the current CF
        position for every active anchor pair.

        The CF must be stationary at `cf_position` when this is called.

        Parameters
        ----------
        cf_position : [x, y, z] in metres
        active_ids  : list returned by build_tof_matrix()
        label       : optional name for this snapshot

        Returns
        -------
        tdoa_m : ndarray shape (N, N), metres, NaN where no data
        """
        if label is None:
            label = f"snap_{len(self._snapshots)}"

        n      = len(active_ids)
        tdoa_m = np.full((n, n), np.nan)

        n_pairs = n * (n - 1) // 2
        log.info("[%s] Capturing TDoA snapshot at %s… (%d pairs, ~%.0f s est.)",
                 label, cf_position, n_pairs,
                 n_pairs * (PAIR_RESET_WAIT_S + 0.5))

        # Only measure each undirected pair (i < j) once — anti-symmetry gives
        # tdoa_m[j,i] = -tdoa_m[i,j] exactly, halving collection time and
        # ensuring perfect anti-symmetry in the recorded snapshot.
        for i, id_i in enumerate(active_ids):
            for j, id_j in enumerate(active_ids):
                if i >= j:
                    continue
                _, tdoa = self._read_pair(id_i, id_j)
                if np.isfinite(tdoa):
                    tdoa_m[i, j] =  tdoa
                    tdoa_m[j, i] = -tdoa   # exact anti-symmetry by construction

        valid = int(np.sum(np.isfinite(tdoa_m)))
        log.info("[%s] Valid pairs: %d / %d (undirected: %d)", label,
                 valid, n * (n - 1), valid // 2)
        self._snapshots[label] = (tdoa_m, list(cf_position))
        return tdoa_m

    def capture_sweep(self, active_ids: List[int],
                      sweep_index: int = 0) -> int:
        """
        Collect TDoA measurements while the CF moves freely through the space.

        Unlike capture_tdoa_snapshot() which fixes the CF at one position and
        measures all pairs there, this method cycles through every anchor pair
        and records the LPS-estimated CF position at the time each pair is read.
        The result is a set of diverse (position, tdoa) measurements spanning the
        room — exactly like the Lighthouse calibration sweep.

        This requires:
          1. Rough anchor geometry already UPLOADED to the CF (so the LPS
             state estimator is running and giving meaningful positions).
          2. The CF is flying, being carried, or otherwise moved around the space
             during the ~60-90 s the sweep takes.

        Each pair's measurement is stored as its own sparse snapshot (one valid
        entry in the n×n matrix) in self._snapshots under the key
        "sw<sweep_index>_<pair_index>".  These are automatically included in the
        next estimate() call.

        Parameters
        ----------
        active_ids  : anchor IDs (same list used throughout)
        sweep_index : counter to distinguish multiple sweeps (default 0)

        Returns
        -------
        n_captured : number of valid pairs measured
        """
        if not self._has_cf_pos:
            log.error("CF position logging is not available — cannot run sweep. "
                      "Ensure anchor geometry has been uploaded to the CF so the "
                      "LPS state estimator is active.")
            return 0

        n       = len(active_ids)
        n_pairs = n * (n - 1) // 2
        log.info("Starting sweep %d (%d pairs, ~%.0f s) — keep the CF moving!",
                 sweep_index, n_pairs, n_pairs * (PAIR_RESET_WAIT_S + 0.5))
        print(f"\n  [Sweep {sweep_index}] Keep the CF flying or moving around the space.")
        print(f"  This will take ~{n_pairs * (PAIR_RESET_WAIT_S + 0.5):.0f} s "
              f"({n_pairs} pairs × ~{PAIR_RESET_WAIT_S + 0.5:.1f} s each).\n")

        n_captured = 0
        pair_idx   = 0

        for i, id_i in enumerate(active_ids):
            for j, id_j in enumerate(active_ids):
                if i >= j:
                    continue

                # Sample CF position before and after the pair reading window
                pos_before = self._get_cf_pos()
                _, tdoa    = self._read_pair(id_i, id_j)
                pos_after  = self._get_cf_pos()

                if np.isfinite(tdoa):
                    # Average position over the measurement window
                    cf_pos = [(a + b) / 2.0 for a, b in zip(pos_before, pos_after)]

                    # Store as a sparse mini-snapshot (one valid pair per matrix)
                    snap = np.full((n, n), np.nan)
                    snap[i, j] =  tdoa
                    snap[j, i] = -tdoa    # exact anti-symmetry
                    label = f"sw{sweep_index}_{pair_idx:02d}"
                    self._snapshots[label] = (snap, cf_pos)
                    n_captured += 1
                    log.info("  sw(%d,%d): tdoa=%.3f m  cf=(%.2f, %.2f, %.2f)",
                             id_i, id_j, tdoa, *cf_pos)

                pair_idx += 1

        log.info("Sweep %d done.  Captured %d / %d pairs.",
                 sweep_index, n_captured, n_pairs)
        return n_captured

    # ── Phase 3: Estimation ───────────────────────────────────────────────────

    def estimate(self, active_ids: List[int],
                 dist_m: np.ndarray) -> Tuple[List[int], np.ndarray]:
        """
        Estimate anchor world-frame positions from the ToF matrix and all
        previously captured TDoA snapshots.

        Returns (active_ids, positions_world) where positions_world is
        ndarray shape (N, 3) in metres.
        """
        return _estimate_and_print(active_ids, dist_m, self._snapshots)

    def upload_positions(self, anchor_ids: List[int], positions: np.ndarray,
                         n_transmissions: int = 3):
        """Push positions to the CF (it will forward them to the anchors via LPP).

        Each position is transmitted n_transmissions times.  LPP short packets
        carry no acknowledgement, so sending multiple copies improves reliability.
        """
        LPP_SHORT_ANCHOR_POSITION = 0x01
        for aid, pos in zip(anchor_ids, positions):
            payload = struct.pack('<Bfff', LPP_SHORT_ANCHOR_POSITION,
                                  float(pos[0]), float(pos[1]), float(pos[2]))
            for _ in range(n_transmissions):
                self._cf.loc.send_short_lpp_packet(int(aid), payload)
                time.sleep(0.02)
            log.info("Uploaded anchor %d → (%.3f, %.3f, %.3f) ×%d",
                     aid, pos[0], pos[1], pos[2], n_transmissions)


# ── Mathematics ───────────────────────────────────────────────────────────────

def _metric_mds(dist_m: np.ndarray, n: int) -> np.ndarray:
    """
    Classical metric MDS on an (n×n) distance matrix that may contain NaN.
    Returns ndarray shape (n, 3) in an arbitrary reference frame.
    """
    # Symmetrise: average D[i,j] and D[j,i]
    # Suppress "Mean of empty slice" which occurs for all-NaN pairs (handled below).
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', category=RuntimeWarning)
        D = np.nanmean(np.stack([dist_m, dist_m.T]), axis=0)

    # Impute remaining NaN with mean of known row/col distances
    for i in range(n):
        for j in range(n):
            if i != j and np.isnan(D[i, j]):
                row_mean = np.nanmean(D[i, :])
                col_mean = np.nanmean(D[:, j])
                D[i, j] = 0.5 * (row_mean + col_mean)
    D = np.nan_to_num(D, nan=0.0)

    # Double-centred Gram matrix → top-3 eigenvectors
    D2 = D ** 2
    H  = np.eye(n) - np.ones((n, n)) / n
    B  = -0.5 * H @ D2 @ H

    eigenvalues, eigenvectors = np.linalg.eigh(B)
    idx = np.argsort(eigenvalues)[::-1][:3]
    lam = np.maximum(eigenvalues[idx], 0.0)
    return eigenvectors[:, idx] * np.sqrt(lam)   # (n, 3)


def _align_to_world(pos_arb: np.ndarray,
                     snapshots: List[Tuple[np.ndarray, List[float]]]
                     ) -> Tuple[np.ndarray, float, float]:
    """
    Find the similarity transform (scale, R, t) that maps pos_arb → world frame,
    minimising for each snapshot at CF position p_k:

        Σ_{i≠j} ( ||s·R·a_i + t – p_k|| – ||s·R·a_j + t – p_k|| – δ_ij^k )²

    Scale `s` absorbs any systematic bias in the ToF-derived MDS distances
    (e.g. if the tof log variable is in different units than assumed).
    The TDoA measurements — which ARE in correct metres — constrain the scale.

    Returns (world_positions [N,3], rms_residual, estimated_scale).
    """
    n = len(pos_arb)

    def residuals(params):
        t     = params[:3]
        R     = Rotation.from_rotvec(params[3:6])
        scale = np.exp(params[6])              # log-parameterised → always positive
        world = R.apply(scale * pos_arb) + t   # (n, 3)
        res   = []
        for snap, p_cf in snapshots:
            p = np.array(p_cf)
            d = np.linalg.norm(world - p, axis=1)   # (n,)
            for i in range(n):
                for j in range(n):
                    v = snap[i, j]
                    if (i != j
                            and np.isfinite(v)
                            and abs(v) >= TDOA_MIN_M
                            and abs(v) <= TDOA_MAX_M):
                        # Firmware convention: tdoa(logId=i, logOthrId=j) =
                        #   d(CF, A_logOthrId) - d(CF, A_logId) = d(CF,A_j) - d(CF,A_i)
                        # So residual = d[j] - d[i] - v
                        res.append(d[j] - d[i] - v)
        return np.array(res) if res else np.zeros(1)

    centroid = np.mean(pos_arb, axis=0)
    # params[6] = 0.0 → initial scale = exp(0) = 1.0
    x0 = np.array([-centroid[0], -centroid[1], -centroid[2],
                   0.0, 0.0, 0.0,
                   0.0])
    result = least_squares(residuals, x0, method='lm', max_nfev=10000)

    scale_est = float(np.exp(result.x[6]))
    R_est     = Rotation.from_rotvec(result.x[3:6])
    t_est     = result.x[:3]
    world_pos = R_est.apply(scale_est * pos_arb) + t_est
    rms = float(np.sqrt(np.mean(result.fun ** 2))) if len(result.fun) > 0 else 0.0
    return world_pos, rms, scale_est


def _direct_tdoa_opt(n: int,
                     snapshots: List[Tuple[np.ndarray, List[float]]],
                     x0: np.ndarray,
                     dist_m: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float, int]:
    """
    Directly optimise all anchor positions against TDoA measurements.

    Unlike the two-step MDS → similarity-transform approach, this treats
    the N×3 anchor coordinates as the free variables and minimises:

        Σ_{k, i≠j} ( ||a_i – p_k|| – ||a_j – p_k|| – δ_ij^k )²
      + Σ_{i<j}    ( ||a_i – a_j|| – d_measured[i,j] )²

    where p_k are the **known** CF positions (fixed in world frame) and
    d_measured are the ToF inter-anchor distances.

    The distance term is critical: TDoA-only optimisation has a spurious
    large-scale global minimum.  When anchors are moved far from the CF
    (e.g. 10× too far), the small CF position offsets between snapshots
    become negligible, all snapshots give nearly the same TDoA values, and
    the optimizer can drive residuals low while the geometry is completely
    wrong.  Adding inter-anchor distance constraints pins the scale.

    TDoA values that are near-zero (likely 'not observed') or physically
    impossible (|δ| > TDOA_MAX_M) are silently skipped.

    Parameters
    ----------
    n         : number of anchors
    snapshots : list of (tdoa_matrix [n×n], cf_position [x,y,z])
    x0        : initial anchor positions, shape (n, 3)
    dist_m    : measured inter-anchor distances [n×n], metres (may contain NaN)

    Returns
    -------
    pos_world : ndarray (n, 3), anchor positions in metres (world frame)
    rms       : RMS residual in metres (lower is better)
    n_constraints : number of TDoA values actually used
    """
    # Pre-compute valid distance pairs (avoid repeated NaN checks in inner loop)
    dist_pairs: List[Tuple[int, int, float]] = []
    if dist_m is not None:
        for i in range(n):
            for j in range(i + 1, n):
                with warnings.catch_warnings():
                    warnings.filterwarnings('ignore', category=RuntimeWarning)
                    d_meas = float(np.nanmean([dist_m[i, j], dist_m[j, i]]))
                if np.isfinite(d_meas):
                    dist_pairs.append((i, j, d_meas))

    def residuals(params):
        pos = params.reshape(n, 3)
        res = []
        for snap, p_cf in snapshots:
            p = np.array(p_cf)
            d = np.linalg.norm(pos - p, axis=1)   # (n,)
            for i in range(n):
                for j in range(n):
                    v = snap[i, j]
                    if (i != j
                            and np.isfinite(v)
                            and abs(v) >= TDOA_MIN_M
                            and abs(v) <= TDOA_MAX_M):
                        # Firmware: tdoa(logId=i, logOthrId=j) = d(CF,A_j) - d(CF,A_i)
                        res.append(d[j] - d[i] - v)
        # Inter-anchor distance constraints (pins the scale, prevents spurious solutions)
        for i, j, d_meas in dist_pairs:
            d_est = float(np.linalg.norm(pos[i] - pos[j]))
            res.append(d_est - d_meas)
        return np.array(res) if res else np.zeros(1)

    # Count valid constraints once (before optimisation)
    n_constraints = sum(
        1
        for snap, _ in snapshots
        for i in range(n)
        for j in range(n)
        if (i != j
            and np.isfinite(snap[i, j])
            and abs(snap[i, j]) >= TDOA_MIN_M
            and abs(snap[i, j]) <= TDOA_MAX_M)
    )

    # Use a robust loss (soft_l1) to reduce sensitivity to inconsistent
    # TDoA measurements caused by clock drift between pair readings.
    result    = least_squares(residuals, x0.flatten(), method='trf',
                              loss='soft_l1', f_scale=0.5, max_nfev=50_000)
    pos_world = result.x.reshape(n, 3)
    rms       = float(np.sqrt(np.mean(result.fun ** 2))) if len(result.fun) > 0 else 1e9
    return pos_world, rms, n_constraints


# ── Session save / load ───────────────────────────────────────────────────────

SESSION_VERSION = 1


def _arr_to_json(a: np.ndarray) -> list:
    """Serialise a 2-D numpy array to a nested list; NaN → None (JSON null)."""
    return [[None if np.isnan(v) else float(v) for v in row] for row in a]


def _arr_from_json(rows: list) -> np.ndarray:
    """Deserialise a nested list back to a 2-D numpy array; None → NaN."""
    return np.array([[np.nan if v is None else float(v) for v in row]
                     for row in rows])


def save_session(path: str,
                 active_ids: List[int],
                 dist_m: np.ndarray,
                 snapshots: Dict[str, Tuple[np.ndarray, List[float]]]) -> None:
    """
    Save all collected data to a JSON file so the estimation can be re-run
    offline without reconnecting to the Crazyflie.

    Parameters
    ----------
    path       : output file path (e.g. 'session_2026-02-27.json')
    active_ids : anchor IDs used
    dist_m     : corrected inter-anchor distance matrix (metres, NaN for missing)
    snapshots  : dict {label: (tdoa_matrix, cf_position)}
    """
    data = {
        'version':                 SESSION_VERSION,
        'timestamp':               datetime.now().isoformat(timespec='seconds'),
        'locodeck_antenna_offset': LOCODECK_ANTENNA_OFFSET,
        'active_ids':              active_ids,
        'dist_m':                  _arr_to_json(dist_m),
        'snapshots': {
            label: {
                'tdoa_m':      _arr_to_json(tdoa_m),
                'cf_position': list(cf_pos),
            }
            for label, (tdoa_m, cf_pos) in snapshots.items()
        },
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    log.info("Session saved → %s", path)


def load_session(path: str
                 ) -> Tuple[List[int], np.ndarray,
                            Dict[str, Tuple[np.ndarray, List[float]]]]:
    """
    Load a previously saved session file.

    Returns (active_ids, dist_m, snapshots).
    """
    with open(path) as f:
        data = json.load(f)

    if data.get('version', 0) != SESSION_VERSION:
        raise ValueError(
            f"Unsupported session file version {data.get('version')}. "
            f"Expected {SESSION_VERSION}.")

    saved_offset = data.get('locodeck_antenna_offset', LOCODECK_ANTENNA_OFFSET)
    if abs(saved_offset - LOCODECK_ANTENNA_OFFSET) > 0.01:
        log.warning(
            "Session was saved with LOCODECK_ANTENNA_OFFSET=%.2f m "
            "but current value is %.2f m — distances may be inconsistent.",
            saved_offset, LOCODECK_ANTENNA_OFFSET)

    active_ids = data['active_ids']
    dist_m     = _arr_from_json(data['dist_m'])
    snapshots  = {
        label: (_arr_from_json(s['tdoa_m']), s['cf_position'])
        for label, s in data['snapshots'].items()
    }

    log.info("Loaded session from %s  (saved %s)", path, data.get('timestamp', '?'))
    log.info("  Anchors  : %s", active_ids)
    n_pairs = len(active_ids) * (len(active_ids) - 1)
    valid   = int(np.sum(np.isfinite(dist_m)) - len(active_ids))
    log.info("  ToF pairs: %d / %d valid", valid, n_pairs)
    log.info("  Snapshots: %s", list(snapshots.keys()))
    return active_ids, dist_m, snapshots


def _estimate_and_print(active_ids: List[int],
                        dist_m: np.ndarray,
                        snapshots: Dict[str, Tuple[np.ndarray, List[float]]]
                        ) -> Tuple[List[int], np.ndarray]:
    """
    Run MDS + alignment from already-collected data (no CF required).
    Used by both the online flow and the offline replay flow.
    """
    if len(snapshots) < 3:
        raise ValueError(
            f"Need at least 3 TDoA snapshots, got {len(snapshots)}.")

    n = len(active_ids)

    # ── Compute pair-specific max TDoA from ToF matrix ────────────────────────
    # |TDoA(i,j)| ≤ d(Ai, Aj) by the triangle inequality, so known inter-anchor
    # distances give a tighter per-pair bound than a global constant.
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', category=RuntimeWarning)
        max_dist = float(np.nanmax(dist_m)) if np.any(np.isfinite(dist_m)) else TDOA_MAX_M
    tdoa_max_m = min(TDOA_MAX_M, max_dist * 1.05)  # 5 % margin for noise
    log.info("Using per-session TDoA_MAX = %.2f m  "
             "(max known inter-anchor dist = %.2f m)", tdoa_max_m, max_dist)

    # ── Symmetrize snapshots ──────────────────────────────────────────────────
    # For each pair (i,j) in every snapshot, the true TDoA satisfies
    #   snap[i,j] = -snap[j,i]  (exact anti-symmetry).
    # If BOTH directions were freshly measured they can be averaged to reduce
    # noise.  If only ONE direction is within range, it is kept and the other
    # is derived from it.  If BOTH are inconsistent (|snap[i,j]+snap[j,i]|
    # above the threshold), both are discarded — one of them is stale.
    ASYM_DISCARD_M = 1.5   # pairs with |asym| above this are discarded
    sym_snapshots: Dict[str, Tuple[np.ndarray, List[float]]] = {}
    n_discarded_total = 0
    for sname, (snap_raw, p_cf) in snapshots.items():
        sym = np.full_like(snap_raw, np.nan)
        n_discarded = 0
        for i in range(n):
            for j in range(i + 1, n):
                v_ij = snap_raw[i, j]
                v_ji = snap_raw[j, i]
                ok_ij = (np.isfinite(v_ij)
                         and abs(v_ij) >= TDOA_MIN_M
                         and abs(v_ij) <= tdoa_max_m)
                ok_ji = (np.isfinite(v_ji)
                         and abs(v_ji) >= TDOA_MIN_M
                         and abs(v_ji) <= tdoa_max_m)
                if ok_ij and ok_ji:
                    asym = abs(v_ij + v_ji)
                    if asym <= ASYM_DISCARD_M:
                        s = (v_ij - v_ji) / 2.0   # symmetrized average
                        sym[i, j] =  s
                        sym[j, i] = -s
                    else:
                        n_discarded += 1           # stale — discard both
                elif ok_ij:
                    sym[i, j] =  v_ij
                    sym[j, i] = -v_ij
                elif ok_ji:
                    sym[i, j] = -v_ji
                    sym[j, i] =  v_ji
        if n_discarded:
            log.info("  [%s] discarded %d pairs with |anti-sym| > %.1f m",
                     sname, n_discarded, ASYM_DISCARD_M)
            n_discarded_total += n_discarded
        sym_snapshots[sname] = (sym, p_cf)

    if n_discarded_total:
        log.warning(
            "Discarded %d measurement pairs across all snapshots due to "
            "anti-symmetry > %.1f m.\n"
            "  ▶ ROOT CAUSE: the firmware's stats->tdoa is stale (from the\n"
            "    previous logId/logOthrId pair) until tdoaStatsUpdate() fires\n"
            "    500 ms after a pair change.  With PAIR_RESET_WAIT_S=%.2f s\n"
            "    the new _read_pair() guarantees freshness for future sessions.",
            n_discarded_total, ASYM_DISCARD_M, PAIR_RESET_WAIT_S)

    snap_list = [(m, p) for m, p in sym_snapshots.values()]

    # Count valid TDoA constraints (after symmetrization + filtering)
    n_valid_tdoa = sum(
        1
        for snap, _ in snap_list
        for i in range(n)
        for j in range(i + 1, n)   # count undirected pairs only (sym gives both)
        if (np.isfinite(snap[i, j]) and abs(snap[i, j]) >= TDOA_MIN_M)
    )
    n_unknowns = n * 3
    log.info("Valid TDoA constraints after symmetrization/filtering: %d undirected pairs "
             "(unknowns: %d)", n_valid_tdoa, n_unknowns)
    if n_valid_tdoa < n_unknowns:
        log.warning(
            "Under-determined system: only %d pairs for %d unknowns. "
            "Result will be unreliable — collect more TDoA snapshots or "
            "move the CF to positions with better multi-anchor visibility.",
            n_valid_tdoa, n_unknowns)

    # ── Anti-symmetry diagnostic ──────────────────────────────────────────────
    # tdoa(logId=i, logOthrId=j) = d(CF,Ai) – d(CF,Aj), so
    # snap[i,j] + snap[j,i] should = 0.  A large violation means the TDoA
    # was measured at different times with different (drifting) clock corrections.
    # This is the MOST COMMON cause of poor geometry estimates.
    all_asym: List[float] = []
    log.info("TDoA anti-symmetry check (ideal: |snap[i,j] + snap[j,i]| = 0):")
    cf_z_nonzero = False
    for sname, (snap, p_cf) in snapshots.items():
        if abs(p_cf[2]) > 0.05:
            cf_z_nonzero = True
        pair_asym: List[float] = []
        for i in range(n):
            for j in range(i + 1, n):
                v_ij = snap[i, j]
                v_ji = snap[j, i]
                if (np.isfinite(v_ij) and np.isfinite(v_ji)
                        and abs(v_ij) >= TDOA_MIN_M and abs(v_ji) >= TDOA_MIN_M
                        and abs(v_ij) <= TDOA_MAX_M and abs(v_ji) <= TDOA_MAX_M):
                    pair_asym.append(abs(v_ij + v_ji))
        if pair_asym:
            log.info("  %-8s  mean=%.3f m  max=%.3f m  (%d pairs checked)",
                     sname, np.mean(pair_asym), np.max(pair_asym), len(pair_asym))
            all_asym.extend(pair_asym)

    if all_asym:
        mean_asym = float(np.mean(all_asym))
        if mean_asym > 0.5:
            log.warning(
                "Anti-symmetry mean error = %.2f m (good data: < 0.1 m).\n"
                "  Root cause: the firmware (tdoaStats.c) only resets stats->tdoa\n"
                "  for a NEW pair after a 500 ms stats-update cycle. If the pair\n"
                "  was read within that window the log returns the OLD pair's\n"
                "  stale value, which can be metres away from the correct one.\n"
                "  ▶ This session was collected with the old _read_pair() code.\n"
                "    The new code (PAIR_RESET_WAIT_S=%.2f s) waits past the\n"
                "    reset and then waits for a fresh non-zero value.\n"
                "    Re-collecting data with the updated script will fix this.\n"
                "  ▶ For this session the estimator will attempt to discard\n"
                "    the worst pairs (|anti-sym| > %.1f m) automatically.",
                mean_asym, PAIR_RESET_WAIT_S, ASYM_DISCARD_M)
        elif mean_asym > 0.1:
            log.warning("Anti-symmetry mean = %.2f m (borderline — "
                        "results may have cm-level bias).", mean_asym)
        else:
            log.info("Anti-symmetry mean = %.3f m — measurement quality looks good.", mean_asym)

    if not cf_z_nonzero:
        log.warning(
            "All CF snapshot positions are at z = 0. "
            "From a flat plane, anchors at (x,y,+z) and (x,y,−z) produce "
            "identical TDoA values — z-coordinates cannot be unambiguously "
            "determined. A z-flip heuristic will be applied post-estimation "
            "(if mean anchor z < 0, all z values are negated).\n"
            "  ▶ The mandatory sweep (Phase 5) samples at varying heights and "
            "will further constrain z-coordinates.  For the best result, add "
            "a snapshot at z > 0 (e.g. place CF on a 50 cm table) before "
            "the sweep.")

    # ── Step 1: MDS initial guess ─────────────────────────────────────────────
    log.info("Running MDS on %d anchors…", n)
    pos_arb = _metric_mds(dist_m, n)

    log.info("MDS → world-frame alignment with %d snapshots…", len(snap_list))
    pos_mds, rms_mds, scale = _align_to_world(pos_arb, snap_list)
    log.info("MDS+alignment RMS: %.4f m  (scale=%.4f)", rms_mds, scale)

    if scale < 0.95 or scale > 1.05:
        factor    = 1.0 / scale if scale < 1.0 else scale
        direction = "too large" if scale < 1.0 else "too small"
        log.warning("MDS scale factor = %.4f  (ToF distances were ~%.1fx %s; "
                    "corrected automatically)",
                    scale, factor, direction)

    # ── Step 2: Direct TDoA optimisation — refine from MDS + random restarts ──
    # Directly optimises the N×3 anchor coordinates against TDoA residuals.
    # This bypasses the fragile two-step (MDS shape → similarity transform)
    # and is far less sensitive to a poor MDS initial guess or sparse ToF data.
    log.info("Direct TDoA optimisation: %d restarts…", N_DIRECT_RESTARTS)
    best_pos, best_rms, _ = _direct_tdoa_opt(n, snap_list, x0=pos_mds, dist_m=dist_m)
    log.info("  [0] MDS seed    RMS=%.4f m", best_rms)

    rng = np.random.default_rng(42)
    for k in range(N_DIRECT_RESTARTS - 1):
        x0_k  = rng.uniform(-8.0, 8.0, size=(n, 3))
        pos_k, rms_k, _ = _direct_tdoa_opt(n, snap_list, x0=x0_k, dist_m=dist_m)
        log.info("  [%d] random     RMS=%.4f m", k + 1, rms_k)
        if rms_k < best_rms:
            best_rms, best_pos = rms_k, pos_k

    pos_world = best_pos
    rms       = best_rms
    log.info("Best direct-opt RMS: %.4f m", rms)

    # ── Distance-fit quality report ───────────────────────────────────────────
    # Compare pairwise distances implied by the estimated positions against the
    # measured distance matrix.  A large mean error means the positions do not
    # explain the ToF data — the geometry estimate is unreliable.
    dist_errors = []
    for i in range(n):
        for j in range(i + 1, n):
            with warnings.catch_warnings():
                warnings.filterwarnings('ignore', category=RuntimeWarning)
                d_meas = float(np.nanmean([dist_m[i, j], dist_m[j, i]]))
            if np.isfinite(d_meas):
                d_est = float(np.linalg.norm(pos_world[i] - pos_world[j]))
                dist_errors.append(abs(d_est - d_meas))

    if dist_errors:
        mean_dist_err = float(np.mean(dist_errors))
        max_dist_err  = float(np.max(dist_errors))
        log.info("Distance fit: mean abs error = %.3f m, max = %.3f m  "
                 "(estimated positions vs measured ToF distances)",
                 mean_dist_err, max_dist_err)

        if mean_dist_err < 0.5:
            quality_label = "GOOD"
        elif mean_dist_err < 1.5:
            quality_label = "MODERATE"
        else:
            quality_label = "POOR"

        if quality_label == "POOR":
            log.warning(
                "Geometry quality: %s  (distance fit error %.2f m — results are "
                "likely unreliable.  Possible causes:\n"
                "  • Anchor clock-sync not stabilised — wait longer before collecting\n"
                "  • Anchor antenna delay miscalibration on specific anchors\n"
                "  • Some anchor pairs too far apart for reliable UWB ranging\n"
                "  • Too many pairs filtered out by symmetry check (too few constraints "
                "for MDS imputation)",
                quality_label, mean_dist_err)
        else:
            log.info("Geometry quality: %s  (distance fit error %.2f m)",
                     quality_label, mean_dist_err)

    print("\n── Estimated anchor positions ──────────────────────────")
    print(f"  {'ID':>3}   {'x (m)':>8}   {'y (m)':>8}   {'z (m)':>8}")
    for aid, pos in zip(active_ids, pos_world):
        print(f"  {aid:>3}   {pos[0]:>8.3f}   {pos[1]:>8.3f}   {pos[2]:>8.3f}")
    print()
    return active_ids, pos_world


# ── CLI entry point ───────────────────────────────────────────────────────────

def _default_save_path() -> str:
    return f"loco_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"


def _upload_with_retry(estimator: LocoGeometryEstimator,
                       anchor_ids: List[int],
                       positions: np.ndarray) -> None:
    """Upload positions with interactive per-anchor retry.

    Sends all positions (3× each for reliability), then prompts the user
    to enter any anchor IDs that need to be retried (e.g. because the
    Crazyflie Client shows them as unconfigured).  Repeats until the
    user presses Enter with no IDs.
    """
    pos_by_id = {int(aid): np.array(pos)
                 for aid, pos in zip(anchor_ids, positions)}
    estimator.upload_positions(anchor_ids, positions)
    print(f"  Sent positions for {len(anchor_ids)} anchors (×3 each for reliability).")
    while True:
        raw = input(
            "  Enter anchor IDs to retry (e.g. '0 3 7'), "
            "or press Enter to continue: ").strip()
        if not raw:
            break
        try:
            retry_ids = [int(x) for x in raw.split()]
        except ValueError:
            print("  Invalid input — enter anchor IDs as integers.")
            continue
        valid   = [aid for aid in retry_ids if aid in pos_by_id]
        unknown = sorted(set(retry_ids) - set(valid))
        if unknown:
            print(f"  Ignoring unknown anchor IDs: {unknown}")
        if valid:
            retry_pos = np.array([pos_by_id[aid] for aid in valid])
            estimator.upload_positions(valid, retry_pos)
            print(f"  Re-sent: {valid}")


def _guided_estimation(uri: str,
                       known_ids: Optional[List[int]] = None,
                       load_file: Optional[str] = None,
                       save_file: Optional[str] = None):

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║        Loco Anchor Geometry Estimation               ║")
    print("║   (no custom firmware — uses existing log vars)      ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    # ── Ask: load saved session or collect new data? ───────────────────────
    if load_file is None:
        # Look for existing session files in the current directory
        existing = sorted(Path('.').glob('loco_session_*.json'))
        if existing:
            print("  Existing session files found:")
            for p in existing[-5:]:   # show up to last 5
                print(f"    {p}")
        print()
        raw = input("  Load a saved session [L] or collect new data [C]? ").strip().upper()
        if raw == 'L':
            if existing:
                default_path = str(existing[-1])
                load_file = input(
                    f"  Session file path  (Enter = {default_path}): ").strip()
                if not load_file:
                    load_file = default_path
            else:
                load_file = input("  Session file path: ").strip()
        print()

    # ── OFFLINE PATH: no CF required ───────────────────────────────────────
    if load_file:
        active_ids, dist_m, snapshots = load_session(load_file)
        print(f"\nPhase 3 — re-estimating anchor geometry from saved session…")
        anchor_ids, positions = _estimate_and_print(active_ids, dist_m, snapshots)
        print("\nDone (offline — no CF upload possible from saved session).")
        return

    # ── ONLINE PATH: collect fresh data ────────────────────────────────────
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        estimator = LocoGeometryEstimator(scf)

        # ── Phase 0: anchor discovery (or use provided IDs) ────────────────
        if known_ids:
            active_ids = sorted(set(known_ids))
            if len(active_ids) != len(known_ids):
                print(f"  WARNING: duplicate IDs removed. Using: {active_ids}")
            print(f"Phase 0 — using provided anchor IDs: {active_ids}\n")
        else:
            print("Phase 0 — discovering active anchor IDs.")
            print(f"  Scanning IDs {list(SCAN_ID_RANGE)}  "
                  f"(~{len(SCAN_ID_RANGE) * DISCOVERY_SETTLE_S:.0f} s).")
            print("  CF should be roughly central so all anchors are reachable.")
            print("  Tip: if an anchor is missed, re-run with --anchor-ids instead.\n")
            input("  Press Enter to start…")
            active_ids = estimator.discover_anchors()
            if not active_ids:
                print("  No anchors found!  Check SCAN_ID_RANGE and LPS setup.")
                estimator.stop()
                return
            print(f"  Active anchors: {active_ids}\n")

        # ── Phase 1: ToF matrix ────────────────────────────────────────────
        print("Phase 1 — building inter-anchor ToF matrix.")
        n_undirected = len(active_ids) * (len(active_ids) - 1) // 2
        print(f"  {len(active_ids)} anchors → {n_undirected} undirected pairs  "
              f"(~{n_undirected * (PAIR_RESET_WAIT_S + 0.5):.0f} s est. for ToF).")
        print("  Moving the CF around the space helps fill in pairs that are")
        print("  out of range from a single spot.  The anchor-to-anchor tof")
        print("  values come from the anchors' own broadcasts — CF position")
        print("  does NOT affect their accuracy, only which pairs you can hear.\n")
        input("  Press Enter when the CF is in position (or has finished roaming)…")
        active_ids, dist_m, tof_quality = estimator.build_tof_matrix(active_ids)
        valid = int(np.sum(np.isfinite(dist_m)) - len(active_ids))
        n_directed = len(active_ids) * (len(active_ids) - 1)
        print(f"  ToF matrix done.  Valid pairs after quality filter: {valid}/{n_directed}\n")

        # ── Phase 2: TDoA snapshots ────────────────────────────────────────
        print("Phase 2 — capture TDoA snapshots at known positions.")
        print("  You need AT LEAST three positions (same logic as Lighthouse):")
        print("    • Position 0: world-frame origin  (0, 0, 0)")
        print("    • Position 1: 1 m along X-axis    (1, 0, 0)   ← sets +X direction")
        print("    • Position 2: any floor point with y > 0      ← breaks ±Y ambiguity")
        print("      (the further from the origin, the better — e.g. 3–4 m away)")
        print("  OPTIONAL extras for better accuracy (recommended):")
        print("    • HIGH Z  e.g. (0, 0, 1.5)  ← improves z-accuracy for ceiling anchors")
        print("      (aim for 1.0–2.0 m; 0.5 m is marginal for anchors at z ≈ 3 m)")
        print("    • More diverse floor points spread across the room")
        print("      → the further from the origin, the more angular information")
        print("  After these static snapshots a mandatory SWEEP will automatically")
        print("  collect diverse measurements at many positions/heights to refine")
        print("  the estimate (like the Lighthouse calibration walk-around).")
        print("  Each position should be held still while the snapshot is taken.\n")

        snapshots_done = 0
        while True:
            if snapshots_done == 0:
                prompt      = "Place CF at the world origin (0, 0, 0)"
                default_pos = [0.0, 0.0, 0.0]
            elif snapshots_done == 1:
                prompt      = "Place CF 1 m along +X axis (1, 0, 0)  [sets +X direction]"
                default_pos = [1.0, 0.0, 0.0]
            elif snapshots_done == 2:
                prompt      = ("Place CF at any floor point with y > 0  "
                               "[breaks ±Y ambiguity — further from origin is better]")
                default_pos = None
            else:
                prompt      = ("Place CF at any diverse position  "
                               "[spread around the room — different x/y/z gives more info]")
                default_pos = None

            print(f"\n  {prompt}")
            raw = input("  Enter [x y z] in metres"
                        + (" (Enter = default)" if default_pos else "")
                        + ", or 'done': ").strip()

            if raw.lower() == 'done':
                if snapshots_done >= 3:
                    break
                else:
                    print(f"  Need at least 3 snapshots ({snapshots_done} done) — continue.")
                continue

            if raw == '' and default_pos is not None:
                pos = default_pos
            else:
                try:
                    pos = [float(v) for v in raw.split()]
                    if len(pos) != 3:
                        raise ValueError
                except ValueError:
                    print("  Invalid.  Enter three numbers separated by spaces.")
                    continue

            label = f"snap_{snapshots_done}"
            estimator.capture_tdoa_snapshot(pos, active_ids, label=label)
            snapshots_done += 1

            if snapshots_done >= 3:
                if input("\n  Add another position for better accuracy? [y/N] ").strip().lower() != 'y':
                    break

        # ── Auto-save session ──────────────────────────────────────────────
        out_path = save_file or _default_save_path()
        save_session(out_path, active_ids, dist_m, estimator._snapshots)
        print(f"  Session saved → {out_path}  (re-run with --load-file to replay)\n")

        # ── Phase 3: rough estimate ────────────────────────────────────────
        print("Phase 3 — estimating rough anchor geometry…")
        anchor_ids, positions = estimator.estimate(active_ids, dist_m)

        # z-flip heuristic: LPS anchors are always above the floor.
        # If the mean estimated z is negative, the ±z reflection ambiguity
        # was resolved the wrong way — flip all z values.
        if float(np.mean(positions[:, 2])) < 0:
            positions = positions.copy()
            positions[:, 2] = -positions[:, 2]
            print("  Applied z-flip (mean anchor z was negative → reflected above floor).")

        # ── Phase 4: upload rough geometry ────────────────────────────────
        print("\nPhase 4 — uploading rough anchor positions to the CF…")
        _upload_with_retry(estimator, anchor_ids, positions)

        # ── Phase 5: sweep refinement (mandatory, Lighthouse-style) ───────
        if not estimator._has_cf_pos:
            print("\n  Note: CF position logging not available — sweep skipped.")
            print("  (This is normal if stateEstimate.x/y/z is not in the log table.)")
            estimator.stop()
            print("\nDone.")
            return

        print("\nPhase 5 — sweep refinement (Lighthouse-style).")
        print("  The CF can now localise itself using the rough anchor geometry.")
        print("  During each sweep, move or fly the CF through the entire space.")
        print("  The sweep records TDoA at the LPS-estimated CF position for every")
        print("  anchor pair — exactly like the Lighthouse calibration walk-around.")
        print("  This significantly improves accuracy (especially anchor z-coordinates).")
        print("  Visit corners, near-anchor areas, and different heights.\n")

        sweep_idx = 0
        while True:
            input(f"  Start moving the CF, then press Enter to begin sweep {sweep_idx}…")
            n_cap = estimator.capture_sweep(active_ids, sweep_index=sweep_idx)
            if n_cap == 0:
                print("  No valid measurements captured — check that LPS is running.")
                break

            save_session(out_path, active_ids, dist_m, estimator._snapshots)
            print(f"  Session updated → {out_path}\n")

            print(f"  Re-estimating with {len(estimator._snapshots)} total snapshots…")
            anchor_ids, positions = estimator.estimate(active_ids, dist_m)

            if float(np.mean(positions[:, 2])) < 0:
                positions = positions.copy()
                positions[:, 2] = -positions[:, 2]
                print("  Applied z-flip.")

            print("\n  Uploading refined positions…")
            _upload_with_retry(estimator, anchor_ids, positions)

            sweep_idx += 1
            if input("\n  Run another sweep for further refinement? [y/N] ").strip().lower() != 'y':
                break

        estimator.stop()

    print("\nDone.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Automatic LPS anchor geometry estimation (no custom firmware)')
    parser.add_argument('--uri', default='radio://0/80/2M/E7E7E7E7E7',
                        help='Crazyflie URI (only needed when collecting new data)')
    parser.add_argument('--anchor-ids', nargs='+', type=int, default=None,
                        metavar='ID',
                        help='Skip auto-discovery and use these anchor IDs directly. '
                             'Example: --anchor-ids 0 1 2 3 4 5 6 7')
    parser.add_argument('--load-file', metavar='FILE', default=None,
                        help='Load a previously saved session JSON and re-run '
                             'estimation offline (no CF required). '
                             'Example: --load-file loco_session_20260227_195200.json')
    parser.add_argument('--save-file', metavar='FILE', default=None,
                        help='Override the auto-generated save filename. '
                             'Default: loco_session_YYYYMMDD_HHMMSS.json')
    args = parser.parse_args()
    _guided_estimation(args.uri,
                       known_ids=args.anchor_ids,
                       load_file=args.load_file,
                       save_file=args.save_file)
