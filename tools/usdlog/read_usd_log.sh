#!/usr/bin/env bash
#
# Read the current uSD log file off a Crazyflie over CRTP (radio or USB) using
# the Rust `cfcli` and save it locally, then optionally decode/replay it.
#
# The uSD deck exposes the log file through the memory subsystem (memory type
# "MicroSD", 0x16), but ONLY once logging is stopped -- while logging is active
# the reported file size is 0. This script therefore stops logging first.
#
# IMPORTANT: do not run this (or keep any radio link up) while the Crazyflie is
# flying / actively collecting a data set -- a radio connection degrades Loco
# performance. Read the log after the run has finished.
#
# Usage:
#   tools/usdlog/read_usd_log.sh <uri> <output.bin> [--replay <anchors.yaml>]
#   tools/usdlog/read_usd_log.sh <uri> --check-only
#
# Examples:
#   tools/usdlog/read_usd_log.sh radio://0/100/2M/F00D2BEFED run01.bin
#   tools/usdlog/read_usd_log.sh usb://0 run01.bin --replay anchors.yaml
#   tools/usdlog/read_usd_log.sh radio://0/100/2M/F00D2BEFED --check-only
#
# --check-only runs the integrity gates (dropped events, SD write errors,
# non-zero file size) and stops logging, but transfers no data. Use it before
# pulling the card by hand: radio readback of a multi-MB log takes far longer
# than reading the card directly, but the counters only exist on the drone.
#
set -euo pipefail

CFCLI="${CFCLI:-cfcli}"

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <uri> <output.bin> [--replay <anchors.yaml>]" >&2
  echo "       $0 <uri> --check-only" >&2
  exit 2
fi

URI="$1"
CHECK_ONLY=0
OUT=""
if [[ "$2" == "--check-only" ]]; then
  CHECK_ONLY=1
  shift 2
else
  OUT="$2"
  shift 2
fi

REPLAY_ANCHORS=""
if [[ "${1:-}" == "--replay" ]]; then
  if [[ "$CHECK_ONLY" == "1" ]]; then
    echo "!! --replay needs a downloaded log; it cannot be combined with --check-only." >&2
    exit 2
  fi
  REPLAY_ANCHORS="${2:?--replay needs an anchors.yaml path}"
fi

echo ">> Stopping logging (usd.logging = 0) so the file becomes readable..."
"$CFCLI" -u "$URI" param set usd.logging=0

echo ">> Checking for dropped uSD events (usd.eventsRequested vs usd.eventsWritten)..."
# The counters are reset when logging starts and keep their final values after
# stop. `log print` streams one CSV line per sample; bound it with --timeout
# and take the last complete sample.
DROP_LINE="$("$CFCLI" -u "$URI" --timeout 3000 --csv log print usd.eventsRequested,usd.eventsWritten 2>/dev/null | tail -n 1 || true)"
EVT_REQ=""
EVT_WR=""
if [[ -n "$DROP_LINE" ]]; then
  EVT_REQ="$(echo "$DROP_LINE" | awk -F',' '{print $(NF-1)}' || true)"
  EVT_WR="$(echo "$DROP_LINE" | awk -F',' '{print $NF}' || true)"
fi

if [[ ! "$EVT_REQ" =~ ^[0-9]+$ || ! "$EVT_WR" =~ ^[0-9]+$ ]]; then
  echo "!! WARNING: could not read usd.eventsRequested/usd.eventsWritten." >&2
  echo "   Firmware without the drop counters? Losslessness NOT verified." >&2
elif [[ "$EVT_REQ" != "$EVT_WR" ]]; then
  echo "!! DROPPED EVENTS: eventsRequested=$EVT_REQ eventsWritten=$EVT_WR ($((EVT_REQ - EVT_WR)) lost)." >&2
  echo "   The log has holes; do NOT trust replay results from it." >&2
  echo "   Mitigations: increase the ring buffer size (line 2 of config.txt)" >&2
  echo "   or switch to config_tdoa_candidates_lean.txt." >&2
  exit 1
else
  echo ">> OK: no dropped events (eventsRequested = eventsWritten = $EVT_REQ)."
fi

echo ">> Checking for SD write failures (usd.writeError)..."
WERR_LINE="$("$CFCLI" -u "$URI" --timeout 3000 --csv log print usd.writeError 2>/dev/null | tail -n 1 || true)"
WERR="$(echo "$WERR_LINE" | awk -F',' '{print $NF}' || true)"
if [[ ! "$WERR" =~ ^[0-9]+$ ]]; then
  echo "!! WARNING: could not read usd.writeError (firmware without it?)." >&2
  echo "   A silently aborted log cannot be ruled out." >&2
elif [[ "$WERR" != "0" ]]; then
  echo "!! SD WRITE FAILURE: usd.writeError=$WERR (FatFS FRESULT)." >&2
  echo "   Logging stopped mid-run; the log is incomplete. Do NOT use it." >&2
  echo "   Check the microSD card seating/card health and retry." >&2
  exit 1
else
  echo ">> OK: no SD write failures."
fi

echo ">> Listing memories to find the microSD file size..."
# mem list --csv is machine readable; find the MicroSD row and take its size.
# The size column is the last numeric field of the MicroSD line. If auto-detect
# fails, override with:  USD_SIZE=<bytes> tools/usdlog/read_usd_log.sh ...
MEM_LIST="$("$CFCLI" -u "$URI" --csv mem list)"
echo "$MEM_LIST"

SIZE="${USD_SIZE:-}"
if [[ -z "$SIZE" ]]; then
  SIZE="$(echo "$MEM_LIST" | awk -F',' 'tolower($0) ~ /microsd/ { for (i=NF;i>=1;i--) if ($i ~ /^[0-9]+$/) { print $i; break } }' | head -n1)"
fi

if [[ -z "$SIZE" || "$SIZE" == "0" ]]; then
  echo "!! Could not determine a non-zero microSD file size." >&2
  echo "   Is a log present and is logging stopped? You can force a size with USD_SIZE=<bytes>." >&2
  echo "   A size of 0 right after a run usually means the file was never closed:" >&2
  echo "   power loss / reboot during logging, or an SD write failure (see the" >&2
  echo "   usd.writeError check above). The file on the card will be 0 bytes too." >&2
  exit 1
fi

if [[ "$CHECK_ONLY" == "1" ]]; then
  echo ">> OK: integrity checks passed; log is $SIZE bytes and closed."
  echo "   Logging is stopped, so the card can be pulled safely."
  echo "   Replay it with:"
  echo "     python3 -m tools.usdlog.replay_tdoa <log.bin> --anchors anchors.yaml"
  exit 0
fi

echo ">> Reading $SIZE bytes from MicroSD into $OUT ..."
"$CFCLI" -u "$URI" mem read MicroSD --offset 0 --length "$SIZE" --output "$OUT"

echo ">> Done. Wrote $OUT ($(wc -c < "$OUT") bytes)."

if [[ -n "$REPLAY_ANCHORS" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
  # The replay needs numpy and the cffirmware bindings built for the *same*
  # interpreter (see bindings/README). The repo venv has both; a bare python3
  # usually has neither. Override with PYTHON=... if you keep them elsewhere.
  PYTHON="${PYTHON:-}"
  if [[ -z "$PYTHON" ]]; then
    if [[ -x "$REPO_ROOT/.venv/bin/python" ]]; then
      PYTHON="$REPO_ROOT/.venv/bin/python"
    else
      PYTHON="python3"
    fi
  fi
  echo ">> Replaying $OUT against $REPLAY_ANCHORS (using $PYTHON) ..."
  ( cd "$REPO_ROOT" && PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}build" \
      "$PYTHON" -m tools.usdlog.replay_tdoa "$OUT" --anchors "$REPLAY_ANCHORS" )
fi
