#!/usr/bin/env bash
# One-minute power-on check of the G1 under SONIC before a task session: status, a 10 s standing hold and the
# arm moves, through the agent's backend. Compare the printed numbers with G1.md; if the tracking RMSE or the
# hand errors are twice the recorded ones, or the tilt climbs, stop and look before starting the agent.
#   bash scripts/g1_check_quick.sh <session_root>
# Needs: SONIC "Init Done", the console open on the same session, s pressed (standing) and SPACE (armed).
set -uo pipefail
ROOT=${1:?session root}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."
PY=${RAX_PYTHON:-python3}
export PYTHONPATH=$PWD
for s in status hold arms; do
    echo; echo "=================== $s"
    "$PY" -u scripts/g1_motion_check.py "$ROOT" "$s" || { echo "step $s failed: do not start the agent"; exit 1; }
done
echo; echo "quick check done: $ROOT/runtime/characterization/g1_motion_check.jsonl (last 3 lines)"
