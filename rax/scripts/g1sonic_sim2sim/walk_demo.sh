#!/bin/bash
# Fresh office sim (lockstep) + gantry, then walk_check.py; frames and trace in OUT (default /tmp/walk).
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-/tmp/walk}
PY="env -u PYTHONPATH LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu /opt/sonic/.venv_sim/bin/python"
LOCKSTEP=1 SCENE=/opt/sonic/gear_sonic/data/robot_model/model_data/g1/scene_office.xml bash "$HERE/run_sim2sim.sh" start >/dev/null
for i in $(seq 1 60); do sleep 5; grep -q "Init Done" /opt/sonic/logs/deploy.log 2>/dev/null && grep -q READY /opt/sonic/logs/camera.log 2>/dev/null && break; done
cd /opt/sonic
$PY "$HERE/gantry.py" > /opt/sonic/logs/gantry.log 2>&1 &
sleep 4
rm -rf "$OUT"
timeout 400 $PY "$HERE/walk_check.py" --out "$OUT" --calls 8 ${PLANNER:+--planner} ${WALKS:+--walks "$WALKS"}
bash "$HERE/run_sim2sim.sh" stop > /dev/null
