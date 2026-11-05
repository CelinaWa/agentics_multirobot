#!/bin/bash
# VERIFY+RETRY (task's reset policy): fresh server per attempt; sweep flat + overhang (lean, standoff) configs;
# verify held->in_box; stop at first success. Configs mirror the validated adaptive_solve + the pin fix.
cd /home/junjieye/workspace/multirobot/r1; SD=$1; PORT=$2; PY=/opt/r1venv/bin/python
for CFG in "flat 0.27 0" "edge 0.28 0" "edge 0.28 1" "edge 0.32 1" "edge 0.32 0" "edge 0.26 1"; do
  set -- $CFG; M=$1; EA=$2; LN=$3
  $PY sim/r1_sim_server.py --scene tools/scratch/scene_test_ft/$SD/scene_r1_dog_box.xml --layout tools/scratch/scene_test_ft/$SD/layout_*.json --port $PORT >/tmp/vr_$PORT.log 2>&1 & SRV=$!
  sleep 14
  R=$(timeout 200 $PY tools/scratch/diag/attempt.py $PORT $M $EA $LN 2>&1 | grep -oE '"IN_BOX": (true|false)' | tail -1)
  kill $SRV 2>/dev/null; sleep 2
  [ "$R" = '"IN_BOX": true' ] && { echo "$SD -> SOLVED by $M(lean=$LN, standoff=$EA)"; exit 0; }
done
echo "$SD -> UNSOLVED"
