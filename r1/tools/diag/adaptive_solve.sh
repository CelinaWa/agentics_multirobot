#!/bin/bash
# BLIND adaptive controller (the task's observe/retry/change-approach policy, <=5 resets): for any seed, try the
# edge-overhang grasp with a sequence of (lean, standoff) configs, then fall back to the flat grasp; stop at the
# first IN_BOX. Same procedure for every pose -> the controller finds the working method+config itself.
cd /home/junjieye/workspace/multirobot/r1
SD=$1; PY=/opt/r1venv/bin/python
for CFG in "0 0.28" "1 0.28" "1 0.32"; do
  set -- $CFG; L=$1; EA=$2
  R=$(timeout 150 $PY tools/scratch/diag/full_pipe_oh_ft.py --seed $SD --lean $L --edge_ahead $EA 2>&1 | grep -oE '"IN_BOX": (true|false)' | tail -1)
  [ "$R" = '"IN_BOX": true' ] && { echo "$SD -> SOLVED by overhang(lean=$L, standoff=$EA)"; exit 0; }
done
R=$(timeout 150 $PY tools/scratch/diag/flat_pipe_ft.py $SD 2>&1 | grep -oE '"IN_BOX": (true|false)' | tail -1)
[ "$R" = '"IN_BOX": true' ] && { echo "$SD -> SOLVED by flat grasp"; exit 0; }
echo "$SD -> UNSOLVED"
