#!/bin/bash
# Full chain from the reset start (session 3d54de): HOME + head .6 -> walk-in v5 -> cl.py (look/waist look, grasp
# depth .045, chain lift, top gate, r1_carry_release, retreat). Then ask the operator (sim) whether it is inside.
#   bash code/puppy_box/full.sh <tag>      logs: output/logs/<tag>_walk.log, output/logs/<tag>.log
cd "$(dirname "$0")/../.."
TAG=${1:-run}
python code/puppy_box/home.py 0.6 2>&1 | tail -1
# R7: paw .378 -> cl.py lift step 0 refused (IK, G x .42) -> accept only paw x <= .37
python code/puppy_box/walkin5.py 0.35 0.375 5 > output/logs/${TAG}_walk.log 2>&1; echo "walkin exit $?"
grep -v '"step": "perceive"' output/logs/${TAG}_walk.log | cut -c1-220
python code/puppy_box/cl.py ${TAG} > output/logs/${TAG}.log 2>&1; echo "cl exit $?"
grep -E '"step": "(look|grasp|lift_refused|lifted|carry|RESULT)"|Traceback|Error' output/logs/${TAG}.log | cut -c1-600
# in-place retry (R8: a safe lift abort left the paw overhanging; the retry placed it): once, only after a lift abort,
# a weak-grip put-back or a grasp miss
if grep -qE '"why": "(lift: lift aborted|lift: weak grip|grasp: )' output/logs/${TAG}.log; then
  echo "retry in place"
  python code/puppy_box/cl.py ${TAG}b > output/logs/${TAG}b.log 2>&1; echo "cl retry exit $?"
  grep -E '"step": "(look|grasp|lift_refused|lifted|carry|RESULT)"|Traceback|Error' output/logs/${TAG}b.log | cut -c1-600
fi
