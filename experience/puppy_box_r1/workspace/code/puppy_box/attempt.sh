#!/bin/bash
# One grasp-stance attempt of the v5 recipe with up to 2 lift aborts/re-grasps (session 2fbfb7).
# usage: bash code/puppy_box/attempt.sh [tag]
cd "$(dirname "$0")/../.."
TAG=${1:-run}
timeout 60 python code/puppy_box/home.py 0.6
PASTS=(0 0.045 0.055 0.035)   # re-grasp depth varies after a lift abort (st19: same depth stayed thin 0/2)
for k in 1 2 3; do
  [ $k -gt 1 ] && timeout 60 python code/puppy_box/home.py 0.6   # the aborted hand occludes the paw
  OVERHANG_MIN=${OVERHANG_MIN:-$([ $k -gt 1 ] && echo 0.02 || echo 0.05)} timeout 120 python code/puppy_box/chain.py look > output/look_${TAG}_$k.log 2>&1 || { grep STOP output/look_${TAG}_$k.log | cut -c1-200; exit 2; }
  grep '"selected"' output/look_${TAG}_$k.log | cut -c1-200
  timeout 120 python code/puppy_box/chain.py pre 2>&1 | grep -v perceive | tail -1 | cut -c1-160
  timeout 400 python code/puppy_box/v4.py go ${PASTS[$k]} -0.004 ${IN_M:-0.06} 4 0.06 1 0 1 > output/v4_${TAG}_$k.log 2>&1
  rc=$?
  grep -v '"perceive"\|"moved"\|"approach"' output/v4_${TAG}_$k.log | cut -c1-260
  echo "exit $rc (try $k)"
  [ $rc -ne 4 ] && exit $rc
done
