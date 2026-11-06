#!/bin/bash
# One grasp-stance attempt with the backend primitives (session 727e14).
# usage: bash code/puppy_box/attempt6.sh <tag> [depth .045] [in_m .10] [lower .06] [lift_z .265] [step_m .07] [shift 0]
cd "$(dirname "$0")/../.."
TAG=${1:-run}; D=${2:-0.045}; IN=${3:-0.10}; LO=${4:-0.06}; LZ=${5:-0.265}; SM=${6:-0.07}; SH=${7:-0}
timeout 120 python code/puppy_box/v6.py look $TAG 2>&1 | grep -v '"perceive"' | cut -c1-700
[ ${PIPESTATUS[0]} -ne 0 ] && { echo "exit 2 (look)"; exit 2; }
timeout 300 python code/puppy_box/v6.py lift $D $LZ 1 $SH | cut -c1-900
rc=${PIPESTATUS[0]}
if [ $rc -ne 0 ]; then
  echo "exit $rc (lift not held / width falling): opening, hand back, HOME"
  timeout 120 python - <<'PY'
import sys; sys.path.insert(0, '.'); sys.path.insert(0, 'code/puppy_box')
from chain import ctx
ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
st = ctx.value('r1_status'); h = st['proprio']['grasp_points_pelvis_m']['right']
ctx.value('r1_move_hand', arm='right', position=[h[0] - 0.04, h[1] - 0.02, max(h[2] + 0.04, 0.12)], approach=[1, 0, 0.2], fingers=[0, 1, 0], gripper_width_m=0.07, secs=1.5)
ctx.value('r1_move_hand', arm='right', position=[0.25, -0.24, 0.13], approach=[1, 0, 0.2], fingers=[0, 1, 0], gripper_width_m=0.07, secs=2)
PY
  timeout 60 python code/puppy_box/home.py 0.6
  exit 4
fi
timeout 300 python code/puppy_box/v6.py carry $IN $LO 5 $SM | cut -c1-900
rc=${PIPESTATUS[0]}; echo "exit $rc (carry)"; echo "NEXT: python code/puppy_box/v6.py out 0.2 (never home.py after a carry)"
