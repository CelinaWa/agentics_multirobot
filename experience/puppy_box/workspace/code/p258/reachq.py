"""Offline IK reach errors on the COMMANDED row with a given waist (yaw, roll) for grasp points and the release.
  PYTHONPATH=. python code/p258/reachq.py"""
import sys, itertools
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd')
import cr  # noqa
pb3 = cr.pb3
from api.runtime import Context
with Context() as ctx:
    q0 = list(pb3.cmd_row(ctx))
top = 0.035
for wy, wr in [(0.7, 0.15), (0.5, 0.15), (0.3, 0.15), (0.0, 0.0)]:
    q = list(q0); q[2], q[5] = wy, wr
    out = []
    for y in (-0.34, -0.38, -0.40, -0.42):
        for yaw in (-30, -12, 0):
            e = pb3.best(q, (0.29, y, top - 0.04), yaw)[0]
            out.append(f'y{y} yaw{yaw}:{e:.3f}')
    er = pb3.best(q, (0.43, 0.02, 0.30), 15.0)[0]
    print(f'waist {wy}/{wr} release {er:.3f} |', ' '.join(out), flush=True)
