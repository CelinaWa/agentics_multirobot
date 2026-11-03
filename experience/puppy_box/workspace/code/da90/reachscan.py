"""Offline reach scan on the commanded row: pressed grasp (x, y, z) over waist yaw x roll (x pitch).
Usage: PYTHONPATH=. python code/da90/reachscan.py x y z [yaw_deg]"""
import sys, json, itertools
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.runtime import Context
x, y, z = map(float, sys.argv[1:4]); gyaw = float(sys.argv[4]) if len(sys.argv) > 4 else 15.0
with Context() as ctx:
    cmd = c3.cmd_row(ctx)
res = []
for w, r, pt in itertools.product((0.3, 0.55, 0.7, 0.9), (0.0, 0.15, 0.25), (0.0, 0.1)):
    q = list(cmd); q[2], q[5], q[8] = w, r, pt
    e, _ = c3.best(q, (x, y, z), gyaw)
    res.append((round(e, 3), w, r, pt))
for t in sorted(res)[:8]:
    print(t)
