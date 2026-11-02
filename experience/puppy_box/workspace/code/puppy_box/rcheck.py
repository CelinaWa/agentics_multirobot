"""IK (commanded row, random seeds) for grasp + release targets over (lean, yaw cmd). Usage: rcheck.py gx gy gz rx ry rz"""
import sys, os, json, numpy as np
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
from api.runtime import Context
g = tuple(float(v) for v in sys.argv[1:4]); r = tuple(float(v) for v in sys.argv[4:7])
with Context() as ctx:
    cmd = pbb.pb3.cmd_row(ctx)
out = []
for lean in (0.0, 0.1):
    for yaw in (0.0, 0.3, 0.5, 0.7):
        q = list(cmd); q[2] = yaw; q[8] = lean
        eg = pbb.best(q, g, 0.0)[0]; er = pbb.best(q, r, -10)[0]
        out.append((lean, yaw, round(eg, 3), round(er, 3))); print(out[-1], flush=True)
