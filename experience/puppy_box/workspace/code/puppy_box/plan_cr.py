"""Reach table: for waist (yaw, roll) candidates, IK error (commanded row) for grasp press, hover and release points."""
import sys, os, json, numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code')); sys.path.insert(0, ROOT)
import pb3
from reach2 import best
from api.runtime import Context

def table(q0, pts, waists):
    out = []
    for w, r in waists:
        q = list(q0); q[2] = w; q[5] = r
        errs = [round(best(q, xyz, yaw)[0], 3) for xyz, yaw in pts]
        out.append(((w, r), errs)); print((w, r), errs, flush=True)
    return out

if __name__ == '__main__':
    gx, gy, top = (float(v) for v in sys.argv[1:4]); rx, ry, rz = (float(v) for v in sys.argv[4:7])
    with Context() as ctx:
        q0 = pb3.cmd_row(ctx)
    pts = [((gx, gy, top - 0.03), 15), ((gx, gy, top + 0.10), 15), ((rx, ry, rz), 15), ((rx, ry, rz), -25), ((rx, ry, rz), 60)]
    W = [(w, r) for w in (-0.5, -0.3, 0.0, 0.3, 0.6, 0.9) for r in (0.0, 0.15)]
    table(q0, pts, W)
