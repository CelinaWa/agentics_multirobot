"""Ramp the right-arm command to the measured right-arm joints (removes an untrackable command). Waist/left unchanged."""
import sys, os, json, time
W = os.getcwd()
for d in ('skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import controller as c
from api.runtime import Context
with Context() as ctx:
    row, meas, _ = c.pps.last_row(ctx)
    t = {n: float(meas[c.J.index(n)]) for n in c.J if n.startswith('right_') and ('shoulder' in n or 'elbow' in n or 'wrist' in n)}
    c.pps.ramp(ctx, t, float(sys.argv[1]) if len(sys.argv) > 1 else 2.0)
    time.sleep(1.0)
    print(json.dumps({k: round(v, 2) for k, v in t.items()}))
