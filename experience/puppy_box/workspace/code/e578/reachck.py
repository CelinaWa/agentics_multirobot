"""No-motion reach check on the commanded row: reachck.py x,y,z[,yaw] ... -> IK err per point (palm down)."""
import sys, os, json
W = os.getcwd()
for d in ('skill-library/skills/plush-pick-place-box-same-side/code', 'skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import mv
from api.runtime import Context
c = mv.c
with Context() as ctx:
    row, meas, _ = c.pps.last_row(ctx)
cmd = list(row[9:38])
for a in sys.argv[1:]:
    v = [float(x) for x in a.split(',')]
    yaw = v[3] if len(v) > 3 else 0.0
    err, t = mv.best(cmd, v[:3], yaw, 'right')
    print(a, 'err', round(err, 4), {k: round(x, 2) for k, x in t.items()} if err < 0.02 else '')
