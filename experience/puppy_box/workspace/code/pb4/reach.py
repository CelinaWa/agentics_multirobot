"""No-motion grid: IK err (commanded row) of the right palm (palm down, fingers forward) for the grasp and the release
over commanded waist (yaw, pitch). reach.py gx gy gz ry"""
import sys, os
W = os.getcwd()
for d in ('skill-library/skills/plush-pick-place-box-same-side/code', 'skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import mv
from api.runtime import Context
c = mv.c
with Context() as ctx:
    row, meas, _ = c.pps.last_row(ctx)
gx, gy, gz, ry = map(float, sys.argv[1:5])
for wy, wp in [(0, 0), (0, 0.1), (0.35, 0), (0.7, 0), (0.7, 0.1)]:
    cmd = list(row[9:38]); cmd[2] = wy; cmd[8] = wp
    out = []
    for x in (0.48, 0.51, 0.54):
        for z in (0.22, 0.27):
            err, t = mv.best(cmd, [x, ry, z], 0.0, 'right')
            out.append(f'{x:.2f}/{z:.2f}:{err:.3f}')
    g, _ = mv.best(cmd, [gx, gy, gz], 0.0, 'right')
    print(f'yaw {wy} pitch {wp} grasp {g:.3f} |', ' '.join(out), flush=True)
