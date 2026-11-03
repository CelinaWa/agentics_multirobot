"""No-motion grid: IK err for palm points over commanded waist (yaw, pitch) candidates. reachgrid.py y"""
import sys, os
W = os.getcwd()
for d in ('skill-library/skills/plush-pick-place-box-same-side/code', 'skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import mv
from api.runtime import Context
c = mv.c
with Context() as ctx:
    row, meas, _ = c.pps.last_row(ctx)
y = float(sys.argv[1])
for wy, wp in [(0, 0), (0.35, 0), (0.7, 0), (0.7, 0.1), (0.7, 0.2), (0, 0.15), (0.35, 0.15)]:
    cmd = list(row[9:38]); cmd[2] = wy; cmd[8] = wp
    out = []
    for x in (0.44, 0.47, 0.50):
        for z in (0.24, 0.28):
            err, t = mv.best(cmd, [x, y, z], 0.0, 'right')
            out.append(f'{x:.2f}/{z:.2f}:{err:.3f}')
    g, _ = mv.best(cmd, [0.243, y, 0.0], 0.0, 'right')
    print(f'yaw {wy} pitch {wp} grasp {g:.3f} |', ' '.join(out), flush=True)
