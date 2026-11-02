"""No-motion reach table: python code/puppy_box/reach.py SIDE 'x,y,z;x,y,z' [yaw_deg] [waist_yaw]
Palm-down IK residual on the measured joints (optionally with a hypothetical waist yaw)."""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library/skills/palm-pose-servo/code'))
from api.runtime import Context
import run as pps

if __name__ == '__main__':
    side = sys.argv[1]; pts = [tuple(map(float, s.split(','))) for s in sys.argv[2].split(';')]
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    with Context() as ctx:
        _, meas, _ = pps.last_row(ctx)
        meas = list(meas)
        if len(sys.argv) > 4:
            meas[2] = float(sys.argv[4])
        for p in pts:
            t, err, _ = pps.solve(meas, side, p, pps.palm_quat(yaw, side))
            print(p, round(float(err), 3), {k.replace('_joint', ''): round(v, 2) for k, v in t.items()})
