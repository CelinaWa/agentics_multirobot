"""Drag the plush toward the robot: with the hand closed on it, move the palm cmd by DX (default -0.03) at cmd z Z, then open
and raise to 0.20. Usage: GY=<yaw> drag.py DX Z"""
import sys, os, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import bb_8cef10 as B
pbb = B.pbb
from api.runtime import Context
dx = float(sys.argv[1]) if len(sys.argv) > 1 else -0.03
z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.04
st = pbb.load(); gx, gy = st['grasp']
with Context() as ctx:
    r = pbb.move(ctx, (gx + dx, gy, z), pbb.pb3.GRASP_YAW, 3.0); r.pop('ob', None)
    out = dict(drag=dict(palm=r.get('palm'), tilt=r.get('tilt'), fingers=r.get('fingers'), why=r.get('why')))
    pbb.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    r2 = pbb.move(ctx, (gx + dx - 0.02, gy, 0.20), pbb.pb3.GRASP_YAW, 3.0, max_jump=1.6); r2.pop('ob', None)
    out['raise'] = dict(palm=r2.get('palm'), tilt=r2.get('tilt'), frame=r2.get('frame'), ok=r2.get('ok'))
    print(json.dumps(out, default=float))
