"""Open the right hand, then raise the palm straight up to z (default plush top + 0.13) at the grasp xy and grasp yaw.
  TABLE_Z=.. PYTHONPATH=. python code/p258/open_up.py [z]"""
import sys, os, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd')
import cr  # noqa
pb3 = cr.pb3
from api.runtime import Context
st = pb3.load(); pb3.GRASP_YAW = cr.lr.GRASP_YAW = st['gyaw']
with Context() as ctx:
    pb3.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    row, _, _ = pb3.c.pps.last_row(ctx)
    pal = pb3.c.pps.link_points(row[9:38], 'right')['right_palm']
    zt = float(sys.argv[1]) if len(sys.argv) > 1 else st['top'] + 0.13
    z = pal[2]; r = None
    while z < zt - 1e-3:
        z = min(zt, z + 0.04)
        r = pb3.move(ctx, (pal[0], pal[1], z), st['gyaw'], 2.5, max_jump=0.8); r.pop('ob', None)
        if not r['ok']:
            print(json.dumps(dict(ok=False, why=r['why'], z=z))); sys.exit(1)
    print(json.dumps(dict(ok=True, palm=r and r['palm'], fingers=r and r['fingers'], tilt=r and r['tilt'], frame=r and r['frame'])))
