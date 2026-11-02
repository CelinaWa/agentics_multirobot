"""Restore standing height after a walk (crouch 0, forward FWD), observe, measure the table edge."""
import sys, os, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import stance as S
from api.runtime import Context
with Context() as ctx:
    if '--nocrouch' not in sys.argv:
        ctx.value('g1_crouch', pelvis_drop_m=0, forward_rad=S.FWD, arms_home=False, secs=3)
    time.sleep(1.0)
    ob, path = S.c.observe(ctx, 'stance')
    e, tz = S.edge_of(path)
    print(json.dumps(dict(edge=round(e, 3), tz=round(tz, 3), yaw=S.yaw_deg(ob), tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'], json=path)))
