"""One more step-up walk (arms already tucked), restore height, re-measure the edge. Usage: walk_more.py FORWARD_M"""
import sys, os, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import stance as S
from api.runtime import Context
with Context() as ctx:
    w = ctx.value('g1_walk', forward_m=float(sys.argv[1]), carry=True, speed_mps=0.2)
    ctx.value('g1_crouch', pelvis_drop_m=0, forward_rad=S.FWD, arms_home=False, secs=3)
    time.sleep(1.0)
    ob, path = S.c.observe(ctx, 'stance')
    e, tz = S.edge_of(path)
    print(json.dumps(dict(odo=(w.get('feedback') or {}).get('odometry_m'), edge=round(e, 3), tz=round(tz, 3), yaw=S.yaw_deg(ob),
                          tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'], json=path)))
