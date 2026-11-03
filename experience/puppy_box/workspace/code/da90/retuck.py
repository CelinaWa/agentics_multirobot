"""Re-tuck both arms to clean values (crouch/walk let the wrists creep: yaw +-0.7 after 3 crouches) and measure the edge.
Usage: PYTHONPATH=. python code/da90/retuck.py"""
import sys, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import stance as s
from api.runtime import Context
c = s.c
TUCK = {'shoulder_pitch': 0.45, 'shoulder_roll': -0.2, 'shoulder_yaw': 0.0, 'elbow': -0.5,
        'wrist_roll': 0.0, 'wrist_pitch': 0.0, 'wrist_yaw': 0.0}
with Context() as ctx:
    c.pps.ramp(ctx, dict(c.mirror('left', TUCK), **c.mirror('right', TUCK)), 3.0, hands=(0, 0))
    time.sleep(1.0)
    ob, path = c.observe(ctx, 'retuck')
    e, tz = s.edge_of(path)
    print(json.dumps(dict(edge=round(e, 3), tz=tz, yaw=s.yaw_deg(ob), tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])))
