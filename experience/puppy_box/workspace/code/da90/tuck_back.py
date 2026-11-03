"""Right arm from a raised pose to the tuck by the back path (shoulder pitch 0.9 + wrists 0 + roll -0.2 first), then
both arms to the clean tuck; waist 0; measure the edge. Usage: PYTHONPATH=. python code/da90/tuck_back.py"""
import sys, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import stance as s
from api.runtime import Context
c = s.c
TUCK = {'shoulder_pitch': 0.45, 'shoulder_roll': -0.2, 'shoulder_yaw': 0.0, 'elbow': -0.5,
        'wrist_roll': 0.0, 'wrist_pitch': 0.0, 'wrist_yaw': 0.0}
with Context() as ctx:
    c.ppg.ctx = None
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, dict(c.mirror('right', {'shoulder_pitch': 0.9, 'shoulder_roll': -0.2, 'shoulder_yaw': 0.0,
                                            'wrist_roll': 0.0, 'wrist_pitch': 0.0, 'wrist_yaw': 0.0}),
                         waist_yaw_joint=0.0, waist_roll_joint=0.0, waist_pitch_joint=0.0), 3.0, hands=(0, 0))
    c.pps.ramp(ctx, dict(c.mirror('left', TUCK), **c.mirror('right', TUCK)), 3.0, hands=(0, 0))
    time.sleep(1.0)
    ob, path = c.observe(ctx, 'tuckback')
    e, tz = s.edge_of(path)
    print(json.dumps(dict(edge=round(e, 3), tz=tz, yaw=s.yaw_deg(ob), tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])))
