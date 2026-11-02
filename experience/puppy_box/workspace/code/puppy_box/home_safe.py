"""Safe end pose at a table (8cef10): open, hand high, waist yaw 0, then waist pitch/roll 0 + right arm to the tucked pose
(sp 0.45, el -0.5, wrists 0) like the left; never swings the forearms down in front (ce59d5 fell lowering them near the table)."""
import sys, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
from api.runtime import Context
c = pbb.c
R = lambda d: c.mirror('right', d)
with Context() as ctx:
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    r = pbb.waypoint(ctx, (0.22, -0.22, 0.36), -10, 0.0, 5.0, max_jump=1.6); r.pop('ob', None)
    print(json.dumps({'hand_high_waist0': r.get('ok'), 'why': r.get('why'), 'palm': r.get('palm')}), flush=True)
    if not r['ok']:
        c.pps.ramp(ctx, R({'shoulder_pitch': -0.8, 'elbow': 0.0}), 3.0)
        c.pps.ramp(ctx, {'waist_yaw_joint': 0.0}, 4.0)
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, dict(R({'shoulder_pitch': 0.1, 'shoulder_roll': -0.2, 'shoulder_yaw': 0, 'elbow': -0.9, 'wrist_roll': 0,
                            'wrist_pitch': 0, 'wrist_yaw': 0}), waist_pitch_joint=0.0, waist_roll_joint=0.0, waist_yaw_joint=0.0), 4.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.45, 'elbow': -0.5}), 3.0, hands=(0.0, 0.0))
    time.sleep(2.0)
    ob, path = c.observe(ctx, 'home_safe')
    f = c.fingers(ob, 'right') + c.fingers(ob, 'left')
    print(json.dumps({'tilt': ob['tilt_deg'], 'fingers_max': round(max(f), 2), 'frame': ob['camera']['frame_id'], 'json': path}))
