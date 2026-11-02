"""End pose for 0878bd: check the joint path from the current command to the high prep pose (waist 0) by FK (lowest right-arm
link must stay > 0.17 m pelvis z, i.e. above the box rim), ramp, then tuck the right arm like the left (sp 0.45, el -0.5)."""
import sys, os, json, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import cr
from api.runtime import Context
c = cr.pb3.c
R = lambda d: c.mirror('right', d)
PREP = dict(R({'shoulder_pitch': 0.1, 'elbow': -0.9, 'wrist_roll': -1.3, 'wrist_pitch': 0.0, 'wrist_yaw': 0.0,
               'shoulder_roll': -0.2, 'shoulder_yaw': 0.0}), waist_yaw_joint=0.0, waist_roll_joint=0.0, waist_pitch_joint=0.0)
with Context() as ctx:
    c.pps.ramp(ctx, {}, 1.0, hands=(0.0, 0.0))
    row, _, _ = c.pps.last_row(ctx); q0 = np.array(row[9:38]); q1 = q0.copy()
    for k, v in PREP.items():
        q1[c.J.index(k)] = v
    lo = cr.lr.path_min_z(q0, q1)
    print(json.dumps({'path_low': lo}), flush=True)
    if lo[0] < 0.17 and not (lo[2][0] < 0.15):
        print(json.dumps({'ok': False, 'why': 'path too low', 'low': lo})); sys.exit(1)
    c.pps.ramp(ctx, PREP, 5.0, hands=(0.0, 0.0))
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.45, 'elbow': -0.5, 'wrist_roll': 0.0}), 3.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, c.mirror('left', {'shoulder_pitch': 0.45, 'elbow': -0.5}), 2.0, hands=(0.0, 0.0))
    time.sleep(2.0)
    ob, path = c.observe(ctx, 'home_cr')
    f = c.fingers(ob, 'right') + c.fingers(ob, 'left')
    print(json.dumps({'ok': ob['tilt_deg'] < 6, 'tilt': ob['tilt_deg'], 'fingers_max': round(max(f), 2),
                      'frame': ob['camera']['frame_id'], 'json': path}))
