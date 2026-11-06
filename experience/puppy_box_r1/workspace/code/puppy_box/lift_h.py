"""Arc lift of a held leg (right hand, vertical fingers). Steps from the current measured hand position along the
leg direction (pelvis-frame yaw deg, argv[1]). Width checked every step; stops (exit 3) if width < 0.02.
python lift_c.py <leg_yaw_deg> [n_steps]"""
import sys, json, math, time
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T033337Z_puppy_box_r1_7c03aa/agent-workspace'
sys.path.insert(0, WS)
from api.runtime import Context
ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
ctx = Context()
yaw = math.radians(float(sys.argv[1]))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
d = (math.cos(yaw), math.sin(yaw))
FING = [-math.sin(yaw), math.cos(yaw), 0.0]
STEPS = [(0.012, 0.035, 0.0), (0.012, 0.035, 0.0), (0.010, 0.04, 0.1), (0.008, 0.04, 0.2), (0.005, 0.04, 0.3),
         (0.0, 0.04, 0.4), (0.0, 0.04, 0.5), (0.0, 0.04, 0.5)][:n]
st = ctx.value('r1_status')
x, y, z = st['proprio']['grasp_points_pelvis_m']['right']
w0 = st['proprio']['gripper_width_m']['right']
print(json.dumps({'start': [x, y, z], 'w0': w0}), flush=True)
if w0 < 0.02:
    print('NOT HELD'); sys.exit(3)
for i, (da, dz, az) in enumerate(STEPS):
    x, y, z = x + da * d[0], y + da * d[1], z + dz
    appr = [d[0], d[1], az]
    q0 = ctx.value('r1_status')['command']['upper_joints']
    pv = ctx.value('r1_move_hand', arm='right', position=[x, y, z], approach=appr, fingers=FING,
                   gripper_width_m=0.0, preview_only=True)['preview']
    jump = max(abs(pv['joints'][k] - q0[k]) for k in ARM)
    if pv['ik_error_m'] > 0.03 or jump > 0.4:
        print(json.dumps({'step': i, 'REFUSED': True, 'ik': pv['ik_error_m'], 'jump': round(jump, 3), 'goal': [x, y, z]}))
        sys.exit(2)
    r = ctx.value('r1_move_hand', arm='right', position=[x, y, z], approach=appr, fingers=FING,
                  gripper_width_m=0.0, secs=1.8)
    w = r['gripper_width_m']['right']
    print(json.dumps({'step': i, 'goal': [round(x, 3), round(y, 3), round(z, 3)], 'hand': r['grasp_points_pelvis_m']['right'],
                      'w': w, 'tilt': r['tilt_deg'], 'steps': r.get('steps'), 'ik': pv['ik_error_m']}), flush=True)
    if w < 0.02:
        print('SLIP'); sys.exit(3)
    if (r.get('tilt_deg') or 0) > 8:
        print('TILT'); sys.exit(2)
print('LIFTED')
