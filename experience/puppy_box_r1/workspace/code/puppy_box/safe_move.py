"""Previewed right-hand move with joint-jump guard. python safe_move.py x y z ax ay az fx fy fz width secs max_jump [go]
Prints the preview; moves only if 'go' is given and ik <= .03 and max arm-joint jump <= max_jump."""
import sys, json
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace'
sys.path.insert(0, WS)
from api.runtime import Context
ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
ctx = Context()
v = [float(s) for s in sys.argv[1:13]]
pos, appr, fing, w, secs, mj = v[0:3], v[3:6], v[6:9], v[9], v[10], v[11]
go = len(sys.argv) > 13 and sys.argv[13] == 'go'
q0 = ctx.value('r1_status')['command']['upper_joints']
r = ctx.value('r1_move_hand', arm='right', position=pos, approach=appr, fingers=fing, gripper_width_m=w, preview_only=True)
pv = r['preview']
jump = {n: round(pv['joints'][n] - q0[n], 3) for n in ARM}
mx = max(abs(x) for x in jump.values())
print(json.dumps({'ik': pv['ik_error_m'], 'appr_err': pv['approach_error_deg'], 'appr': pv['approach_achieved'],
                  'fing': pv['fingers_axis_achieved'], 'jump': jump, 'max_jump': mx}))
if go:
    if pv['ik_error_m'] > 0.03 or mx > mj:
        print('REFUSED'); sys.exit(2)
    m = ctx.value('r1_move_hand', arm='right', position=pos, approach=appr, fingers=fing, gripper_width_m=w, secs=secs)
    print(json.dumps({k: m.get(k) for k in ('grasp_points_pelvis_m', 'gripper_width_m', 'tilt_deg', 'steps',
                                             'servo_correction_m', 'grasp_error_m', 'servo_errors_m')}))
