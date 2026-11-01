"""Small g1_step helpers: cosine ramps from the last commanded row (or measured joints)."""
import os, sys, json, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
from api.runtime import Context
J = ["left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint", "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint", "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint", "left_knee_joint", "right_knee_joint", "left_shoulder_pitch_joint", "right_shoulder_pitch_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint", "left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_ankle_roll_joint", "right_ankle_roll_joint", "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint", "right_wrist_roll_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint"]
IDX = {n: i for i, n in enumerate(J)}

def last_row(ctx):
    s = ctx.value('g1_status')
    r = s.get('command', {}).get('last_row')
    meas = s['proprio']['joint_positions_rad'] if 'proprio' in s else None
    return (list(r) if r else None), meas, s

def ramp(ctx, targets, secs=2.0, hands=None):
    """targets: {joint_name: rad}; hands: (left, right) fractions or None to keep."""
    row, meas, s = last_row(ctx)
    if row is None:
        raise RuntimeError('no last row; call g1_hold first')
    start = list(row)
    end = list(row)
    for k, v in targets.items():
        end[9 + IDX[k]] = float(v)
    if hands is not None:
        end[38], end[39] = hands
    n = max(5, int(secs * 50))
    rows = []
    for i in range(1, n + 1):
        a = 0.5 - 0.5 * math.cos(math.pi * i / n)
        rows.append([s0 + a * (e - s0) for s0, e in zip(start, end)])
    out = None
    for k in range(0, len(rows), 95):
        chunk = rows[k:k + 95]
        out = ctx.value('g1_step', actions=chunk + [chunk[-1]] * 5)
    return out

def plan_exec(ctx, arm, pos, quat=None, frame='pelvis', ee='palm', execute=True, use_waist=False):
    """Plan with cuRobo (preview), then execute; returns a compact summary."""
    args = dict(arm=arm, position=list(pos), frame=frame, end_effector=ee, preview_only=True, use_waist=use_waist)
    if quat is not None:
        args['quaternion_wxyz'] = list(quat)
    p = ctx.value('g1_free_space_move', **args)
    if not p.get('planned'):
        return {'ok': False, 'why': p.get('plan', {}).get('status')}
    if not execute:
        return {'ok': True, 'plan': p['plan']['motion_plan_id']}
    r = ctx.value('g1_execute_motion_plan', motion_plan_id=p['plan']['motion_plan_id'])
    fb = r.get('transition', {}).get('feedback', {})
    return {'ok': True, 'frame': r.get('camera', {}).get('frame_id'), 'tilt': r.get('tilt_deg'),
            'fb': {k: fb.get(k) for k in ('tilt_deg', 'max_joint_tracking_error_rad', 'fallen')},
            'joints': r.get('proprio', {}).get('joint_positions_rad'),
            'hand': r.get('proprio', {}).get('hand_state', {}).get('positions_rad')}
