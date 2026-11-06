"""Pivot pinch of a plush paw ball with the right PiPER gripper, FINGERS HORIZONTAL (opening axis = pelvis y),
so the ball can rotate about the pinch axis while the puppy hangs (no lever torque on the pinch, only shear).
r1sonic sim, session 20261105T222431Z_puppy_box_r1_e0ac0c. Run ONE step at a time and look between steps.

  PYTHONPATH=/tmp/claude-1001/ws_c4f6c3/pylib python pivot_pinch.py <step> [args]
    look                         head 0.6, observe, print scene (edge, paw blobs, box) - pelvis frame
    seed                         wrist-roll seed so the IK picks the horizontal-finger branch (do it with the hand
                                 in front of the edge, e.g. after `pregrasp` failed or from home)
    pregrasp X Y Z [W]           safe move (preview, branch + jump check) to X,Y,Z, approach (1,0,0), fingers (0,1,0)
    grasp X Y Z                  safe move to X,Y,Z (ball centre, 0-0.5 cm short in x), then close, report width
    lift DZ DX [N]               N safe steps of (DX, 0, DZ)/N each, gripper kept at 0, width check each step
    release                      open, straight back 8 cm, HOME joints
Safety: every move is previewed first; a move is REFUSED when the preview's finger axis is not horizontal
(|fy| < 0.9), the wrist roll jumps > 0.5 rad, or any arm joint jumps > 0.8 rad (0.35 while lifting) from the
commanded joints (an IK branch flip while holding = drop).
"""
import json, sys, subprocess, time
from pathlib import Path
WSR = '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T232821Z_puppy_box_r1_c4f6c3/agent-workspace'
sys.path.insert(0, WSR)
from api.runtime import Context  # noqa: E402

HERE = Path(__file__).resolve().parent
ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
HOME = dict(zip(ARM, [0.35, -0.18, 0.0, 0.87, 0.0]))
SEED = dict(zip(ARM, [0.2615, -0.1115, 0.0409, -0.2435, 0.1061]))   # hand ~(0.25,-0.17,0.02), fingers horizontal
APPROACH, FINGERS = [1, 0, 0], [0, 1, 0]
HELD_MIN = 0.012
ctx = Context()


def log(**kw):
    print(json.dumps({'t': time.strftime('%H:%M:%S'), **kw}, default=str), flush=True)


def commanded():
    st = ctx.value('r1_status')
    return st['command']['upper_joints'], st


def safe_move(pos, width, secs=2.0, approach=APPROACH, max_jump=0.8):
    q0, _ = commanded()
    rep = ctx.value('r1_move_hand', arm='right', position=pos, approach=approach, fingers=FINGERS,
                    gripper_width_m=width, preview_only=True)
    pv = rep['preview']
    pv['reachable'] = rep.get('reachable', pv.get('reachable', False))
    fy = abs(pv['fingers_axis_achieved'][1])
    jump = max(abs(pv['joints'][n] - q0[n]) for n in ARM)
    wjump = abs(pv['joints']['right_wrist_roll_joint'] - q0['right_wrist_roll_joint'])
    if not pv['reachable'] or fy < 0.9 or jump > max_jump or wjump > 0.5:
        log(step='refused', pos=pos, reachable=pv['reachable'], ik_error=pv['ik_error_m'], finger_y=round(fy, 3),
            max_joint_jump=round(jump, 3), joints=pv['joints'])
        return None
    r = ctx.value('r1_move_hand', arm='right', position=pos, approach=approach, fingers=FINGERS,
                  gripper_width_m=width, secs=secs)
    log(step='moved', goal=pos, hand=r['grasp_points_pelvis_m']['right'], width=r['gripper_width_m']['right'],
        tilt=r['tilt_deg'], servo=r['servo_correction_m']['right'])
    return r


def main():
    step = sys.argv[1]
    a = [float(v) for v in sys.argv[2:]]
    if step == 'look':
        ctx.value('r1_head', pitch=0.6, secs=1.0)
        o = ctx.value('r1_observe')
        fid = o['camera']['frame_id']
        sc = subprocess.run([sys.executable, str(HERE / 'scene.py'), fid], capture_output=True, text=True).stdout
        log(step='look', frame=fid, waist_yaw=o['proprio']['joint_positions_rad'][5], scene=sc.strip())
    elif step == 'seed':
        r = ctx.value('r1_move_joints', joints=SEED, secs=2.5)
        log(step='seed', hand=r['grasp_points_pelvis_m']['right'])
    elif step == 'pregrasp':
        safe_move(a[:3], a[3] if len(a) > 3 else 0.07)
    elif step == 'grasp':
        if safe_move(a[:3], 0.07) is None:
            return
        g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.5)['gripper']
        log(step='grasp', width=g['measured_m'], held=g['measured_m'] > HELD_MIN)
    elif step == 'lift':
        dz, dx = a[0], a[1]
        n = int(a[2]) if len(a) > 2 else max(1, int(round(abs(dz) / 0.03)))
        for i in range(n):
            hand = ctx.value('r1_status')['proprio']['grasp_points_pelvis_m']['right']
            goal = [hand[0] + dx / n, hand[1], hand[2] + dz / n]
            r = safe_move(goal, 0.0, secs=2.0, max_jump=0.35)
            if r is None:
                return
            w = r['gripper_width_m']['right']
            if w < HELD_MIN:
                log(step='lift', result='SLIPPED', i=i, width=w)
                return
        log(step='lift', result='held', width=w)
    elif step == 'release':
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        hand = ctx.value('r1_status')['proprio']['grasp_points_pelvis_m']['right']
        safe_move([hand[0] - 0.08, hand[1], hand[2]], 0.07)
        ctx.value('r1_move_joints', joints=HOME, secs=2.5)
        log(step='release', done=True)


if __name__ == '__main__':
    main()
