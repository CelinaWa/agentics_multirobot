"""Guarded hand moves while holding (right arm, gripper kept at 0).
python lift.py "x,y,z[,ax,ay,az]" ["x,y,z,..."] ...   (pelvis frame; default approach (1,0,0), fingers (0,1,0))
or python lift.py rel dx,dy,dz [n]  -> n equal steps relative to the current grasp point.
Each step: preview; refuse if the fingers axis leaves horizontal (|fy|<0.85), a joint jumps > 0.35 rad or the wrist
roll > 0.4 rad (IK branch flip = drop); execute 2.5 s; stop when width < 0.012 or it fell > 0.008 below the start."""
import sys, json, time
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace')
from api.runtime import Context
ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
import os
WAIST = os.environ.get('USE_WAIST') == '1'
ctx = Context()


def log(**kw):
    print(json.dumps({'t': time.strftime('%H:%M:%S'), **kw}, default=str), flush=True)


def guarded(pos, approach=(1, 0, 0), fingers=(0, 1, 0), secs=2.5, width=0.0):
    st = ctx.value('r1_status')
    q0 = st['command']['upper_joints']
    rep = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(approach), fingers=list(fingers),
                    gripper_width_m=width, preview_only=True, use_waist=WAIST)
    pv = rep.get('preview', rep)
    reach = rep.get('reachable', pv.get('reachable', False))
    fy = abs(pv['fingers_axis_achieved'][1])
    jump = max(abs(pv['joints'][n] - q0[n]) for n in ARM + (['waist_roll_joint', 'waist_yaw_joint'] if WAIST else []))
    wj = abs(pv['joints']['right_wrist_roll_joint'] - q0['right_wrist_roll_joint'])
    if not reach or fy < 0.85 or jump > 0.35 or wj > 0.4:
        log(step='REFUSED', pos=pos, reach=reach, ik=pv.get('ik_error_m'), fy=round(fy, 3), jump=round(jump, 3), wj=round(wj, 3))
        return None
    r = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(approach), fingers=list(fingers),
                  gripper_width_m=width, secs=secs, use_waist=WAIST)
    log(step='moved', goal=[round(v, 3) for v in pos], hand=r['grasp_points_pelvis_m']['right'],
        width=r['gripper_width_m']['right'], tilt=r['tilt_deg'], ik=pv.get('ik_error_m'), steps=r.get('steps'))
    return r


def main():
    w0 = ctx.value('r1_status')['proprio']['gripper_width_m']['right']
    if sys.argv[1] == 'rel':
        d = [float(v) for v in sys.argv[2].split(',')]
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 1
        ap = [float(v) for v in sys.argv[4].split(',')] if len(sys.argv) > 4 else [1, 0, 0]
        hand = ctx.value('r1_status')['proprio']['grasp_points_pelvis_m']['right']
        wps = [[hand[k] + d[k] * (i + 1) / n for k in range(3)] + ap for i in range(n)]
    else:
        wps = [[float(v) for v in a.split(',')] for a in sys.argv[1:]]
    for wp in wps:
        ap = wp[3:6] if len(wp) >= 6 else [1, 0, 0]
        r = guarded(wp[:3], ap)
        if r is None:
            return 2
        w = r['gripper_width_m']['right']
        if w < 0.012 or w < w0 - 0.008:
            log(step='SLIP', width=w, w0=w0)
            return 3
    log(step='done', width=w, w0=w0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
