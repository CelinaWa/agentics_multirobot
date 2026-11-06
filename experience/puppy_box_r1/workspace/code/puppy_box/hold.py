"""Holding-phase steps for puppy_box_r1 (session 7c03aa). Each step is a subcommand; JSON log lines on stdout.

  python code/puppy_box/hold.py lift [yaw_deg]        servo-off arc lift from the current grasp (leg yaw in pelvis deg)
  python code/puppy_box/hold.py walk <fwd_m>          r1_walk while holding; width before/after
  python code/puppy_box/hold.py move x y z [az]       guarded servo-off hand move while holding (preview, jump guard)
  python code/puppy_box/hold.py release               open, hand up, out, HOME
Exit: 0 ok, 2 refused, 3 slipped (width < HELD_MIN).
"""
import sys, json, math, time
from pathlib import Path
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS))
from api.runtime import Context

ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
HOME = {'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0.0, 'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}
HELD_MIN = 0.02
STATE = WS / 'output' / 'hold_state.json'
# (along leg m, up m, approach z) per step: ~+1.5 cm along per 4 cm up first, then +1, tilting the approach up
LIFT = [(0.013, 0.035, 0.0), (0.013, 0.035, 0.05), (0.012, 0.035, 0.1), (0.01, 0.04, 0.2), (0.008, 0.04, 0.3),
        (0.005, 0.04, 0.35), (0.0, 0.04, 0.4), (0.0, 0.04, 0.45), (0.0, 0.04, 0.5)]
ctx = Context()


def log(**kw):
    print(json.dumps({'t': time.strftime('%H:%M:%S'), **kw}, default=str), flush=True)


def save(**kw):
    s = json.loads(STATE.read_text()) if STATE.exists() else {}
    s.update(kw)
    STATE.write_text(json.dumps(s, default=str))


def load():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def guarded(pos, appr, fing, w=0.0, secs=2.0, max_jump=0.4, servo=False, use_waist=False):
    q0 = ctx.value('r1_status')['command']['upper_joints']
    pv = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(appr), fingers=list(fing),
                   gripper_width_m=w, preview_only=True, use_waist=use_waist)['preview']
    jump = max(abs(pv['joints'][k] - q0[k]) for k in ARM)
    if pv['ik_error_m'] > 0.03 or jump > max_jump:
        log(step='REFUSED', goal=pos, ik=pv['ik_error_m'], jump=round(jump, 3))
        return None
    r = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(appr), fingers=list(fing),
                  gripper_width_m=w, secs=secs, servo=servo, use_waist=use_waist)
    log(step='moved', goal=[round(v, 4) for v in pos], hand=r['grasp_points_pelvis_m']['right'],
        width=r['gripper_width_m']['right'], tilt=r['tilt_deg'], ik=pv['ik_error_m'], jump=round(jump, 3))
    return r


def main():
    step = sys.argv[1]
    a = [float(v) for v in sys.argv[2:]]
    if step == 'lift':
        yaw = math.radians(a[0] if a else 12.0)
        dx, dy = math.cos(yaw), math.sin(yaw)
        fing = [-dy, dx, 0.0]
        st = ctx.value('r1_status')
        x, y, z = st['proprio']['grasp_points_pelvis_m']['right']
        w0 = st['proprio']['gripper_width_m']['right']
        log(step='lift0', hand=[x, y, z], width=w0)
        if w0 < HELD_MIN:
            log(step='NOT_HELD'); sys.exit(3)
        widths = []
        for i, (da, dz, az) in enumerate(LIFT):
            x, y, z = x + da * dx, y + da * dy, z + dz
            r = guarded((x, y, z), (dx, dy, az), fing, secs=1.5)
            if r is None:
                save(leg=[dx, dy], lift_widths=widths); sys.exit(2)
            widths.append(r['gripper_width_m']['right'])
            if widths[-1] < HELD_MIN:
                log(step='SLIP', at=i, widths=widths); sys.exit(3)
        save(leg=[dx, dy], lift_widths=widths, appr=[dx, dy, LIFT[-1][2]])
        log(step='lifted', widths=widths)
    elif step == 'walk':
        w0 = ctx.value('r1_status')['proprio']['gripper_width_m']['right']
        r = ctx.value('r1_walk', forward_m=a[0], turn_deg=(a[1] if len(a) > 1 else 0.0))
        w1 = ctx.value('r1_status')['proprio']['gripper_width_m']['right']
        log(step='walk', cmd=a, walked=r.get('walked_m'), turned=r.get('turned_deg'), pelvis=r.get('pelvis'),
            tilt=r.get('tilt_deg'), w_before=w0, w_after=w1)
        sys.exit(0 if w1 >= HELD_MIN else 3)
    elif step == 'move':
        s = load()
        dx, dy = s.get('leg', [1.0, 0.0])
        az = a[3] if len(a) > 3 else s.get('appr', [dx, dy, 0.4])[2]
        r = guarded(a[:3], (dx, dy, az), [-dy, dx, 0.0], secs=2.0)
        if r is None:
            sys.exit(2)
        sys.exit(0 if r['gripper_width_m']['right'] >= HELD_MIN else 3)
    elif step == 'release':
        g = ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        hp = g['grasp_points_pelvis_m']['right']
        log(step='opened', hand=hp)
        s = load()
        dx, dy = s.get('leg', [1.0, 0.0])
        up = [hp[0], hp[1], hp[2] + 0.06]
        guarded(up, (dx, dy, 0.4), [-dy, dx, 0.0], w=0.07, secs=1.5, max_jump=3.0, servo=True)
        guarded((0.25, -0.32, max(hp[2] + 0.06, 0.2)), (0.9, -0.3, 0.3), (0, 0, 1), w=0.07, secs=2.0, max_jump=3.0,
                servo=True)
        r = ctx.value('r1_move_joints', joints=HOME, secs=2.0)
        log(step='home', hand=r['grasp_points_pelvis_m']['right'], tilt=r['tilt_deg'])
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
