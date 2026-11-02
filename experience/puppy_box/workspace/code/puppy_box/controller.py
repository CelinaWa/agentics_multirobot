"""puppy_box controller: one-hand pinch of the plush puppy, lift, carry across with a waist turn, release over the
clear box, return to a safe standing pose. Steps run one by one so the frames can be looked at in between:

  python code/puppy_box/controller.py prep        # tuck the box-side arm, raise the puppy-side arm out of view
  python code/puppy_box/controller.py perceive    # puppy + box from a fresh frame -> output/pb_state.json + overlay
  python code/puppy_box/controller.py grasp       # pinch the puppy mid-body, finger-stall check
  python code/puppy_box/controller.py lift        # slow lift to palm z ~0.27 toward the body centre, grip check
  python code/puppy_box/controller.py carry       # waist yaw toward the box, lean 0.15, palm over the box, grip check
  python code/puppy_box/controller.py release     # open, raise the hand, unlean
  python code/puppy_box/controller.py home        # arms home via safe waypoints, waist 0
  python code/puppy_box/controller.py verify      # fresh frame: puppy blob vs box footprint, overlay

Every step prints a JSON report {ok, why, measured, frames} and stops (exit 1) when a check fails. State between
steps lives in output/pb_state.json. Pelvis frame, metres. The puppy side is the hand used (task rule); the box is on
the other side. Written from the 2026-11-02 by-hand success (memory/PROGRESS.md attempt 1)."""
import json, os, sys, time
import numpy as np

import importlib.util
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
_p = Path(HERE).resolve()
ROOT = str(next((q for q in _p.parents if (q / '.agents').is_dir() and (q / 'runtime').exists()), _p.parents[1]))
SKILLS = Path(ROOT) / 'skill-library' / 'skills'
sys.path.insert(0, HERE); sys.path.insert(0, ROOT)
sys.path.insert(0, str(SKILLS / 'plush-pinch-grasp' / 'code'))
_spec = importlib.util.spec_from_file_location('plush_pinch_grasp_run', SKILLS / 'plush-pinch-grasp' / 'code' / 'run.py')
ppg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ppg)            # plush-pinch-grasp; loads palm-pose-servo as ppg.pps
import percep as p  # noqa: E402
from boxdet import locate_box  # noqa: E402

pps = ppg.pps
J = pps.M.canonical29
STATE = os.path.join(ROOT, 'output', 'pb_state.json')
MIRROR = ('shoulder_roll', 'shoulder_yaw', 'wrist_roll', 'wrist_yaw', 'waist_yaw', 'waist_roll')

# parameters (right-hand values; the left hand mirrors them)
GRASP_BACK = 0.07        # palm x = puppy far edge - this
MID_FROM_INNER = 0.15    # mid-body y from the inner (box-side) end of the blob when the blob leaves the image
LIFT_Z = (0.08, 0.12, 0.16, 0.20)
LIFT_TO = ((0.28, -0.18, 0.24), (0.31, -0.12, 0.27))
WAIST_YAW = (0.5, 0.8)   # commanded toward the box; measured ~0.29 / 0.41-0.47
WAIST_PITCH = 0.15
RELEASE_DZ = 0.12        # palm z above the rim
RELEASE_YAW = 30.0
STALL = 1.3


def mirror(side, d):
    """Right-hand joint targets {short name: rad} -> canonical names for `side`."""
    out = {}
    for k, v in d.items():
        n = k if k.startswith('waist') else f'{side}_{k}'
        flip = side == 'left' and any(m in k for m in MIRROR)
        out[n + '_joint'] = -v if flip else v
    return out


def my(side, y):
    return y if side == 'right' else -y


def other(side):
    return 'left' if side == 'right' else 'right'


def load():
    return json.load(open(STATE)) if os.path.exists(STATE) else {}


def save(st):
    json.dump(st, open(STATE, 'w'), indent=1, default=float)


def observe(ctx, tag):
    ob = ctx.value('g1_observe')
    path = os.path.join(ROOT, 'output', f"pb_{tag}_{ob['step_id']}.json")
    json.dump(ob, open(path, 'w'))
    return ob, path


def fingers(ob, side):
    return ppg._fingers(ob['proprio']['hand_state']['positions_rad'], side)


class StepFailed(Exception):
    pass


def report(ok, why='', **measured):
    r = {'ok': ok, 'why': why, 'measured': measured}
    print(json.dumps(r, default=float))
    if not ok:
        raise StepFailed(r)
    return r


def step_prep(ctx, st):
    side = st.get('side', 'right')
    o = other(side)
    pps.ramp(ctx, mirror(o, {'shoulder_pitch': 0.45, 'elbow': -0.5}), 3.0, hands=(0.0, 0.0))
    pps.ramp(ctx, mirror(side, {'shoulder_pitch': 0.5}), 2.0)
    pps.ramp(ctx, mirror(side, {'shoulder_pitch': 0.0, 'elbow': -0.3}), 2.5)
    pps.ramp(ctx, mirror(side, {'shoulder_roll': -0.6}), 2.5)
    time.sleep(1.0)
    ob, path = observe(ctx, 'prep')
    report(ob['tilt_deg'] < 6, f"tilt {ob['tilt_deg']}", frame=ob['camera']['frame_id'], tilt=ob['tilt_deg'])


def step_perceive(ctx, st):
    side = st.get('side', 'right')
    box_side = other(side)
    ob, path = observe(ctx, 'perceive')
    P = p.load(path); tz = p.table_z(P)
    box = locate_box(P, box_side, tz)
    if box is None:
        report(False, 'no clear box found on the box side: look at the frame', frame=ob['camera']['frame_id'])
    # puppy: cream blob on the puppy side of the box's inner wall
    ex = (-1, 2, box['y_inner'] - 0.02, 2) if box_side == 'left' else (-1, 2, -2, box['y_inner'] + 0.02)
    loc = ppg.locate(path, exclude=ex)
    if not loc['ok']:
        report(False, 'no puppy blob: ' + loc['why'], frame=ob['camera']['frame_id'])
    m = loc['measured']
    ys = m['y_range']
    inner = max(ys) if side == 'right' else min(ys)          # the end toward the robot centre / the box
    span = ys[1] - ys[0]
    if loc['measured']['touches_image_edge'] and span < 0.30:
        mid_y = inner - my(side, MID_FROM_INNER)
    else:
        mid_y = (ys[0] + ys[1]) / 2
    gx = m['x_range'][1] - GRASP_BACK
    cx = box['x'][0] + 0.15                                   # near wall + half the 28-30 cm depth (far rim reads long)
    cy = box['centre'][1]
    rel = [cx, cy, box['rim_z'] + RELEASE_DZ]          # only z is used: carry searches x, y inside the box
    st.update(side=side, grasp_xy=[gx, mid_y], top=m['top_pelvis'], puppy=m, box=box, release=rel,
              edge_frame=ob['camera']['frame_id'])
    save(st)
    p.draw_box(P, box['x'][0], box['x'][1], box['y'][0], box['y'][1], box['rim_z'], tz,
               os.path.join(ROOT, 'output', f"pb_perceive_box_{ob['step_id']}.jpg"))
    ok = 0.15 < gx < 0.38 and abs(mid_y) < 0.40 and m['top_pelvis'] < 0.15
    report(ok, '' if ok else 'puppy out of the tested range', grasp_xy=[round(gx, 3), round(mid_y, 3)],
           top=round(m['top_pelvis'], 3), puppy_x=m['x_range'], puppy_y=ys, box_x=box['x'], box_y=box['y'],
           release=[round(v, 3) for v in rel], frame=ob['camera']['frame_id'],
           overlay=f"output/pb_perceive_box_{ob['step_id']}.jpg")


def step_grasp(ctx, st):
    side = st['side']
    ppg.settle_waist(ctx)
    ppg.set_pinch_shape(ctx, side)
    x, y = st['grasp_xy']
    press_z = st['top'] - 0.03
    for k in range(6):                     # run 2: palm x 0.327 only rested on the top (reach floor); 0.30 pressed
        if ppg.reach_check(ctx, side, (x, y, press_z), max_err=0.008)['ok']:
            break
        x -= 0.01
    st['grasp_xy'] = [x, y]; save(st)
    r = ppg.grasp(ctx, side, (x, y), descent=(0.12, 0.09, 0.06, press_z + 0.01, press_z - 0.01), log=lambda *a: None)
    if not r['ok'] and 'descent' in r.get('why', ''):
        # IK refused the deepest step: close where the palm is if it already rests on the plush
        pps.ramp(ctx, {}, ppg.CLOSE_SECS, hands=(1.0, 0.0) if side == 'left' else (0.0, 1.0))
        ob = ctx.value('g1_observe')
        ok = ppg.grip_ok(ob['proprio']['hand_state']['positions_rad'], side)
        r = {'ok': ok, 'why': '' if ok else 'fingers reached the fist after a reach-limited descent',
             'measured': {'fingers': [round(v, 2) for v in fingers(ob, side)], 'frame_id': ob['camera']['frame_id']}}
    st['bias'] = r['measured'].get('bias') or st.get('bias') or [0.0136, 0.012, -0.0096]; save(st)
    report(r['ok'], r['why'], grasp_xy=[round(x, 3), round(y, 3)], fingers=r['measured'].get('fingers'),
           frame=r['measured'].get('frame_id'), bias=st['bias'], stall=r['measured'].get('palm_stall_z'))


def step_lift(ctx, st):
    side = st['side']
    x, y = st['grasp_xy']
    pts = [(x, y, z) for z in LIFT_Z] + [(a, my(side, b), c) for a, b, c in LIFT_TO]
    r = ppg.lift(ctx, side, [q[:2] for q in pts], [q[2] for q in pts], st['bias'], log=lambda *a: None)
    h = r['measured']['history']
    report(r['ok'], r['why'], palm=h[-1]['palm'] if h else None, fingers=h[-1]['fingers'] if h else None,
           frame=h[-1]['frame_id'] if h else None)


def step_carry(ctx, st):
    side = st['side']
    o = other(side)
    s = -1 if side == 'right' else 1      # waist yaw toward the box (positive = left)
    pps.ramp(ctx, mirror(o, {'shoulder_pitch': 0.8, 'shoulder_roll': -0.2, 'elbow': -0.3}), 2.5)
    pps.ramp(ctx, {'waist_yaw_joint': -s * WAIST_YAW[0]}, 4.0)
    pps.ramp(ctx, {'waist_yaw_joint': -s * WAIST_YAW[1]}, 3.0)
    pps.ramp(ctx, {'waist_pitch_joint': WAIST_PITCH}, 3.0)
    time.sleep(3.0)                        # run 2: yaw crept 0.47 -> 0.605 after the IK was solved
    ob, _ = observe(ctx, 'carry_lean')
    if ob['tilt_deg'] > 8 or min(fingers(ob, side)[3], fingers(ob, side)[5]) > STALL:
        report(False, f"tilt {ob['tilt_deg']} or grip lost before the release move", frame=ob['camera']['frame_id'])
    # palm over the box: IK on the measured joints, back off toward the robot until reachable
    _, meas, _ = pps.last_row(ctx)
    box = st['box']
    z = st['release'][2]
    q = pps.palm_quat(RELEASE_YAW if side == 'right' else -RELEASE_YAW, side)
    s_in = 1 if box['y'][0] == box['y_inner'] else -1        # +1: box extends to +y from its inner wall
    best = None
    # the plush hangs toward the robot and the hand's side of the palm (2026-11-02 run 1: body ~5 cm behind the
    # palm): prefer the palm 10 cm inside the near wall, 19 cm inside the inner wall; accept 6 / 10 cm
    for dx in (0.10, 0.09, 0.08, 0.07, 0.06):
        for dy in (0.19, 0.17, 0.15, 0.13, 0.11, 0.10):
            cand = (box['x'][0] + dx, box['y_inner'] + s_in * dy, z)
            t, err, _ = pps.solve(list(meas), side, cand, q)
            if err < 0.015:
                best = (cand, t, err)
                break
        if best:
            break
    if best is None:
        report(False, 'no release point reachable inside the box from this stance and waist',
               frame=ob['camera']['frame_id'], waist=[meas[2], meas[8]])
    tgt, t, err = np.array(best[0]), best[1], best[2]
    pps.ramp(ctx, t, 4.0)
    _, meas, _ = pps.last_row(ctx)         # correct for the waist that kept moving during the ramp
    t2, err2, _ = pps.solve(list(meas), side, tuple(tgt), q)
    if err2 < 0.02:
        pps.ramp(ctx, t2, 3.0)
    ob, _ = observe(ctx, 'carry_over')
    f = fingers(ob, side)
    palm = pps.link_points(ob['proprio']['joint_positions_rad'], side)[f'{side}_palm']
    st['release_cmd'] = tgt.tolist(); save(st)
    report(min(f[3], f[5]) < STALL and ob['tilt_deg'] < 9, '' if min(f[3], f[5]) < STALL else 'grip lost on the carry',
           palm=[round(v, 3) for v in palm], target=tgt.round(3).tolist(), fingers=[round(v, 2) for v in f],
           tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])


def step_release(ctx, st):
    side = st['side']
    pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(1.0)
    ob1, _ = observe(ctx, 'released')
    pps.ramp(ctx, dict({'waist_pitch_joint': 0.0}, **mirror(side, {'shoulder_pitch': -1.7})), 3.0)
    ob, _ = observe(ctx, 'release_up')
    report(ob['tilt_deg'] < 8, '', frame_open=ob1['camera']['frame_id'], frame=ob['camera']['frame_id'], tilt=ob['tilt_deg'])


def step_home(ctx, st):
    side = st.get('side', 'right')
    o = other(side)
    ctx.value('g1_hand_shape', side=side)                    # default open/fist shapes back
    pps.ramp(ctx, mirror(side, {'elbow': 0.3, 'wrist_roll': 0, 'wrist_pitch': 0, 'wrist_yaw': 0, 'shoulder_yaw': 0}), 3.0, hands=(0.0, 0.0))
    pps.ramp(ctx, mirror(side, {'shoulder_pitch': -0.8, 'shoulder_roll': -0.25}), 3.0)
    pps.ramp(ctx, {'waist_yaw_joint': 0.0, 'waist_pitch_joint': 0.0, 'waist_roll_joint': 0.0}, 4.0)
    pps.ramp(ctx, mirror(side, {'shoulder_pitch': 0.7, 'elbow': -0.3, 'shoulder_roll': -0.2}), 4.0)
    pps.ramp(ctx, dict(mirror(side, {'shoulder_pitch': 0.5, 'elbow': 0.6}),
                       **mirror(o, {'shoulder_pitch': 0.5, 'elbow': 0.6, 'shoulder_roll': -0.2})), 3.0)
    time.sleep(2.0)
    ob, _ = observe(ctx, 'home')
    f = fingers(ob, side) + fingers(ob, o)
    if max(f) >= 0.3:                      # run 3: left index read 0.38 right after the ramp, 0 after a nudge
        pps.ramp(ctx, mirror(o, {'shoulder_roll': -0.35}), 2.0)
        time.sleep(3.0)                    # run 4: still 0.37 one second after the nudge, 0 two seconds later
        ob, _ = observe(ctx, 'home2')
        f = fingers(ob, side) + fingers(ob, o)
    report(ob['tilt_deg'] < 6 and max(f) < 0.3, '', tilt=ob['tilt_deg'], fingers_max=round(max(f), 2),
           frame=ob['camera']['frame_id'])


def step_verify(ctx, st):
    ob, path = observe(ctx, 'verify')
    P = p.load(path); tz = p.table_z(P)
    box = st['box']
    pel, lev = p.cloud(P)
    import cv2
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 40) & (hsv[..., 1] < 140) & (hsv[..., 2] > 120)
    m = cream & (lev[..., 2] > tz + 0.03) & (P['depth'] > 0)
    pts = lev[m]
    inside = ((pts[:, 0] > box['x'][0]) & (pts[:, 0] < box['x'][0] + 0.30) &
              (pts[:, 1] > box['y'][0]) & (pts[:, 1] < box['y'][1]))
    frac = float(inside.mean()) if len(pts) else 0.0
    p.draw_box(P, box['x'][0], box['x'][0] + 0.30, box['y'][0], box['y'][1], box['rim_z'], tz,
               os.path.join(ROOT, 'output', f"pb_verify_{ob['step_id']}.jpg"))
    report(frac > 0.6, 'most cream points are outside the box footprint: look at the frame' if frac <= 0.6 else '',
           cream_points=int(len(pts)), inside_frac=round(frac, 2), frame=ob['camera']['frame_id'],
           overlay=f"output/pb_verify_{ob['step_id']}.jpg")


STEPS = dict(prep=step_prep, perceive=step_perceive, grasp=step_grasp, lift=step_lift, carry=step_carry,
             release=step_release, home=step_home, verify=step_verify)

if __name__ == '__main__':
    st = load()
    if len(sys.argv) > 2:
        st['side'] = sys.argv[2]; save(st)
    from api.runtime import Context
    with Context() as ctx:
        try:
            STEPS[sys.argv[1]](ctx, st)
        except StepFailed:
            sys.exit(1)
