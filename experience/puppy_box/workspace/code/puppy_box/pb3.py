"""puppy_box v3 controller (right hand only, box on either side): one step per call so the frames can be looked at.

  python code/puppy_box/pb3.py prep                 # tuck left arm, raise the right hand behind the box wall
  python code/puppy_box/pb3.py perceive [head]      # puppy slices + box -> output/pb3_state.json + overlay; head=left|right
  python code/puppy_box/pb3.py hover                # pick a commanded waist reaching grasp AND release, hover over mid-body
  python code/puppy_box/pb3.py grasp                # press until the palm stalls, close, finger-stall check
  python code/puppy_box/pb3.py lift                 # 4 cm / 3 s steps to carry height, grip check each
  python code/puppy_box/pb3.py carry                # arm-only (waist unchanged) 5 s steps <= 8 cm to above the box centre
  python code/puppy_box/pb3.py release              # open, raise the hand
  python code/puppy_box/pb3.py home                 # waist 0 with the hand high, both arms home, hands open

Key fact (2026-11-02 session 810c60): SONIC tracks the whole-body pose, so when the waist under-tracks (cmd 0.45 ->
meas ~0.2) the arm compensates: FK of the COMMANDED row predicts the palm within ~1.5 cm. All IK here is solved on
the commanded row (mv.move / carry.waypoint). The grip survives slow vertical and arm-only moves but not a waist swing
while the plush hangs: the waist is set once before the grasp and kept until after the release.
Each step prints {ok, why, measured} JSON and exits 1 on a failed check. Pelvis frame, metres."""
import json, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controller as c          # noqa: E402  (pps, ppg, observe, fingers, mirror)
import percep as p              # noqa: E402
from boxdet import locate_box   # noqa: E402
from mv import move             # noqa: E402
from carry import waypoint      # noqa: E402
from reach2 import best         # noqa: E402

ROOT = c.ROOT
STATE = os.path.join(ROOT, 'output', 'pb3_state.json')
STALL = 1.3
WAISTS = (0.0, 0.2, 0.3, 0.45, 0.55, 0.7)  # commanded waist yaw candidates (measured ~35-45 % of these); 0.7 needed for a low table (152e7f)
HEAD_SHIFT = 0.03       # body centre = blob y-centre shifted this far away from the head (0.06 grasped the rear twice in 152e7f: the press/close pushes the plush toward its head)
GRASP_BACK = 0.07       # palm x = body far edge - this
PRESS = (0.04, 0.0, -0.03, -0.06)   # descent below the top (palm z cmd = top + d); stops when the palm stalls
LIFT_Z = 0.28
CARRY_Z = 0.30
GRASP_YAW, RELEASE_YAW = 15.0, -25.0


def load():
    return json.load(open(STATE)) if os.path.exists(STATE) else {}


def save(st):
    json.dump(st, open(STATE, 'w'), indent=1, default=float)


class Fail(Exception):
    pass


def report(ok, why='', **m):
    print(json.dumps({'ok': ok, 'why': why, 'measured': m}, default=float))
    if not ok:
        raise Fail(why)


def grip(f):
    return min(f[3], f[5]) < STALL


def cmd_row(ctx):
    row, _, _ = c.pps.last_row(ctx)
    return list(row[9:38])


def step_prep(ctx, st):
    R = lambda d: c.mirror('right', d)
    c.pps.ramp(ctx, c.mirror('left', {'shoulder_pitch': 0.45, 'elbow': -0.5}), 3.0, hands=(0.0, 0.0))
    # shoulder back first so the fingers stay at x < 0.22 (behind a box near wall) while the elbow lifts the hand
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.9}), 2.5)
    c.pps.ramp(ctx, R({'elbow': -0.9}), 3.0)
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.1}), 3.0)
    c.pps.ramp(ctx, R({'wrist_roll': -1.3}), 2.5)        # palm down while high: later IK moves stay < 1 rad
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'pb3_prep')
    report(ob['tilt_deg'] < 6, f"tilt {ob['tilt_deg']}", tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])


def step_lean(ctx, st, pitch=0.1):
    """Waist pitch in its own slow ramp with the hand high, then two observations 3 s apart (tilt, pelvis settle).
    Only for a stance at the far end of 0.15-0.20 m from the edge (152e7f run 4: far edge 0.40, hover found no reach).
    Never above 0.15 here (fall at 0.30, 2026-11-01). Re-run perceive afterwards: the pelvis may slide back."""
    pitch = min(float(pitch), 0.15)
    c.pps.ramp(ctx, {'waist_pitch_joint': pitch}, 4.0)
    tilts = []
    for k in range(2):
        time.sleep(1.5)
        ob, _ = c.observe(ctx, f'pb3_lean{k}')
        tilts.append(ob['tilt_deg'])
    j = ob['proprio']['joint_positions_rad']
    report(max(tilts) < 7, f'tilt {tilts}', tilts=tilts, waist_pitch=round(j[8], 3), frame=ob['camera']['frame_id'])


def step_perceive(ctx, st, head=None, move_hand=True, paths=None):
    """View A (hand raised on the right, after prep): the puppy. View B (hand moved up-left): the box."""
    if move_hand:      # waist turned left: the tucked left hand leaves the view of the puppy's head
        # look toward the puppy's side with the hand moved to the other side (head right: puppy on the right)
        va = ((0.24, 0.10, 0.32), 20, -0.35) if head == 'right' else ((0.30, 0.0, 0.29), 0, 0.45)
        r = waypoint(ctx, va[0], va[1], va[2], 5.0, max_jump=1.6)   # free space, hand high
        if not r['ok']:
            report(False, 'puppy-view move refused: ' + r['why'])
        time.sleep(1.0)
    obA, pathA = c.observe(ctx, 'pb3_puppy') if paths is None else (json.load(open(paths[0])), paths[0])
    if move_hand:
        r = waypoint(ctx, (0.24, 0.08, 0.32), 20, 0.0, 5.0, max_jump=1.6)
        if not r['ok']:
            report(False, 'hand-left view move refused: ' + r['why'])
    ob, path = c.observe(ctx, 'pb3_perceive') if paths is None else (json.load(open(paths[1])), paths[1])
    P = p.load(path); tz = p.table_z(P)
    box = None
    for side in ('right', 'left'):
        b = locate_box(P, side, tz)
        if b and (box is None or (b['x'][1] - b['x'][0]) > (box['x'][1] - box['x'][0])):
            box, bside = b, side
    if box is None:
        report(False, 'no clear box found: look at the frame', frame=ob['camera']['frame_id'])
    ex = (-1, 2, -2, box['y_inner'] + 0.03) if bside == 'right' else (-1, 2, box['y_inner'] - 0.03, 2)
    PA = p.load(pathA); tzA = p.table_z(PA)
    pp = p.puppy(PA, tzA, exclude=ex)
    if pp is None:
        report(False, 'no puppy blob', frame=obA['camera']['frame_id'])
    pel, _ = p.cloud(PA)
    pts = pel[pp['mask']]
    ppB = p.puppy(P, tz, exclude=ex)                  # view B sees the end the raised arm hides in view A
    if ppB is not None:
        pts = np.vstack([pts, p.cloud(P)[0][ppB['mask']]])
    pts = pts[(pts[:, 2] < tzA + 0.15) & (pts[:, 2] > tzA + 0.02)]   # drop the hand
    y0, y1 = np.percentile(pts[:, 1], 3), np.percentile(pts[:, 1], 97)
    yc = (y0 + y1) / 2
    if head in ('left', 'right'):
        yc += -HEAD_SHIFT if head == 'left' else HEAD_SHIFT
    mid = pts[np.abs(pts[:, 1] - yc) < 0.05]
    far = float(np.percentile(mid[:, 0], 97)); top = float(np.percentile(mid[:, 2], 97))
    gx = far - GRASP_BACK
    cx = box['x'][0] + 0.145; cy = box['centre'][1]
    st.update(box=box, box_side=bside, grasp=[gx, float(yc)], top=top, table_z=tz, release=[cx, cy, CARRY_Z - 0.01],
              puppy_y=[float(y0), float(y1)], far=far, head=head)
    save(st)
    over = os.path.join(ROOT, 'output', f"pb3_perceive_{ob['step_id']}.jpg")
    p.draw_box(P, box['x'][0], box['x'][1], box['y'][0], box['y'][1], box['rim_z'], tz, over)
    img = cv2_mark(P, [(gx, yc, top)], over)
    ok = 0.18 < gx < 0.36 and abs(yc) < 0.35 and top < 0.15
    report(ok, '' if ok else 'puppy out of the tested range: ask the operator', grasp=[round(gx, 3), round(yc, 3)],
           top=round(top, 3), puppy_y=[round(y0, 3), round(y1, 3)], far=round(far, 3), box_side=bside,
           box_x=[round(v, 3) for v in box['x']], box_y=[round(v, 3) for v in box['y']],
           release=[round(cx, 3), round(cy, 3)], overlay=over, frame_puppy=obA['camera']['frame_id'], frame=ob['camera']['frame_id'])


def cv2_mark(P, pts, path):
    import cv2
    img = cv2.imread(path)
    for q in pts:
        uv = p.project(P, np.array(q))[0].astype(int)
        cv2.circle(img, tuple(int(v) for v in uv), 7, (0, 255, 0), -1)
    cv2.imwrite(path, img)


def step_hover(ctx, st):
    """Pick the smallest commanded waist yaw reaching the release and a pressed grasp pose; when the deepest press is
    out of reach (table low relative to the pelvis), back the palm off toward the robot (<= far - 0.10) and press
    less deep (>= top - 0.03) before giving up. 2026-11-02 152e7f: table at pelvis z -0.05, far-0.07/top-0.06 out."""
    gx, gy = st['grasp']; top = st['top']
    rel = st['release']
    cmd = cmd_row(ctx)
    pick = None
    for depth in (PRESS[-1], -0.045, -0.03):
        for back in (0.0, 0.01, 0.02, 0.03):
            for w in WAISTS:
                q = list(cmd); q[2] = w
                e1, _ = best(q, (gx - back, gy, top + depth), GRASP_YAW)
                if e1 >= 0.01:
                    continue
                for rx in (rel[0], max(st['box']['x'][0] + 0.12, rel[0] - 0.03)):   # body hangs ~5 cm behind the palm
                    e2, _ = best(q, (rx, rel[1], rel[2]), RELEASE_YAW)
                    if e2 < 0.01:
                        pick = w; rel[0] = rx
                        break
                if pick is not None:
                    break
            if pick is not None:
                break
        if pick is not None:
            break
    if pick is not None:
        gx -= back
        st['grasp'] = [gx, gy]; st['press_depth'] = depth; st['release'] = rel
    if pick is None:
        report(False, 'no commanded waist reaches both the grasp and the release point: ask the operator to place the robot closer to the table (no lean)', grasp=st['grasp'], release=rel)
    st['waist_cmd'] = pick; save(st)
    c.ppg.set_pinch_shape(ctx, 'right')       # pre-curled pinch; home restores the default flat shape (run 4: fist)
    r = waypoint(ctx, (gx, gy, top + 0.13), GRASP_YAW, pick, 5.0, max_jump=1.6)   # free space above the puppy
    if r['ok']:
        r2 = move(ctx, (gx, gy, top + 0.06), GRASP_YAW, 3.0); r2.pop('ob', None); r = r2
    report(r['ok'], r.get('why', ''), waist_cmd=pick, palm=r.get('palm'), waist=r.get('waist'), frame=r.get('frame'))


def step_grasp(ctx, st):
    gx, gy = st['grasp']; top = st['top']
    stall = None
    for d in [v for v in PRESS if v >= st.get('press_depth', PRESS[-1]) - 1e-6]:
        z = top + d
        r = move(ctx, (gx, gy, z), GRASP_YAW, 2.5); r.pop('ob', None)
        if not r['ok'] or r['tilt'] > 7:
            report(False, r.get('why', f"tilt {r.get('tilt')}"))
        if r['palm'][2] > z + 0.04:          # palm held up by the plush: pressed in
            stall = r['palm'][2]
            break
    c.pps.ramp(ctx, {}, 3.0, hands=(0.0, 1.0))
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'pb3_grasp')
    f = c.fingers(ob, 'right')
    st['grasp_palm'] = r['palm']; st['press_z'] = z; save(st)
    report(grip(f), '' if grip(f) else 'fingers reached the fist: nothing in the hand (open, raise, re-perceive)',
           stall_z=stall, palm=r['palm'], fingers=[round(v, 2) for v in f], frame=ob['camera']['frame_id'])


def step_lift(ctx, st):
    gx, gy = st['grasp']
    x = gx + 0.015
    z = st.get('press_z', st['top'] - 0.06)   # start from the press COMMAND: a 10 cm first step lost the grip (run 6)
    hist = []
    while z < LIFT_Z - 1e-3:
        z = min(LIFT_Z, z + 0.04)
        r = move(ctx, (x, gy, z), GRASP_YAW, 3.0); r.pop('ob', None)
        hist.append([round(r.get('palm', [0, 0, 0])[2], 3), r.get('fingers')])
        if not r['ok'] or not grip(r['fingers']):
            report(False, r.get('why') or f'grip lost at palm z {r["palm"][2]:.3f}', history=hist, frame=r.get('frame'))
    report(True, palm=r['palm'], fingers=r['fingers'], frame=r['frame'], history=hist)


def step_carry(ctx, st):
    row, meas, _ = c.pps.last_row(ctx)
    start = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])   # commanded palm (what move() targets)
    end = np.array(st['release'], float)
    start[2] = max(start[2], CARRY_Z)
    n = max(2, int(np.ceil(np.linalg.norm((end - start)[:2]) / 0.08)))
    hist = []
    for k in range(1, n + 1):
        a = k / n
        pos = start + a * (end - start)
        pos[2] = CARRY_Z if k < n else end[2]
        yaw = GRASP_YAW + a * (RELEASE_YAW - GRASP_YAW)
        r = move(ctx, tuple(pos), yaw, 5.0); ob = r.pop('ob', None)
        if not r['ok']:
            # the far end may be out of reach: stop one step short and report (release decision is the agent's)
            report(False, r['why'], history=hist, at=pos.round(3).tolist())
        hist.append({'cmd': pos.round(3).tolist(), 'palm': r['palm'], 'fingers': r['fingers'][3:6:2], 'tilt': r['tilt']})
        if not grip(r['fingers']) or r['tilt'] > 8:
            report(False, 'grip lost or tilt on the carry', history=hist, frame=r['frame'])
    path = os.path.join(ROOT, 'output', f"pb3_over_{ob['step_id']}.json")
    json.dump(ob, open(path, 'w'))
    P = p.load(path); b = st['box']
    over = os.path.join(ROOT, 'output', f"pb3_over_{ob['step_id']}.jpg")
    p.draw_box(P, b['x'][0], b['x'][1], b['y'][0], b['y'][1], b['rim_z'], st['table_z'], over)
    pa = r['palm']
    inside = b['x'][0] + 0.08 < pa[0] < b['x'][1] - 0.06 and b['y'][0] + 0.08 < pa[1] < b['y'][1] - 0.08
    report(inside, '' if inside else 'palm not well inside the box footprint: look before releasing', palm=pa,
           box_x=b['x'], box_y=b['y'], history=hist, overlay=over, frame=r['frame'])


def step_release(ctx, st):
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(1.5)
    ob1, _ = c.observe(ctx, 'pb3_released')
    row, _, _ = c.pps.last_row(ctx)
    pa = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])
    r = move(ctx, (pa[0] - 0.05, pa[1] + 0.04, pa[2] + 0.08), RELEASE_YAW, 3.0, max_jump=1.6); r.pop('ob', None)  # high above the box: 152e7f run 5 refused at jump 1.05
    report(r['ok'], r.get('why', ''), frame_open=ob1['camera']['frame_id'], frame=r.get('frame'), palm=r.get('palm'))


def step_home(ctx, st):
    R = lambda d: c.mirror('right', d); L = lambda d: c.mirror('left', d)
    r = waypoint(ctx, (0.22, -0.22, 0.38), -10, 0.0, 4.0)        # waist back to 0 with the hand high
    if not r['ok']:
        c.pps.ramp(ctx, R({'shoulder_pitch': -0.8, 'elbow': 0.0}), 3.0)
        c.pps.ramp(ctx, {'waist_yaw_joint': 0.0}, 4.0)
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, dict(R({'shoulder_pitch': 0.1, 'shoulder_roll': -0.2, 'shoulder_yaw': 0, 'elbow': -0.9, 'wrist_roll': 0,
                            'wrist_pitch': 0, 'wrist_yaw': 0}), waist_pitch_joint=0.0, waist_roll_joint=0.0), 4.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.9}), 3.0)
    c.pps.ramp(ctx, dict(R({'elbow': 0.6}), **L({'shoulder_pitch': 0.9})), 3.0)
    c.pps.ramp(ctx, L({'elbow': 0.6, 'shoulder_roll': 0.2, 'shoulder_yaw': 0, 'wrist_roll': 0, 'wrist_pitch': 0, 'wrist_yaw': 0}), 3.0)
    c.pps.ramp(ctx, dict(R({'shoulder_pitch': 0.5}), **L({'shoulder_pitch': 0.5})), 3.0)
    time.sleep(2.0)
    ob, _ = c.observe(ctx, 'pb3_home')
    f = c.fingers(ob, 'right') + c.fingers(ob, 'left')
    report(ob['tilt_deg'] < 6 and max(f) < 0.5, '', tilt=ob['tilt_deg'], fingers_max=round(max(f), 2),
           frame=ob['camera']['frame_id'])


STEPS = dict(prep=step_prep, lean=step_lean, perceive=step_perceive, hover=step_hover, grasp=step_grasp, lift=step_lift,
             carry=step_carry, release=step_release, home=step_home)

if __name__ == '__main__':
    st = load()
    from api.runtime import Context
    with Context() as ctx:
        try:
            if sys.argv[1] == 'lean':
                step_lean(ctx, st, *sys.argv[2:3])
            elif sys.argv[1] == 'perceive':
                st = {}
                step_perceive(ctx, st, sys.argv[2] if len(sys.argv) > 2 else None)
            else:
                STEPS[sys.argv[1]](ctx, st)
        except Fail:
            sys.exit(1)
