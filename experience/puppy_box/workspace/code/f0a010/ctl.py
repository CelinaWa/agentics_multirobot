"""puppy_box, box directly BEHIND the puppy with a small gap (~3.5 cm), right hand only, waist 0, no lean (session f0a010, sim).
Run from the workspace root, one step at a time, and LOOK at the frame each step prints:
  PYTHONPATH=.:code/puppy_box python code/f0a010/ctl.py STEP     (state: output/f0_state.json)
Steps:
  perceive  no motion: puppy depth profile per 3 cm y slice (far side, top), grasp y = middle of the high slices + Y_SHIFT
  hover     palm-down, fingers forward (yaw 0); every arm move is FK-path-checked (no hand link below top + 0.04 over the
            puppy, no finger link past the wall - WALL_MARGIN); high side point -> over the grasp at top + .22 -> top + .13; LOOK
  hover_cu  alternative to hover: g1_free_space_move (cuRobo) to (target, top + .15), then the same press command (run 3)
  grasp     press levels with the forward palm drift compensated on the MEASURED palm (target far - BACK), each level gated
            on finger-link FK x <= wall - WALL_MARGIN; close 4 s; closed raise 3 cm; held = a base joint < GRIP_LOST
  lift      3 cm / 4 s steps up to LIFT_Z; where straight up is out of reach (palm close to the body) step 3 cm forward too,
            never past wall - RAISE_X_MAX; grip check each step. LOOK: the hanging paws must be above the rim
  carry     forward 4 cm / 5 s steps at LIFT_Z to x = wall + REL_PAST, grip check
  release   open flat, wait 2 s, up 5 cm if reachable
  home      home_safe.py (both arms tucked, waist 0, hands open)
The wall x comes from the box rim (perceive arg or state 'wall'); the clear box is invisible in depth."""
import sys, os, json, time
import numpy as np
W = os.getcwd()
for d in ('skill-library/skills/plush-pick-place-box-same-side/code', 'skill-library/skills/plush-pick-place-box-behind/code',
          'skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import mv                                  # noqa: E402  same-side skill: commanded-row IK move
from common import M                       # noqa: E402
from api.params import param               # noqa: E402

c = mv.c
ST = 'output/f0_state.json'
BACK = param('grasp.palm_back_from_far_m', 0.105)        # measured palm short of the puppy's far side (fingers forward)
WALL_MARGIN = param('grasp.finger_wall_margin_m', 0.007)  # finger links (index_1/middle_1) stay this far short of the wall
GRIP_LOST = param('grasp.grip_lost_rad', 1.45)          # base joint (index_0/middle_0) at or above this = empty
LIFT_Z = param('lift.palm_z_pelvis_m', 0.33)            # palm z for the carry (rim at table + 0.14)
REL_PAST = param('release.past_near_wall_m', 0.13)      # release palm this far past the near wall
Y_SHIFT = 0.0
PRESS_BACK = param('grasp.press_cmd_back_m', 0.01)       # press command this far behind the hover-corrected target
RAISE_X_MAX = 0.07                                       # diagonal raise keeps the palm this far short of the wall
FINGERS = ('right_hand_index_1_link', 'right_hand_middle_1_link', 'right_hand_index_0_link', 'right_hand_middle_0_link',
           'right_hand_thumb_2_link', 'right_palm', 'right_wrist_yaw_link')


def load():
    return json.load(open(ST)) if os.path.exists(ST) else {}


def save(st):
    json.dump(st, open(ST, 'w'), indent=1, default=float)


def hand_links(j29, hand7=None):
    h = dict(zip(M.hand_joints['right'], hand7)) if hand7 is not None else None
    P = M.fk(list(j29), hand=h) if h else M.fk(list(j29))
    return {n: np.array(P.frame_pose(n)[1], float) for n in FINGERS}


def path_ok(cmd29, tgt, st, n=12):
    """FK along the joint ramp: hand links stay above the puppy (top + 0.04 inside its footprint) and short of the wall."""
    a = np.array(cmd29, float); b = a.copy()
    for k, v in tgt.items():
        b[c.J.index(k)] = v
    x0, x1, y0, y1 = st['foot']
    hand = st.get('hand7', [0, 0.4, 0, 0.5, 0, 0.5, 0])                    # pinch-open shape (fingers pre-curled)
    L0 = hand_links(a, hand)
    for s in np.linspace(0, 1, n)[1:]:
        L = hand_links(a + (b - a) * (0.5 - 0.5 * np.cos(np.pi * s)), hand)
        for name, p in L.items():
            closer = p[0] > L0[name][0] + 0.005 or p[2] < L0[name][2] - 0.005      # the start pose may already be near
            if x0 - 0.03 < p[0] < x1 + 0.03 and y0 - 0.03 < p[1] < y1 + 0.03 and p[2] < st['top'] + 0.04 and not st.get('contact_ok') and closer:
                return False, f'{name} at {np.round(p, 3).tolist()} passes through the puppy (s={s:.2f})'
            if 'palm' not in name and p[0] > st['wall'] - WALL_MARGIN and p[2] < st['rim'] + 0.01 and closer:
                return False, f'{name} at {np.round(p, 3).tolist()} reaches the box wall {st["wall"]:.3f} (s={s:.2f})'
    return True, ''


def move(ctx, st, pos, secs, hands=None, check=True):
    row, meas, _ = c.pps.last_row(ctx)
    cmd = list(row[9:38])
    t, err, _ = c.pps.solve(cmd, 'right', pos, c.pps.palm_quat(0.0, 'right'))
    if err > 0.01:
        err, t = mv.best(cmd, pos, 0.0, 'right')
    jump = max(abs(t[k] - cmd[c.J.index(k)]) for k in t)
    if err > 0.01 or jump > 1.6:
        return dict(ok=False, why=f'refused: IK err {err:.3f} jump {jump:.2f}')
    if check:
        ok, why = path_ok(cmd, t, st)
        if not ok:
            return dict(ok=False, why='path check: ' + why)
    c.pps.ramp(ctx, t, secs, hands=hands)
    time.sleep(0.5)
    return measure(ctx, dict(cmd=[round(v, 3) for v in pos]))


def measure(ctx, extra=None):
    ob = ctx.value('g1_observe')
    j = ob['proprio']['joint_positions_rad']; h = ob['proprio']['hand_state']['positions_rad']
    L = hand_links(j, h[7:])
    path = f"output/f0_{ob['step_id']}.json"; json.dump(ob, open(path, 'w'))
    return dict(extra or {}, ok=True, palm=np.round(L['right_palm'], 3).tolist(),
                tips_x=round(float(max(L['right_hand_index_1_link'][0], L['right_hand_middle_1_link'][0])), 3),
                fingers=[round(abs(v), 2) for v in h[7:]], left_palm=c.pps.link_points(j, 'left')['left_palm'],
                tilt=ob['tilt_deg'], frame=f"runtime/frames/{ob['camera']['frame_id']}/cam_front.jpg", json=path)


def held(r):
    f = r.get('fingers') or [9] * 7
    return r.get('ok') and min(f[3], f[5]) < GRIP_LOST and r.get('tilt', 99) < 7


def perceive(ctx, st, wall=None, gy_override=None):
    import percep as p
    st['contact_ok'] = False
    r = measure(ctx)
    P = p.load(r['json']); tz = p.table_z(P)
    wall = float(wall) if wall is not None else st.get('wall', 0.288)
    pp = p.puppy(P, tz, exclude=(wall - 0.005, 2, -2, 2)); pel, _ = p.cloud(P)
    pts = pel[pp['mask']]; pts = pts[(pts[:, 2] > tz + 0.02) & (pts[:, 2] < tz + 0.14)]
    rows = []
    for y0 in np.arange(-0.40, 0.10, 0.03):
        s = pts[(pts[:, 1] >= y0) & (pts[:, 1] < y0 + 0.03)]
        if len(s) > 150:
            rows.append((y0 + 0.015, float(np.percentile(s[:, 0], 98)), float(np.percentile(s[:, 0], 2)), float(np.percentile(s[:, 2], 97))))
    if len(rows) < 4:
        return dict(ok=False, why='puppy not found in depth', rows=rows, frame=r['frame'])
    tops = np.array([q[3] for q in rows]); hi = [q for q in rows if q[3] > tops.max() - 0.012]
    gy = float(np.mean([q[0] for q in hi])) + Y_SHIFT
    if gy_override is not None:                                            # e.g. head hidden behind the tucked hand
        gy = float(gy_override)
    far = float(np.median([q[1] for q in rows if abs(q[0] - gy) < 0.05]))
    top = float(tops.max())
    st.update(far=far, gy=gy, top=top, tz=tz, wall=wall, rim=tz + 0.14,
              foot=[min(q[2] for q in rows), max(q[1] for q in rows), min(q[0] for q in rows) - 0.015, max(q[0] for q in rows) + 0.015])
    gap = wall - far
    ok = gap > 0.025 and far - BACK > 0.13
    return dict(ok=ok, why='' if ok else f'gap {gap:.3f} or grasp x {far - BACK:.3f} outside the envelope', far=round(far, 3),
                gy=round(gy, 3), top=round(top, 3), wall=wall, foot=np.round(st['foot'], 3).tolist(),
                rows=[[round(v, 3) for v in q] for q in rows], frame=r['frame'], left_palm=r['left_palm'])


def hover(ctx, st):
    ctx.value('g1_hand_shape', side='right', open=[0, 0.4, 0, 0.5, 0, 0.5, 0], closed=[0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74])
    st['contact_ok'] = False
    tx, ty = st['far'] - BACK, st['gy']
    for pos in [(0.10, min(ty - 0.17, -0.32), st['top'] + 0.25), (0.13, ty - 0.07, st['top'] + 0.24), (tx, ty, st['top'] + 0.22), (tx, ty, st['top'] + 0.13)]:
        r = move(ctx, st, pos, 4.0)
        if not r['ok']:
            return r
    # one correction on the measured palm, then 1 cm back: the press pushes the palm forward (run 2: links stayed <= wall - .014)
    cx, cy = tx + (tx - r['palm'][0]) - PRESS_BACK, ty + (ty - r['palm'][1])
    st['cmd'] = [cx, cy]
    return dict(r, target=[round(tx, 3), round(ty, 3)])


def hover_cu(ctx, st):
    """cuRobo approach (self-collision-free) to (target, top + 0.15), then the press command = hover-corrected target - PRESS_BACK.
    Use when the hand starts tucked low in front of the puppy (run 3: the IK-waypoint hover left the arm untrackable, 0.6-0.9 rad
    off its command): sync the arm to the measured joints first, then this."""
    ctx.value('g1_hand_shape', side='right', open=[0, 0.4, 0, 0.5, 0, 0.5, 0], closed=[0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74])
    st['contact_ok'] = False
    tx, ty = st['far'] - BACK, st['gy']
    q = [float(v) for v in c.pps.palm_quat(0.0, 'right')]
    pl = ctx.value('g1_free_space_move', arm='right', position=[tx, ty, st['top'] + 0.15], quaternion_wxyz=q, end_effector='palm',
                   frame='pelvis', preview_only=True, time_scale=2)
    if not pl.get('planned') or not pl.get('plan', {}).get('success'):
        return dict(ok=False, why=f"cuRobo plan failed: {pl.get('plan', {}).get('status')}")
    ctx.value('g1_execute_motion_plan', motion_plan_id=pl['plan']['motion_plan_id'])
    time.sleep(0.5)
    r = measure(ctx)
    st['cmd'] = [tx + (tx - r['palm'][0]) - PRESS_BACK, ty + (ty - r['palm'][1])]
    return dict(r, target=[round(tx, 3), round(ty, 3)], cmd=[round(v, 3) for v in st['cmd']])


def grasp(ctx, st):
    tx, ty = st['far'] - BACK, st['gy']
    cx, cy = st['cmd']; st['contact_ok'] = True; r = None
    for d in (0.0, -0.015, -0.03):
        r2 = move(ctx, st, (cx, cy, st['top'] + d), 2.5)
        if not r2['ok']:
            break
        r = r2
        print(json.dumps(dict(level=d, palm=r['palm'], tips_x=r['tips_x'], tilt=r['tilt'])), flush=True)
        if r['tips_x'] > st['wall'] - WALL_MARGIN or r['tilt'] > 6.5:
            return dict(r, ok=False, why=f"fingers at x {r['tips_x']} near the wall / tilt: open and raise")
        cx += float(np.clip(tx - r['palm'][0], -0.02, 0.02))                 # forward drift under the press
        if r['palm'][2] <= st['top'] + 0.012:
            break
    if r is None:
        return dict(ok=False, why='press refused: ' + r2.get('why', ''))
    st['cmd'] = [cx, cy]; st['press'] = r['palm']
    c.pps.ramp(ctx, {}, 4.0, hands=(0.0, 1.0)); time.sleep(1.0)
    rc = measure(ctx)
    print(json.dumps(dict(close_fingers=rc['fingers'], tips_x=rc['tips_x'])), flush=True)
    r = move(ctx, st, (cx, cy, r['palm'][2] + 0.03), 3.0, check=False)   # closed raise: measured palm + 3 cm
    ok = held(r)
    return dict(r, ok=ok, why='' if ok else 'empty after the closed raise: open, raise straight up, re-perceive', close=rc['fingers'])


def lift(ctx, st):
    cx, cy = st['cmd']; r = None
    zc = c.pps.link_points(c.pps.last_row(ctx)[0][9:38], 'right')['right_palm'][2]
    x, z = cx, zc
    while z < LIFT_Z - 1e-3:
        z = min(z + 0.03, LIFT_Z)
        r = move(ctx, st, (x, cy, float(z)), 4.0, check=False)
        if not r['ok'] and x < st['wall'] - RAISE_X_MAX:                  # close to the body straight up is out of reach (run 2)
            x = min(x + 0.03, st['wall'] - RAISE_X_MAX)
            r = move(ctx, st, (x, cy, float(z)), 4.0, check=False)
        print(json.dumps(dict(x=round(float(x), 3), z=round(float(z), 3), palm=r.get('palm'), fingers=r.get('fingers'), why=r.get('why'))), flush=True)
        if not held(r):
            return dict(r, ok=False, why=f"grip lost / refused at z {z:.2f}: {r.get('why')}")
        st['cmd'] = [x, cy]
    return r


def carry(ctx, st):
    cx, cy = st['cmd']; xr = st['wall'] + REL_PAST; r = None
    for x in list(np.arange(cx + 0.04, xr, 0.04)) + [xr]:
        r = move(ctx, st, (float(x), cy, LIFT_Z), 5.0, check=False)
        print(json.dumps(dict(x=round(float(x), 3), palm=r.get('palm'), fingers=r.get('fingers'))), flush=True)
        if not held(r):
            return dict(r, ok=False, why=f'grip lost / refused at x {x:.2f}')
    st['cmd'] = [xr, cy]
    return r


def release(ctx, st):
    ctx.value('g1_hand_shape', side='right')                                 # flat default open
    c.pps.ramp(ctx, {}, 2.0, hands=(0.0, 0.0)); time.sleep(2.0)
    cx, cy = st['cmd']
    r = move(ctx, st, (cx, cy, LIFT_Z + 0.05), 3.0, check=False)
    if not r['ok']:                                                       # hand stays open where it is (run 2: z +.10 refused)
        r = measure(ctx, dict(raise_refused=r['why']))
    return r


def home(ctx, st):
    import subprocess
    p = subprocess.run([sys.executable, 'skill-library/skills/plush-pick-place-box-behind/code/home_safe.py'], capture_output=True,
                       text=True, env=dict(os.environ, PYTHONPATH='.:code/puppy_box'))
    return measure(ctx, dict(home=p.stdout.strip().split('\n')[-1][:300]))


STEPS = dict(perceive=perceive, hover=hover, hover_cu=hover_cu, grasp=grasp, lift=lift, carry=carry, release=release, home=home)

def main(ctx, steps='perceive,hover', wall=None, gy=None):
    """Run the comma-separated steps in order; stops at the first step that is not ok. Returns {step: result}. LOOK at each
    step's frame before the next one (see SKILL.md perception checkpoints); never chain grasp..release unattended."""
    st = load(); out = {}
    for s in steps.split(','):
        r = STEPS[s](ctx, st, wall, gy) if s == 'perceive' else STEPS[s](ctx, st)
        save(st)
        out[s] = r
        print(json.dumps({s: r}, default=float), flush=True)
        if not r.get('ok'):
            print(json.dumps({'stopped_at': s, 'why': r.get('why')}))
            break
    return out


if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        main(ctx, sys.argv[1], *sys.argv[2:])          # STEP[,STEP...] [WALL [GY]]
