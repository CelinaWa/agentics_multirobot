"""puppy_box, box directly BEHIND the puppy (gap ~4 cm), right hand only, waist yaw cmd 0.7 held from before the approach to
after the release (session e578ed, sim). Built on skill-library plush-pick-place-box-behind run_f0.py (state output/f0_state.json).
Run from the workspace root, one step at a time, and LOOK at the frame each step prints:
  PYTHONPATH=.:code/puppy_box python code/e578/ctl.py STEP[,STEP...] [ARG]
Steps:
  hover    palm-down fingers forward over (far - BACK, gy) at top + .22 then top + .13; then the press command is corrected by the
           measured palm (as run_f0 hover). Run perceive (run_f0) first; the right hand must already be raised (prep_side) and the
           waist at its command (waist.py 0.35 then 0.7).
  grasp    press levels top + (0, -.015, -.03, -.045) until the measured palm is <= top + PRESS_GATE or stalls, FK finger-vs-wall
           gate, close 4 s, closed raise 3 cm; held = a base joint < GRIP_LOST.
  lift     3 cm up + 1.5 cm forward per 4 s step to palm cmd z LIFT_Z (never past wall - .07), grip check each step.
  carry    4 cm / 5 s steps at LIFT_Z to x = min(wall + REL_PAST, X_MAX), grip check.
  release  open flat, wait 2 s, hand up 5 cm.
Every IK move refuses a joint jump > MAX_JUMP (attempt 1: the multi-seed fallback flipped the arm branch in the lift, the palm
jumped 7-12 cm off its command and the grip loosened, then let go on the first carry step)."""
import sys, os, json, time
import numpy as np
W = os.getcwd()
sys.path.insert(0, os.path.join(W, 'skill-library/skills/plush-pick-place-box-behind/code'))
import run_f0 as F                         # noqa: E402
from api.params import param               # noqa: E402

c, mv = F.c, F.mv
MAX_JUMP = param('ik.max_joint_jump_holding_rad', 0.5)
PRESS_GATE = float(os.environ.get('PRESS_GATE', param('grasp.press_gate_above_top_m', 0.0)))
LIFT_DX = float(os.environ.get('LIFT_DX', 0.0))         # forward per lift step; 0.015 lost the grip at lift-off twice (att. 2-3)
LIFT_DZ = float(os.environ.get('LIFT_DZ', param('lift.step_dz_m', 0.02)))
YAW = float(os.environ.get('GYAW', 0.0))                 # finger yaw, deg (+ = left); attempt 3 uses -40
LIFT_Z = param('lift.palm_cmd_z_behind_m', 0.24)
X_MAX = param('release.palm_cmd_x_max_m', 0.505)
CARRY_JUMP = param('ik.max_joint_jump_carry_rad', 0.8)        # att. 5: the last 3-5 cm of the carry needed .53-.64
REL_PAST = param('release.past_near_wall_m', 0.13)


def move(ctx, st, pos, secs, check=True, max_jump=MAX_JUMP):
    row, meas, _ = c.pps.last_row(ctx)
    cmd = list(row[9:38])
    t, err, _ = c.pps.solve(cmd, 'right', pos, c.pps.palm_quat(YAW, 'right'))
    jump = max(abs(t[k] - cmd[c.J.index(k)]) for k in t)
    if err > 0.01 or jump > max_jump:
        e2, t2 = mv.best(cmd, pos, YAW, 'right')
        j2 = max(abs(t2[k] - cmd[c.J.index(k)]) for k in t2)
        if e2 <= 0.01 and j2 <= max_jump:
            t, err, jump = t2, e2, j2
    if err > 0.01 or jump > max_jump:
        return dict(ok=False, why=f'refused: IK err {err:.3f} jump {jump:.2f}')
    if check:
        ok, why = F.path_ok(cmd, t, st)
        if not ok:
            return dict(ok=False, why='path check: ' + why)
    c.pps.ramp(ctx, t, secs)
    time.sleep(0.5)
    return F.measure(ctx, dict(cmd=[round(v, 3) for v in pos], jump=round(jump, 2)))


REGRASP_ABOVE = 1.3                                     # attempt 1 (the only lift-off that held) closed both bases at .66/.72
FWD = float(os.environ.get('GRASP_FWD', 0.0))           # palm this much further forward than far - BACK (approach 3: 0.02)


def target(st):
    """Palm target for fingers yawed YAW: fingertips (palm + BACK along the fingers) at the far side, finger span centred on gy."""
    a = np.radians(YAW)
    return st['far'] - (F.BACK - FWD) * np.cos(a), st['gy'] - 0.5 * F.BACK * np.sin(a)


def hover(ctx, st):
    ctx.value('g1_hand_shape', side='right', open=[0, 0.4, 0, 0.5, 0, 0.5, 0], closed=[0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74])
    st['contact_ok'] = False
    tx, ty = target(st)
    for pos in [(tx, ty, st['top'] + 0.22), (tx, ty, st['top'] + 0.13)]:
        r = move(ctx, st, pos, 4.0, max_jump=1.0)
        if not r['ok']:
            return r
        off = float(np.linalg.norm(np.subtract(r['palm'], pos)))
        if off > 0.03:                                  # untracked IK branch (attempt 2): sync to measured, re-approach
            return dict(r, ok=False, why=f'palm {off:.3f} m off its command: untracked arm, do not press')
    st['cmd'] = [tx + (tx - r['palm'][0]) - F.PRESS_BACK, ty + (ty - r['palm'][1])]
    return dict(r, target=[round(tx, 3), round(ty, 3)], press_cmd=[round(v, 3) for v in st['cmd']])


def grasp(ctx, st):
    tx, _ = target(st)
    cx, cy = st['cmd']; st['contact_ok'] = True; r = None; last_z = None
    for d in (0.0, -0.015, -0.03, -0.045):
        r2 = move(ctx, st, (cx, cy, st['top'] + d), 2.5, max_jump=1.0)
        if not r2['ok']:
            break
        r = r2
        print(json.dumps(dict(level=d, palm=r['palm'], tips_x=r['tips_x'], tilt=r['tilt'])), flush=True)
        if r['tips_x'] > st['wall'] - F.WALL_MARGIN or r['tilt'] > 6.5:
            return dict(r, ok=False, why=f"fingers at x {r['tips_x']} near the wall / tilt: open and raise")
        cx += float(np.clip(tx - r['palm'][0], -0.02, 0.02))
        if r['palm'][2] <= st['top'] + PRESS_GATE or (last_z is not None and last_z - r['palm'][2] < 0.004):
            break                                                           # pressed in, or stalled
        last_z = r['palm'][2]
    if r is None:
        return dict(ok=False, why='press refused: ' + r2.get('why', ''))
    st['press'] = r['palm']
    c.pps.ramp(ctx, {}, 4.0, hands=(0.0, 1.0)); time.sleep(1.0)
    rc = F.measure(ctx)
    print(json.dumps(dict(close_fingers=rc['fingers'], tips_x=rc['tips_x'])), flush=True)
    if max(rc['fingers'][3], rc['fingers'][5]) > REGRASP_ABOVE:          # one finger at the fist: half-open, re-close once
        c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.3)); c.pps.ramp(ctx, {}, 4.0, hands=(0.0, 1.0)); time.sleep(1.0)
        rc = F.measure(ctx)
        print(json.dumps(dict(reclose_fingers=rc['fingers'])), flush=True)
    # raise from the MEASURED palm xy: raising at the command xy pulled the palm 3-4.6 cm back and slid the hook off (att. 1-4)
    cx, cy = rc['palm'][0], rc['palm'][1]; st['cmd'] = [cx, cy]
    r = move(ctx, st, (cx, cy, rc['palm'][2] + 0.03), 3.0, check=False, max_jump=1.0)
    ok = F.held(r)
    return dict(r, ok=ok, why='' if ok else 'empty after the closed raise: open, raise straight up, re-perceive', close=rc['fingers'])


def lift(ctx, st):
    cx, cy = st['cmd']; r = None
    z = c.pps.link_points(c.pps.last_row(ctx)[0][9:38], 'right')['right_palm'][2]
    x = cx
    while z < LIFT_Z - 1e-3:
        z = min(z + LIFT_DZ, LIFT_Z); x = min(x + LIFT_DX, st['wall'] - F.RAISE_X_MAX)
        r = move(ctx, st, (x, cy, float(z)), 4.0, check=False)
        print(json.dumps(dict(x=round(float(x), 3), z=round(float(z), 3), palm=r.get('palm'), fingers=r.get('fingers'),
                              jump=r.get('jump'), why=r.get('why'))), flush=True)
        if not F.held(r):
            return dict(r, ok=False, why=f"grip lost / refused at z {z:.2f}: {r.get('why')}")
        st['cmd'] = [x, cy]
    return r


def carry(ctx, st):
    cx, cy = st['cmd']; xr = min(st['wall'] + REL_PAST, X_MAX); r = None
    for x in list(np.arange(cx + 0.04, xr, 0.04)) + [xr]:
        r = move(ctx, st, (float(x), cy, LIFT_Z), 5.0, check=False, max_jump=CARRY_JUMP)
        print(json.dumps(dict(x=round(float(x), 3), palm=r.get('palm'), fingers=r.get('fingers'), jump=r.get('jump'), why=r.get('why'))), flush=True)
        if not F.held(r):
            return dict(r, ok=False, why=f'grip lost / refused at x {x:.2f}')
        st['cmd'] = [float(x), cy]
    return r


def release(ctx, st):
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, {}, 2.0, hands=(0.0, 0.0)); time.sleep(2.0)
    cx, cy = st['cmd']
    r = move(ctx, st, (cx, cy, LIFT_Z + 0.05), 3.0, check=False)
    if not r['ok']:
        r = F.measure(ctx, dict(raise_refused=r['why']))
    return r


STEPS = dict(hover=hover, grasp=grasp, lift=lift, carry=carry, release=release)


def main(ctx, steps='hover', **state):
    """Run comma-separated steps on output/f0_state.json (after run_f0 perceive); state kwargs (e.g. gy=-0.13, wall=0.39)
    override the state first. Stops at the first failed step. Returns {ok, measured: {step: result}, why}. LOOK between steps."""
    st = F.load(); st.update(state); out = {}
    for s in steps.split(','):
        r = STEPS[s](ctx, st)
        F.save(st); out[s] = r
        if not r.get('ok'):
            return dict(ok=False, measured=out, why=f"{s}: {r.get('why')}")
    return dict(ok=True, measured=out, why='')

if __name__ == '__main__':
    from api.runtime import Context
    st = F.load()
    with Context() as ctx:
        for s in sys.argv[1].split(','):
            r = STEPS[s](ctx, st)
            F.save(st)
            print(json.dumps({s: r}, default=float), flush=True)
            if not r.get('ok'):
                print(json.dumps({'stopped_at': s, 'why': r.get('why')}))
                break
