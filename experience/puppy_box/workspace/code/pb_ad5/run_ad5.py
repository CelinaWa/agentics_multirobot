"""Box-behind puppy_box controller, session ad5d0f (sim, right hand only). The sequence of attempt 5 (first lift of the session):
  prep      hand raised at the side (prep_side.py), left arm tucked
  perceive  lean 0.1 + run_eed perceive (hand aside, box rim corners, plush cloud) + per-y depth profile: far side at mid-body
  hover     approach at waist 0 -> waist cmd 0.7, hover yaw 0 (fingers across the body); one xy correction on the MEASURED palm
            so that palm = far side - 0.096 (fingertips land on the back), mid-body y
  grasp     light press (DEPTHS 0,-.015,-.03 GATE .012), close; then raise 3 cm CLOSED: the fingers lying on the back roll the
            plush into the hand (base joints 0 at the close is expected here). Held if a base joint < 1.4 after the raise.
  lift      4 cm / 3 s steps to palm cmd z 0.25, grip check each step
  carry     5 s steps straight forward at the grasp y to cmd x 0.48, grip check each step
  release   lean_release.py 0.2 <wall+0.045> 0.50 y REL_Z: open only past the wall, then hand up + unlean
  home      home_safe.py
Usage (workspace root): PYTHONPATH=.:code/puppy_box python code/pb_ad5/run_ad5.py STEP[,STEP...]   (state: output/ad5_state.json)
LOOK at the hover frame before `grasp`, and at the lift frame before `carry`."""
import sys, os, json, subprocess
import numpy as np
W = os.getcwd()
SK = 'skill-library/skills/plush-pick-place-box-behind/code/'
ST = os.path.join(W, 'output', 'ad5_state.json')
PB3 = os.path.join(W, 'output', 'pb3_state.json')
BACK = 0.096      # palm (measured) short of the far side along the fingers (8cef10 wrap, ad5d0f attempt 5)
OFF = (0.03, 0.02)  # measured palm - command at waist cmd 0.7 / lean 0.1 (ad5d0f hovers: +0.024..0.05, +0.02)
LIFT_Z, CARRY_X, REL_Z = 0.25, 0.48, 0.20
GRIP_LOST = 1.45


def sh(args, env=None):
    e = dict(os.environ, PYTHONPATH='.:code/puppy_box', **(env or {}))
    p = subprocess.run([sys.executable] + args, capture_output=True, text=True, env=e, timeout=600)
    lines = [l for l in p.stdout.strip().split('\n') if l.startswith('{')]
    return json.loads(lines[-1]) if lines else dict(ok=False, why=(p.stderr or p.stdout)[-400:])


def eed(frm, to, env=None):
    """run_eed.py steps frm..to; returns the report of step `to` (its last line is a stopped_at line when ok is False)."""
    e = dict(os.environ, PYTHONPATH='.:code/puppy_box', **(env or {}))
    p = subprocess.run([sys.executable, SK + 'run_eed.py', '--from', frm, '--to', to], capture_output=True, text=True, env=e, timeout=900)
    for l in reversed(p.stdout.strip().split('\n')):
        if l.startswith('{') and to in json.loads(l):
            return json.loads(l)[to]
    return dict(ok=False, why=(p.stderr or p.stdout)[-400:])


def load():
    return json.load(open(ST)) if os.path.exists(ST) else {}


def save(st):
    json.dump(st, open(ST, 'w'), indent=1)


def up(x, y, z, yaw=0, secs=3):
    return sh(['code/pb_ad5/up.py', str(x), str(y), str(z), str(yaw), str(secs)])


def held(r):
    f = r.get('fingers') or [9] * 7
    return bool(r.get('ok')) and min(f[3], f[5]) < GRIP_LOST and r.get('tilt', 99) < 7


def profile(json_path, wall_x):
    import percep as p
    P = p.load(json_path); tz = p.table_z(P)
    pp = p.puppy(P, tz, exclude=(wall_x - 0.005, 2, -2, 2)); pel, _ = p.cloud(P)
    pts = pel[pp['mask']]; pts = pts[pts[:, 2] > tz + 0.02]
    rows = []
    for y0 in np.arange(-0.40, 0.10, 0.03):
        s = pts[(pts[:, 1] >= y0) & (pts[:, 1] < y0 + 0.03)]
        if len(s) > 300:
            rows.append((y0 + 0.015, float(np.percentile(s[:, 0], 98)), float(np.percentile(s[:, 2], 97))))
    return rows, tz


def step(s, st):
    if s == 'prep':
        return sh(['code/pb_ad5/prep_side.py']) | {'ok': True}
    if s == 'perceive':
        r = eed('lean', 'perceive')
        if 'json' not in r:
            return dict(ok=False, why='perceive failed', r=r)
        rows, tz = profile(r['json'], r['wall_grasp'])
        if len(rows) < 4:
            return dict(ok=False, why='plush body not found in depth', rows=rows)
        ys = np.array([q[0] for q in rows]); body = rows[1:-1] if len(rows) > 4 else rows   # drop the thin ends (tail / head)
        my = float(np.median([q[0] for q in body]))
        far = float(np.median([q[1] for q in body if abs(q[0] - my) < 0.05]))
        (x1, y1), (x2, y2) = r['near']; wall = lambda y: x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        st.update(far=far, my=my, wall_g=wall(my), top=r['top'], tz=tz, perceive=r)
        gap = wall(my) - far
        ok = gap > 0.03 and 0.30 < far < 0.42
        return dict(ok=ok, why='' if ok else f'gap {gap:.3f} / far {far:.3f} outside the tested envelope', far=round(far, 3),
                    mid_y=round(my, 3), wall=round(wall(my), 3), rows=[[round(v, 3) for v in q] for q in rows], png=r.get('png'))
    if s == 'hover':
        tx, ty = st['far'] - BACK, st['my'] + 0.03          # mid-body, 3 cm away from the head (head right)
        cx, cy = tx - OFF[0], ty - OFF[1]
        if not os.environ.get('SKIP_VIA'):   # SKIP_VIA=1: the arm is already over the plush at waist 0.7 (approached by hand)
            r = sh([SK + 'via.py', '0', '0.0', '4', f'0.22,{cy - 0.05:.3f},0.30', f'{cx:.3f},{cy:.3f},0.28'])
            if not r.get('ok'):
                return r
            # a close plush (far < 0.33) makes the waist-0 -> 0.7 switch at the target jump 2.7 rad: switch at x 0.265 first
            r = sh([SK + 'via.py', '0', '0.7', '4', f'{max(cx, 0.265):.3f},{cy:.3f},0.26', f'{cx:.3f},{cy:.3f},0.24'])
            if not r.get('ok'):
                return r
        pb = json.load(open(PB3)); pb.update(grasp=[cx, cy], yaw=0.0, top=st['top']); json.dump(pb, open(PB3, 'w'), indent=1)
        h = eed('hover', 'hover', dict(GY='0'))
        m = h.get('measured', {}).get('palm')
        if m:   # one correction on the measured palm
            cx, cy = cx + float(np.clip(tx - m[0], -0.03, 0.03)), cy + float(np.clip(ty - m[1], -0.03, 0.03))   # clamp: a bad hover overshot (run 3)
            pb.update(grasp=[cx, cy]); json.dump(pb, open(PB3, 'w'), indent=1)
            h = eed('hover', 'hover', dict(GY='0'))
        st.update(cmd=[cx, cy], target=[tx, ty])
        fk = subprocess.run([sys.executable, 'code/pb_ad5/fk.py'], capture_output=True, text=True,
                            env=dict(os.environ, PYTHONPATH='.:code/puppy_box')).stdout
        return dict(h, target=[round(tx, 3), round(ty, 3)], cmd=[round(cx, 3), round(cy, 3)], fk=fk.split('\n')[:3])
    if s == 'grasp':
        cx, cy = st['cmd']
        g = sh([SK + 'bb_8cef10.py', 'grasp_deep'], dict(GY='0', DEPTHS='0.0,-0.015,-0.03', GATE='0.012'))
        r = up(cx, cy, st['top'] + 0.015, 0, 2.5)          # closed raise ~3 cm: rolls the plush into the hand
        st['grasp_fingers'] = r.get('fingers')
        ok = held(r)
        return dict(ok=ok, why='' if ok else 'empty after the closed raise: open, raise straight up, re-perceive', close=g, raise_=r)
    if s == 'lift':
        cx, cy = st['cmd']
        for z in np.arange(st['top'] + 0.055, LIFT_Z + 1e-6, 0.04).tolist() + [LIFT_Z]:
            r = up(cx, cy, round(z, 3), 0, 3)
            if not held(r):
                return dict(r, ok=False, why=f'grip lost at z {z:.2f}: lower over the table, open')
        return r
    if s == 'carry':
        cx, cy = st['cmd']
        for x in np.arange(cx + 0.045, CARRY_X + 1e-6, 0.045).tolist() + [CARRY_X]:
            r = up(round(x, 3), cy, LIFT_Z, 0, 5)
            if not held(r):
                return dict(r, ok=False, why=f'grip lost / refused at x {x:.2f}')
        return r
    if s == 'release':
        cx, cy = st['cmd']
        r = sh(['code/pb_ad5/release_up.py', '0.2', f"{st['wall_g'] + 0.045:.3f}", '0.50', f'{cy:.3f}', str(REL_Z)])
        r['ok'] = bool(r.get('released'))
        return r
    if s == 'home':
        r = sh([SK + 'home_safe.py'])
        r['ok'] = r.get('tilt', 99) < 6 and r.get('fingers_max', 9) < 0.5
        return r
    raise ValueError(s)


if __name__ == '__main__':
    st = {} if sys.argv[1].startswith('prep') else load()
    for s in sys.argv[1].split(','):
        r = step(s, st)
        save(st)
        print(json.dumps({s: r}, default=float)[:1500], flush=True)
        if not r.get('ok'):
            print(json.dumps({'stopped_at': s, 'why': r.get('why')}))
            break
