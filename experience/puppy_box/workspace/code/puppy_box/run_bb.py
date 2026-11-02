"""Box-behind puppy_box controller (session 8cef10 run 5 sequence, right hand only). Chains CLI steps, stops at the first failed
check and prints the frames to look at. Usage: PYTHONPATH=. python code/puppy_box/run_bb.py [--from STEP]
Steps: prep, perceive, lean, approach, hover, grasp, lift, carry, release, home.
Grasp that wraps: fingers perpendicular to the body axis (PCA), palm 0.096 short of the far side, light press (gate palm <= top+0.012).
Release: carry straight forward at the grasp y, then the library's gated lean release (open only past near wall + 0.03)."""
import sys, os, json, subprocess, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
PBB = 'skill-library/skills/plush-pick-place-box-behind/code/pbb.py'
STATE = os.path.join(ROOT, 'output', 'pb3_state.json')
STEPS = ['prep', 'perceive', 'lean', 'approach', 'hover', 'grasp', 'lift', 'carry', 'release', 'home']


def sh(args, env=None):
    e = dict(os.environ, PYTHONPATH='.', **(env or {}))
    r = subprocess.run([sys.executable] + args, cwd=ROOT, env=e, capture_output=True, text=True, timeout=300)
    lines = [l for l in r.stdout.strip().splitlines() if l.startswith('{')]
    out = json.loads(lines[-1]) if lines else {'ok': False, 'why': r.stderr[-400:]}
    if r.returncode != 0 and out.get('ok', True):
        out['ok'] = False
    return out


def perceive():
    sh(['code/puppy_box/via.py', '0', '0.0', '4', '0.14,-0.33,0.30'])     # hand aside: full view of the puppy and the box
    L = sh(['code/puppy_box/look.py', 'rb'])
    import percep as p
    P = p.load(L['json']); tz = p.table_z(P)
    c4 = np.array(L['box']['corners'])
    near = c4[np.argsort(c4[:, 0])[:2]]                                   # near wall = the two corners closest to the robot
    pp = p.puppy(P, tz, exclude=(float(near[:, 0].min()) + 0.0, 2, -2, 2))
    pel, _ = p.cloud(P); pts = pel[pp['mask']]
    pts = pts[(pts[:, 2] > tz + 0.02) & (pts[:, 2] < tz + 0.15)]
    top = float(np.percentile(pts[:, 2], 97)); body = pts[pts[:, 2] > tz + 0.7 * (top - tz)]
    c = body[:, :2].mean(0); a = np.linalg.svd(body[:, :2] - c)[2][0]
    n = np.array([abs(a[1]), -a[0] * np.sign(a[1])]); n = n if n[0] > 0 else -n      # finger direction: perpendicular, forward
    far = float(np.percentile((pts[:, :2] - c) @ n, 98))
    palm = c + (far - 0.105) * n
    yaw = float(np.degrees(np.arctan2(n[1], n[0])))
    (x1, y1), (x2, y2) = near
    wall = x1 + (palm[1] - y1) * (x2 - x1) / (y2 - y1)                 # near wall x at the grasp y
    rel = [float(min(wall + 0.055, 0.47)), float(palm[1]), 0.26]
    st = dict(box=dict(x=[float(wall), float(c4[:, 0].max())], y=[float(c4[:, 1].min()), float(c4[:, 1].max())], rim_z=tz + 0.14,
                       table_z=tz, centre=c4.mean(0).tolist(), y_inner=float(c4[:, 1].max())),
              box_side='behind', grasp=[float(palm[0]), float(palm[1])], top=top, table_z=tz, release=rel, far=float((c + far * n)[0]),
              waist_cmd=0.7, press_depth=-0.03, yaw=yaw)
    json.dump(st, open(STATE, 'w'), indent=1)
    ok = 0.18 < palm[0] < 0.33 and abs(yaw) < 45 and wall < 0.44
    return dict(ok=bool(ok), why='' if ok else 'puppy/box outside the tested envelope: look and decide', grasp=st['grasp'],
                yaw=round(yaw, 1), top=round(top, 3), near_wall=round(float(wall), 3), release=rel, frame=L['frame'], png=L['png'])


def main(ctx=None, start='prep'):
    """ctx is unused: every step runs as its own CLI process (one robot user at a time; do not pass a live Context while it runs)."""
    rep = {}
    for s in STEPS[STEPS.index(start):]:
        st = json.load(open(STATE)) if os.path.exists(STATE) else {}
        gy = str(round(st.get('yaw', 0.0), 1)); g = st.get('grasp', [0, 0])
        if s == 'prep':
            r = sh([PBB, 'prep'])
        elif s == 'perceive':
            r = perceive()
        elif s == 'lean':
            r = sh([PBB, 'lean', '0.1'])
        elif s == 'approach':
            r = sh(['code/puppy_box/via.py', gy, '0.0', '4', f'0.22,{g[1] - 0.05:.3f},0.30', f'{g[0]:.3f},{g[1]:.3f},0.28'])
            for w, z in (('0.35', 0.27), ('0.7', 0.24)):
                if r.get('ok'):
                    r = sh(['code/puppy_box/via.py', gy, w, '4', f'{g[0]:.3f},{g[1]:.3f},{z}'])
        elif s == 'hover':
            r = sh(['code/puppy_box/bb.py', 'hover'], dict(GY=gy))
        elif s == 'grasp':
            r = sh(['code/puppy_box/bb.py', 'grasp_deep'], dict(GY=gy, DEPTHS='0,-0.015,-0.03', GATE='0.012'))
        elif s == 'lift':
            r = sh(['code/puppy_box/bb.py', 'lift'], dict(GY=gy))
        elif s == 'carry':
            rel = st['release']
            r = sh(['code/puppy_box/via.py', '-8', '0.7', '5', f'0.37,{g[1]:.3f},0.27', f'{rel[0]:.3f},{rel[1]:.3f},{rel[2]}'])
        elif s == 'release':
            r = sh(['code/puppy_box/bb.py', 'lean_release'], dict(GY=gy))
        else:
            r = sh(['code/puppy_box/home_safe.py'])
            r['ok'] = r.get('tilt', 99) < 6 and r.get('fingers_max', 9) < 0.5
        rep[s] = r
        print(json.dumps({s: r}, default=float), flush=True)
        if not r.get('ok'):
            print(json.dumps({'stopped_at': s, 'why': r.get('why'), 'frame': r.get('frame')}))
            if s in ('grasp', 'lift', 'carry', 'release'):
                print('hand may hold the puppy: look at the frame, then open/raise (code/puppy_box/openup.py) or home_safe')
            return rep
    print(json.dumps({'done': True, 'check': 'look into the box in the home frame and ask the operator'}))
    return rep


if __name__ == '__main__':
    main(None, sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] == '--from' else 'prep')
