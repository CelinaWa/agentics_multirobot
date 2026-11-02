"""Box-behind puppy_box controller, session eed312 (right hand only), on top of the library run_bb.py steps.
Changes from run_bb (evidence: memory/PROGRESS.md attempt 1):
  - grasp: palm over the body's CREST (highest x of the body at mid-body y, from the depth profile) + 1 cm (palm lands short),
    light press DEPTHS .01,-.01 GATE .02. Palm at far-0.105 (run_bb) sat on the near slope: fingers blocked on the crest.
  - release: the near wall is slanted in the pelvis frame (box rotated with the robot heading). Release at the y inside the box
    where the wall is closest to the robot (>= 0.12 from the side walls); open only if palm >= wall(y) + OPEN_MARGIN.
Usage (workspace root): PYTHONPATH=. python code/puppy_box/run_eed.py [--from STEP] [--to STEP]
Steps: prep, lean, perceive, approach, hover, grasp, lift, carry, release, home. Look at hover/lift/release frames."""
import sys, os, json
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
SK = 'skill-library/skills/plush-pick-place-box-behind/code/'
sys.path.insert(0, os.path.join(ROOT, SK)); sys.path.insert(1, os.path.join(ROOT, 'code', 'puppy_box'))
import run_bb as R

STEPS = ['prep', 'lean', 'perceive', 'approach', 'hover', 'grasp', 'lift', 'carry', 'release', 'home']
CREST_DX = 0.01        # palm lands ~1-1.7 cm short in x
OPEN_MARGIN = 0.06     # body half-width 6.5 cm: opened at wall + 0.04 the body fell back out (attempt 1)
REL_X_CMD = 0.47       # commanded release x before the lean (reach limit at edge ~0.26)
SIDE_CLEAR = 0.12
WALL_BACK = 0.09      # palm cmd <= near wall - 0.09 (lands ~1 cm short): lifted at wall-0.10, rim contact at wall-0.077 / -0.058


def perceive():
    R.sh([SK + 'via.py', '0', '0.0', '4', '0.14,-0.33,0.30'])     # hand aside: full view of the puppy and the box
    L = R.sh([SK + 'look.py', 'rb'])
    import percep as p
    P = p.load(L['json']); tz = p.table_z(P)
    c4 = np.array(L['box']['corners'])
    near = c4[np.argsort(c4[:, 0])[:2]]
    pp = p.puppy(P, tz, exclude=(float(near[:, 0].min()), 2, -2, 2))
    pel, _ = p.cloud(P); pts = pel[pp['mask']]
    pts = pts[(pts[:, 2] > tz + 0.02) & (pts[:, 2] < tz + 0.15)]
    top = float(np.percentile(pts[:, 2], 97)); body = pts[pts[:, 2] > tz + 0.7 * (top - tz)]
    yc = float(np.median(body[:, 1]))
    sl = body[np.abs(body[:, 1] - yc) < 0.04]
    crest = sl[sl[:, 2] > np.percentile(sl[:, 2], 80)]
    gx_crest = float(np.median(crest[:, 0])) + CREST_DX
    c = body[:, :2].mean(0); a = np.linalg.svd(body[:, :2] - c)[2][0]
    n = np.array([abs(a[1]), -a[0] * np.sign(a[1])]); n = n if n[0] > 0 else -n
    yaw = float(np.degrees(np.arctan2(n[1], n[0])))
    (x1, y1), (x2, y2) = near
    wall = lambda y: x1 + (y - y1) * (x2 - x1) / (y2 - y1)
    ylo, yhi = float(c4[:, 1].min()) + SIDE_CLEAR, float(c4[:, 1].max()) - SIDE_CLEAR
    ys = np.linspace(ylo, yhi, 21) if ylo < yhi else np.array([float(c4[:, 1].mean())])
    ry = float(ys[np.argmin([wall(y) for y in ys])])
    w_rel, w_g = float(wall(ry)), float(wall(yc))
    gx = min(gx_crest, w_g - WALL_BACK)       # fingers forward reach ~0.10 past the palm: keep the tips off the rim
    st = dict(box=dict(x=[w_rel + OPEN_MARGIN - 0.03, float(c4[:, 0].max())], y=[float(c4[:, 1].min()), float(c4[:, 1].max())],
                       rim_z=tz + 0.14, table_z=tz, centre=c4.mean(0).tolist(), y_inner=float(c4[:, 1].max()), near=near.tolist()),
              box_side='behind', grasp=[gx, yc], top=top, table_z=tz, release=[REL_X_CMD, ry, 0.26], waist_cmd=0.7,
              press_depth=-0.01, yaw=yaw, wall_release=w_rel, wall_grasp=w_g)
    json.dump(st, open(R.STATE, 'w'), indent=1)
    ok = 0.22 < gx < 0.36 and abs(yaw) < 45 and w_rel < 0.42 and float(c4[:, 1].max()) - float(c4[:, 1].min()) < 0.6
    return dict(ok=bool(ok), why='' if ok else 'outside the tested envelope: look and decide', grasp=[round(gx, 3), round(yc, 3)], crest=round(gx_crest - CREST_DX, 3),
                yaw=round(yaw, 1), top=round(top, 3), wall_grasp=round(w_g, 3), wall_release=round(w_rel, 3), release=st['release'],
                open_min_x=round(w_rel + OPEN_MARGIN, 3), near=np.round(near, 3).tolist(), frame=L['frame'], png=L['png'], json=L['json'])


def finger_gate(st):
    """FK of the closed right hand (measured arm + hand joints, newest grasp observation) against the near wall: refuse the lift if
    a finger link is within 1 cm of the wall (or past it) below rim + 2 cm. eed312 run 3: a close that read 'held' (base 1.08/1.06)
    had the finger bases ON the rim; the run-2 yaw -40 close had the finger links 5-7 cm past the wall."""
    import glob
    sys.path.insert(0, os.path.join(ROOT, 'skill-library', 'skills', 'palm-pose-servo', 'code'))
    from common import M
    path = max(glob.glob(os.path.join(ROOT, 'output', 'pb_bb_grasp_*.json')), key=os.path.getmtime)
    ob = json.load(open(path)); j = ob['proprio']['joint_positions_rad']; h = ob['proprio']['hand_state']['positions_rad'][7:]
    P = M.fk(list(j), hand=dict(zip(M.hand_joints['right'], h)))
    (x1, y1), (x2, y2) = st['box']['near']; rim = st['box']['rim_z']
    worst = None
    for n in ('index_0', 'index_1', 'middle_0', 'middle_1'):
        p = P.frame_pose(f'right_hand_{n}_link')[1]
        d = p[0] - (x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1))
        if p[2] < rim + 0.02 and (worst is None or d > worst[1]):
            worst = (n, round(float(d), 3), round(float(p[2] - rim), 3))
    ok = worst is None or worst[1] < -0.01
    # NOT a reliable discriminator (replay of eed312): the one close that lifted also read +0.03 past the start-frame wall (the wall
    # estimate drifts ~2 cm as the pelvis slides). So a warning stops the chain for a LOOK; it does not decide.
    return dict(ok=ok, why='' if ok else f'WARNING finger {worst[0]} near/past the box wall (x-wall {worst[1]}, z-rim {worst[2]}): '
                'LOOK at the grasp frame; if the fingers may be on the rim, open and raise straight up (never back)', worst=worst, obs=path)


def main(start='prep', stop='home'):
    rep = {}
    for s in STEPS[STEPS.index(start):STEPS.index(stop) + 1]:
        st = json.load(open(R.STATE)) if os.path.exists(R.STATE) else {}
        gy = str(round(st.get('yaw', 0.0), 1)); g = st.get('grasp', [0, 0])
        if s == 'prep':
            r = R.sh([R.PBB, 'prep'])
        elif s == 'lean':
            r = R.sh([R.PBB, 'lean', '0.1'])
        elif s == 'perceive':
            r = perceive()
        elif s == 'approach':
            r = R.sh([SK + 'via.py', gy, '0.0', '4', f'0.22,{g[1] - 0.05:.3f},0.30', f'{g[0]:.3f},{g[1]:.3f},0.28'])
            for w, z in (('0.35', 0.27), ('0.7', 0.24)):
                if r.get('ok'):
                    r = R.sh([SK + 'via.py', gy, w, '4', f'{g[0]:.3f},{g[1]:.3f},{z}'])
        elif s == 'hover':
            r = R.sh([SK + 'bb_8cef10.py', 'hover'], dict(GY=gy))
        elif s == 'grasp':
            r = R.sh([SK + 'bb_8cef10.py', 'grasp_deep'], dict(GY=gy, DEPTHS='0.01,-0.01', GATE='0.02'))
            if r.get('ok'):
                r['finger_gate'] = fg = finger_gate(st)
                if not fg['ok']:
                    r.update(ok=False, why=fg['why'])
        elif s == 'lift':
            r = R.sh([SK + 'bb_8cef10.py', 'lift'], dict(GY=gy))
            f = (r.get('measured') or {}).get('fingers') or [0] * 7
            if r.get('ok') and min(f[3], f[5]) > 1.45:
                r.update(ok=False, why=f'grip lost in the lift (base {f[3]}/{f[5]})')
        elif s == 'carry':
            rel = st['release']
            r = R.sh([SK + 'via.py', '-8', '0.7', '5', f'0.37,{(g[1] + rel[1]) / 2:.3f},0.27', f'{rel[0]:.3f},{rel[1]:.3f},{rel[2]}'])
        elif s == 'release':
            r = R.sh([SK + 'bb_8cef10.py', 'lean_release'], dict(GY=gy))
        else:
            r = R.sh([SK + 'home_safe.py'])
            r['ok'] = r.get('tilt', 99) < 6 and r.get('fingers_max', 9) < 0.5
        rep[s] = r
        print(json.dumps({s: r}, default=float), flush=True)
        if not r.get('ok'):
            print(json.dumps({'stopped_at': s, 'why': r.get('why')}))
            return rep
    return rep


if __name__ == '__main__':
    a = sys.argv[1:]
    main(a[a.index('--from') + 1] if '--from' in a else 'prep', a[a.index('--to') + 1] if '--to' in a else 'home')
