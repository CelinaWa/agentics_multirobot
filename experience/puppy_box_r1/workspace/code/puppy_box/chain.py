"""puppy_box_r1 chained controller (session 7c03aa, sim). The ball pinch creeps out within ~15-30 s when the robot
stands still, so everything from closing the gripper to the release runs in ONE call without pauses.

  python code/puppy_box/chain.py look                 perceive: edge, paws (overhang), target paw + reference, box
  python code/puppy_box/chain.py pre                  waypoint + pregrasp 10 cm short of the grasp point (open)
  python code/puppy_box/chain.py go [walk_m] [past_m] re-localize, advance, close, lift, measure the hang, walk while
                                                      holding, reach over the box, lower, open, report
  python code/puppy_box/chain.py retreat              open, hand up/out, HOME (no walk)
State: output/chain_state.json. Every frame id is printed for checkpoint review.
"""
import sys, json, math, time
from pathlib import Path
import numpy as np, cv2
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
from api.runtime import Context
import scene, pawfit, edges
from percep import load_frame, cloud

ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
HOME = {'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0.0, 'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}
HELD_MIN = 0.02
LEG_YAW = 12.0          # deg, paw->hip vs the table normal (measured: 11.6, 12-13)
LEG_PITCH = 17.0
RIM_H = 0.145           # box rim above the table
STATE = WS / 'output' / 'chain_state.json'
LIFT = [(0.013, 0.035, 0.0), (0.013, 0.035, 0.05), (0.012, 0.035, 0.1), (0.01, 0.04, 0.2), (0.008, 0.04, 0.3),
        (0.005, 0.04, 0.35), (0.0, 0.04, 0.4), (0.0, 0.04, 0.45), (0.0, 0.04, 0.5), (0.0, 0.045, 0.5),
        (0.0, 0.035, 0.5)]
YAW_END = 2.0           # approach yaw at the end of the lift: the body hangs along the finger axis
REACH = False           # at hand z .40-.43 the arm cannot extend past x ~.46 (st10)
MAX_WALKS = 2           # the grip survived 2 walks in 3/3 runs and the 3rd in 0/3
WALK_STEP = 0.07        # walk while holding in short calls, width checked after each
HOLD_Z = 0.43           # hand z while carrying: hanging body lowest point ~.27 below the hand, rim at table+.145
REACH_X = 0.50          # holding reach at z .44 with use_waist (preview ik .008 at .48, .028 at .52)
BODY_IN = 0.08          # body centroid this far behind the box front wall
BODY_OFF = (0.05, 0.13) # hanging body centroid - hand (x, y), front-right paw grasp (stances 2, 4)
HANG = 0.27             # lowest point of the hanging puppy below the hand
ctx = Context()
T0 = time.time()


def log(**kw):
    print(json.dumps({'t': round(time.time() - T0, 1), **kw}, default=str), flush=True)


def save(**kw):
    s = load(); s.update(kw); STATE.write_text(json.dumps(s, default=str))


def load():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def width():
    return ctx.value('r1_status')['proprio']['gripper_width_m']['right']


def guarded(pos, appr, fing, w=0.07, secs=2.0, max_jump=0.8, servo=True, use_waist=False, dry=False):
    q0 = ctx.value('r1_status')['command']['upper_joints']
    try:
        pv = ctx.value('r1_move_hand', arm='right', position=[float(v) for v in pos], approach=[float(v) for v in appr],
                       fingers=[float(v) for v in fing], gripper_width_m=w, preview_only=True,
                       use_waist=use_waist)['preview']
    except RuntimeError as ex:
        log(step='REFUSED', goal=[round(float(v), 3) for v in pos], why=str(ex)[:200])
        return None
    jump = max(abs(pv['joints'][k] - q0[k]) for k in ARM)
    if pv['ik_error_m'] > 0.03 or jump > max_jump:
        log(step='REFUSED', goal=[round(float(v), 3) for v in pos], ik=pv['ik_error_m'], jump=round(jump, 3))
        return None
    if dry:
        return pv
    r = ctx.value('r1_move_hand', arm='right', position=[float(v) for v in pos], approach=[float(v) for v in appr],
                  fingers=[float(v) for v in fing], gripper_width_m=w, secs=secs, servo=servo, use_waist=use_waist)
    log(step='moved', goal=[round(float(v), 3) for v in pos], hand=r['grasp_points_pelvis_m']['right'],
        width=r['gripper_width_m']['right'], tilt=r['tilt_deg'], ik=round(pv['ik_error_m'], 4))
    return r


def front_edge(fid, zt):
    """table front edge: per 2 cm y-bin the nearest table-height point (|z-zt|<.008), robust line x = a + b y."""
    meta, rgb, dep = load_frame(fid)
    P, ok = cloud(meta, dep, stride=1)
    p = P[ok & (np.abs(P[..., 2] - zt) < 0.008) & (P[..., 0] > 0.15) & (P[..., 0] < 0.9)]
    ys, xs = [], []
    for y in np.arange(-0.45, 0.45, 0.02):
        b = p[(p[:, 1] >= y) & (p[:, 1] < y + 0.02)]
        if len(b) > 20:
            ys.append(y + 0.01); xs.append(float(np.percentile(b[:, 0], 1)))
    if len(ys) < 6:
        return None
    ys = np.array(ys); xs = np.array(xs)
    best = None
    for i in range(len(ys)):
        for j in range(i + 1, len(ys)):
            if abs(ys[j] - ys[i]) < 0.1:
                continue
            b_ = (xs[j] - xs[i]) / (ys[j] - ys[i]); a_ = xs[i] - b_ * ys[i]
            inl = np.abs(a_ + b_ * ys - xs) < 0.006
            if best is None or inl.sum() > best[0]:
                best = (inl.sum(), inl)
    inl = best[1]
    b_, a_ = np.polyfit(ys[inl], xs[inl], 1)
    yaw = -math.degrees(math.atan(b_))          # edge normal yaw (+ = normal points left)
    dist = a_ * math.cos(math.radians(yaw))
    return {'dist': round(float(dist), 3), 'yaw': round(float(yaw), 1), 'n_in': int(inl.sum()), 'n': int(len(ys))}


def perceive(tag):
    o = ctx.value('r1_observe')
    fid = o['camera']['frame_id']
    b = scene.brief(fid)
    zt = b['table_z']
    e = front_edge(fid, zt) if zt is not None else None
    if e is not None:
        e = {'dist': e['dist'], 'normal_yaw_deg': e['yaw'], 'n_in': e['n_in']}
    paws = []
    for pb in b['paw_blobs']:
        if pb['area'] < 150:
            continue
        try:
            ft = pawfit.fit(fid, *pb['pix'])
        except Exception:
            ft = None
        if ft:
            d = {'pix': pb['pix'], 'c': ft['centre'], 'n': ft['n'], 'rms': ft['rms']}
            if e:
                th = math.radians(e['normal_yaw_deg'])
                d['overhang'] = round(e['dist'] - (math.cos(th) * ft['centre'][0] + math.sin(th) * ft['centre'][1]), 3)
            paws.append(d)
    out = {'frame': fid, 'tag': tag, 'table_z': zt, 'edge': e and {'dist': e['dist'], 'yaw': e['normal_yaw_deg']},
           'paws': paws, 'box': b.get('box'), 'hand': o['proprio']['grasp_points_pelvis_m']['right'],
           'width': o['proprio']['gripper_width_m']['right'], 'waist_yaw': round(o['proprio']['joint_positions_rad'][5], 3),
           'tilt': o['tilt_deg']}
    log(step='perceive', **out)
    return out


def good(p):
    return p['n'] >= 800 and p['rms'] <= 0.001


def select(o):
    c = [p for p in o['paws'] if good(p) and p.get('overhang', 0) > 0.05]
    c = [p for p in c if min([abs(q['c'][1] - p['c'][1]) for q in o['paws'] if q is not p] or [1]) >= 0.09]
    if not c:
        return None, None
    T = min(c, key=lambda p: p['c'][1])
    refs = [q for q in o['paws'] if q is not T and good(q)]
    R = min(refs, key=lambda q: math.dist(q['c'][:2], T['c'][:2])) if refs else None
    return T, R


def legdir(edge_yaw):
    a = math.radians(LEG_YAW + (edge_yaw or 0.0))
    return np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a), 0.0])


def relocalize(o, s):
    """target paw from the reference paw + the stored offset (the target is usually hidden with the arm out)."""
    ref0 = np.array(s['ref']); off = np.array(s['target']) - ref0
    best = None
    for p in o['paws']:
        if not good(p):
            continue
        d = math.dist(p['c'][:2], ref0[:2])
        if d < 0.06 and (best is None or d < best[0]):
            best = (d, p)
    if best is None:
        return None, 'reference not found'
    return (np.array(best[1]['c']) + off).tolist(), 'ref %s moved %.3f' % (best[1]['pix'], best[0])


def body_points(fid, zt, hand):
    """3-D points of the lifted puppy: cream pixels above table+0.03, within 0.45 m of the hand."""
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 2] > 110) & (hsv[..., 1] > 35) & (hsv[..., 1] < 140) & (hsv[..., 0] > 8) & (hsv[..., 0] < 35)
    P, ok = cloud(meta, dep, stride=1)
    m = ok & cream
    p = P[m]
    p = p[(p[:, 2] > zt + 0.03) & (np.linalg.norm(p - np.array(hand), axis=1) < 0.45)]
    if len(p) < 200:
        return None
    return {'n': int(len(p)), 'centroid': np.round(np.median(p, 0), 3).tolist(),
            'zmin': round(float(np.percentile(p[:, 2], 2)), 3), 'xmin': round(float(np.percentile(p[:, 0], 2)), 3),
            'xmax': round(float(np.percentile(p[:, 0], 98)), 3), 'ymin': round(float(np.percentile(p[:, 1], 2)), 3),
            'ymax': round(float(np.percentile(p[:, 1], 98)), 3)}


def rim_box(fid, zt, gate=None):
    """box rim from depth: low-saturation bright points at table+RIM_H. gate=(front_expected, (ylo, yhi), hand):
    keep x in [front-.06, front+.34], y in [ylo-.12, yhi+.12], drop points near the arm (|y-hand_y|<.09, x<hand_x+.05)."""
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    P, ok = cloud(meta, dep, stride=1)
    m = ok & (np.abs(P[..., 2] - (zt + RIM_H)) < 0.015) & (P[..., 0] > 0.25) & (P[..., 0] < 1.12) & (hsv[..., 1] < 40)
    p = P[m]
    if gate is not None:
        fe, (ylo, yhi), hand = gate
        p = p[(p[:, 0] > fe - 0.06) & (p[:, 0] < fe + 0.34) & (p[:, 1] > ylo - 0.12) & (p[:, 1] < yhi + 0.12)]
        p = p[~((np.abs(p[:, 1] - hand[1]) < 0.09) & (p[:, 0] < hand[0] + 0.05))]
    if len(p) < 150:
        return None
    lo, hi = np.percentile(p[:, :2], 2, 0), np.percentile(p[:, :2], 98, 0)
    return {'n': int(len(p)), 'x': [round(float(lo[0]), 3), round(float(hi[0]), 3)],
            'y': [round(float(lo[1]), 3), round(float(hi[1]), 3)]}


def leg_axis(fid, c, r_leg=0.016):
    """Leg axis (paw -> hip) of the target paw from x-slices of the leg band ahead of the paw: the RIGHT edge (-y side)
    of the run that starts at the paw is the clean leg silhouette (the left side merges with the body/other leg).
    Returns yaw (deg, pelvis frame), the axis point at the paw's x, the leg top z, and the fit residual."""
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    P, ok = cloud(meta, dep, stride=1)
    c = np.array(c)
    p = P[ok & (hsv[..., 2] > 110) & (np.linalg.norm(P - c, axis=2) < 0.12)]
    xs, ys, zt = [], [], []
    for dx in np.arange(0.03, 0.0701, 0.005):
        sl = p[(np.abs(p[:, 0] - (c[0] + dx)) < 0.0025) & (p[:, 1] > c[1] - 0.04) & (p[:, 1] < c[1] + 0.08)]
        if len(sl) < 10:
            continue
        yy = np.sort(sl[:, 1])
        runs = []; st = yy[0]; pv = yy[0]
        for y in yy[1:]:
            if y - pv > 0.004:
                runs.append((st, pv)); st = y
            pv = y
        runs.append((st, pv))
        # the run containing / nearest the paw's y line (leg starts at the paw)
        rr = [r for r in runs if r[1] > c[1] - 0.01 and r[1] - r[0] > 0.015]
        if not rr:
            continue
        r0 = min(rr, key=lambda r: r[0] if r[0] > c[1] - 0.03 else 9)
        xs.append(dx); ys.append(r0[0] - c[1]); zt.append(float(np.percentile(sl[(sl[:, 1] > r0[0]) & (sl[:, 1] < r0[0] + 0.03)][:, 2], 95)))
    if len(xs) < 4:
        return None
    xs = np.array(xs); ys = np.array(ys)
    b, a = np.polyfit(xs, ys, 1)
    res = float(np.sqrt(np.mean((a + b * xs - ys) ** 2)))
    yaw = math.degrees(math.atan(b))
    lat0 = a + r_leg / math.cos(math.radians(yaw))     # axis y offset at dx = 0
    return {'yaw': round(yaw, 1), 'axis_at_paw': [float(c[0]), float(c[1] + lat0), float(c[2])], 'lat0': round(lat0, 4),
            'res': round(res, 4), 'n': int(len(xs)), 'top_z': round(float(np.median(zt)), 3),
            'edge': [[round(float(x), 3), round(float(y), 4)] for x, y in zip(xs, ys)]}


def choose_approach(pos, leg_yaw_deg, fing):
    """the commanded approach (yaw offset, pitch) whose ACHIEVED yaw matches the leg yaw with a small IK error
    (without the waist the IK trades yaw for position: cmd 20.3 -> achieved 17.0, ik .016; cmd 22/-15 -> 21.2, ik .003)."""
    best = None
    for dy in (0, 2, 4, 6):
        for pc in (0, -8, -15):
            a = math.radians(leg_yaw_deg + dy); pp = math.radians(pc)
            ap = [math.cos(a) * math.cos(pp), math.sin(a) * math.cos(pp), math.sin(pp)]
            try:
                pv = ctx.value('r1_move_hand', arm='right', position=[float(v) for v in pos], approach=ap,
                               fingers=[float(v) for v in fing], gripper_width_m=0.07, preview_only=True)['preview']
            except RuntimeError:
                continue
            aa = pv['approach_achieved']
            ya = math.degrees(math.atan2(aa[1], aa[0]))
            cost = abs(ya - leg_yaw_deg) + 400 * max(0.0, pv['ik_error_m'] - 0.005)
            if best is None or cost < best[0]:
                best = (cost, ap, round(ya, 1), round(pv['ik_error_m'], 4), dy, pc)
    log(step='approach', cost=round(best[0], 2), yaw_ach=best[2], ik=best[3], dyaw=best[4], pitch=best[5])
    return best[1]


def main():
    step = sys.argv[1]
    a = [float(v) for v in sys.argv[2:]]
    if step == 'look':
        if STATE.exists():
            STATE.unlink()
        o = perceive('look')
        T, R = select(o)
        if T is None:
            log(step='STOP', why='no clean overhanging paw'); sys.exit(2)
        ax = leg_axis(o['frame'], T['c'])
        if ax is None or ax['res'] > 0.004 or not (-10 < ax['yaw'] < 45) or \
                min(e_[1] for e_ in ax['edge']) < -0.037:      # edge at the slice window boundary = not the leg
            log(step='STOP', why='leg axis not measured reliably', axis=ax); sys.exit(2)
        a_ = math.radians(ax['yaw'])
        ld = np.array([math.cos(a_), math.sin(a_)]); f = np.array([-math.sin(a_), math.cos(a_), 0.0])
        save(target=ax['axis_at_paw'], paw_fit=T['c'], ref=R and R['c'], edge=o['edge'], table_z=o['table_z'],
             box=o['box'], leg=ld.tolist(), fing=f.tolist(), axis=ax)
        log(step='selected', target=T, ref=R, axis=ax)
    elif step == 'pre':
        s = load()
        P = np.array(s['target']); ld = np.array(s['leg']); f = s['fing']
        appr = [ld[0], ld[1], 0.0]
        G = P[:2] + 0.05 * ld
        wp = [G[0] - 0.12 * ld[0], G[1] - 0.12 * ld[1] - 0.01, 0.12]
        if guarded(wp, appr, f, secs=2.5, max_jump=3.0) is None:
            sys.exit(2)
        pre = [G[0] - 0.10 * ld[0], G[1] - 0.10 * ld[1], P[2] + 0.012]
        appr = choose_approach([G[0], G[1], P[2]], math.degrees(math.atan2(ld[1], ld[0])), f)
        save(grasp_appr=appr)
        if guarded(pre, appr, f, secs=2.0) is None:
            sys.exit(2)
        perceive('pre')
    elif step == 'go':
        walk_m = a[0] if a else 0.15
        past = a[1] if len(a) > 1 else 0.05
        dzg = a[2] if len(a) > 2 else 0.008
        s = load()
        ld = np.array(s['leg']); f = s['fing']
        o = perceive('grasp0')
        P, how = relocalize(o, s)
        if P is None:
            log(step='STOP', why=how); sys.exit(2)
        P = np.array(P)
        hand = np.array(o['hand'])
        lat = float((P[:2] - hand[:2]) @ np.array([-ld[1], ld[0]]))
        log(step='target', P=P.round(3).tolist(), how=how, lateral=round(lat, 4))
        if abs(lat) > 0.02:
            log(step='STOP', why='lateral misalignment %.3f > .02: re-run pre' % lat); sys.exit(2)
        G = [P[0] + past * ld[0], P[1] + past * ld[1], P[2] + dzg]
        appr_g = choose_approach(G, math.degrees(math.atan2(ld[1], ld[0])), f)
        if guarded(G, appr_g, f, secs=1.5) is None:
            sys.exit(2)
        g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
        w = g['gripper']['measured_m']
        log(step='closed', width=w)
        if w < HELD_MIN:
            log(step='MISS'); sys.exit(3)
        # lift (servo off), no pauses
        x, y, z = g['grasp_points_pelvis_m']['right']
        yaw0 = math.atan2(ld[1], ld[0]); yaw1 = math.radians(YAW_END)
        for i, (da, dz, az) in enumerate(LIFT):
            x, y, z = x + da * ld[0], y + da * ld[1], z + dz
            if i == len(LIFT) - 1:
                x += 0.02
            t_ = min(1.0, max(0.0, (i - 2) / (len(LIFT) - 3)))     # keep the grasp yaw for 2 steps, then rotate
            yw = yaw0 + (yaw1 - yaw0) * t_
            ap = (math.cos(yw), math.sin(yw), az); fi = (-math.sin(yw), math.cos(yw), 0.0)
            r = guarded((x, y, z), ap, fi, w=0.0, secs=1.2, max_jump=0.45, servo=False)
            if r is None:
                break
            if r['gripper_width_m']['right'] < HELD_MIN:
                log(step='SLIP', at=i); sys.exit(3)
        o2 = ctx.value('r1_observe')
        fid = o2['camera']['frame_id']; hand = o2['proprio']['grasp_points_pelvis_m']['right']
        bp = body_points(fid, s['table_z'], hand)
        log(step='lifted', frame=fid, hand=hand, width=o2['proprio']['gripper_width_m']['right'], body=bp)
        if o2['proprio']['gripper_width_m']['right'] < HELD_MIN:
            log(step='SLIP', at='after lift'); sys.exit(3)
        # carry: hand up to HOLD_Z, walk in short steps (re-raising the hand after each), reach, lower, open
        zt = s['table_z']; rim_z = zt + RIM_H
        box0 = s['box']; front0 = box0['x'][0]; y0 = box0['y']
        hx, hy, hz = o2['proprio']['grasp_points_pelvis_m']['right']
        appr = (math.cos(math.radians(YAW_END)), math.sin(math.radians(YAW_END)), 0.5)
        f = (-math.sin(math.radians(YAW_END)), math.cos(math.radians(YAW_END)), 0.0)

        def hold_at(x, y, z, waist=False, secs=1.5):
            return guarded((x, y, z), appr, f, w=0.0, secs=secs, max_jump=0.6, servo=False, use_waist=waist)

        if hz < 0.40:
            hold_at(hx, hy, 0.43)
        o3 = ctx.value('r1_observe'); fid3 = o3['camera']['frame_id']
        hand3 = o3['proprio']['grasp_points_pelvis_m']['right']
        bp3 = body_points(fid3, zt, hand3) or {'zmin': hand3[2] - HANG}
        off = np.array([BODY_OFF[0], BODY_OFF[1], -0.12])   # measured (stances 2, 4); camera sees little of it
        hang = HANG
        walk_need = front0 + BODY_IN - ((REACH_X if REACH else hand3[0]) + off[0])
        log(step='hang', frame=fid3, hand=hand3, body=bp3, off=off.round(3).tolist(), hang=round(hang, 3),
            walk_need=round(walk_need, 3), rim_clear=round(bp3['zmin'] - rim_z, 3))
        edge0 = (s['edge'] or {}).get('dist', 0.39)
        walk_need = min(walk_need, edge0 - 0.15)
        walked = 0.0
        nwalk = 0
        while walked < walk_need - 0.02 and nwalk < MAX_WALKS:
            nwalk += 1
            cmd = min(WALK_STEP, walk_need - walked)
            w0 = width()
            wr = ctx.value('r1_walk', forward_m=round(cmd, 3))
            wk = float(wr.get('walked_m') or cmd); walked += wk
            st = ctx.value('r1_status')['proprio']
            hp = st['grasp_points_pelvis_m']['right']; w1 = st['gripper_width_m']['right']
            log(step='walk', cmd=round(cmd, 3), walked=wk, total=round(walked, 3), tilt=wr.get('tilt_deg'),
                w_before=w0, w_after=w1, hand=hp)
            if w1 < HELD_MIN:
                log(step='SLIP', at='walk'); sys.exit(3)
            if hp[2] < 0.40 and nwalk < MAX_WALKS:   # only if the body would touch the rim; moves under load cost grip
                hold_at(hp[0], hp[1], 0.42, secs=1.5)
        # box now: dead reckoning in x, rim (gated) for y if seen
        front = front0 - walked
        o4 = ctx.value('r1_observe'); fid4 = o4['camera']['frame_id']
        hand4 = o4['proprio']['grasp_points_pelvis_m']['right']
        rb = rim_box(fid4, zt, gate=(front, y0, hand4))
        ylo, yhi = (rb['y'] if rb and 0.34 < rb['y'][1] - rb['y'][0] < 0.44 else y0)
        bp4 = body_points(fid4, zt, hand4)
        tx = min(REACH_X, front + BODY_IN + 0.03 - off[0])
        ty = (ylo + yhi) / 2 - off[1]
        log(step='box_now', frame=fid4, front=round(front, 3), rim=rb, y=[ylo, yhi], body=bp4, hand=hand4,
            reach_goal=[round(tx, 3), round(ty, 3)])
        if REACH:
            for k in range(4):
                if hold_at(max(tx, hand4[0]) - 0.012 * k, ty, max(hand4[2], rim_z + 0.03 + HANG), secs=2.0):
                    break
        w5 = width()
        log(step='release_now', hand=hand4, width=w5, body_est=(np.array(hand4) + off).round(3).tolist(),
            front=round(front, 3), y=[ylo, yhi])
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        time.sleep(1.5)
        o6 = ctx.value('r1_observe')
        log(step='released', frame=o6['camera']['frame_id'], hand=o6['proprio']['grasp_points_pelvis_m']['right'],
            walked=round(walked, 3))
        save(walked_total=walked, released_frame=o6['camera']['frame_id'])
        sys.exit(0)
    elif step == 'retreat':
        # open; hand up; out to the right side at low height; walk back (if near the table); HOME
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        st = ctx.value('r1_status'); hp = st['proprio']['grasp_points_pelvis_m']['right']
        guarded([hp[0] - 0.02, hp[1], min(hp[2] + 0.06, 0.45)], (0.9, 0.0, 0.4), (0, 1, 0), secs=1.5, max_jump=3.0)
        guarded((0.14, -0.40, 0.08), (0.9, -0.3, 0.3), (0, 0, 1), secs=2.5, max_jump=3.5)
        back = a[0] if a else 0.15
        if back > 0:
            wr = ctx.value('r1_walk', forward_m=-back)
            log(step='walk_back', walked=wr.get('walked_m'), tilt=wr.get('tilt_deg'))
        r = ctx.value('r1_move_joints', joints=HOME, secs=2.0)
        log(step='home', hand=r['grasp_points_pelvis_m']['right'], tilt=r['tilt_deg'])
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
