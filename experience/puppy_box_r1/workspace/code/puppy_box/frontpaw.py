"""puppy_box_r1 step controller, session cdc45b (sim): horizontal-finger PIVOT PINCH of the clean overhanging FRONT
paw ("F") + servo-off arc lift. Run one step at a time, read the JSON, look at the printed frame, then decide.

  python code/puppy_box/frontpaw.py look              observe; sphere-fit all paws; pick F and its reference paw
  python code/puppy_box/frontpaw.py walk <fwd_m>      r1_walk straight (arms home), then look
  python code/puppy_box/frontpaw.py pre               waypoint above/behind F, then pregrasp 4.5 cm short of F (open)
  python code/puppy_box/frontpaw.py grasp [past_m]    re-measure F with the arm out, advance so the grasp point is
                                                      past_m (default .045) beyond the ball centre, close, report width
  python code/puppy_box/frontpaw.py lift              smooth arc lift (servo off) to hand z ~.36-.40, width each step
  python code/puppy_box/frontpaw.py retreat           open, hand up/out, HOME joints, walk back .2

F = the overhanging paw (overhang >= .05) with the lowest y (rightmost) whose nearest other paw is >= .09 m away in y.
Its pelvis position is the direct sphere fit when that fit is good (n >= 800, rms <= .001), else reference paw +
offset (offset measured at the start pose and rotated by the heading change). Leg yaw (vs table normal) LEG_YAW_T.
Every arm move is previewed: refused if ik > .03 or an arm joint jumps > max_jump (IK branch flip guard).
Exit codes: 0 ok, 2 refused/stopped, 3 not held/slipped. State is kept in output/frontpaw_state.json.
"""
import sys, json, math, time
from pathlib import Path
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
from api.runtime import Context
from api.params import param

ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
HOME = {'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0.0, 'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}
LEG_YAW_T = param('puppy.front_paw_leg_yaw_vs_table_deg', default=11.6)   # measured twice in sim (11.6, 11-13 f6b331)
LEG_PITCH = math.radians(17)
HELD_MIN = 0.02
STATE = WS / 'output' / 'frontpaw_state.json'
LIFT = [(0.012, 0.035, 0.0), (0.012, 0.035, 0.0), (0.010, 0.035, 0.0), (0.008, 0.04, 0.1), (0.005, 0.04, 0.2),
        (0.0, 0.04, 0.3), (0.0, 0.04, 0.35), (0.0, 0.04, 0.4), (0.0, 0.04, 0.4)]   # (along leg, up, approach z)
ctx = Context()


def log(**kw):
    print(json.dumps({'t': time.strftime('%H:%M:%S'), **kw}, default=str), flush=True)


def save(**kw):
    s = json.loads(STATE.read_text()) if STATE.exists() else {}
    s.update(kw)
    STATE.write_text(json.dumps(s, default=str))


def load():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def look(tag):
    import scene, pawfit, edges
    o = ctx.value('r1_observe')
    fid = o['camera']['frame_id']
    pr = o['proprio']
    b = scene.brief(fid)
    L = edges.lines(fid, b['table_z'])['lines']
    fr = [l for l in L if abs(l['normal_yaw_deg']) < 35 and l['dist'] > 0.2]
    e = max(fr, key=lambda l: l['n_in']) if fr else None
    paws = []
    for pb in b['paw_blobs']:
        if pb['area'] < 150:
            continue
        ft = pawfit.fit(fid, *pb['pix'])
        if ft:
            paws.append({'pix': pb['pix'], 'c': ft['centre'], 'n': ft['n'], 'rms': ft['rms']})
    out = {'frame': fid, 'hand': pr['grasp_points_pelvis_m']['right'], 'width': pr['gripper_width_m']['right'],
           'waist': round(pr['joint_positions_rad'][5], 3), 'tilt': o['tilt_deg'], 'table_z': b['table_z'],
           'edge': e and {'dist': e['dist'], 'yaw': e['normal_yaw_deg']}, 'paws': paws}
    if out['edge']:
        save(edge=out['edge'])
    elif load().get('edge'):
        out['edge'] = dict(load()['edge'], stale=True)   # last seen edge (the pelvis may have moved 1-2 cm)
    log(step='look', tag=tag, **out)
    return out


def heading(o):
    """robot heading vs the table normal (deg, + = turned left) from the edge line, else the saved one."""
    if o.get('edge'):
        return -o['edge']['yaw']
    return load().get('heading', 0.0)


def good(p):
    return p['n'] >= 800 and p['rms'] <= 0.001


def locate_F(o):
    """F and its reference paw. At the start (first call) choose F by geometry and store the offset ref->F in the
    table frame; later use the direct fit if good, else the reference + rotated offset."""
    h = math.radians(heading(o))
    s = load()
    paws = o['paws']
    if 'offset_T' not in s:
        if not o.get('edge'):
            return None, 'no edge line in the first look'
        cand = []
        for p in paws:
            dx = o['edge']['dist'] - (p['c'][0] * math.cos(-h) + p['c'][1] * math.sin(-h))   # overhang past the edge
            others = [q for q in paws if q is not p]
            gap = min([abs(q['c'][1] - p['c'][1]) for q in others] or [1])
            if dx > 0.05 and gap >= 0.09 and good(p):
                cand.append(p)
        if not cand:
            return None, 'no clean overhanging paw (overhang > .05, y-gap >= .09)'
        F = min(cand, key=lambda p: p['c'][1])
        ref = max([q for q in paws if q is not F and good(q)], key=lambda q: q['n'])
        d = [F['c'][i] - ref['c'][i] for i in range(3)]
        offT = [d[0] * math.cos(h) - d[1] * math.sin(h), d[0] * math.sin(h) + d[1] * math.cos(h), d[2]]
        save(offset_T=offT, ref_pix=ref['pix'], F0=F['c'], heading=math.degrees(h))
        return F['c'], 'direct (first look); reference %s' % ref['pix']
    offT = s['offset_T']
    # 1) the same geometric rule as the first look (clean overhanging rightmost paw), if the edge is seen
    if o.get('edge'):
        cand = []
        for p in paws:
            dx = o['edge']['dist'] - (p['c'][0] * math.cos(-h) + p['c'][1] * math.sin(-h))
            gap = min([abs(q['c'][1] - p['c'][1]) for q in paws if q is not p] or [1])
            if dx > 0.05 and gap >= 0.09 and good(p):
                cand.append(p)
        if cand:
            F = min(cand, key=lambda p: p['c'][1])
            if abs(F['c'][1] - min(q['c'][1] for q in paws)) < 1e-9:
                return F['c'], 'direct fit, geometric rule (overhang, y-gap >= .09, rightmost)'
    # 2) direct fit: the good paw nearest the predicted F
    best = None
    for ref in [q for q in paws if good(q)]:
        pred = [ref['c'][0] + offT[0] * math.cos(-h) - offT[1] * math.sin(-h),
                ref['c'][1] + offT[0] * math.sin(-h) + offT[1] * math.cos(-h), ref['c'][2] + offT[2]]
        for p in paws:
            dd = math.dist(p['c'][:2], pred[:2])
            if p is not ref and dd < 0.03 and good(p):
                return p['c'], 'direct fit (pred err %.3f)' % dd
        if best is None:
            best = pred
    # no good direct fit: use the reference that sits where the stored reference should be (largest n)
    refs = sorted([q for q in paws if good(q)], key=lambda q: -q['n'])
    for ref in refs:
        pred = [ref['c'][0] + offT[0] * math.cos(-h) - offT[1] * math.sin(-h),
                ref['c'][1] + offT[0] * math.sin(-h) + offT[1] * math.cos(-h), ref['c'][2] + offT[2]]
        partial = [p for p in paws if math.dist(p['c'][:2], pred[:2]) < 0.03]
        if partial or len(refs) == 1:
            return pred, 'reference %s + offset (partial fit agrees: %s)' % (ref['pix'], bool(partial))
    return None, 'F not found (no reference paw consistent with the stored offset)'


def guarded(pos, appr, fing, w=0.07, secs=2.0, max_jump=0.8, servo=True):
    q0 = ctx.value('r1_status')['command']['upper_joints']
    pv = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(appr), fingers=list(fing),
                   gripper_width_m=w, preview_only=True)['preview']
    jump = max(abs(pv['joints'][k] - q0[k]) for k in ARM)
    if pv['ik_error_m'] > 0.03 or jump > max_jump:
        log(step='REFUSED', goal=pos, ik=pv['ik_error_m'], jump=round(jump, 3))
        return None
    r = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(appr), fingers=list(fing),
                  gripper_width_m=w, secs=secs, servo=servo)
    log(step='moved', goal=[round(v, 4) for v in pos], hand=r['grasp_points_pelvis_m']['right'],
        width=r['gripper_width_m']['right'], tilt=r['tilt_deg'], ik=pv['ik_error_m'])
    return r


def leg_dir(o):
    a = math.radians(LEG_YAW_T - heading(o))
    return (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a), 0.0)


def main():
    step = sys.argv[1]
    a = [float(v) for v in sys.argv[2:]]
    if step == 'look':
        if STATE.exists() and a and a[0] == 0:
            STATE.unlink()
        o = look('look')
        F, how = locate_F(o)
        log(step='F', F=F, how=how, heading=heading(o))
    elif step == 'walk':
        r = ctx.value('r1_walk', forward_m=a[0])
        log(step='walk', walked=r.get('walked_m'), pelvis=r.get('pelvis'), tilt=r.get('tilt_deg'), fallen=r.get('fallen'))
        if r.get('fallen') or (r.get('tilt_deg') or 0) > 8:
            sys.exit(2)
        o = look('walk')
        F, how = locate_F(o)
        save(heading=heading(o))
        log(step='F', F=F, how=how, heading=heading(o))
    elif step == 'pre':
        o = look('pre0')
        F, how = locate_F(o)
        if F is None:
            log(step='STOP', why=how); sys.exit(2)
        (dx, dy), fing = leg_dir(o)
        appr = (dx, dy, 0.0)
        z = F[2] + 0.012
        wp = (F[0] - 0.12 * dx, F[1] - 0.12 * dy - 0.01, 0.12)
        if guarded(wp, appr, fing, secs=2.5, max_jump=3.0) is None:
            sys.exit(2)
        P = (F[0] - 0.045 * dx, F[1] - 0.045 * dy, z)
        if guarded(P, appr, fing, secs=2.0, max_jump=0.8) is None:
            sys.exit(2)
        save(F_pre=F, appr=appr, fing=fing)
        look('pre')
    elif step == 'grasp':
        past = a[0] if a else 0.045
        o = look('grasp0')
        F, how = locate_F(o)
        if F is None:
            log(step='STOP', why=how); sys.exit(2)
        (dx, dy), fing = leg_dir(o)
        hand = o['hand']
        lat = (F[0] - hand[0]) * (-dy) + (F[1] - hand[1]) * dx
        log(step='F_vs_hand', F=F, how=how, lateral=round(lat, 4), width=o['width'])
        if o['width'] > 0.0712:
            log(step='WARN', why='width > .071 before the advance: a jaw may touch a paw')
        G = (F[0] + past * dx, F[1] + past * dy, F[2] + past * math.tan(LEG_PITCH) + 0.002)
        s = load()
        bias = s.get('bias', [0.0, 0.0, 0.0])
        Gc = [G[i] - bias[i] for i in range(3)]
        r = guarded(Gc, (dx, dy, 0.0), fing, secs=2.0, max_jump=0.8)
        if r is None:
            sys.exit(2)
        g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.5)
        w = g['gripper']['measured_m']
        save(G=G, grasp_width=w, leg=[dx, dy])
        log(step='closed', width=w, held=w >= HELD_MIN, hand=g['grasp_points_pelvis_m']['right'],
            note='.031-.040 = shaft, .041-.048 = ball pinch (both lifted here), < .012 = miss')
        sys.exit(0 if w >= HELD_MIN else 3)
    elif step == 'lift':
        s = load()
        dx, dy = s['leg']
        fing = [-dy, dx, 0.0]
        st = ctx.value('r1_status')
        x, y, z = st['proprio']['grasp_points_pelvis_m']['right']
        if st['proprio']['gripper_width_m']['right'] < HELD_MIN:
            log(step='NOT_HELD'); sys.exit(3)
        widths = []
        for i, (da, dz, az) in enumerate(LIFT):
            x, y, z = x + da * dx, y + da * dy, z + dz
            r = guarded((x, y, z), (dx, dy, az), fing, w=0.0, secs=2.0, max_jump=0.4, servo=False)
            if r is None:
                sys.exit(2)
            widths.append(r['gripper_width_m']['right'])
            if widths[-1] < HELD_MIN:
                log(step='SLIP', at=i, widths=widths); sys.exit(3)
        save(lift_widths=widths)
        look('lifted')
    elif step == 'retreat':
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        guarded((0.28, -0.32, 0.25), (0.9, -0.3, 0.3), (0, 0, 1), secs=2.0, max_jump=3.0)
        ctx.value('r1_move_joints', joints=HOME, secs=2.0)
        r = ctx.value('r1_walk', forward_m=-0.2)
        log(step='retreat', walked=r.get('walked_m'), tilt=r.get('tilt_deg'))
        look('retreat')
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
