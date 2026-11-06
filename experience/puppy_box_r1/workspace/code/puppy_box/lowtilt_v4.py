"""r1-leg-lowtilt-lift-advance-carry (session 90b103, sim): pick a plush puppy by an overhanging leg and drop it into a
box ~.22 m behind the table edge, from the grasp stance (paw ~(.31-.33, -.19..-.22) in the pelvis frame).

Primitives (each `f(ctx, ...) -> dict`, chained by main):
  look(ctx)                    perception only: paw-ball sphere fit, leg axis (paw -> hip yaw), front edge, box front
                               wall (rim RANSAC) - from r1-leg-axis-grasp-short-carry/chain.py
  grasp(ctx, scene, depth_m)   pregrasp 8 cm short along the leg (servo on), advance to paw + depth_m along the leg,
                               close to 0; ok when the width is on the leg (>= .015)
  lift(ctx, scene, ...)        servo-OFF arc lift with the approach kept nearly LEVEL (z component <= az_max = .15),
                               +1.2..0 cm along the leg per 3-4 cm up, to commanded z <= z_cmd; then a level ADVANCE
                               of advance_m along the leg (2 x 3 cm). Aborts (puts the paw back low, opens, backs off,
                               HOME) if the width falls > 4 mm below its peak in the first 6 steps.
  carry_release(ctx, scene, hand, ...)  r1_carry_release(target = wall + in_m - (hand x + .06), lower_m, step_m)
  retreat(ctx, back_m)         v4.out: open, up, back, out beside the body (.14,-.40,.14), walk back, HOME both arms

    main(ctx, depth_m=0.03, in_m=0.10, lower_m=0.06, step_m=0.10, az_max=0.15, z_cmd=0.29, advance_m=0.06,
         back_m=0.2, do_retreat=True, lift_secs=2.0) -> {ok, measured, why, checkpoint}
v2: running width peak starts at lift step 0 (the close width is a transient); an abort retraces the lift path
downward and opens only at cmd z < .08 (s12: a direct put-back from the top was refused, it opened high and the
puppy fell off the table); lift_secs 2.0 (s11, s13 unattended successes).
v3: depth_m default .02 -> .03 (s16, s17: widths ROSE through the lift 2/2 and both placed; .02 lifted 5/9).

Why low tilt (90b103): r1_leg_lift tilts the approach up to .5 during its arc; with the hanging body's COM offset
sideways from the leg that tilt becomes a yaw torque on the leg inside the pads -> width +3-4 mm at lift step 2,
then the leg slides out (4/5 r1_leg_lift aborts). With the approach level the widths stayed flat (3/4 lifts).
Why the advance: the carry walk is the lossy stage (~3-10 mm of width per r1_walk); ending the lift 6 cm further
forward cut the walk from .22-.25 m (3 steps, drop) to ~.16 m (2 steps, held 2/2).
The success check is NOT automated (clear box: depth/images mislead): ask the operator (sim) or look into the box.
"""
import sys, math, importlib.util
import numpy as np
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
from pathlib import Path

SKILLS = Path(__file__).resolve().parents[2] / 'skill-library' / 'skills' if not (Path(__file__).resolve().parents[2] / 'r1-leg-axis-grasp-short-carry').exists() else Path(__file__).resolve().parents[2]
BASE = SKILLS / 'r1-leg-axis-grasp-short-carry' / 'code'
sys.path.insert(0, str(BASE))          # its chain.py imports scene, pawfit, edges, percep from its own folder
import chain                           # noqa: E402  (r1-leg-axis-grasp-short-carry/code/chain.py)
import v4                              # noqa: E402

BODY_OFF_X = 0.06       # hanging body centroid ahead of the hand (727e14 st2, 90b103 s7/s9: torso -5 cm of box centre)
HELD_MIN = 0.015
ABORT_FALL = 0.004
PRE_BACK = 0.08         # pregrasp distance short of the grasp point along the leg
# (advance along the leg m, rise m, fraction of az_max for the approach z component)
PROFILE = [(0.012, 0.03, 0.0), (0.012, 0.03, 0.0), (0.012, 0.04, 0.33), (0.01, 0.04, 0.53), (0.008, 0.04, 0.67),
           (0.005, 0.04, 0.8), (0.0, 0.04, 1.0), (0.0, 0.04, 1.0)]
ADV_STEP = 0.03
BALL_W = 0.045        # v4: close width above this = clearly on the paw ball -> one regrasp 1.5 cm deeper (j1 .040 lift-aborted, but
#                       deeper regrasps of horizontal grips WIDENED .037->.043 and pushed the paw (j11, R3); vertical .039/.044 -> .029)
REGRASP_STEP = 0.015
MAX_REGRASP = 1
EDGE_MIN = 0.22       # v4: min pelvis-to-edge distance at the end of the carry
CARRY_MIN_W = 0.031   # v4: min width to start the carry
CARRY_FALL = 0.003    # v4: max fall from the lift peak to start the carry
PULL_BACK = 0.0       # v4: 0.025 dragged the puppy OFF the table in j12 (R1: rotated it) - disabled
MIN_PAW_N = 300      # v4: was 800 (chain.good)
CLOSE_EDGE = 0.375   # v4: edge closer than this + no paw -> waist look
WAIST_LOOK = -0.35


def _good_v4(p):
    # v4 (j3): a paw at x <= .30 is cut by the image bottom at head pitch .6 (n 355, rms .0002) - the fixed-radius
    # sphere fit still agrees with the operator's paw within ~1 cm; fragments at the border have n < 150
    return p['n'] >= MIN_PAW_N and p['rms'] <= 0.001


def rigid2d(A, B):
    A = np.asarray(A)[:, :2]; B = np.asarray(B)[:, :2]
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb); U, S, Vt = np.linalg.svd(H); R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1; R = Vt.T @ U.T
    return R, cb - R @ ca


def _shared(oA, oB, maxd=0.08, exclude=None):
    """greedy one-to-one matching of good paws A -> B by distance (v5, R2: a paw hidden in B was matched to the
    neighbour's B paw a second time -> residual > .01 -> look refused)."""
    A = [p for p in oA['paws'] if chain.good(p) and (exclude is None or math.dist(p['c'][:2], exclude[:2]) >= 0.005)]
    B = [q for q in oB['paws'] if chain.good(q)]
    pairs = sorted(((math.dist(p['c'][:2], q['c'][:2]), i, j) for i, p in enumerate(A) for j, q in enumerate(B)))
    ua, ub, pa, pb = set(), set(), [], []
    for d, i, j in pairs:
        if d >= maxd or i in ua or j in ub:
            continue
        ua.add(i); ub.add(j); pa.append(A[i]); pb.append(B[j])
    return pa, pb


def _select_via_B(oA, oB, log=print):
    """v5: map the yawed view's paws into the waist-0 view (mean translation of the shared paws), compute each paw's
    overhang from the waist-0 edge, then apply chain.select's rules (overhang > .045, lateral gap >= .09, rightmost)."""
    e = oB.get('edge')
    if not e:
        return None
    pa, pb = _shared(oA, oB)
    if not pa:
        return None
    t = np.mean([np.asarray(q['c'][:2]) - np.asarray(p['c'][:2]) for p, q in zip(pa, pb)], axis=0)
    th = math.radians(e['yaw'])
    cand = []
    for p in sorted(oA['paws'], key=lambda q: -q['n']):
        if not chain.good(p):
            continue
        if any(math.dist(p['c'][:2], c[0]['c'][:2]) < 0.02 for c in cand):     # R9: one paw split into two blobs
            continue
        cb = np.asarray(p['c'][:2]) + t
        ovh = e['dist'] - (math.cos(th) * cb[0] + math.sin(th) * cb[1])
        cand.append((p, cb, ovh))
    ok = [c for c in cand if c[2] > 0.045 and min([abs(o[1][1] - c[1][1]) for o in cand if o is not c] or [1]) >= 0.09]
    log({'step': 'select_via_B', 't': [round(float(v), 4) for v in t], 'cand': [[round(float(v), 3) for v in c[1]] + [round(c[2], 3)] for c in cand]})
    if not ok:
        return None
    p, cb, ovh = min(ok, key=lambda c: c[1][1])
    return dict(p, overhang=round(ovh, 3))


def look_waist(ctx, wy=-0.35, log=print):
    chain.ctx = ctx; chain.good = _good_v4
    ctx.value('r1_move_joints', joints={'waist_yaw_joint': wy}, secs=1.5)
    oA = chain.perceive('waistA')
    TA, _ = chain.select(oA)
    if TA is None:
        # v5 (3d54de R2): the yawed-view edge fit read yaw -17.5 (true ~0) -> every overhang wrong, no paw selected.
        # Select in the waist-0 frame instead: map A's paws by the shared paws, overhang from B's edge.
        ctx.value('r1_move_joints', joints={'waist_yaw_joint': 0.0}, secs=1.5)
        oB = chain.perceive('waistB')
        TA = _select_via_B(oA, oB, log=log)
        if TA is None:
            return {'ok': False, 'why': 'no clean paw with the waist yawed', 'measured': {'A': oA['paws'], 'B': oB['paws']}}
        ctx.value('r1_move_joints', joints={'waist_yaw_joint': wy}, secs=1.5)
        oA2 = chain.perceive('waistA2')
        TA2 = min([p for p in oA2['paws'] if chain.good(p)] or [None], key=lambda p: 9 if p is None else math.dist(p['c'][:2], TA['c'][:2]))
        if TA2 is not None and math.dist(TA2['c'][:2], TA['c'][:2]) < 0.03:
            oA, TA = oA2, dict(TA2, overhang=TA.get('overhang'))
        ax = chain.leg_axis(oA['frame'], TA['c'])
        ctx.value('r1_move_joints', joints={'waist_yaw_joint': 0.0}, secs=1.5)
        oB = chain.perceive('waistB2')
    else:
        ax = chain.leg_axis(oA['frame'], TA['c'])
        ctx.value('r1_move_joints', joints={'waist_yaw_joint': 0.0}, secs=1.5)
        oB = chain.perceive('waistB')
    sa, sb = _shared(oA, oB, exclude=TA['c'])
    pa = [p['c'] for p in sa]; pb = [q['c'] for q in sb]
    if not pa:
        return {'ok': False, 'why': 'no reference paw seen in both', 'measured': {'A': oA['paws'], 'B': oB['paws']}}
    R = np.eye(2)
    if len(pa) >= 2:
        R2, _ = rigid2d(pa, pb)
        if abs(math.degrees(math.atan2(R2[1, 0], R2[0, 0]))) < 4.0:
            R = R2
    # translation from the shared paw NEAREST the target (j6v: an edge fit in the yawed view read -9.9 deg and the
    # edge-based rotation threw the paw 4 cm off; the pelvis counter-rotation is ~2 deg -> <= 5 mm over 12 cm)
    k = min(range(len(pa)), key=lambda i: math.dist(pa[i][:2], TA['c'][:2]))
    t = np.asarray(pb[k][:2]) - R @ np.asarray(pa[k][:2])
    res = float(np.max(np.linalg.norm((np.asarray(pa)[:, :2] @ R.T + t) - np.asarray(pb)[:, :2], axis=1)))
    rot = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    cB = list(R @ np.asarray(TA['c'][:2]) + t) + [TA['c'][2] - (oA['table_z'] - oB['table_z']) if oA['table_z'] and oB['table_z'] else TA['c'][2]]
    out = {'paw_A': TA['c'], 'paw': [round(float(v), 4) for v in cB], 'rot_deg': round(rot, 2), 't': [round(float(v), 4) for v in t],
           'ref_res': round(res, 4), 'n_ref': len(pa), 'axis_A': ax, 'leg_yaw': None if ax is None else round(ax['yaw'] + rot, 1),
           'frameA': oA['frame'], 'frameB': oB['frame'], 'overhang': TA.get('overhang'), 'edgeA': oA['edge'], 'edgeB': oB['edge'], 'table_z': oB['table_z']}
    log(out)
    return {'ok': ax is not None and res < 0.01, 'measured': out, 'why': ''}



def look(ctx, log=print):
    chain.ctx = ctx
    chain.good = _good_v4
    o = chain.perceive('lowtilt')
    T, R = chain.select(o)
    if T is None and o['edge'] is not None and o['edge']['dist'] < CLOSE_EDGE:
        # v4 (j5, j6): the paw is below the view at a close stance (paw x <= .29): look with the waist yawed and map
        # the paw into the waist-0 frame via the paw/edge seen in both configurations (look_waist.py)
        r = look_waist(ctx, WAIST_LOOK, log=log)
        if not r['ok']:
            return {'ok': False, 'why': 'paw hidden; waist look failed: ' + r['why'], 'measured': {'frame': o['frame'], **r['measured']}}
        wm = r['measured']
        e0 = o['edge']
        o = chain.perceive('lowtilt_b')
        # v5 (3d54de R4): after the waist look the edge fit read .499 (true .355: the near edge left the view) -> the
        # wall (.719) and the carry's edge bound were wrong. Keep the first waist-0 edge unless they agree within 3 cm.
        if e0 is not None and (o['edge'] is None or abs(o['edge']['dist'] - e0['dist']) > 0.03):
            o = dict(o, edge=e0)
        T = {'c': wm['paw'], 'overhang': wm['overhang']}
        ax = {'yaw': wm['leg_yaw'], 'res': wm['axis_A']['res']}
    elif T is None:
        return {'ok': False, 'why': 'no clean overhanging paw', 'measured': {'frame': o['frame'], 'paws': o['paws']}}
    else:
        ax = chain.leg_axis(o['frame'], T['c'])
    if ax is None or ax['res'] > 0.004 or not (-10 < ax['yaw'] < 45):
        return {'ok': False, 'why': 'leg axis not measured reliably', 'measured': {'frame': o['frame'], 'axis': ax}}
    e = o['edge']
    bp = chain.box_pose(o['frame'], o['table_z'])
    wall = None
    if bp is not None and e is not None and 0.15 < bp['x0'] - e['dist'] < 0.30:
        # front rim line x = x0 - tan(yaw) (y - ym): evaluate it on the carry line (the paw's y) - matters when the
        # robot stands rotated to the table (walk-in), negligible when square
        wall = bp['x0'] - math.tan(math.radians(bp['yaw'])) * (T['c'][1] - bp['ym'])
    if wall is None and e is not None:
        wall = e['dist'] + 0.22
    if wall is None:
        return {'ok': False, 'why': 'no table edge / box wall', 'measured': {'frame': o['frame']}}
    sc = {'frame': o['frame'], 'paw': [float(v) for v in T['c']], 'leg_yaw': ax['yaw'], 'edge': e, 'wall': float(wall),
          'overhang': T.get('overhang'), 'table_z': o['table_z']}
    log({'step': 'look', **sc})
    return {'ok': True, 'measured': sc, 'why': ''}


def _back_off(G, ap, f, ld):
    chain.ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
    chain.guarded([G[0] - PRE_BACK * ld[0], G[1] - PRE_BACK * ld[1], G[2] + 0.02], ap, f, w=0.07, secs=1.5, max_jump=0.9)
    chain.guarded([0.22, -0.26, 0.10], (1, 0, 0.2), (0, 1, 0), w=0.07, secs=1.5, max_jump=1.5)
    chain.ctx.value('r1_move_joints', joints=v4.HOME_BOTH, secs=1.5)


def _put_back(path, f, ld=None):
    """move the held paw back down through the commanded lift path (reverse order, servo off); True when the
    command is back at the grasp height (open is then safe), False if a step was refused (still holding).
    v4: then drag the paw PULL_BACK toward the robot along the leg before opening - the leg slides ~2-3 cm into the
    pads before an abort, so a plain put-back left the paw only ~3 cm over the edge (j1b: no retry possible)."""
    for pos, api in reversed(path[:-1]):
        if chain.guarded(pos, api, f, w=0.0, secs=1.2, max_jump=0.6, servo=False) is None:
            return pos[2] < 0.08
    if ld is not None and PULL_BACK > 0:
        pos, api = path[0]
        chain.guarded((pos[0] - PULL_BACK * ld[0], pos[1] - PULL_BACK * ld[1], pos[2]), api, f, w=0.0, secs=1.2,
                      max_jump=0.4, servo=False)
    return True


def grasp(ctx, sc, depth_m=0.02, log=print, vertical=False, ball_w=None):
    ball_w = ball_w or BALL_W
    chain.ctx = ctx
    yaw = math.radians(sc['leg_yaw']); ld = (math.cos(yaw), math.sin(yaw))
    # v4: vertical straddle (fingers (0,0,1): one pad UNDER the shaft carries the hanging weight by contact, not
    # friction) or the horizontal cross grip (pads left/right of the leg, v3)
    f = [0.0, 0.0, 1.0] if vertical else [-ld[1], ld[0], 0.0]
    P = sc['paw']
    G = [P[0] + depth_m * ld[0], P[1] + depth_m * ld[1], P[2]]
    pre = [G[0] - PRE_BACK * ld[0], G[1] - PRE_BACK * ld[1], G[2]]
    ap = chain.choose_approach(pre, sc['leg_yaw'], f)
    if chain.guarded(pre, ap, f, w=0.07, secs=2.5, max_jump=2.0) is None:        # empty hand from HOME (jump ~1.05)
        return {'ok': False, 'why': 'pregrasp refused', 'measured': {'pre': pre}}
    ap = chain.choose_approach(G, sc['leg_yaw'], f)
    if chain.guarded(G, ap, f, w=0.07, secs=1.5) is None:
        return {'ok': False, 'why': 'grasp pose refused', 'measured': {'G': G}}
    tries = []
    for k in range(MAX_REGRASP + 1):
        g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
        w = g['gripper']['measured_m']; tries.append(round(w, 4))
        m = {'G': G, 'ap': list(ap), 'f': f, 'ld': list(ld), 'close_width': w, 'hand': g['grasp_points_pelvis_m']['right'],
             'tries': tries}
        log({'step': 'closed', 'try': k, **m})
        if w < HELD_MIN:
            _back_off(G, ap, f, ld)
            return {'ok': False, 'why': 'missed the leg (width %.4f)' % w, 'measured': m}
        if w <= ball_w or k == MAX_REGRASP:
            break
        # v4: width says the pads are on the paw BALL (j1: .040 -> lift abort at step 2): open, slide deeper up the
        # shaft by REGRASP_STEP along the leg (servo on, no load), close again
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
        G = [G[0] + REGRASP_STEP * ld[0], G[1] + REGRASP_STEP * ld[1], G[2]]
        if chain.guarded(G, ap, f, w=0.07, secs=1.2) is None:
            _back_off(G, ap, f, ld)
            return {'ok': False, 'why': 'deeper regrasp refused', 'measured': m}
    return {'ok': True, 'measured': m, 'why': ''}


def lift(ctx, gm, az_max=0.15, z_cmd=0.29, advance_m=0.06, lift_secs=1.2, log=print, arc=1.0):
    chain.ctx = ctx
    G, ap, f, ld = gm['G'], gm['ap'], gm['f'], gm['ld']
    hz = math.hypot(ap[0], ap[1]); apx, apy = ap[0] / hz, ap[1] / hz
    x, y, z = G; last = None; path = [(tuple(G), (apx, apy, 0.0))]
    widths = [gm['close_width']]; peak = 0.0     # the close width is often a transient peak (R1: .0378 -> .0343
    #                                               at step 0, 2fbfb7 st20): the running peak starts at lift step 0
    for i, (da, dz, k) in enumerate(PROFILE):
        if z + dz > z_cmd + 1e-6:
            break
        da = da * arc                     # v4 experiment: arc > 1 moves the hand toward the body faster while it rises
        x, y, z = x + da * ld[0], y + da * ld[1], z + dz
        api = (apx, apy, az_max * k)
        r = chain.guarded((x, y, z), api, f, w=0.0, secs=lift_secs, max_jump=0.45, servo=False)
        if r is None:
            x, y, z = last if last else G
            if z < z_cmd - 0.08:          # refused while the body is still low: never carry it into the box wall
                if not _put_back(path, f):
                    return {'ok': False, 'why': 'lift refused low and the put-back was refused: HOLDING, ask for help',
                            'measured': {'widths': widths, 'holding': True}}
                _back_off(G, ap, f, ld)
                return {'ok': False, 'why': 'lift step refused at cmd z %.3f (IK)' % z, 'measured': {'widths': widths}}
            break
        last = (x, y, z); path.append(((x, y, z), api))
        wi = r['gripper_width_m']['right']; widths.append(wi); peak = max(peak, wi)
        log({'step': 'lift', 'i': i, 'cmd': [round(v, 3) for v in (x, y, z)], 'width': wi})
        if wi < HELD_MIN:
            return {'ok': False, 'why': 'slipped in the lift (step %d)' % i, 'measured': {'widths': widths}}
        if i <= 5 and wi < peak - ABORT_FALL:
            low = _put_back(path, f, ld)      # retrace the lift downwards (s12: a direct put-back from the top was
            if not low:                       # refused, it opened high and the puppy fell off the table)
                return {'ok': False, 'why': 'lift aborted high and the put-back was refused: HOLDING, ask for help',
                        'measured': {'widths': widths, 'aborted_at': i, 'holding': True}}
            _back_off(G, ap, f, ld)
            return {'ok': False, 'why': 'lift aborted: width falling (%.4f < peak %.4f - .004)' % (wi, peak),
                    'measured': {'widths': widths, 'aborted_at': i}}
    for j in range(int(round(advance_m / ADV_STEP))):
        nx, ny = x + ADV_STEP * ld[0], y + ADV_STEP * ld[1]
        r = chain.guarded((nx, ny, z), (apx, apy, az_max), f, w=0.0, secs=1.2, max_jump=0.45, servo=False)
        if r is None:
            break
        x, y = nx, ny
        path.append(((x, y, z), (apx, apy, az_max)))
        wi = r['gripper_width_m']['right']; widths.append(wi)
        log({'step': 'advance', 'j': j, 'width': wi})
        if wi < HELD_MIN:
            return {'ok': False, 'why': 'slipped in the advance', 'measured': {'widths': widths}}
    st = ctx.value('r1_status')['proprio']
    hand = st['grasp_points_pelvis_m']['right']; w = st['gripper_width_m']['right']
    if w < HELD_MIN:
        return {'ok': False, 'why': 'slipped after the lift', 'measured': {'widths': widths}}
    pk = max(widths[1:]) if len(widths) > 1 else w
    if w < CARRY_MIN_W or w < pk - CARRY_FALL:
        # v4 (j10b): a lift that ends with a thin or falling width drops in the first carry walk (puppy in front of
        # the box); carries held 8/8 entering at >= .0337. Put it back (retrace incl. the advance) for a retry.
        low = _put_back(path, f, ld)
        if not low:
            return {'ok': False, 'why': 'weak grip at the top and the put-back was refused: HOLDING, ask for help',
                    'measured': {'widths': widths, 'holding': True}}
        _back_off(G, ap, f, ld)
        return {'ok': False, 'why': 'lift aborted: weak grip for the carry (%.4f, peak %.4f)' % (w, pk),
                'measured': {'widths': widths, 'aborted_at': 'top'}}
    return {'ok': True, 'measured': {'widths': widths, 'hand': hand, 'width': w, 'cmd': [x, y, z]}, 'why': ''}


def carry_release(ctx, sc, hand, in_m=0.10, lower_m=0.06, step_m=0.10, max_steps=5, log=print):
    tgt = round(min(0.40, max(0.05, sc['wall'] + in_m - (hand[0] + BODY_OFF_X))), 3)
    # v4 (j15a): never carry the pelvis closer than EDGE_MIN to the front edge (a .203 m carry from an edge .39 ahead
    # left the torso against the table and the retreat stuck); equal steps <= step_m (a 2nd .10 step after .106 of a
    # .126 target overshot)
    if sc.get('edge') and sc['edge'].get('dist'):
        tgt = round(max(0.05, min(tgt, sc['edge']['dist'] - EDGE_MIN)), 3)
    n = max(1, int(math.ceil(tgt / step_m - 1e-6)))
    step_m = round(min(0.10, max(0.04, tgt / n)), 3); max_steps = n
    c = ctx.value('r1_carry_release', target_forward_m=tgt, lower_m=lower_m, max_steps=max_steps, step_m=step_m)
    o = ctx.value('r1_observe')
    m = {'target_forward_m': tgt, 'reply': c, 'after_frame': o['camera']['frame_id']}
    log({'step': 'carry', **m})
    if c.get('dropped'):
        return {'ok': False, 'why': 'dropped during the carry at %.3f m' % (c.get('walked_total_m') or 0), 'measured': m}
    return {'ok': bool(c.get('released')), 'measured': m, 'why': '' if c.get('released') else 'not released'}


def free_arm(ctx, log=print):
    """v4 (j9v): after the leg slipped out with the hand up, open and route the empty hand back (x .18 at its height),
    out beside the body (.12,-.40,.10) and HOME - never straight down onto the dropped puppy."""
    chain.ctx = ctx
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
    h = ctx.value('r1_status')['proprio']['grasp_points_pelvis_m']['right']
    for pos in ([0.18, -0.32, max(h[2], 0.12)], [0.12, -0.40, 0.10]):
        try:
            ctx.value('r1_move_hand', arm='right', position=pos, gripper_width_m=0.07, secs=2.0)
        except RuntimeError as ex:
            log({'step': 'free_arm_refused', 'pos': pos, 'why': str(ex)[:120]})
    ctx.value('r1_move_joints', joints=v4.HOME_BOTH, secs=2.0)


def retreat(ctx, back_m=0.2):
    chain.ctx = ctx
    v4.out([back_m])
    return {'ok': True, 'measured': {}, 'why': ''}


def main(ctx, depth_m=0.03, in_m=0.10, lower_m=0.06, step_m=0.10, az_max=0.15, z_cmd=0.29, advance_m=0.06,
         back_m=0.2, do_retreat=True, lift_secs=2.0, vertical=0.0, ball_w=0.0, arc=1.0, log=print):
    chain.ctx = ctx
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.5)        # stale commands after operator placements
    ctx.value('r1_move_joints', joints=v4.HOME_BOTH, secs=1.0)
    ctx.value('r1_head', pitch=0.6, secs=1.0)
    meas = {}
    r = look(ctx, log=log); meas['look'] = r['measured']
    if not r['ok']:
        return {'ok': False, 'measured': meas, 'why': 'look: ' + r['why']}
    sc = r['measured']
    r = grasp(ctx, sc, depth_m, log=log, vertical=bool(vertical), ball_w=ball_w or None); meas['grasp'] = r['measured']
    if not r['ok']:
        return {'ok': False, 'measured': meas, 'why': 'grasp: ' + r['why']}
    r = lift(ctx, r['measured'], az_max, z_cmd, advance_m, lift_secs, log=log, arc=arc); meas['lift'] = r['measured']
    if not r['ok']:
        if 'slipped' in r['why']:
            free_arm(ctx, log=log)
        return {'ok': False, 'measured': meas, 'why': 'lift: ' + r['why']}
    r = carry_release(ctx, sc, r['measured']['hand'], in_m, lower_m, step_m, log=log); meas['carry'] = r['measured']
    ck = {'subagent': 'sonnet', 'frames': ['runtime/frames/%s/cam_head.jpg' % r['measured']['after_frame']],
          'question': 'Is the cream plush puppy lying INSIDE the clear box (body below the rim, not across the rim, '
                      'not in front of the front wall)? The box is transparent: a puppy in front of it looks inside.',
          'answer': 'inside | across the rim | in front of the box | elsewhere',
          'pass_if': 'inside, and confirmed by the operator ground truth in sim',
          'on_fail': 'retreat; across the rim: do NOT push with the arm (made it worse 2/2); report'}
    ok = r['ok']; why = '' if ok else 'carry: ' + r['why']
    if do_retreat:
        try:
            retreat(ctx, back_m)
        except Exception as ex:                 # v4 (j15a): a crash here left the robot against the table
            ctx.value('r1_hold')
            return {'ok': False, 'measured': meas, 'why': 'retreat failed (HOLDING, ask for help): %s' % str(ex)[:200],
                    'checkpoint': ck}
    return {'ok': ok, 'measured': meas, 'why': why or 'released over the box; verify inside', 'checkpoint': ck}
