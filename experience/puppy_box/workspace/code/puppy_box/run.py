"""puppy_box: ONE program for every layout (right hand only, left arm tucked). Session 3a46e1 (2026-11-02, sim).

  PYTHONPATH=. python code/puppy_box/run.py scene            # tuck, measure, classify the layout, plan waist/grasp/release (no reach over the table)
  PYTHONPATH=. python code/puppy_box/run.py approach         # raise the right hand by the back path, set the waist, hover over mid-body -> LOOK
  PYTHONPATH=. python code/puppy_box/run.py grasp            # light press to the stall, close, closed 3 cm raise, grip check -> LOOK
  PYTHONPATH=. python code/puppy_box/run.py lift,carry       # 3 cm / 4 s lift, arm-only carry over the box, hanging-body check -> LOOK
  PYTHONPATH=. python code/puppy_box/run.py release,home,verify
  PYTHONPATH=. python code/puppy_box/run.py abort            # open the hand, straight up, home (from anywhere after approach)
Trial (evaluation): run.py scene -> LOOK at the overlay -> run.py approach -> LOOK at the hover (optional shift DX DY) -> run.py pick -> LOOK at the lift frame -> run.py carry -> LOOK at the carry overlay -> run.py place
  pick = [approach if no hover yet],grasp,lift with one built-in retry (abort + fresh scene) after an empty close or a slip;
  place = release (open, check the plush left the hand), home (arms tucked, hands open), verify (cream inside the box).

Steps can be chained with commas; the chain stops at the first failed check (exit 1) and prints why.
Layouts (box relative to the puppy): 'right' (same side as the hand), 'left' (cross-body), 'behind'.
All IK on the COMMANDED row (SONIC compensates an under-tracked waist with the arm: FK(cmd) ~ measured palm).
The waist is set once before the arm goes over the puppy and kept until after the release.
Numbers come from memory/PARAMS.json (api.params.param, names 'pb.*'). Pelvis frame, metres.
State: output/run_state.json. Frames: runtime/frames/<frame_id>/, overlays in output/run_*.jpg."""
import json, os, sys, time
import numpy as np
import cv2

from pathlib import Path
HERE = os.path.dirname(os.path.abspath(__file__))
# workspace root (has api/ and runtime/) whether this file is code/puppy_box/run.py or the library copy
ROOT = str(next((q for q in Path(HERE).resolve().parents if (q / 'api').is_dir() and (q / 'runtime').exists()), Path.cwd()))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'skill-library', 'skills', 'plush-pick-place-box-same-side', 'code'))
import controller3 as c3          # noqa: E402  (commanded-row move / waypoint / best, observe, fingers)
from api.params import param      # noqa: E402

c = c3.c
p = c3.p
STATE = os.path.join(ROOT, 'output', 'run_state.json')

TUCK = {'shoulder_pitch': 0.45, 'elbow': -0.5}
EDGE_MIN_IN = 0.06                                        # mid-body centre at least this far inside the table edge (legs may overhang)
EYE_MIN_H, EYE_TO_MID = 0.055, 0.20                    # eye height above the table; eye -> mid-body along the axis
SIDE_LOOK_YAW = 0.7                                       # waist yaw cmd for the second view of a clipped puppy
LOOK = {'shoulder_pitch': 0.9, 'elbow': 0.6}                  # arms down behind the edge, out of view. el 0 put the right fingertips
# at x .20 / 6 cm below the table top once the side look turned the waist (.28 measured) and swept an edge puppy to the floor
# (d65273 run 2); el .6: every hand link x <= .05, z <= -.13 for waist yaw -0.7..0.7 (FK)
LEFT_GUARD = {'shoulder_pitch': 0.9, 'elbow': -0.9}            # left arm back and up during approach..release
HOME_BLOCKED_DEV = 0.4                                     # rad: an arm this far off its LOOK command after home is resting on something
GRASP_BACK = param('pb.palm_back_from_far_m', 0.10)      # palm x = body far side - this (fingertips past the far side)
PRESS_LEVELS = tuple(param('pb.press_levels_rel_top_m', [0.02, 0.0, -0.015, -0.03, -0.045]))   # palm cmd z = top + d, stop at the first stall
EXTRA_PRESS = 1                                           # levels pressed past the first stall (run 6 box behind: 2 slips without)
STALL_DZ = 0.03                                           # palm > cmd + this = held up by the plush
RIM_H = param('pb.box_rim_above_table_m', 0.14)
LIFT_ABOVE_RIM = param('pb.lift_palm_above_rim_m', 0.19)  # palm z at the end of the lift
CARRY_ABOVE_RIM = param('pb.carry_palm_above_rim_m', 0.21)
LIFT_STEP = param('lift.step_dz_m', 0.03)
LIFT_DRIFT, LIFT_DRIFT_MAX = 0.01, 0.0                    # sideways drift toward the box during the lift: OFF (2 of 3 slips came at its onset, runs 5, 7)
LIFT_IN_STEP = 0.03                                       # lift: palm step toward the body centre when straight up is IK-refused
LIFT_DRIFT_AFTER = 0.12                                   # only once the plush is clear of the table (run 5: drift from 5 cm -> slip at z .10)
REL_IN = param('pb.release_inside_wall_m', 0.09)          # minimum palm distance inside the near/side walls at release
GRIP_LOST = param('grasp.grip_lost_rad', 1.45)
TILT_MAX = 7.0
PRESS_TILT = 5.5                                          # keep pressing while the palm is above the plush top and tilt < this
HANG_PX = 1500                                            # cream pixels near the raised open hand = plush still on it
FINGER_WALL_MARGIN = param('grasp.finger_wall_margin_m', 0.007)   # finger links stay this far short of a box wall behind
VERIFY_MIN_IN_PX = 1500                                   # cream pixels inside the box footprint after home
SET_DOWN_ABOVE_RIM = 0.13                                 # palm z above the rim where a hanging plush rests on the box floor
RETREAT_Z, RETREAT_X = param('pb.home_retreat_z_m', 0.40), 0.24                       # home: palm up to this z, then back to this x, at the current waist
DUMP_ABOVE_RIM = 0.19                                     # palm z above the rim for the wrist-roll dump (trial 8 recovery .29 pelvis)
HEAD_WALL_MEAS = param('pb.head_wall_clearance_measured_m', 0.18)   # measured palm from the head-side wall before release
HEAD_WALL_IN = 0.19                                       # release palm at least this far from the wall the head hangs toward
NEAR_WALL_SIDE = param('pb.release_near_wall_side_m', 0.13)                                    # box beside the plush: measured palm at least this far past the near wall
HANG_IN_MIN, CENTRE_STEP = 0.6, 0.025                     # carry: hanging body fraction over the interior; nudge step toward the centre
PALM_TOL, OFF_MAX = 0.015, 0.06                          # measured-palm aim tolerance / max command offset at hover and press
ARM_SYNC_DEV = 0.5                                        # rad: measured arm this far from its command = other IK branch -> sync
SYNC_PALM_ERR = param('pb.sync_palm_err_m', 0.04)                                     # ... but only when the measured palm is this far from FK(command)
TAIL_SHIFT = param('pb.grasp_shift_toward_tail_m', 0.02)   # run 1: hover over the chest, shifted 2 cm -> mid-body hold
LATERAL_Y = -0.25                                         # grasp this far right with the box on the left = lateral reach
LATERAL_GYAW = param('pb.lateral_grasp_yaw_deg', -45.0)
BOX_LONG, BOX_SHORT = 0.39, 0.28                          # task: outer size 39 x 28 cm
WAISTS = [(0.0, 0.0), (0.3, 0.0), (0.45, 0.0), (0.45, 0.15), (0.6, 0.15), (0.7, 0.15)]   # no 0.9: d65273 run 3 the measured
# yaw oscillated .27 <-> .46 under cmd .9 (palm swinging 15 cm) and the box left the table during the press; da90 tilt 7+
# box LEFT (lateral grasp, mid y ~-.28): f20de8 run 1 at .45/.15 the pressed grasp was IK-reachable with no margin; SONIC held the
# arm off its command (wrist roll -.8 vs -1.2) -> weak close, lost on the lift. Waist ROLL lowers the right shoulder: .25-.35 reaches
# 3-6 cm deeper (offline IK). Left layouts take the first of these whose grasp reaches GRASP_MARGIN below the press target.
WAISTS_LEFT = [tuple(w) for w in param('pb.lateral_grasp_waists', [[0.3, 0.25], [0.45, 0.25], [0.3, 0.35], [0.45, 0.35]])] + WAISTS
GRASP_MARGIN = param('pb.lateral_grasp_ik_margin_m', 0.03)
HANG_DEFAULT = param('pb.hang_below_palm_default_m', 0.22)   # held plush lowest point below the palm when not measured
REL_Z_TOL = param('pb.release_z_tol_m', 0.04)                                         # release: measured palm at most this far above the planned release z
HANG_CLEAR = 0.05                                         # carry: hanging plush this far above the rim
CARRY_MAX_ABOVE_RIM = 0.30
SWING_STEPS = 3                                            # waist turn grasp -> release waist with the plush held, ramps of 4 s


class Fail(Exception):
    pass


def load():
    return json.load(open(STATE)) if os.path.exists(STATE) else {}


def save(st):
    json.dump(st, open(STATE, 'w'), indent=1, default=float)


def out(ok=True, why='', **m):
    print(json.dumps(dict(ok=ok, why=why, **m), default=float), flush=True)
    if not ok:
        raise Fail(why)


def R(d):
    return c.mirror('right', d)


def L(d):
    return c.mirror('left', d)


def held(f):
    """Hold = middle base short of the fist (index alone short = blocked by the thumb, ad5d0f; middle alone holds, 996d97)."""
    return f[3] < GRIP_LOST          # middle base short of the fist (empty fist ~1.53; held plush 0.6-1.39 seen)


def left_tucked(ob):
    j = ob['proprio']['joint_positions_rad']
    lp = c.pps.link_points(j, 'left')
    ok = j[11] > 0.3 and j[21] < -0.3 and lp['left_palm'][0] < 0.22 and lp['left_hand_index_1_link'][0] < 0.30
    return ok, {'l_sp': round(j[11], 2), 'l_el': round(j[21], 2), 'l_palm': lp['left_palm']}


HAND_CLEAR_MIN = 0.06                                     # m: plush this close to a hand before the look move = stop
LEFT_CLEAR_MIN = 0.03                                     # m: plush points this close to a left hand link = possible touch


def left_clearance(ob, path):
    """Min distance from plush points (sat 60-125: the white hands' warm edges read ~42, d65273) above the table to the
    left hand links (FK on the measured joints)."""
    P = p.load(path); pel, lev = p.cloud(P)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 60) & (hsv[..., 1] < 125) & (hsv[..., 2] > 120) & (P['depth'] > 0)
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool) & (lev[..., 2] > p.table_z(P) + 0.02)
    q = pel[m]
    if len(q) < 50:
        return None
    pts = np.array(list(c.pps.link_points(ob['proprio']['joint_positions_rad'], 'left').values()))
    return round(float(np.min(np.linalg.norm(q[None, ::2, :] - pts[:, None, :], axis=2))), 3)


ARM = [f'right_{a}_joint' for a in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]


def sync_arm(ctx, secs=3.0):
    """Set the right ARM command (shoulder/elbow/wrist only, never the legs) to the measured joints."""
    row, meas, _ = c.pps.last_row(ctx)
    c.pps.ramp(ctx, {k: meas[c.J.index(k)] for k in ARM}, secs)


def mv(ctx, pos, yaw, secs, max_jump=1.0):
    """c3.move (commanded-row IK) + tracking check: if SONIC holds the arm on a different branch than the command
    (run 4: cmd shoulder pitch .88 / roll -2.07, measured -1.36 / -1.58, palm 4-5 cm off; the next IK step then jumped
    1.62 rad), sync the arm command to the measured joints and solve this target again from there."""
    r = c3.move(ctx, pos, yaw, secs, max_jump=max_jump)
    if not r['ok']:
        return r
    j = r['ob']['proprio']['joint_positions_rad']; row, _, _ = c.pps.last_row(ctx)
    dev = max(abs(j[c.J.index(k)] - row[9 + c.J.index(k)]) for k in ARM)
    r['arm_dev'] = round(dev, 2)
    # at a turned waist SONIC compensates the under-tracked waist with the arm: the joints differ from the command by design
    # while the palm is where FK(command) says (f20de8 run 3: dev .5-.94 with the palm within 1-3 cm -> a sync + re-move on
    # every carry step jerked the plush). Sync only when the PALM is off too.
    pe = float(np.linalg.norm(np.array(c.pps.link_points(row[9:38], 'right')['right_palm']) - np.array(r['palm'])))
    r['palm_err'] = round(pe, 3)
    if dev > ARM_SYNC_DEV and pe > SYNC_PALM_ERR:
        sync_arm(ctx)
        r2 = c3.move(ctx, pos, yaw, max(2.0, secs * 0.6), max_jump=max_jump)
        if r2['ok']:
            r = r2
        r['synced'] = round(dev, 2)
    return r


# ---------------------------------------------------------------- perception (no motion)

def view_puppy(P, tz, box):
    """One view: cream body points (largest blob above the table, outside the box) and dark blobs (paws, nose, eyes)
    in the level frame; border = which image sides the body blob touches (clipped puppy)."""
    pel, lev = p.cloud(P); h = lev[..., 2] - tz; ok = P['depth'] > 0
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 40) & (hsv[..., 1] < 140) & (hsv[..., 2] > 120)
    m = cream & (h > 0.015) & (h < 0.16) & ok & (lev[..., 0] < 0.75)
    if box:
        m &= ~((lev[..., 0] > box['x'][0] - 0.01) & (lev[..., 0] < box['x'][1] + 0.01) &
               (lev[..., 1] > box['y'][0] - 0.01) & (lev[..., 1] < box['y'][1] + 0.01))
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stt, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None, [], []
    k = 1 + int(np.argmax(stt[1:, 4]))
    H, W = m.shape; bx, by, bw, bh = stt[k, :4]
    border = [n_ for n_, v_ in (('left', bx <= 2), ('right', bx + bw >= W - 3)) if v_]
    dark = (hsv[..., 2] < 110) & (hsv[..., 1] > 60) & (hsv[..., 0] <= 30) & (h > 0.003) & (h < 0.12) & ok
    dark = cv2.morphologyEx(dark.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n2, lab2, st2, _ = cv2.connectedComponentsWithStats(dark)
    raw = [(int(st2[q, 4]), lev[lab2 == q][:, :2].mean(0)) for q in range(1, n2) if st2[q, 4] >= 40]
    # eye: small near-black blob ~6-8 cm above the table (paws and their shadows sit lower)
    blk = (hsv[..., 2] < 50) & (h > 0.01) & (h < 0.15) & ok & (lev[..., 0] < 0.75)
    blk = cv2.morphologyEx(blk.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n3, lab3, st3, _ = cv2.connectedComponentsWithStats(blk)
    for q in range(1, n3):
        if 30 <= st3[q, 4] <= 400 and h[lab3 == q].mean() > EYE_MIN_H:
            raw.append((-1, lev[lab3 == q][:, :2].mean(0)))      # size -1 marks an eye
    return lev[lab == k], raw, border


def dense_run(v, bin_=0.02, frac=0.25):
    """Mask of the values inside the longest run of histogram bins holding >= frac x the densest bin (drops thin attached strips)."""
    edges = np.arange(v.min(), v.max() + bin_, bin_)
    if len(edges) < 3:
        return np.ones(len(v), bool)
    hc, edges = np.histogram(v, bins=edges)
    good = hc >= frac * hc.max()
    best, cur = (0, 0), None
    for i, g in enumerate(good):
        if g:
            cur = (cur[0], i) if cur else (i, i)
            if cur[1] - cur[0] >= best[1] - best[0]:
                best = cur
        else:
            cur = None
    return (v >= edges[best[0]]) & (v <= edges[best[1] + 1])


def perceive(path, extra=()):
    P = p.load(path); pel, lev = p.cloud(P); tz = p.table_z(P)
    h = lev[..., 2] - tz; ok = P['depth'] > 0
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    # table edge: per y-bin nearest table point (the hands cover a few bins), median
    tab = (np.abs(h) < 0.01) & ok & (np.abs(lev[..., 1]) < 0.4)
    xs, ys = lev[..., 0][tab], lev[..., 1][tab]
    near = [xs[(ys >= a) & (ys < a + 0.05)].min() for a in np.arange(-0.4, 0.4, 0.05) if ((ys >= a) & (ys < a + 0.05)).sum() > 30]
    edge = float(np.median(near)) if near else None
    # box: sim depth sees the rim top (table + ~0.145); real depth will not -> colour fallback (boxdet)
    box = None
    rim = (np.abs(h - 0.145) < 0.012) & ok
    rim = cv2.morphologyEx(rim.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stt, _ = cv2.connectedComponentsWithStats(rim)
    if n > 1:
        k = 1 + int(np.argmax(stt[1:, 4]))
        if stt[k, 4] > 3000:
            pts = lev[lab == k]
            pts = pts[dense_run(pts[:, 0]) & dense_run(pts[:, 1])]   # f20de8 dev 9: a thin strip at rim height (400 px/bin vs 5000)
            x0, x1 = np.percentile(pts[:, 0], [1, 99]); y0, y1 = np.percentile(pts[:, 1], [1, 99])   # joined the rim: y +.16 -> +.48
            box = dict(x=[float(x0), float(x1)], y=[float(y0), float(y1)], rim_z=tz + RIM_H, src='rim_depth',
                       px=int(stt[k, 4]), corners=cv2.boxPoints(cv2.minAreaRect(pts[:, :2].astype(np.float32))).tolist())
            # clipped by the image border: extend that side to the known outer size (0.39 x 0.28)
            bx, by, bw_, bh_ = stt[k, :4]; H, W = rim.shape
            clip = dict(right=bx + bw_ >= W - 2, left=bx <= 1, top=by <= 1)
            sx, sy = x1 - x0, y1 - y0
            if clip['right'] or clip['left']:
                full = BOX_LONG if abs(sx - BOX_SHORT) < abs(sx - BOX_LONG) else BOX_SHORT
                if sy < full - 0.02:
                    if clip['right']:
                        box['y'][0] = float(y1 - full)
                    else:
                        box['y'][1] = float(y0 + full)
            if clip['top']:
                full = BOX_SHORT if box['y'][1] - box['y'][0] > 0.34 else BOX_LONG
                if sx < full - 0.02:
                    box['x'][1] = float(x0 + full)
            box['clipped'] = [k_ for k_, v_ in clip.items() if v_]
    if box is None:
        for side in ('right', 'left'):
            b = c3.locate_box(P, side, tz)
            if b and (box is None or b['x'][1] - b['x'][0] > box['x'][1] - box['x'][0]):
                box = dict(x=list(b['x']), y=list(b['y']), rim_z=tz + RIM_H, src='colour_' + side)
    body, raw, border = view_puppy(P, tz, box)
    for path2 in extra:                                      # extra views (waist turned toward a clipped end)
        P2 = p.load(path2); b2, r2, _ = view_puppy(P2, tz, box)
        if b2 is not None:
            body = b2 if body is None else np.vstack([body, b2]); raw += r2
    if body is None:
        return dict(P=P, tz=tz, edge=edge, box=box, puppy=None, border=border)
    # body axis (PCA in xy)
    xy = body[:, :2] - body[:, :2].mean(0)
    w, v = np.linalg.eigh(np.cov(xy.T)); ax = v[:, 1]
    if ax[1] < 0:
        ax = -ax                                            # axis points to +y (left)
    # paws: dark brown blobs just above the table next to the body; 2 groups along the axis = front / hind legs
    blobs = []
    for size, cxy in raw:                                    # dedupe blobs seen in two views
        dist = np.min(np.linalg.norm(body[::7, :2] - cxy, axis=1))
        if dist < 0.14 and not any(np.linalg.norm(cxy - b_[1]) < 0.03 and (b_[0] < 0) == (size < 0) for b_ in blobs):
            blobs.append((size, cxy, dist))
    eyes = [b for b in blobs if b[0] < 0 and b[2] < 0.06]
    blobs = [b for b in blobs if b[0] >= 0]
    paws = sorted([b for b in blobs if b[0] >= 300 and b[2] < 0.06], key=lambda b: float(b[1] @ ax))
    small = [b for b in blobs if b[0] < 300]                # nose / eyes (the head may be hidden by the tucked left hand)
    mid_xy, head = None, None
    if len(paws) >= 2:
        s = [float(b[1] @ ax) for b in paws]
        gaps = np.diff(s); i = int(np.argmax(gaps))
        if gaps[i] > 0.06:
            g1 = np.mean([b[1] for b in paws[:i + 1]], 0); g2 = np.mean([b[1] for b in paws[i + 1:]], 0)
            mid_xy = (g1 + g2) / 2
    pc = body[:, :2].mean(0)
    s_body = (body[:, :2] - pc) @ ax
    if mid_xy is None:
        mid_xy = pc
    nrm0 = np.array([ax[1], -ax[0]])
    votes = [np.sign(float((b[1] - mid_xy) @ ax)) for b in small
             if 0.15 < abs(float((b[1] - mid_xy) @ ax)) < 0.35 and abs(float((b[1] - mid_xy) @ nrm0)) < 0.09]
    if votes and abs(sum(votes)) == len(votes):
        head = 'left' if votes[0] > 0 else 'right'           # head end along +y = left
    mid_src = 'paws' if mid_xy is not pc else 'blob'
    if eyes:                                                 # the eye decides the head end and checks the mid-body
        e = np.mean([b[1] for b in eyes], 0)
        sgn = np.sign(float((e - pc) @ ax)) or 1.0
        head = 'left' if sgn > 0 else 'right'
        d_eye = float((e - mid_xy) @ ax) * sgn                # eye ahead of the mid point toward the head
        if not 0.12 < d_eye < 0.30:
            mid_xy = pc + ax * (float((e - pc) @ ax) - sgn * EYE_TO_MID)
            mid_src = 'eye'
    # mid-body slice: body points within 4 cm of the mid point along the axis
    sm = float((mid_xy - pc) @ ax)
    sl = body[np.abs(s_body - sm) < 0.04]
    nrm = np.array([ax[1], -ax[0]])                         # axis normal
    if nrm[0] < 0:
        nrm = -nrm                                          # points away from the robot (+x)
    across = (sl[:, :2] - mid_xy) @ nrm
    far_off = float(np.percentile(across, 97)); near_off = float(np.percentile(across, 3))
    top = float(np.percentile(sl[:, 2], 97))
    centre = mid_xy + nrm * (far_off + near_off) / 2
    pup = dict(n=int(len(body)), mid=mid_xy.tolist(), mid_src=mid_src, eyes=len(eyes), axis=ax.tolist(), axis_deg=float(np.degrees(np.arctan2(-ax[0], ax[1]))),
               centre=centre.tolist(), far=(mid_xy + nrm * far_off).tolist(), top=top, head=head,
               paws=[[round(float(v), 3) for v in b[1]] for b in paws], ends=[float(s_body.min()), float(s_body.max())],
               x=[float(np.percentile(body[:, 0], 2)), float(np.percentile(body[:, 0], 98))],
               y=[float(np.percentile(body[:, 1], 2)), float(np.percentile(body[:, 1], 98))])
    return dict(P=P, tz=tz, edge=edge, box=box, puppy=pup, border=border)


def classify(pup, box):
    cy = pup['centre'][1]
    if box['y'][1] < cy - 0.05 and box['y'][1] <= pup['y'][0] + 0.06:
        return 'right'
    if box['y'][0] > cy + 0.05 and box['y'][0] >= pup['y'][1] - 0.06:
        return 'left'
    if box['x'][0] > pup['centre'][0]:
        return 'behind'
    return 'unknown'


def release_candidates(layout, box, gy, head=None):
    """Release palm (x, y) in order of preference. Box beside the plush: box centre first, then nearer the wall the carry
    comes from - but keep >= HEAD_WALL_IN from that wall when the head hangs toward it (eval trials 3, 7: released 12 cm
    inside the left wall with the head on that side -> the plush tipped over the wall onto the table)."""
    x0, x1 = box['x']; y0, y1 = box['y']; cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    out_ = []
    if layout in ('right', 'left'):
        head_to_wall = (layout == 'right' and head == 'left') or (layout == 'left' and head == 'right')
        near = HEAD_WALL_IN if head_to_wall else 0.12
        ys = (cy, y1 - near, y1 - near + 0.03) if layout == 'right' else (cy, y0 + near, y0 + near - 0.03)
        ys = [yy for yy in ys if y0 + REL_IN <= yy <= y1 - REL_IN] or [cy]
        for dx in (0.15, 0.12, REL_IN):
            for yy in ys:
                out_.append((min(x0 + dx, cx), yy))
    else:   # behind: as far past the near wall as reachable, at the grasp y kept inside the side walls
        yy = float(np.clip(gy, y0 + 0.10, y1 - 0.10))
        for dx in (0.14, 0.12, 0.10, REL_IN):
            out_.append((x0 + dx, yy))
    return out_


def grasp_yaw(pup):
    """Fingers across the body: palm yaw = body axis angle from +y, wrapped into [-50, 50] deg."""
    a = pup['axis_deg']
    while a > 90:
        a -= 180
    while a < -90:
        a += 180
    return float(np.clip(a + 10.0, -50, 50))   # +10: v5 used 15 for a body along y


def plan(st, cmd):
    pup, box, layout = st['puppy'], st['box'], st['layout']
    gx = pup['far'][0] - GRASP_BACK; gy = pup['far'][1] - GRASP_BACK * np.sin(np.radians(pup['axis_deg']))
    gy = pup['centre'][1] if abs(gy - pup['centre'][1]) > 0.05 else gy
    if pup.get('head') in ('left', 'right'):   # the press and close shove the plush toward its head: grasp toward the tail
        ax = np.array(pup['axis']) * (1 if pup['head'] == 'left' else -1)
        gx -= TAIL_SHIFT * ax[0]; gy -= TAIL_SHIFT * ax[1]
    top = pup['top']; gyaw = float(os.environ['GYAW']) if os.environ.get('GYAW') else grasp_yaw(pup)   # GYAW: experiment override (deg)
    if layout == 'left' and gy < LATERAL_Y and not os.environ.get('GYAW'):
        # d65273: at mid y <= -.27 fingers across the body (yaw ~10) pressed only the top skin (5 empty fists in 7 closes, all at
        # waists with NO IK margin); fingers forward-right (-45) held 1/2. f20de8 runs 2-5: with the waist-roll margin, -45 closed
        # but crept/slipped in 4/4 (diagonal fingers span ~18 cm of a 13 cm body). Across-body yaw when it reaches with margin.
        across = gyaw
        gyaw = LATERAL_GYAW
        for wy, wr in WAISTS_LEFT[:4]:
            q = list(cmd); q[2] = wy; q[5] = wr; q[8] = 0.0
            if c3.best(q, (gx, gy, top - 0.03 - GRASP_MARGIN), across)[0] < 0.01:
                gyaw = across
                break
    if layout == 'behind':     # finger links reach ~0.115 m ahead of the palm: keep them short of the near wall
        gx = min(gx, box['x'][0] - 0.135)
    ryaw = {'right': -25.0, 'left': 30.0, 'behind': 0.0}[layout]
    rz = box['rim_z'] + CARRY_ABOVE_RIM - 0.01
    tried = []; best_ = None
    cands = release_candidates(layout, box, gy, pup.get('head'))
    if layout == 'left':
        return plan_left(cmd, gx, gy, top, gyaw, rz, cands)
    for wy, wr in WAISTS:              # every waist: keep the one whose release is most preferred (box centre first)
        q = list(cmd); q[2] = wy; q[5] = wr; q[8] = 0.0
        e1, _ = c3.best(q, (gx, gy, top - 0.03), gyaw)
        e1b, _ = c3.best(q, (gx, gy, top + 0.13), gyaw)
        if e1 > 0.01 or e1b > 0.01:
            tried.append([wy, wr, round(e1, 3), None]); continue
        for rank, (rx, ry) in enumerate(cands[:best_[0] if best_ else None]):
            e2, _ = c3.best(q, (rx, ry, rz), ryaw)
            if e2 < 0.01:
                best_ = (rank, dict(waist=[wy, wr], grasp=[gx, gy], top=top, gyaw=gyaw, release=[rx, ry, rz], ryaw=ryaw, release_rank=rank))
                break
        tried.append([wy, wr, round(e1, 3), best_[0] if best_ else 'release'])
        if best_ and (best_[0] == 0 or layout == 'behind'):   # behind: the smallest waist that reaches any release
            break                                            # (run 10: the rank search asked for waist .9 = tilt risk)
    return (best_[1] if best_ else None), tried


RELEASE_WAISTS_LEFT = [(0.45, 0.15), (0.6, 0.15), (0.7, 0.15), (0.45, 0.25), (0.6, 0.25)]


def plan_left(cmd, gx, gy, top, gyaw, rz, cands):
    """Box LEFT of the plush (cross-body). Grasp waist = the first of WAISTS_LEFT whose pressed grasp reaches GRASP_MARGIN deeper
    than the press target (f20de8 run 1: no margin -> SONIC held the arm off its command, weak close). Release waist = the one
    (grasp waist first: no swing) with the most preferred release, then the palm yaw nearest the grasp yaw (run 2: a 75 deg
    rotation on the carry -> slip); 2 cm IK margin below the release. The carry turns the waist with the plush lifted (swing)."""
    tried = []; gw = None
    for wy, wr in WAISTS_LEFT:
        q = list(cmd); q[2] = wy; q[5] = wr; q[8] = 0.0
        e1, _ = c3.best(q, (gx, gy, top - 0.03 - GRASP_MARGIN), gyaw)
        e1b, _ = c3.best(q, (gx, gy, top + 0.13), gyaw)
        tried.append([wy, wr, round(e1, 3), round(e1b, 3)])
        if e1 < 0.01 and e1b < 0.01:
            gw = [wy, wr]; break
    if gw is None:
        return None, tried
    key, rel = None, None
    for wy, wr in [tuple(gw)] + [w for w in RELEASE_WAISTS_LEFT if list(w) != gw]:
        q = list(cmd); q[2] = wy; q[5] = wr; q[8] = 0.0
        for rk, (rx, ry) in enumerate(cands if key is None else cands[:key[0] + 1]):
            ok_y = [yw for yw in sorted({round(gyaw, 1), round(gyaw + 30.0, 1), 0.0, 15.0, 30.0})
                    if yw <= 30.0 and c3.best(q, (rx, ry, rz), yw)[0] < 0.01 and c3.best(q, (rx, ry, rz - 0.02), yw)[0] < 0.01]
            if ok_y:
                yw = min(ok_y, key=lambda v: abs(v - gyaw))
                if key is None or (rk, abs(yw - gyaw)) < key:
                    key = (rk, abs(yw - gyaw)); rel = dict(release=[rx, ry, rz], release_rank=rk, ryaw=yw, release_waist=[wy, wr])
                    tried.append(['release_waist', wy, wr, rk, yw])
                break
    if rel is None:
        return None, tried
    if rel['release_waist'] == gw:
        rel.pop('release_waist')
    return dict(waist=gw, grasp=[gx, gy], top=top, gyaw=gyaw, **rel), tried


def mark(P, pts, path, box=None, tz=None):
    if box is not None:
        p.draw_box(P, box['x'][0], box['x'][1], box['y'][0], box['y'][1], box['rim_z'], tz, path)
        img = cv2.imread(path)
    else:
        img = cv2.imread(path) if os.path.exists(path) else P['img'].copy()
    for q, col in pts:
        uv = p.project(P, p.to_pelvis(P, np.array(q)))[0].astype(int)
        cv2.circle(img, (int(uv[0]), int(uv[1])), 6, col, -1)
    cv2.imwrite(path, img)


# ---------------------------------------------------------------- steps

def step_tuck(ctx, st):
    """Both arms back and up (shoulder pitch first so the fingers leave the table / box rim upward)."""
    row, meas, _ = c.pps.last_row(ctx)
    both = lambda d: dict(R(d), **L(d))
    c.pps.ramp(ctx, both({'shoulder_pitch': 0.8}), 3.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, both({'elbow': -0.5, 'wrist_roll': 0, 'wrist_pitch': 0, 'wrist_yaw': 0, 'shoulder_yaw': 0}), 3.0)
    c.pps.ramp(ctx, dict(both(TUCK), waist_yaw_joint=0.0, waist_roll_joint=0.0, waist_pitch_joint=0.0), 3.0)
    time.sleep(1.0)


def in_look(j):
    return (abs(j[11] - LOOK['shoulder_pitch']) < 0.15 and abs(j[12] - LOOK['shoulder_pitch']) < 0.15 and abs(j[21] - LOOK['elbow']) < 0.15
            and abs(j[22] - LOOK['elbow']) < 0.15 and abs(j[2]) < 0.1)


def to_look(ctx):
    """Both arms back and DOWN behind the table edge (shoulders first, then elbows), waist 0."""
    both = lambda d: dict(R(d), **L(d))
    c.pps.ramp(ctx, dict(both({'shoulder_pitch': LOOK['shoulder_pitch']}), waist_yaw_joint=0.0, waist_roll_joint=0.0,
                         waist_pitch_joint=0.0), 3.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, both(dict(LOOK, wrist_roll=0, wrist_pitch=0, wrist_yaw=0, shoulder_yaw=0)), 3.0)
    time.sleep(1.5)
    # f20de8 dev 9/10: the LEFT hand came down onto the drawer unit beside the robot (wrist pitch -1.13 vs 0, elbow .25-.49 vs .6);
    # pushing forward did not free it, going back up to the guard pose did. A blocked arm returns to its guard pose.
    row, meas, _ = c.pps.last_row(ctx); blocked = []
    for side in ('left', 'right'):
        dev = max(abs(meas[c.J.index(f'{side}_{a}_joint')] - row[9 + c.J.index(f'{side}_{a}_joint')])
                  for a in ('shoulder_pitch', 'elbow', 'wrist_pitch', 'wrist_roll'))
        if dev > HOME_BLOCKED_DEV:
            blocked.append([side, round(dev, 2)])
            c.pps.ramp(ctx, c.mirror(side, LEFT_GUARD), 5.0, hands=(0.0, 0.0))
            time.sleep(1.0)
    return blocked


def hands_clearance(ob, path):
    """Min distance from plush points to either hand's links (left_clearance for both sides). Plush lying on the table only:
    points below table + 0.16 (f20de8 dev 11: the raised guard-pose hand's own shading passed the cream mask -> 'plush 2 cm away')."""
    P = p.load(path); pel, lev = p.cloud(P); tz = p.table_z(P)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 60) & (hsv[..., 1] < 125) & (hsv[..., 2] > 120) & (P['depth'] > 0)
    m = (cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool) & (lev[..., 2] > tz + 0.02)
         & (lev[..., 2] < tz + 0.16))
    q = pel[m]
    if len(q) < 50:
        return None
    j = ob['proprio']['joint_positions_rad']
    pts = np.array(list(c.pps.link_points(j, 'left').values()) + list(c.pps.link_points(j, 'right').values()))
    return round(float(np.min(np.linalg.norm(q[None, ::2, :] - pts[:, None, :], axis=2))), 3)


def step_scene(ctx, st, tuck=True):
    if tuck:   # look pose: both arms back and DOWN behind the table edge, out of the camera view (tucked hands hid the puppy, run 3)
        ob, path0 = c.observe(ctx, 'run_prelook'); j = ob['proprio']['joint_positions_rad']
        if not in_look(j):
            hc = hands_clearance(ob, path0)                   # run 6: the reset laid the plush between the tucked hands and the
            if hc is not None and hc < HAND_CLEAR_MIN:         # move to the look pose dragged it off the table
                out(False, f'plush {hc} m from a hand: do not move the arms - ask the operator to move the plush/table',
                    frame=ob['camera']['frame_id'])
            to_look(ctx)
            time.sleep(1.0)
    ob, path = c.observe(ctx, 'run_scene')
    S = perceive(path)
    views = [ob['camera']['frame_id']]
    sides = []
    if S.get('border'):                                       # puppy runs off the image: look toward that side too
        sides = [SIDE_LOOK_YAW * (-1 if 'right' in S['border'] else 1)]
    elif S['puppy'] is None:                                  # not in view at all (d65273 run 6): look right, then left
        sides = [-SIDE_LOOK_YAW, SIDE_LOOK_YAW]
    extra = []
    for yaw in sides:
        c.pps.ramp(ctx, {'waist_yaw_joint': yaw}, 3.0)
        time.sleep(1.5)
        ob2, path2 = c.observe(ctx, 'run_scene_side')
        extra.append(path2); views.append(ob2['camera']['frame_id'])
        S = perceive(path, extra)
        S['side_yaw'] = [yaw, round(ob2['proprio']['joint_positions_rad'][2], 3)]
        if S['puppy'] is not None and not S.get('border_side'):
            break
    if sides:
        c.pps.ramp(ctx, {'waist_yaw_joint': 0.0}, 3.0)
        time.sleep(1.0)
    pup, box = S['puppy'], S['box']
    st.clear(); st.update(scene_obs=path, frame=ob['camera']['frame_id'], table_z=S['tz'], edge=S['edge'], tilt=ob['tilt_deg'])
    if pup is None or box is None:
        save(st)
        out(False, f"not found: puppy {pup is not None} box {box is not None} - look at the frame / ask the operator", frame=st['frame'])
    pup = {k: v for k, v in pup.items()}
    st.update(puppy=pup, box=box, layout=classify(pup, box))
    over = os.path.join(ROOT, 'output', f"run_scene_{ob['step_id']}.jpg")
    mark(S['P'], [(pup['centre'] + [pup['top']], (0, 255, 0)), (pup['far'] + [pup['top']], (255, 0, 255))], over, box, S['tz'])
    st['overlay'] = over
    checks = []
    if st['edge'] is None or not 0.14 < st['edge'] < 0.26:
        checks.append(f"edge {st['edge']} not 0.14-0.26: ask the operator to bring the table to ~0.20 m")
    bw = box['x'][1] - box['x'][0]; bl = box['y'][1] - box['y'][0]
    if not (0.2 < bw < 0.45 and 0.2 < bl < 0.45):
        checks.append(f'box size {bw:.2f} x {bl:.2f} implausible (clipped / not the box)')
    if st['layout'] == 'unknown':
        checks.append('layout unknown (box overlaps the puppy?)')
    if st['edge'] and pup['centre'][0] < st['edge'] + EDGE_MIN_IN:
        checks.append(f"mid-body only {pup['centre'][0] - st['edge']:.3f} m inside the front edge: ask the operator to push it in / reset")
    cmd = c3.cmd_row(ctx)
    pl, tried = (None, []) if checks else plan(st, cmd)
    if not checks and pl is None:
        checks.append('no waist reaches both the pressed grasp and a release 9 cm inside the walls: ask the operator')
    if pl:
        st.update(pl)
        mark(S['P'], [(pl['grasp'] + [pup['top']], (0, 0, 255)), (pl['release'][:2] + [box['rim_z']], (255, 255, 0))], over)
    save(st)
    out(not checks, '; '.join(checks), views=views, side_yaw=S.get('side_yaw'), layout=st['layout'], edge=st['edge'], table_z=round(S['tz'], 3),
        puppy=dict(centre=np.round(pup['centre'], 3).tolist(), far=np.round(pup['far'], 3).tolist(), top=round(pup['top'], 3),
                   axis_deg=round(pup['axis_deg'], 1), head=pup['head'], paws=pup['paws'], y=np.round(pup['y'], 3).tolist()),
        box=dict(x=np.round(box['x'], 3).tolist(), y=np.round(box['y'], 3).tolist(), src=box['src']),
        plan=pl, tried=tried, overlay=over, frame=st['frame'])


def step_approach(ctx, st):
    """Raise the right hand by the back path (no sweep over the table), set the waist with the hand high, hover."""
    c.pps.ramp(ctx, L(LEFT_GUARD), 3.0)                    # left arm back and up BEFORE the right hand moves
    ob = ctx.value('g1_observe')
    ok, lt = left_tucked(ob)
    if not ok:
        out(False, 'left arm not tucked', **lt)
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.9}), 2.5)
    c.pps.ramp(ctx, R({'elbow': -0.9}), 3.0)
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.1, 'shoulder_roll': -0.35}), 3.0)
    c.pps.ramp(ctx, R({'wrist_roll': -1.3}), 2.5)
    wy, wr = st['waist']
    if wy or wr:
        c.pps.ramp(ctx, {'waist_yaw_joint': wy, 'waist_roll_joint': wr}, 5.0)
        time.sleep(1.5)
    # left arm further back and up while the right hand works: with the waist turned left the 0.45/-0.5 tuck put the
    # left fingertips at (0.22, 0.27, 0.10-0.16) = where a head-left plush's head hangs after the lift (run 2)
    c.pps.ramp(ctx, L(LEFT_GUARD), 3.0)
    ob, _ = c.observe(ctx, 'run_waist')
    if ob['tilt_deg'] > 6.5:
        out(False, f"tilt {ob['tilt_deg']} after the waist", frame=ob['camera']['frame_id'])
    c.ppg.set_pinch_shape(ctx, 'right')
    gx, gy = st['grasp']; top = st['top']; hist = []
    for z, secs in ((top + 0.13, 5.0), (top + 0.06, 3.0)):
        r = mv(ctx, (gx, gy, z), st['gyaw'], secs, max_jump=1.6); ob = r.pop('ob', None)
        hist.append([round(z, 3), r.get('palm'), r.get('tilt')])
        if not r['ok']:
            out(False, r['why'], hist=hist)
    off = np.zeros(2)
    err = np.array(r['palm'][:2]) - np.array([gx, gy])
    if np.linalg.norm(err) > PALM_TOL:          # lateral reach limit: palm short of its command (run 3: 4.6 cm) -> offset the command once
        off = np.clip(-err, -OFF_MAX, OFF_MAX)
        r = mv(ctx, (gx + off[0], gy + off[1], top + 0.06), st['gyaw'], 3.0, max_jump=1.6); ob = r.pop('ob', None)
        hist.append(['corr', off.round(3).tolist(), r.get('palm')])
        if not r['ok']:
            out(False, r['why'], hist=hist)
        err = np.array(r['palm'][:2]) - np.array([gx, gy])
    path = os.path.join(ROOT, 'output', f"run_hover_{ob['step_id']}.json"); json.dump(ob, open(path, 'w'))
    P = p.load(path)
    over = os.path.join(ROOT, 'output', f"run_hover_{ob['step_id']}.jpg")
    mark(P, [((gx, gy, top), (0, 0, 255))], over)
    st['hovered'] = True; st['off'] = off.tolist(); save(st)
    out(float(np.linalg.norm(err)) < 0.035, 'hover palm off its command' if np.linalg.norm(err) >= 0.035 else '',
        palm=r['palm'], err_xy=np.round(err, 3).tolist(), hist=hist, overlay=over, frame=r['frame'],
        look='hand between the front and hind legs, over the body? if not: run.py shift DX DY')


def step_shift(ctx, st, dx=0.0, dy=0.0):
    st['grasp'] = [st['grasp'][0] + float(dx), st['grasp'][1] + float(dy)]; save(st)
    o = st.get('off', [0.0, 0.0])
    r = mv(ctx, (st['grasp'][0] + o[0], st['grasp'][1] + o[1], st['top'] + 0.06), st['gyaw'], 3.0); r.pop('ob', None)
    out(r['ok'], r.get('why', ''), grasp=st['grasp'], palm=r.get('palm'), frame=r.get('frame'))


def step_grasp(ctx, st):
    ob = ctx.value('g1_observe')
    ok, lt = left_tucked(ob)
    if not ok:
        out(False, 'left arm not tucked', **lt)
    gx, gy = st['grasp']; top = st['top']; hist = []; z = None; extra = EXTRA_PRESS
    off = np.array(st.get('off', [0.0, 0.0]), float)
    for d in PRESS_LEVELS + (PRESS_LEVELS[-1] - 0.015, PRESS_LEVELS[-1] - 0.03):
        zc = top + d
        r = c3.move(ctx, (gx + off[0], gy + off[1], zc), st['gyaw'], 2.5); r.pop('ob', None)   # no sync: contact holds the arm off its command
        if not r['ok']:
            break
        e = np.array([gx, gy]) - np.array(r['palm'][:2])
        if r['palm'][2] < zc + 0.015 and np.linalg.norm(e) > PALM_TOL:   # free (not on the plush yet): re-aim on the measured palm
            off = np.clip(off + e, -OFF_MAX, OFF_MAX)                  # (run 3: the hover offset held the palm 5 cm off at press height)
            r2 = c3.move(ctx, (gx + off[0], gy + off[1], zc), st['gyaw'], 2.0); r2.pop('ob', None)
            if r2['ok']:
                r = r2
        z = zc; hist.append([round(zc, 3), [round(v, 3) for v in r['palm']], r['tilt'], off.round(3).tolist()])
        if st['layout'] == 'behind':       # finger links vs the near wall: step the palm back instead of closing on the top skin (run 6)
            for _ in range(3):
                lp = c.pps.link_points(c3.cmd_row(ctx), 'right')
                tip = max(lp['right_hand_index_1_link'][0], lp['right_hand_middle_1_link'][0])
                if tip <= st['box']['x'][0] - FINGER_WALL_MARGIN:
                    break
                gx -= 0.01
                r = c3.move(ctx, (gx + off[0], gy + off[1], zc), st['gyaw'], 2.0); r.pop('ob', None)
                if not r['ok']:
                    break
            st['grasp'] = [gx, gy]; save(st)
            hist[-1].append(round(tip, 3))
        if r['tilt'] > TILT_MAX:
            break
        if r['palm'][2] > zc + STALL_DZ:              # first stall: one level deeper while the tilt allows (deeper hook)
            if r['palm'][2] > top + 0.005 and r['tilt'] < PRESS_TILT:
                continue                              # still resting ON the plush (run 4: thumb in, palm top+.016 -> fist): press on
            if extra <= 0 or r['tilt'] > 6.0:
                break
            extra -= 1
    if z is None:
        out(False, 'no press level reachable', hist=hist)
    c.pps.ramp(ctx, {}, 4.0, hands=(0.0, 1.0))
    time.sleep(1.0)
    obc = ctx.value('g1_observe'); f0 = c.fingers(obc, 'right')
    # raise and lift from the MEASURED palm xy: the press pushes the palm ~3 cm past its command (run 7: cmd .275, palm .307);
    # going back to the command drags the hook back through the plush (e578 lesson)
    pm = c.pps.link_points(obc['proprio']['joint_positions_rad'], 'right')['right_palm']
    if abs(pm[0] - gx) < 0.06 and abs(pm[1] - gy) < 0.06:
        gx, gy = pm[0] + off[0], pm[1] + off[1]             # command that keeps the palm where it closed
    z += 0.03                                              # closed raise: rolls the plush into the hand
    r = mv(ctx, (gx, gy, z), st['gyaw'], 4.0); ob = r.pop('ob', None)
    st['press_z'] = z; st['lift_xy'] = [gx, gy]; save(st)
    f = r.get('fingers', [9] * 7)
    out(r['ok'] and held(f), '' if held(f) else 'nothing held (fingers at the fist): run.py abort, then scene again',
        base_idx_mid_close=[round(f0[5], 2), round(f0[3], 2)], base_idx_mid=[f[5], f[3]], palm=r.get('palm'),
        hist=hist, frame=r.get('frame'), look='plush in the hand, raised a few cm?')


def hang_below(ob, path):
    """How far the held plush hangs below the measured palm: cream points (sat 60-125, not the white hand) within 0.22 m
    of the palm in xy, above the table; palm z - their 3rd percentile z. None when too few are in view."""
    P = p.load(path); pel, lev = p.cloud(P)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 60) & (hsv[..., 1] < 125) & (hsv[..., 2] > 120) & (P['depth'] > 0)
    m &= lev[..., 2] > p.table_z(P) + 0.02
    pa = np.array(c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm'])
    q = pel[m]
    q = q[(np.linalg.norm(q[:, :2] - pa[:2], axis=1) < 0.22) & (q[:, 2] < pa[2] + 0.02)]
    if len(q) < 200:
        return None
    return round(float(pa[2] - np.percentile(q[:, 2], 3)), 3)


def step_lift(ctx, st):
    ob = ctx.value('g1_observe')
    ok, lt = left_tucked(ob)
    if not ok:
        out(False, 'left arm not tucked', **lt)
    gx, gy = st.get('lift_xy', st['grasp']); x = gx; z = st['press_z']; hist = []; y = gy; z0 = z
    lift_z = st['box']['rim_z'] + LIFT_ABOVE_RIM
    drift = {'right': -1, 'left': 1}.get(st['layout'], 0) * LIFT_DRIFT   # toward the box: the hanging head swings off the left hand
    while z < lift_z - 1e-3:
        z = min(lift_z, z + LIFT_STEP)
        if z - z0 > LIFT_DRIFT_AFTER and abs(y - gy) < LIFT_DRIFT_MAX:
            y += drift
        r = mv(ctx, (x, y, z), st['gyaw'], 4.0); ob = r.pop('ob', None)
        for _ in range(2):        # eval trials 1, 4: straight up at the lateral reach -> IK branch jump (2.5-2.85 rad) refused
            if r['ok'] or 'refused' not in r.get('why', ''):
                break
            y -= np.sign(y) * LIFT_IN_STEP if abs(y) > 0.05 else 0.0
            x -= 0.01                                        # rise toward the body instead
            r = mv(ctx, (x, y, z), st['gyaw'], 4.0); ob = r.pop('ob', None)
            hist.append(['inward', round(x, 3), round(y, 3)])
        hist.append([round(r.get('palm', [0, 0, 0])[2], 3), r.get('fingers', [])[3:6:2]] + (['sync', r['synced']] if r.get('synced') else []))
        if not r['ok'] or not held(r['fingers']):
            out(False, r.get('why') or 'grip lost', hist=hist, frame=r.get('frame'))
    path = os.path.join(ROOT, 'output', f"run_lift_{ob['step_id']}.json"); json.dump(ob, open(path, 'w'))
    st['lifted'] = [x, y, z]; st['hang'] = hang_below(ob, path); save(st)
    lc = left_clearance(ob, path)
    out(lc is None or lc > LEFT_CLEAR_MIN, '' if lc is None or lc > LEFT_CLEAR_MIN else f'plush {lc:.3f} m from the left hand: look',
        palm=r['palm'], hist=hist, left_clear=lc, hang=st['hang'], frame=r['frame'])


def hanging_check(ob, st):
    """Cream points of the hanging plush (between table + 2 cm and the palm) vs the box footprint."""
    path = os.path.join(ROOT, 'output', f"run_carry_{ob['step_id']}.json"); json.dump(ob, open(path, 'w'))
    P = p.load(path); pel, lev = p.cloud(P); tz = st['table_z']
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 40) & (hsv[..., 1] < 140) & (hsv[..., 2] > 120)
    m = cream & (lev[..., 2] > tz + 0.02) & (P['depth'] > 0)
    b = st['box']
    pts = lev[m]
    over = os.path.join(ROOT, 'output', f"run_carry_{ob['step_id']}.jpg")
    p.draw_box(P, b['x'][0], b['x'][1], b['y'][0], b['y'][1], b['rim_z'], tz, over)
    pa = np.array(c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm'])
    uv = p.project(P, (pa - [0.0, 0.0, 0.08])[None])[0]          # the plush hangs below the palm
    H, W = P['img'].shape[:2]
    if len(pts) < 50 or not (-60 <= uv[0] < W + 60 and -60 <= uv[1] < H + 60):
        return None, over                                   # palm (and the plush under it) out of view: unknown (run 7)
    inside = ((pts[:, 0] > b['x'][0] + 0.02) & (pts[:, 0] < b['x'][1] - 0.02) &
              (pts[:, 1] > b['y'][0] + 0.02) & (pts[:, 1] < b['y'][1] - 0.02))
    return float(inside.mean()), over


def swing(ctx, waist_to, pos, yaw, n=SWING_STEPS, secs=4.0):
    """Turn the waist (yaw, roll) to waist_to in n ramps while the arm IK (commanded row) keeps the palm at pos in the
    pelvis frame, so the held plush barely moves. Returns the per-step history; stops on a refused IK step or a lost grip."""
    row, _, _ = c.pps.last_row(ctx)
    w0 = np.array([row[9 + 2], row[9 + 5]]); w1 = np.array(waist_to, float)
    hist = []
    for k in range(1, n + 1):
        w = w0 + (w1 - w0) * k / n
        row, _, _ = c.pps.last_row(ctx); q = list(row[9:38]); q[2], q[5] = float(w[0]), float(w[1])
        t, e, _ = c.pps.solve(q, 'right', pos, c.pps.palm_quat(yaw, 'right'))
        if e > 0.015:
            e, t = c3.best(q, pos, yaw)
        jump = max(abs(t[k_] - row[9 + c.J.index(k_)]) for k_ in t)
        if e > 0.015 or jump > 0.8:
            hist.append({'refused': [round(float(w[0]), 2), round(float(w[1]), 2)], 'err': round(e, 3), 'jump': round(jump, 2)})
            return False, hist, None
        c.pps.ramp(ctx, dict(t, waist_yaw_joint=float(w[0]), waist_roll_joint=float(w[1])), secs)
        time.sleep(0.5)
        ob = ctx.value('g1_observe'); f = c.fingers(ob, 'right')
        j = ob['proprio']['joint_positions_rad']
        hist.append({'waist_cmd': np.round(w, 2).tolist(), 'waist': [round(j[2], 2), round(j[5], 2)], 'f': [round(f[3], 2), round(f[5], 2)],
                     'palm': c.pps.link_points(j, 'right')['right_palm'], 'tilt': ob['tilt_deg']})
        if not held(f) or ob['tilt_deg'] > TILT_MAX:
            return False, hist, ob
    return True, hist, ob


def step_carry(ctx, st):
    row, _, _ = c.pps.last_row(ctx)
    start = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])
    end = np.array(st['release'], float)
    # carry high enough that the hanging plush clears the rim (f20de8 run 3: palm rim+.22, the dangling plush caught the box's
    # side wall and pushed the box 13 cm); unknown hang (plush out of view) = HANG_DEFAULT
    hang = max(st.get('hang') or 0.0, HANG_DEFAULT)   # f20de8 run 6: only the raised head was in view -> 'hang' .025: never below the default
    cz = st['box']['rim_z'] + min(CARRY_MAX_ABOVE_RIM, max(CARRY_ABOVE_RIM, hang + HANG_CLEAR))
    # eval f20de8 trial 6 (behind, release x .52): the last step was IK-refused at cz, lowered + re-synced, and the grip let go there.
    # Choose the carry height reachable AT THE RELEASE xy (release waist, release yaw) before moving
    q = list(row[9:38])
    if st.get('release_waist'):
        q[2], q[5] = st['release_waist']
    while cz > st['box']['rim_z'] + CARRY_ABOVE_RIM + 1e-3 and c3.best(q, (end[0], end[1], cz), st['ryaw'])[0] > 0.01:
        cz -= 0.02
    start[2] = max(start[2], cz)
    hist = [{'carry_z': round(cz, 3), 'hang': st.get('hang')}]
    rw = st.get('release_waist')
    if rw and (abs(rw[0] - row[9 + 2]) > 0.01 or abs(rw[1] - row[9 + 5]) > 0.01):
        # first up to carry height at the grasp waist, then turn the waist with the palm fixed (pelvis frame)
        r = mv(ctx, tuple(start), st['gyaw'], 5.0); r.pop('ob', None)
        hist.append({'up': start.round(3).tolist(), 'palm': r.get('palm'), 'f': (r.get('fingers') or [0] * 7)[3:6:2], 'why': r.get('why', ''),
                     'arm_dev': r.get('arm_dev'), 'synced': r.get('synced')})
        if not r['ok'] or not held(r['fingers']):
            out(False, r.get('why') or 'grip lost before the waist swing', history=hist, frame=r.get('frame'))
        ok, sh, _ = swing(ctx, rw, tuple(start), st['gyaw'])
        hist += sh
        if not ok:
            out(False, 'waist swing refused or grip lost', history=hist)
    n = max(2, int(np.ceil(np.linalg.norm((end - start)[:2]) / 0.06)))
    riding = False
    for k in range(1, n + 1):
        a = k / n
        pos = start + a * (end - start); pos[2] = cz
        yaw = st['gyaw'] + a * (st['ryaw'] - st['gyaw'])
        r = mv(ctx, tuple(pos), yaw, 5.0); ob = r.pop('ob', None)
        while not r['ok'] and pos[2] > max(end[2], st['box']['rim_z'] + CARRY_ABOVE_RIM) + 0.005:   # high carry out of reach: lower
            pos[2] -= 0.02; cz = pos[2]
            r = mv(ctx, tuple(pos), yaw, 5.0); ob = r.pop('ob', None)
            hist.append({'lowered': round(cz, 3), 'ok': r['ok']})
        if not r['ok']:
            out(False, r['why'], history=hist, at=pos.round(3).tolist())
        hist.append({'cmd': pos.round(3).tolist(), 'palm': r['palm'], 'f': r['fingers'][3:6:2], 'tilt': r['tilt'],
                     'yaw': round(float(yaw), 1), 'arm_dev': r.get('arm_dev'), 'synced': r.get('synced')})
        if r['tilt'] > 8:
            out(False, 'tilt on the carry', history=hist, frame=r['frame'])
        if not held(r['fingers']) and not riding:
            # f20de8 run 8: the plush rolled onto the back of the fist and rode along; carried on over the box and tipped
            # off by the release step it landed INSIDE. Continue only while it is still on the hand.
            n_on, _ = near_hand(ob, st)
            hist.append({'grip_lost_at': r['palm'], 'near_hand_px': n_on})
            if n_on <= HANG_PX:
                out(False, 'grip lost on the carry (plush not on the hand)', history=hist, frame=r['frame'])
            riding = True
    if pos[2] > end[2] + 0.005:            # over the box: straight down to the release height
        pos = np.array(end, float)
        r2 = mv(ctx, tuple(pos), st['ryaw'], 4.0); ob2 = r2.pop('ob', None)
        if r2['ok']:
            r, ob = r2, ob2
        hist.append({'down': pos.round(3).tolist(), 'palm': r2.get('palm'), 'ok': r2['ok']})
        # eval f20de8 trial 2: the palm stayed at z .378 for a .268 command, the plush was opened 31 cm above the rim and fell over
        # the far wall. Step the command down while the MEASURED palm is high (never below the rim + SET_DOWN_ABOVE_RIM)
        for _ in range(3):
            if r['palm'][2] <= end[2] + REL_Z_TOL:
                break
            pos = pos - np.array([0.0, 0.0, 0.03])
            if pos[2] < st['box']['rim_z'] + SET_DOWN_ABOVE_RIM:
                break
            r2 = mv(ctx, tuple(pos), st['ryaw'], 3.0); ob2 = r2.pop('ob', None)
            if not r2['ok']:
                break
            r, ob = r2, ob2
            hist.append({'down_more': pos.round(3).tolist(), 'palm': r['palm']})
    # head-side wall clearance on the MEASURED palm (head-toward-wall releases: placed at 18.5-20.5 cm, tipped out at
    # 11 / 12 / 14.5 cm - eval trials 3, 7, post-eval run 9; the palm lags its command up to 5 cm at the reach limit)
    lay, head, b = st['layout'], st['puppy'].get('head'), st['box']
    if (lay == 'right' and head == 'left') or (lay == 'left' and head == 'right'):
        for _ in range(2):
            pa = r['palm']; clear = (b['y'][1] - pa[1]) if lay == 'right' else (pa[1] - b['y'][0])
            if clear >= HEAD_WALL_MEAS:
                break
            dy = (HEAD_WALL_MEAS - clear + 0.01) * (-1 if lay == 'right' else 1)
            pos = pos + np.array([0.0, dy, 0.0])
            r2 = mv(ctx, tuple(pos), st['ryaw'], 4.0); ob2 = r2.pop('ob', None)
            if not r2['ok'] or not held(r2['fingers']):
                break
            r, ob = r2, ob2
            hist.append({'nudge': pos.round(3).tolist(), 'palm': r['palm'], 'f': r['fingers'][3:6:2], 'tilt': r['tilt']})
    frac, over = hanging_check(ob, st)
    # body hanging over a wall (run 5: frac .45, body toward the right wall): step the palm toward the box centre while IK allows
    cxy = np.array([(b['x'][0] + b['x'][1]) / 2, (b['y'][0] + b['y'][1]) / 2])
    for _ in range(3):
        if frac is None or frac >= HANG_IN_MIN or lay == 'behind':
            break
        d = cxy - pos[:2]
        if np.linalg.norm(d) < 0.02:
            break
        pos = pos + np.append(d / np.linalg.norm(d) * min(CENTRE_STEP, np.linalg.norm(d)), 0.0)
        r2 = mv(ctx, tuple(pos), st['ryaw'], 4.0, max_jump=0.8); ob2 = r2.pop('ob', None)
        if not r2['ok']:
            hist.append({'centre_nudge_refused': pos.round(3).tolist()}); break
        r, ob = r2, ob2
        frac, over = hanging_check(ob, st)
        hist.append({'centre_nudge': pos.round(3).tolist(), 'palm': r['palm'], 'f': r['fingers'][3:6:2], 'frac': frac})
        if not held(r['fingers']):
            break                                        # dropped (run 5 it fell inside): the release step checks the hand
    for _ in range(2):         # side layouts: body hangs toward the robot; eval trial 1 opened at near wall + .12 -> over the wall, floor
        if lay == 'behind' or r['palm'][0] >= b['x'][0] + NEAR_WALL_SIDE:
            break
        pos = pos + np.array([0.025, 0.0, 0.0])
        r2 = mv(ctx, tuple(pos), st['ryaw'], 4.0, max_jump=0.8); ob2 = r2.pop('ob', None)
        if not r2['ok'] or not held(r2['fingers']):
            hist.append({'x_nudge_refused': pos.round(3).tolist()}); break
        r, ob = r2, ob2
        hist.append({'x_nudge': pos.round(3).tolist(), 'palm': r['palm'], 'f': r['fingers'][3:6:2]})
    pa = r['palm']
    if lay != 'behind' and pa[0] < b['x'][0] + NEAR_WALL_SIDE:
        hist.append({'near_wall_short': round(pa[0] - b['x'][0], 3)})
    inside = b['x'][0] + 0.07 < pa[0] < b['x'][1] - 0.05 and b['y'][0] + 0.07 < pa[1] < b['y'][1] - 0.07
    st['riding'] = riding; save(st)
    out(inside, '' if inside else 'palm not well inside the box footprint: look, then release or abort', palm=pa, riding=riding,
        hanging_inside_frac=frac, history=hist, overlay=over, frame=r['frame'],
        look='hanging body over the box interior (not over a wall)? then release')


def near_hand(ob, st):
    """Cream points around the measured palm (xy within 0.20 m, z above palm - 0.10): a plush still hanging on or
    draped over the hand. A plush that dropped into the box lies below (sitting head <= rim + ~0.12)."""
    path = os.path.join(ROOT, 'output', f"run_rel_{ob['step_id']}.json"); json.dump(ob, open(path, 'w'))
    P = p.load(path); pel, lev = p.cloud(P)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    # plush saturation 77-89; the white hand's warm-lit edges read 40-46 (d65273): sat > 60 keeps the hand out
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 60) & (hsv[..., 1] < 125) & (hsv[..., 2] > 120) & (P['depth'] > 0)
    pa = np.array(c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm'])
    q = pel[cream]
    m = (np.linalg.norm(q[:, :2] - pa[:2], axis=1) < 0.20) & (q[:, 2] > pa[2] - 0.10)
    return int(m.sum()), pa.round(3).tolist()


def raise_open(ctx, pa, yaw, secs=3.0):
    """Open hand up and away from the release point. At the reach limit the straight 10 cm raise is IK-refused
    (d65273 run 1, palm .465 past a box-behind wall): fall back to smaller / backed-off raises. Always returns a
    fresh observation (a refused move left near_hand() counting nothing = a false 'released')."""
    for dx, dz in ((0.0, 0.10), (-0.04, 0.10), (-0.06, 0.07), (0.0, 0.05), (-0.03, 0.04)):
        r = c3.move(ctx, (pa[0] + dx, pa[1], pa[2] + dz), yaw, secs, max_jump=1.6)
        if r['ok']:
            ob = r.pop('ob'); r['raise'] = [dx, dz]
            return r, ob
    return r, ctx.value('g1_observe')


def step_release(ctx, st):
    """Open (flat default shape), wait, 10 cm straight up, then CHECK the plush left the hand (run 4: it stayed draped
    over the open hand and home dragged it through the box). Still on the hand: lower to rim + 0.19, roll the wrist so the
    back of the hand slopes toward the box, roll back, up again; if still there stop (no home) for a look / the operator."""
    ctx.value('g1_hand_shape', side='right')                # flat default open shape (pinch open stays curled)
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(2.0)
    ob1, _ = c.observe(ctx, 'run_released')
    row, _, _ = c.pps.last_row(ctx)
    pa = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])
    r, ob = raise_open(ctx, pa, st['ryaw'])
    n, palm = near_hand(ob, st)                                # checked ~10 cm up: a plush sitting in the box stays below
    tries = [n]
    if n > HANG_PX:
        # eval trial 2 (by hand): lower until the plush sits on the box floor (palm rim + .13), then raise slowly -> it slid off
        r = c3.move(ctx, (pa[0], pa[1], st['box']['rim_z'] + SET_DOWN_ABOVE_RIM), st['ryaw'], 4.0, max_jump=1.0); r.pop('ob', None)
        time.sleep(1.0)
        r = c3.move(ctx, (pa[0], pa[1], st['box']['rim_z'] + SET_DOWN_ABOVE_RIM + 0.05), st['ryaw'], 4.0, max_jump=1.0); r.pop('ob', None)
        r, ob = raise_open(ctx, pa, st['ryaw'], 4.0)
        n, palm = near_hand(ob, st); tries.append(n)
    if n > HANG_PX:
        # draped over the back of the (palm-down) hand: lower into the box and roll the wrist so the back of the hand slopes
        # toward the box centre (post-eval trial 8 recovery: roll +0.8 at palm z .29 -> slid into the box, operator INSIDE)
        r = c3.move(ctx, (pa[0], pa[1], st['box']['rim_z'] + DUMP_ABOVE_RIM), st['ryaw'], 4.0, max_jump=1.0); r.pop('ob', None)
        row, _, _ = c.pps.last_row(ctx); q = list(row[9:38]); k = c.J.index('right_wrist_roll_joint')
        lo, hi = -1.9722 + 0.08, 1.9722 - 0.08
        b = st['box']; to_c = np.array([(b['x'][0] + b['x'][1]) / 2, (b['y'][0] + b['y'][1]) / 2]) - np.array(pa[:2])
        best_d, best_s = None, -9
        for d in (0.8, -0.8, 0.6, -0.6):
            if not lo < q[k] + d < hi:
                continue
            qq = list(q); qq[k] += d
            R, _ = c.pps.M.fk(qq).frame_pose('right_palm'); back = -np.array(R)[:, 1]
            sc = float(back[:2] @ to_c) / (np.linalg.norm(to_c) + 1e-6)
            if sc > best_s:
                best_d, best_s = d, sc
        if best_d is not None:
            c.pps.ramp(ctx, {'right_wrist_roll_joint': q[k] + best_d}, 2.5)
            time.sleep(2.0)
            c.pps.ramp(ctx, {'right_wrist_roll_joint': q[k]}, 2.0)
        r, ob = raise_open(ctx, pa, st['ryaw'], 4.0)
        n, palm = near_hand(ob, st); tries.append(n)
    if n > HANG_PX:
        out(False, 'plush still on the hand after opening: do NOT go home (it drags the box); look, then ask the operator',
            near_hand_px=tries, palm=palm, frame_open=ob1['camera']['frame_id'], frame=ob['camera']['frame_id'])
    out(True, near_hand_px=tries, frame_open=ob1['camera']['frame_id'], up_ok=r['ok'], raise_=r.get('raise'),
        frame=ob['camera']['frame_id'])


def retreat(ctx, st):
    """Cartesian retreat at the CURRENT waist: straight up to z >= RETREAT_Z, then back toward the body at that height.
    Run 10 (d65273): home's single joint-space ramp from the release pose (waist .6/.15) to a waist-0 waypoint dipped the
    hand 30 cm (index at z .07 beside the box wall), dragged the box with the plush in it off the table."""
    row, _, _ = c.pps.last_row(ctx)
    pa = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])
    yaw = st.get('ryaw', st.get('gyaw', 0.0))
    hist = []
    for tgt in ((pa[0], pa[1], max(pa[2], RETREAT_Z)), (0.5 * (pa[0] + RETREAT_X), pa[1], max(pa[2], RETREAT_Z)),
                (RETREAT_X, pa[1], max(pa[2], RETREAT_Z))):
        if tgt[0] > pa[0] + 1e-3:
            continue
        r = c3.move(ctx, tgt, yaw, 3.0, max_jump=0.8); r.pop('ob', None)
        hist.append([np.round(tgt, 3).tolist(), r.get('palm'), r.get('why', '')])
        if not r['ok']:
            break
    return hist


def step_home(ctx, st):
    """Hand high, waist 0, then both arms tucked like the left (back and up, clear of the table edge), hands open."""
    ctx.value('g1_hand_shape', side='right')
    retreat(ctx, st)
    c.pps.ramp(ctx, {'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0, 'waist_pitch_joint': 0.0}, 4.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, dict(R({'shoulder_pitch': 0.1, 'shoulder_roll': -0.25, 'shoulder_yaw': 0, 'elbow': -0.9, 'wrist_roll': 0,
                            'wrist_pitch': 0, 'wrist_yaw': 0}), waist_yaw_joint=0.0, waist_pitch_joint=0.0, waist_roll_joint=0.0),
               4.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.8}), 3.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, R({'shoulder_roll': -0.2, 'elbow': -0.5}), 2.5, hands=(0.0, 0.0))
    # no TUCK (sp .45 / el -.5) on the way: its fingertips reach x .275, z .11-.16 = inside a box whose near wall is < .28 m,
    # and the move to LOOK then hooked it off the table (eval 4f16bd trial 2). Guard/sp .8 -> LOOK keeps every link x <= .203 (FK).
    blocked = to_look(ctx)   # end with the arms down behind the edge: a reset then cannot lay the plush into tucked hands (run 6)
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'run_home')
    f = c.fingers(ob, 'right') + c.fingers(ob, 'left')
    j = ob['proprio']['joint_positions_rad']
    ok = ob['tilt_deg'] < 6 and max(f) < 0.5
    out(ok, '' if ok else 'tilt or fingers not open', tilt=ob['tilt_deg'], fingers_max=round(max(f), 2), blocked_in_look=blocked,
        waist=[round(j[2], 3), round(j[5], 3), round(j[8], 3)], frame=ob['camera']['frame_id'])


def step_verify(ctx, st):
    """Fresh frame (arms tucked): cream points inside the box footprint vs outside on the table."""
    ob, path = c.observe(ctx, 'run_verify')
    P = p.load(path); pel, lev = p.cloud(P); tz = st['table_z']
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 40) & (hsv[..., 1] < 140) & (hsv[..., 2] > 120)
    m = cream & (lev[..., 2] > tz + 0.015) & (lev[..., 2] < tz + 0.30) & (P['depth'] > 0) & (lev[..., 0] < 0.8)
    b = st['box']; pts = lev[m]
    ins = ((pts[:, 0] > b['x'][0] - 0.01) & (pts[:, 0] < b['x'][1] + 0.01) & (pts[:, 1] > b['y'][0] - 0.01) & (pts[:, 1] < b['y'][1] + 0.01))
    over = os.path.join(ROOT, 'output', f"run_verify_{ob['step_id']}.jpg")
    p.draw_box(P, b['x'][0], b['x'][1], b['y'][0], b['y'][1], b['rim_z'], tz, over)
    n_in, n_out = int(ins.sum()), int((~ins).sum())
    ok = n_in > VERIFY_MIN_IN_PX        # a sitting plush's raised head projects outside the footprint (runs 3, 8): look + operator decide
    out(ok, '' if ok else 'little or no plush inside the box footprint: look',
        cream_in_box=n_in, cream_outside=n_out, tilt=ob['tilt_deg'], overlay=over, frame=ob['camera']['frame_id'],
        look='confirm with the operator')


def step_abort(ctx, st):
    """Open (flat), raise the palm straight up from where the COMMAND is, then home."""
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    row, meas, _ = c.pps.last_row(ctx)
    pa = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])
    yaw = st.get('gyaw', 0.0)
    ob = None
    for dz in (0.04, 0.08, 0.14):
        r = c3.move(ctx, (pa[0], pa[1], pa[2] + dz), yaw, 2.5, max_jump=1.6); ob = r.pop('ob', None) or ob
        if not r['ok']:
            break
    # run 12 (d65273): the open hand (yaw -45) lifted the plush with it and home's retreat dragged it off the front edge
    n, palm = near_hand(ob or ctx.value('g1_observe'), st)
    if n > HANG_PX:
        r = c3.move(ctx, (pa[0], pa[1], pa[2] + 0.01), yaw, 4.0, max_jump=1.6); r.pop('ob', None)   # set it back down on the table
        time.sleep(1.0)
        r = c3.move(ctx, (pa[0], pa[1], pa[2] + 0.08), yaw, 5.0, max_jump=1.6); ob = r.pop('ob', None)
        n, palm = near_hand(ob or ctx.value('g1_observe'), st)
        if n > HANG_PX:
            out(False, 'abort: plush still on the open hand - do NOT go home (it drags the plush off the table); ask the operator',
                near_hand_px=n, palm=palm)
    step_home(ctx, st)


def step_pick(ctx, st):
    """approach, grasp, lift (from the last `scene`); on an empty close or a slip: abort, re-perceive (the press moves the plush) and try once
    more (run 5: slip at z .10, retry held). Stops on any scene/approach failure (layout not doable from this stance)."""
    for attempt in (1, 2):
        if attempt == 2 or not st.get('waist'):
            step_scene(ctx, st); st = load()
        if attempt == 2 or not st.get('hovered'):
            step_approach(ctx, st); st = load()
        try:
            step_grasp(ctx, st); st = load()
            step_lift(ctx, st)
            return
        except Fail as e:
            if attempt == 2:
                raise
            print(json.dumps({'retry': str(e)}), flush=True)
            step_abort(ctx, load())
            st = load()


STEPS = dict(tuck=lambda ctx, st: step_tuck(ctx, st), look=lambda ctx, st: to_look(ctx), scene=step_scene, approach=step_approach, shift=step_shift,
             grasp=step_grasp, lift=step_lift, carry=step_carry, release=step_release, home=step_home,
             verify=step_verify, abort=step_abort, pick=step_pick)
CHAINS = dict(place=['release', 'home', 'verify'])

if __name__ == '__main__':
    from api.runtime import Context
    st = load()
    args = sys.argv[2:]
    with Context() as ctx:
        names = [n for s_ in sys.argv[1].split(',') for n in CHAINS.get(s_, [s_])]
        for s in names:
            try:
                if s == 'shift':
                    STEPS[s](ctx, st, *args)
                else:
                    STEPS[s](ctx, st)
                if s in ('tuck', 'look'):
                    out(True, step=s)
            except Fail:
                sys.exit(1)
            st = load() if s != 'scene' else st
