"""v8 (session 90b103): own grasp along the measured leg + LOW-TILT servo-off lift, then r1_carry_release.
Hypothesis (s1-s5): r1_leg_lift's approach tilt-up (to .5) turns the hanging body's sideways COM offset into a yaw
torque about the pad normal-free axis -> the width jumps +3-4 mm at lift step 2 and the leg slides out. Keeping the
approach nearly horizontal (az <= .15) leaves only a pivot torque about the finger axis.

    python code/puppy_box/v8.py go [depth .02] [in_m .10] [lower .06] [step_m .10] [az_max .15] [z_cmd .29]
Exit: 0 released, 2 STOP before contact, 3 dropped/miss, 4 lift aborted (put back low, opened, HOME).
"""
import sys, json, math, time
from pathlib import Path
import numpy as np
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
import v6, chain
from v6 import ctx, log, load, save
from chain import guarded, choose_approach

BODY_OFF_X = 0.06
HELD_MIN = 0.015
ABORT_FALL = 0.004
# (advance along the leg m, rise m, approach z component / az_max scale 0..1)
PROFILE = [(0.012, 0.03, 0.0), (0.012, 0.03, 0.0), (0.012, 0.04, 0.33), (0.01, 0.04, 0.53), (0.008, 0.04, 0.67),
           (0.005, 0.04, 0.8), (0.0, 0.04, 1.0), (0.0, 0.04, 1.0)]


def go(a):
    depth = a[0] if len(a) > 0 else 0.02
    in_m = a[1] if len(a) > 1 else 0.10
    lower = a[2] if len(a) > 2 else 0.06
    step_m = a[3] if len(a) > 3 else 0.10
    az_max = a[4] if len(a) > 4 else 0.15
    z_cmd = a[5] if len(a) > 5 else 0.29
    adv = a[6] if len(a) > 6 else 0.06       # s7: level forward advance at the top (shortens the carry walk)
    v6.look(['v8'])
    s = load()
    P = np.array(s['paw']); yaw = math.radians(s['leg_yaw'])
    ld = np.array([math.cos(yaw), math.sin(yaw)]); f = [-ld[1], ld[0], 0.0]
    G = [P[0] + depth * ld[0], P[1] + depth * ld[1], P[2]]
    pre = [G[0] - 0.08 * ld[0], G[1] - 0.08 * ld[1], G[2]]
    ap = choose_approach(pre, s['leg_yaw'], f)
    if guarded(pre, ap, f, w=0.07, secs=2.5, max_jump=2.0) is None:   # empty hand from HOME (jump ~1.05)
        log(step='STOP', why='pregrasp refused'); v6.home(); sys.exit(2)
    ap = choose_approach(G, s['leg_yaw'], f)
    r = guarded(G, ap, f, w=0.07, secs=1.5)
    if r is None:
        log(step='STOP', why='grasp pose refused'); v6.home(); sys.exit(2)
    g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
    w = g['gripper']['measured_m']
    hand = g['grasp_points_pelvis_m']['right']
    log(step='closed', width=w, hand=hand, G=[round(v, 3) for v in G])
    if w < HELD_MIN:
        log(step='MISS'); back_off(G, ap, f, ld); sys.exit(3)
    x, y, z = G          # command frame (servo off from here; the measured hand differs by the servo/sag offset)
    hz = math.hypot(ap[0], ap[1]); apx, apy = ap[0] / hz, ap[1] / hz
    widths = [w]; peak = w; last = None
    for i, (da, dz, k) in enumerate(PROFILE):
        if z + dz > z_cmd + 1e-6:
            break
        x, y, z = x + da * ld[0], y + da * ld[1], z + dz
        api = (apx, apy, az_max * k)
        r = guarded((x, y, z), api, f, w=0.0, secs=1.2, max_jump=0.45, servo=False)
        if r is None:
            log(step='lift_refused', i=i); x, y, z = last if last else G; break
        last = (x, y, z)
        wi = r['gripper_width_m']['right']; widths.append(wi); peak = max(peak, wi)
        log(step='lift', i=i, cmd=[round(x, 3), round(y, 3), round(z, 3)], az=round(az_max * k, 3),
            hand=r['grasp_points_pelvis_m']['right'], width=wi)
        if wi < HELD_MIN:
            log(step='SLIP', at=i, widths=widths); sys.exit(3)
        if i <= 5 and wi < peak - ABORT_FALL:
            log(step='ABORT_LIFT', at=i, widths=widths)
            guarded(G, api, f, w=0.0, secs=1.5, max_jump=0.6, servo=False)   # put the paw back low, then open
            back_off(G, ap, f, ld); sys.exit(4)
    nadv = int(round(adv / 0.03))
    for j in range(nadv):                      # level advance along the leg at the top, preview-guarded
        nx, ny = x + 0.03 * ld[0], y + 0.03 * ld[1]
        api = (apx, apy, az_max)
        r = guarded((nx, ny, z), api, f, w=0.0, secs=1.2, max_jump=0.45, servo=False)
        if r is None:
            log(step='advance_refused', j=j); break
        x, y = nx, ny
        wi = r['gripper_width_m']['right']; widths.append(wi)
        log(step='advance', j=j, cmd=[round(x, 3), round(y, 3), round(z, 3)], hand=r['grasp_points_pelvis_m']['right'], width=wi)
        if wi < HELD_MIN:
            log(step='SLIP', at='advance', widths=widths); sys.exit(3)
    st = ctx.value('r1_status')['proprio']
    handm = st['grasp_points_pelvis_m']['right']; wl = st['gripper_width_m']['right']
    log(step='lifted', hand=handm, width=wl, widths=widths)
    if wl < HELD_MIN:
        log(step='SLIP', at='after lift'); sys.exit(3)
    tgt = round(min(0.40, max(0.05, s['wall'] + in_m - (handm[0] + BODY_OFF_X))), 3)
    try:
        c = ctx.value('r1_carry_release', target_forward_m=tgt, lower_m=lower, max_steps=5, step_m=step_m)
    except RuntimeError as ex:
        log(step='carry_refused', why=str(ex)[:400]); sys.exit(2)
    log(step='carry', target=tgt, wall=s['wall'], hand=handm, reply=c)
    save(carry=c, carry_target=tgt, widths=widths)
    o = ctx.value('r1_observe')
    log(step='after', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'],
        width=o['proprio']['gripper_width_m']['right'], tilt=o['tilt_deg'])
    sys.exit(3 if c.get('dropped') else 0)


def back_off(G, ap, f, ld):
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
    guarded([G[0] - 0.08 * ld[0], G[1] - 0.08 * ld[1], G[2] + 0.02], ap, f, w=0.07, secs=1.5, max_jump=0.9)
    guarded([0.22, -0.26, 0.10], (1, 0, 0.2), (0, 1, 0), w=0.07, secs=1.5, max_jump=1.5)
    v6.home()


if __name__ == '__main__':
    go([float(v) for v in sys.argv[2:]])
