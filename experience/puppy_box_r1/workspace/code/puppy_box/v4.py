"""puppy_box_r1 v4 (session 2fbfb7, sim): TASK.md engineering recipe = LOW lift + several short walks.

Run after `chain.py look` and `chain.py pre` (leg-axis measurement, pregrasp along the leg):

  python code/puppy_box/v4.py go [past_m] [dz_m] [in_m] [max_walks] [lower_m] [eng] [probe_deg] [chainlift]
      probe_deg (7th arg): close at yaw offsets 0, d, 2d about the measured leg axis first, grasp at the narrowest
      eng=1: the engineering chain - cmd z .265, no look, no raise, walk at once (in_m = torso centre past the wall)
      re-localize the paw, advance along the leg axis, close (width 0), servo-OFF arc lift to hand z ~.265 keeping
      the leg-yaw approach (pitch 0 -> .5), head-up look at the hanging body, then r1_walk cmd .07 repeatedly (no arm
      or waist move) until the body centroid is in_m past the box front wall (dead reckoning on walked_m), release.
      Exit 0 released, 2 stopped before contact, 3 slip/miss.
  python code/puppy_box/v4.py out [back_m]   open, hand up + back, walk back, HOME, head .6, observe
  python code/puppy_box/v4.py check          head .6 observe + depth points inside the box volume (dead reckoned)
State: output/chain_state.json (shared with chain.py). Frame ids are printed for checkpoint review.
"""
import sys, json, math, time
from pathlib import Path
import numpy as np
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
import chain
from chain import ctx, log, save, load, guarded, body_points, relocalize, perceive, choose_approach, HOME, HELD_MIN

# (along-leg advance m, rise m, approach pitch z-component): +1.5 cm per 4 cm over the first ~10 cm, then +1 cm per 4 cm
LIFT_LOW = [(0.015, 0.04, 0.1), (0.015, 0.04, 0.2), (0.008, 0.025, 0.3), (0.01, 0.04, 0.4), (0.01, 0.04, 0.45),
            (0.01, 0.04, 0.5), (0.005, 0.04, 0.5), (0.005, 0.03, 0.5)]
Z_MAX = 0.265          # MEASURED hand z (pelvis) to reach - no higher: a higher raise makes the body hang and swing
Z_CMD_MAX = 0.31       # commanded cap (servo off, the loaded hand sags 2.5-3.5 cm below the command; st1)
RIM_CLEAR = 0.03       # wanted body-bottom clearance over the box rim before walking
RIM_H = 0.145
WALK_CMD = 0.07
EDGE_MIN = 0.12        # never walk so the front edge is closer than this to the pelvis (dead reckoned)
LOWER = 0.05
ENG = False
PROBE = []
CHAIN = False
ABORT_FALL = 0.004   # abort the lift (exit 4) if the width falls this much below its peak in steps 0-5 (st18 ok: -.0006)
REGRASP = 0      # st6, st13: re-grasping 2 cm further up (hip) made it worse
W_CLEAN_MAX = 0.037
HEAD_LOOK = 0.45     # head pitch for the hang look (.25 cut the lowest part off, st3)
Z_RAISE_MAX = 0.44     # cmd; the hanging body reaches ~.26 below the hand (st4: zmin .095 at hand .358)
REWALK_RAISE = 0.03
HEAD_CARRY = 0.25      # head pitch during the carry (sees the hanging body)


HOME_BOTH = dict(HOME, left_shoulder_pitch_joint=0.35, left_shoulder_roll_joint=0.18, left_shoulder_yaw_joint=0.0,
                 left_elbow_joint=0.87, left_wrist_roll_joint=0.0)


def width():
    return ctx.value('r1_status')['proprio']['gripper_width_m']['right']


def go(a):
    past = a[0] if len(a) > 0 else 0.045     # st14/st15: ball partly in the pads -> width RISES under load = stopper
    dzg = a[1] if len(a) > 1 else -0.004
    IN = a[2] if len(a) > 2 else 0.10
    max_walks = int(a[3]) if len(a) > 3 else 5
    global LOWER, ENG, Z_CMD_MAX, PROBE, CHAIN
    CHAIN = len(a) > 7 and a[7] > 0        # chain.py LIFT profile (z ~.43-.46, yaw -> 2 deg; walks held 6/12 in 7c03aa)
    PROBE = [0.0, a[6], 2 * a[6]] if len(a) > 6 and a[6] != 0 else []
    LOWER = a[4] if len(a) > 4 else 0.05
    ENG = len(a) > 5 and a[5] > 0          # engineering chain (TASK.md + operator notice): cmd z .265, no look/raise
    if ENG:
        Z_CMD_MAX = 0.265
    s = load()
    ld = np.array(s['leg']); f = s['fing']
    o = perceive('grasp0')
    P, how = relocalize(o, s)
    if P is None:
        log(step='STOP', why=how); sys.exit(2)
    P = np.array(P); hand = np.array(o['hand'])
    lat = float((P[:2] - hand[:2]) @ np.array([-ld[1], ld[0]]))
    log(step='target', P=P.round(3).tolist(), how=how, lateral=round(lat, 4))
    if abs(lat) > 0.02:
        log(step='STOP', why='lateral misalignment %.3f > .02: re-run pre' % lat); sys.exit(2)
    yaw_m = math.atan2(ld[1], ld[0])
    if PROBE:     # close at several approach yaws about the measured leg axis, keep the narrowest (= cleanest) clamp
        res = []
        for off in PROBE:
            yw = yaw_m + math.radians(off)
            ldo = np.array([math.cos(yw), math.sin(yw)]); fo = [-ldo[1], ldo[0], 0.0]
            Go = [P[0] + past * ldo[0], P[1] + past * ldo[1], P[2] + dzg]
            ap = choose_approach(Go, math.degrees(yw), fo)
            if guarded(Go, ap, fo, secs=1.2) is None:
                continue
            g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
            wv = g['gripper']['measured_m']
            res.append((wv if wv > HELD_MIN else 9.0, off))
            log(step='probe', off=off, width=wv)
            ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
            back = [Go[0] - 0.04 * ldo[0], Go[1] - 0.04 * ldo[1], Go[2]]
            guarded(back, ap, fo, secs=1.0)
        if res:
            best = min(res)
            yaw_m += math.radians(best[1])
            ld = np.array([math.cos(yaw_m), math.sin(yaw_m)]); f = [-ld[1], ld[0], 0.0]
            log(step='probe_best', off=best[1], width=best[0])
    G = [P[0] + past * ld[0], P[1] + past * ld[1], P[2] + dzg]
    yaw0 = math.atan2(ld[1], ld[0])
    appr_g = choose_approach(G, math.degrees(yaw0), f)
    if guarded(G, appr_g, f, secs=1.5) is None:
        sys.exit(2)
    g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
    w = g['gripper']['measured_m']
    log(step='closed', width=w, past=past)
    for k in range(0 if PROBE else REGRASP):       # TASK.md: >= .040 = paw ball / diagonal -> open, re-grasp 2 cm further up the shaft
        if w < W_CLEAN_MAX:
            break
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
        past += 0.02
        G = [P[0] + past * ld[0], P[1] + past * ld[1], P[2] + dzg]
        if guarded(G, appr_g, f, secs=1.2) is None:
            sys.exit(2)
        g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
        w = g['gripper']['measured_m']
        log(step='regrasp', width=w, past=round(past, 3))
    if w < HELD_MIN:
        log(step='MISS'); sys.exit(3)
    w_close = w
    w_peak = w       # st17/R3: widths rose for 3 steps, then fell .006-.007 at step 4 (z ~.2) -> later slip
    # low arc lift, servo OFF, approach yaw kept on the leg axis, pitch rising
    x, y, z = g['grasp_points_pelvis_m']['right']
    last = (x, y, z)
    prof = chain.LIFT if CHAIN else LIFT_LOW
    yaw_end = math.radians(chain.YAW_END) if CHAIN else yaw0
    for i, (da, dz, az) in enumerate(prof):
        nz = z + dz if CHAIN else min(z + dz, Z_CMD_MAX)
        x, y, z = x + da * ld[0], y + da * ld[1], nz
        if CHAIN and i == len(prof) - 1:
            x += 0.02
        t_ = min(1.0, max(0.0, (i - 2) / (len(prof) - 3))) if CHAIN else 0.0
        yw = yaw0 + (yaw_end - yaw0) * t_      # chain.py (7c03aa): rotate the fingers toward y so the body hangs ahead
        ap = (math.cos(yw), math.sin(yw), az); fi = (-math.sin(yw), math.cos(yw), 0.0)
        r = guarded((x, y, z), ap, fi, w=0.0, secs=1.2, max_jump=0.45, servo=False)
        if r is None and CHAIN:     # st10: step 9 refused (ik .032) -> body too low; retry a half step
            x, y, z = (last[0] + x) / 2, (last[1] + y) / 2, (last[2] + z) / 2
            r = guarded((x, y, z), ap, fi, w=0.0, secs=1.2, max_jump=0.45, servo=False)
        if r is None:
            x, y, z = last; break
        last = (x, y, z)
        wi = r['gripper_width_m']['right']
        log(step='lift', i=i, cmd=[round(x, 3), round(y, 3), round(z, 3)], hand=r['grasp_points_pelvis_m']['right'],
            width=wi)
        if wi < HELD_MIN:
            log(step='SLIP', at=i); sys.exit(3)
        w_peak = max(w_peak, wi)
        if ABORT_FALL and i <= 5 and wi < w_peak - ABORT_FALL:
            # st11/st12/st16: a width FALLING during the lift = the leg slides out (no paw-ball stopper) -> put it back
            # down while it is still low, open, back off along the leg; the caller re-runs look/pre/go (exit 4)
            guarded(G, ap, fi, w=0.0, secs=1.5, max_jump=0.6, servo=False)
            ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
            guarded([G[0] - 0.08 * ld[0], G[1] - 0.08 * ld[1], G[2] + 0.01], ap, fi, w=0.07, secs=1.5, max_jump=0.8)
            log(step='ABORT_LIFT', at=i, width=wi, w_close=w_close); sys.exit(4)
        if not CHAIN and (r['grasp_points_pelvis_m']['right'][2] >= Z_MAX or z >= Z_CMD_MAX - 1e-6):
            break
    save(hold_pos=[float(v) for v in last], hold_yaw=math.degrees(yaw_end), hold_apz=prof[-1][2])
    carry(s, last, yaw_end, prof[-1][2], IN, max_walks, look=not ENG)


def carry(s, last, yaw0, apz, IN, max_walks, look=True):
    """holding the puppy (servo-off command `last`, approach yaw0 rad / apz): look, raise until the body clears the
    rim, walk until the body rear is IN past the box front wall, lower LOWER, open."""
    if look:     # head-down look at the hanging body (one frame, no arm motion)
        ctx.value('r1_head', pitch=HEAD_LOOK, secs=0.8)
        o2 = ctx.value('r1_observe')
        fid = o2['camera']['frame_id']; hand2 = o2['proprio']['grasp_points_pelvis_m']['right']
        w2 = o2['proprio']['gripper_width_m']['right']
        bp = body_points(fid, s['table_z'], hand2)
    else:        # engineering chain: walk at once (no pause, no raise); torso centre ~hand x + .03 (st1)
        st = ctx.value('r1_status')['proprio']
        hand2 = st['grasp_points_pelvis_m']['right']; w2 = st['gripper_width_m']['right']; bp = None; fid = None
    log(step='lifted', frame=fid, hand=hand2, width=w2, body=bp)
    if w2 < HELD_MIN:
        log(step='SLIP', at='after lift'); sys.exit(3)
    rim_z = s['table_z'] + RIM_H
    for k in range(5 if look else 0):      # body bottom must clear the rim before walking into the box front wall (st1, st3 dropped)
        if not bp or bp['zmin'] >= rim_z + RIM_CLEAR or last[2] + 0.025 > Z_RAISE_MAX:
            break
        nl = (last[0], last[1], last[2] + 0.025)
        r = guarded(nl, (math.cos(yaw0), math.sin(yaw0), apz), (-math.sin(yaw0), math.cos(yaw0), 0.0), w=0.0, secs=1.2,
                    max_jump=0.45, servo=False)
        if r is None:
            break
        last = nl
        o2 = ctx.value('r1_observe'); fid = o2['camera']['frame_id']; hand2 = o2['proprio']['grasp_points_pelvis_m']['right']
        bp = body_points(fid, s['table_z'], hand2)
        log(step='raise', frame=fid, cmd=[round(v, 3) for v in nl], hand=hand2, width=o2['proprio']['gripper_width_m']['right'],
            body=bp, rim_z=round(rim_z, 3))
        if o2['proprio']['gripper_width_m']['right'] < HELD_MIN:
            log(step='SLIP', at='raise'); sys.exit(3)
    save(hold_pos=[float(v) for v in last])
    edge0 = (s.get('edge') or {}).get('dist') or 0.39
    bx = s.get('box') or {}
    front = bx['x'][0] if bx.get('x') and 0.45 < bx['x'][0] < 0.75 else edge0 + 0.22
    offx = -0.035 if look else (0.06 if CHAIN else 0.03)    # body xmin - hand x (st2: -.033); torso centre - hand x (st1: +.026)
    if bp and abs(bp['xmin'] - hand2[0]) < 0.12:
        offx = bp['xmin'] - hand2[0]
    need = front + IN - (hand2[0] + offx)      # IN: body REAR (xmin) this far past the front wall
    need = min(need, edge0 - EDGE_MIN)
    if max_walks < 0:          # exit holding (manual carry.py steps next)
        save(hold_pos=[float(v) for v in last], hold_yaw=math.degrees(yaw0), hold_apz=apz)
        log(step='holding_exit', hand=hand2, width=w2); sys.exit(0)
    log(step='plan_walk', front=round(front, 3), edge0=round(edge0, 3), hand_x=round(hand2[0], 3), offx=round(offx, 3),
        need=round(need, 3))
    walked = 0.0; n = 0; frames = []
    z_hold = hand2[2]
    while walked < need - 0.015 and n < max_walks:
        n += 1
        wr = ctx.value('r1_walk', forward_m=WALK_CMD)
        wk = float(wr.get('walked_m') or 0.0); walked += wk
        st = ctx.value('r1_status')['proprio']
        wN = st['gripper_width_m']['right']; hp = st['grasp_points_pelvis_m']['right']
        log(step='walk', n=n, walked=round(wk, 3), total=round(walked, 3), tilt=wr.get('tilt_deg'), width=wN, hand=hp,
            pelvis=wr.get('pelvis'))
        if wN < HELD_MIN:
            log(step='SLIP', at='walk %d' % n); sys.exit(3)
        if not ENG and walked < need - 0.015 and n < max_walks and hp[2] < z_hold - REWALK_RAISE and last[2] + 0.02 <= Z_RAISE_MAX + 0.02:
            nl = (last[0], last[1], last[2] + 0.02)     # the hand sags under load on each walk: re-raise a little
            r = guarded(nl, (math.cos(yaw0), math.sin(yaw0), apz), (-math.sin(yaw0), math.cos(yaw0), 0.0), w=0.0,
                        secs=1.2, max_jump=0.45, servo=False)
            if r is not None:
                last = nl
                log(step='reraise', cmd=[round(v, 3) for v in nl], hand=r['grasp_points_pelvis_m']['right'],
                    width=r['gripper_width_m']['right'])
    o4 = ctx.value('r1_observe'); fid4 = o4['camera']['frame_id']
    hand4 = o4['proprio']['grasp_points_pelvis_m']['right']
    bp4 = body_points(fid4, s['table_z'], hand4)
    while walked < need + 0.03 and bp4 and bp4['centroid'][0] < front - walked + 0.04 and n < max_walks + 1 and edge0 - walked > EDGE_MIN + 0.04:
        # R1: the body hung 9 cm BEHIND the hand and was released in front of the wall -> one more walk
        n += 1
        wr = ctx.value('r1_walk', forward_m=WALK_CMD)
        walked += float(wr.get('walked_m') or 0.0)
        o4 = ctx.value('r1_observe'); fid4 = o4['camera']['frame_id']
        hand4 = o4['proprio']['grasp_points_pelvis_m']['right']
        bp4 = body_points(fid4, s['table_z'], hand4)
        log(step='extra_walk', walked=wr.get('walked_m'), total=round(walked, 3), width=o4['proprio']['gripper_width_m']['right'],
            body=bp4)
        if o4['proprio']['gripper_width_m']['right'] < HELD_MIN:
            log(step='SLIP', at='extra walk'); sys.exit(3)
    log(step='before_release', frame=fid4, hand=hand4, width=o4['proprio']['gripper_width_m']['right'], body=bp4,
        walked=round(walked, 3), front_now=round(front - walked, 3), edge_now=round(edge0 - walked, 3))
    if LOWER > 0:            # lower the body into the opening before letting go (st14 upright on the rim, st15 inside)
        r = None
        for dz in (LOWER, LOWER / 2):          # st17: the full step was refused (jump .49) -> released high -> floor
            nl = (last[0], last[1], last[2] - dz)
            r = guarded(nl, (math.cos(yaw0), math.sin(yaw0), apz), (-math.sin(yaw0), math.cos(yaw0), 0.0), w=0.0,
                        secs=1.5, max_jump=0.6, servo=False)
            if r is not None:
                break
        log(step='lowered', ok=r is not None, hand=r and r['grasp_points_pelvis_m']['right'],
            width=r and r['gripper_width_m']['right'])
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
    time.sleep(1.5)
    o6 = ctx.value('r1_observe')
    save(walked_total=walked, released_frame=o6['camera']['frame_id'], front_now=front - walked,
         edge_now=edge0 - walked)
    log(step='released', frame=o6['camera']['frame_id'], hand=o6['proprio']['grasp_points_pelvis_m']['right'],
        tilt=o6['tilt_deg'])


def out(a):
    back = a[0] if a else 0.2
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
    st = ctx.value('r1_status'); h = st['proprio']['grasp_points_pelvis_m']['right']
    # up first (fingers open, clear of the released puppy), then back toward the body
    guarded([h[0] - 0.02, h[1], min(h[2] + 0.06, 0.36)], (0.9, 0.0, 0.45), (0, 1, 0), w=0.07, secs=1.5, max_jump=3.0)
    guarded([0.22, -0.32, 0.26], (0.9, -0.2, 0.4), (0, 1, 0), w=0.07, secs=2.0, max_jump=3.5)
    guarded([0.14, -0.40, 0.14], (0.9, -0.3, 0.3), (0, 0, 1), w=0.07, secs=2.0, max_jump=3.5)   # out beside the body
    done = 0.0
    for k in range(3):        # st9: a -0.2 walk went only .031 and HOME then drove the hand into the table edge
        if back <= 0 or done >= back - 0.05:
            break
        wr = ctx.value('r1_walk', forward_m=-(back - done))
        done += abs(float(wr.get('walked_m') or 0.0))
        log(step='walk_back', walked=wr.get('walked_m'), total=round(done, 3), tilt=wr.get('tilt_deg'))
    if back > 0 and done < back - 0.05:
        log(step='STOP', why='walk back incomplete (%.3f): hand left out at (.14,-.40,.14), no HOME' % done); sys.exit(2)
    r = ctx.value('r1_move_joints', joints=HOME_BOTH, secs=2.0)
    ctx.value('r1_head', pitch=0.6, secs=1.0)
    o = ctx.value('r1_observe')
    log(step='home', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'],
        width=o['proprio']['gripper_width_m']['right'], tilt=o['tilt_deg'])


def main():
    step = sys.argv[1] if len(sys.argv) > 1 else ''
    a = [float(v) for v in sys.argv[2:]]
    if step == 'go':
        go(a)
    elif step == 'carry':      # after `chain.py go 0 <past> <dz> 1` (lift_only): python v4.py carry [in_m] [max_walks] [lower_m]
        global LOWER
        st = load()
        LOWER = a[2] if len(a) > 2 else LOWER
        carry(st, tuple(st['hold_pos']), math.radians(st.get('hold_yaw', 2.0)), st.get('hold_apz', 0.5),
              a[0] if a else 0.03, int(a[1]) if len(a) > 1 else 4)
    elif step == 'out':
        out(a)
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
