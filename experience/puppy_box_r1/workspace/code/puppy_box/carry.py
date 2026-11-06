"""Carry steps for a puppy already held after `chain.py go ... lift_only` (session cf6e88, sim).
Each step is one short call; look at the printed frame before the next one.

  python code/puppy_box/carry.py look [pitch]     head to pitch (default .15), observe: hand, width, cream body pts, rim box
  python code/puppy_box/carry.py yaw <deg> [step] rotate the approach yaw (servo off) in <= step-deg increments to deg
  python code/puppy_box/carry.py move dx dy dz    move the held hand by (dx,dy,dz), servo off, same approach
  python code/puppy_box/carry.py walk <cmd>       one r1_walk holding; width before/after
  python code/puppy_box/carry.py waist <rad> [secs] slow waist yaw (+ left) while holding
  python code/puppy_box/carry.py release          open, wait 2 s, observe (head as is)
  python code/puppy_box/carry.py push             closed gripper push-in of a puppy left on the box front rim
  python code/puppy_box/carry.py out [back]       open, hand up/back, out, HOME, head .6, optional walk back
State: output/chain_state.json keys hold_pos, hold_yaw, hold_apz.
"""
import sys, json, math, time
from pathlib import Path
import numpy as np
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
import chain
from chain import ctx, log, save, load, guarded, body_points, rim_box, HOME, HELD_MIN

STEP_MAX_JUMP = 0.45


def appr_of(yaw_deg, apz):
    a = math.radians(yaw_deg)
    return (math.cos(a), math.sin(a), apz), (-math.sin(a), math.cos(a), 0.0)


def status():
    st = ctx.value('r1_status')
    return st['proprio']['grasp_points_pelvis_m']['right'], st['proprio']['gripper_width_m']['right'], st


def look(pitch=0.15):
    ctx.value('r1_head', pitch=pitch, secs=1.0)
    o = ctx.value('r1_observe')
    fid = o['camera']['frame_id']
    s = load()
    zt = s.get('table_z', -0.033)
    hand = o['proprio']['grasp_points_pelvis_m']['right']
    bp = body_points(fid, zt, hand)
    rb = rim_box(fid, zt)
    out = {'frame': fid, 'pitch': pitch, 'hand': hand, 'width': o['proprio']['gripper_width_m']['right'],
           'tilt': o['tilt_deg'], 'body': bp, 'rim': rb, 'waist_yaw': round(o['proprio']['joint_positions_rad'][5], 3)}
    log(step='look', **out)
    return out


def hold_move(pos, yaw, apz, secs=1.5):
    ap, fi = appr_of(yaw, apz)
    r = guarded(pos, ap, fi, w=0.0, secs=secs, max_jump=STEP_MAX_JUMP, servo=False)
    if r is None:
        return None
    save(hold_pos=[float(v) for v in pos], hold_yaw=yaw, hold_apz=apz)
    w = r['gripper_width_m']['right']
    if w < HELD_MIN:
        log(step='SLIP', width=w); sys.exit(3)
    return r


def main():
    step = sys.argv[1]
    a = [float(v) for v in sys.argv[2:]]
    s = load()
    pos = s.get('hold_pos'); yaw = s.get('hold_yaw', 2.0); apz = s.get('hold_apz', 0.5)
    if step == 'look':
        look(a[0] if a else 0.15)
    elif step == 'yaw':
        target = a[0]; inc = a[1] if len(a) > 1 else 5.0
        n = max(1, int(math.ceil(abs(target - yaw) / inc)))
        y0 = yaw
        for i in range(1, n + 1):
            yw = y0 + (target - y0) * i / n
            r = hold_move(pos, yw, apz, secs=1.2)
            if r is None:
                log(step='yaw_stopped', at=round(yw, 1), kept=load().get('hold_yaw')); break
        h, w, _ = status()
        log(step='yaw_done', yaw=load().get('hold_yaw'), hand=h, width=w)
    elif step == 'move':
        p = [pos[0] + a[0], pos[1] + a[1], pos[2] + a[2]]
        r = hold_move(p, yaw, apz, secs=2.0)
        h, w, _ = status()
        log(step='move_done', ok=r is not None, hand=h, width=w)
    elif step == 'waist':
        # slow waist yaw (+ = left): swings the held hand/body sideways without changing the arm joints
        h0, w0, _ = status()
        r = ctx.value('r1_move_joints', joints={'waist_yaw_joint': a[0]}, secs=a[1] if len(a) > 1 else 2.5)
        time.sleep(0.5)
        h1, w1, st = status()
        log(step='waist', cmd=a[0], measured=round(st['proprio']['joint_positions_rad'][5], 3), w_before=w0, w_after=w1,
            hand_before=h0, hand_after=h1, tilt=r.get('tilt_deg'))
        if w1 < HELD_MIN:
            log(step='SLIP', at='waist'); sys.exit(3)
    elif step == 'walk':
        h0, w0, _ = status()
        wr = ctx.value('r1_walk', forward_m=a[0])
        h1, w1, _ = status()
        log(step='walk', cmd=a[0], walked=wr.get('walked_m'), pelvis=wr.get('pelvis'), tilt=wr.get('tilt_deg'),
            w_before=w0, w_after=w1, hand_before=h0, hand_after=h1)
        if w1 < HELD_MIN:
            log(step='SLIP', at='walk'); sys.exit(3)
    elif step == 'release':
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        time.sleep(2.0)
        o = ctx.value('r1_observe')
        log(step='released', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'])
    elif step == 'push':
        # closed gripper, waypoint behind the edge at z .16, push forward along x at rim + .04
        y = a[0] if a else -0.20
        x_end = a[1] if len(a) > 1 else 0.52
        z = a[2] if len(a) > 2 else 0.16
        ctx.value('r1_gripper', arm='right', width_m=0.0, secs=0.8)
        for p, secs in (([0.20, -0.28, z + 0.06], 2.5), ([0.36, y, z], 2.0), ([x_end, y, z], 3.0)):
            r = guarded(p, (1, 0, 0), (0, 0, 1), w=0.0, secs=secs, max_jump=3.0)
            if r is None:
                log(step='push_refused', at=p); break
        o = ctx.value('r1_observe')
        log(step='pushed', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'],
            tilt=o['tilt_deg'])
    elif step == 'out':
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        h, w, _ = status()
        guarded([h[0] - 0.04, h[1], min(h[2] + 0.05, 0.45)], (0.9, 0.0, 0.4), (0, 1, 0), secs=1.5, max_jump=3.0)
        guarded([0.20, -0.36, max(0.12, min(h[2], 0.30))], (0.9, -0.3, 0.3), (0, 0, 1), secs=2.0, max_jump=3.5)
        guarded((0.14, -0.40, 0.08), (0.9, -0.3, 0.3), (0, 0, 1), secs=2.0, max_jump=3.5)
        r = ctx.value('r1_move_joints', joints=HOME, secs=2.0)
        ctx.value('r1_head', pitch=0.6, secs=1.0)
        back = a[0] if a else 0.0
        if back > 0:
            wr = ctx.value('r1_walk', forward_m=-back)
            log(step='walk_back', walked=wr.get('walked_m'), tilt=wr.get('tilt_deg'))
        o = ctx.value('r1_observe')
        log(step='home', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'],
            width=o['proprio']['gripper_width_m']['right'], tilt=o['tilt_deg'])
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
