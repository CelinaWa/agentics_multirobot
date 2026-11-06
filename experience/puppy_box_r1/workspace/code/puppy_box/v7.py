"""v7 (session 90b103): ONE process from look to release so the grip never waits between the lift and the carry.

    python code/puppy_box/v7.py go [yaw_off_deg 5] [depth .03] [in_m .10] [lower .06] [step_m .07] [lift_z .265] [max_steps 5]
    then look at the after-frame, ask the operator (question only), python code/puppy_box/v6.py out 0.2

yaw_off: added to the measured leg yaw (the x-slice edge fit reads ~5 deg low; s1: +5 -> width ROSE in the lift).
Exit: 0 released, 2 look STOP, 3 dropped in the carry, 4 lift not held / width falling (hand opened + backed off + HOME).
"""
import sys, json, math, time
from pathlib import Path
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
import v6
from v6 import ctx, log, load, save

BODY_OFF_X = 0.06


def back_off():
    """after an aborted lift: open, hand up/back/out beside the body, HOME (v4 out without the walk back).
    Sim: the puppy drops from the abort height (z .15-.20). Real robot: lower the paw onto the table first."""
    import v4
    v4.out([0.0])


def go(a):
    yoff = a[0] if len(a) > 0 else 0.0
    depth = a[1] if len(a) > 1 else 0.03
    in_m = a[2] if len(a) > 2 else 0.10
    lower = a[3] if len(a) > 3 else 0.06
    step_m = a[4] if len(a) > 4 else 0.07
    lz = a[5] if len(a) > 5 else 0.265
    max_steps = int(a[6]) if len(a) > 6 else 5
    v6.look(['v7'])                      # exits 2 on STOP
    s = load()
    p = list(s['paw'])          # ENGINEERING NOTICE (90b103 s4): pass the paw-ball centre itself, exact measured yaw
    import os
    if os.environ.get('PAW_OVERRIDE'):   # sim diagnostic: the operator-stated stance paw centre
        p = [float(v) for v in os.environ['PAW_OVERRIDE'].split(',')]
    y0 = math.radians(s['leg_yaw'] + yoff)
    args = dict(paw_position=[round(float(v), 4) for v in p], leg_direction=[round(math.cos(y0), 4), round(math.sin(y0), 4)],
                depth_m=depth, lift_z_m=lz)
    log(step='leg_lift_call', yaw_off=yoff, **args)
    r = ctx.value('r1_leg_lift', **args)
    log(step='leg_lift', reply=r)
    save(lift=r, lift_args=args, yaw_off=yoff)
    if not r.get('held') or r.get('width_falling'):
        log(step='ABORT_LIFT', why='not held' if not r.get('held') else 'width falling')
        back_off(); sys.exit(4)
    hand = r['hand_pelvis_m']
    tgt = round(min(0.40, max(0.05, s['wall'] + in_m - (hand[0] + BODY_OFF_X))), 3)
    t0 = time.time()
    c = ctx.value('r1_carry_release', target_forward_m=tgt, lower_m=lower, max_steps=max_steps, step_m=step_m)
    log(step='carry', target=tgt, wall=s['wall'], hand=hand, reply=c, dt_call=round(time.time() - t0, 1))
    save(carry=c, carry_target=tgt)
    o = ctx.value('r1_observe')
    log(step='after', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'],
        width=o['proprio']['gripper_width_m']['right'], tilt=o['tilt_deg'])
    sys.exit(3 if c.get('dropped') else 0)


if __name__ == '__main__':
    go([float(v) for v in sys.argv[2:]])
