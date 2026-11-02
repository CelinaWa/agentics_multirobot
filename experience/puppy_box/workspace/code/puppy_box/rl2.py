"""Box LEFT-ahead, puppy RIGHT beside the hip (head left), right hand only (session ce59d5). Library pb3/lr steps with
this layout's constants: waist yaw 0 + roll 0.2 + pitch 0.12 set once, palm yaw 0, higher lift/carry (rim z 0.11).
python code/puppy_box/rl2.py <step>   steps: check, waist, above, hover, grasp, lift, carry, release, home"""
import sys, os, json, time
import numpy as np
L = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../skill-library/skills/plush-pick-place-box-cross-right-hand/code'))
sys.path.insert(0, L)
import pb3, lr, reach2
from pb3 import c, move, report, load, save

WAIST = dict(waist_yaw_joint=0.0, waist_roll_joint=0.2, waist_pitch_joint=0.12)
pb3.GRASP_YAW = pb3.RELEASE_YAW = lr.GRASP_YAW = 0.0
pb3.LIFT_Z, pb3.CARRY_Z = 0.32, 0.34
pb3.STALL = float(os.environ.get('STALL', pb3.STALL))
lr.RELEASE_YAW = 0.0


def step_check(ctx, st):
    q = pb3.cmd_row(ctx)
    q[2], q[5], q[8] = WAIST['waist_yaw_joint'], WAIST['waist_roll_joint'], WAIST['waist_pitch_joint']
    gx, gy = st['grasp']; top = st['top']
    pts = {'press': (gx, gy, top - 0.06), 'above': (gx, gy, 0.30), 'lift': (gx + 0.015, gy, pb3.LIFT_Z),
           'release': tuple(st['release'])}
    errs = {k: round(reach2.best(q, v, 0.0)[0], 4) for k, v in pts.items()}
    report(max(errs.values()) < 0.01, '', errs=errs)


def step_waist(ctx, st):
    c.pps.ramp(ctx, WAIST, 4.0)
    tilts = []
    for k in range(2):
        time.sleep(1.5)
        ob, _ = c.observe(ctx, f'rl2_waist{k}')
        tilts.append(ob['tilt_deg'])
    j = ob['proprio']['joint_positions_rad']
    report(max(tilts) < 6, f'tilt {tilts}', tilts=tilts, waist_meas=[round(j[2], 3), round(j[5], 3), round(j[8], 3)],
           frame=ob['camera']['frame_id'])


def step_above(ctx, st):
    gx, gy = st['grasp']
    q0 = np.array(pb3.cmd_row(ctx))
    t = reach2.best(list(q0), (gx, gy, 0.30), 0.0)[1]
    q1 = q0.copy()
    for k, v in t.items():
        q1[c.J.index(k)] = v
    low = lr.path_min_z(q0, q1)
    if low[0] < st['top'] + 0.05:
        report(False, f'path too low: {low}')
    c.pps.ramp(ctx, t, 5.0)
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'rl2_above')
    palm = c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm']
    report(ob['tilt_deg'] < 6, f"tilt {ob['tilt_deg']}", palm=palm, low=low, frame=ob['camera']['frame_id'])


STEPS = dict(check=step_check, waist=step_waist, above=step_above, hover=lr.step_hover, grasp=pb3.step_grasp,
             lift=pb3.step_lift, carry=pb3.step_carry, release=pb3.step_release, home=pb3.step_home)

if __name__ == '__main__':
    st = load()
    from api.runtime import Context
    with Context() as ctx:
        try:
            STEPS[sys.argv[1]](ctx, st, *sys.argv[2:])
        except pb3.Fail:
            sys.exit(1)
