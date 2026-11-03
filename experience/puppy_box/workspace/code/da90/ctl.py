"""Session da90c5 controller: box RIGHT, puppy centre-left head LEFT, right hand only, from a FAR stance (edge ~0.25 m,
walks did not advance) and a LOW table (pelvis z -0.07, puppy top ~+0.004). On top of the library
plush-pick-place-box-same-side v5 primitives (controller3 state file output/pb3_state.json, commanded-row IK moves).
Changes vs v5 for the far stance:
  waist  yaw cmd 0.9 + roll 0.15 set ONCE in its own ramp with the hand high (offline IK: the v5 pressed grasp at
         far-0.07 needs yaw .9 + roll .25 + lean .1; lean slid the pelvis back 1.7 cm), held through the release
  hover  palm x = far - GRASP_BACK (0.10, nearer the robot, fingers still past the far side), y from perceive + DY
  grasp  light press levels top+0.02 .. top-PRESS_MAX, stop at the first palm stall (palm > cmd + 0.03) or tilt > 7, close 4 s
  raise  closed 3 cm raise from the press command, then require a finger base joint < GRIP_BASE_MAX before lifting
  lift   3 cm / 4 s steps to LIFT_Z with the grip check
Then: run_v5.py carry,release,home (arm-only carry on the commanded row keeps this waist).
Usage: PYTHONPATH=. python code/da90/ctl.py restore|prep|waist|hover|grasp|raise|lift  (LOOK at the hover and lift frames)"""
import sys, os, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.params import param
from api.runtime import Context

c = c3.c
WAIST_YAW = float(os.environ.get('WAIST_YAW') or param('grasp.far_stance_waist_yaw_cmd', 0.9))
WAIST_ROLL = param('grasp.far_stance_waist_roll', 0.15)
GRASP_BACK = param('grasp.far_stance_palm_back_from_far_m', 0.10)
PRESS_MAX = param('grasp.far_stance_press_below_top_m', 0.045)
LIFT_Z = param('lift.palm_z_pelvis_m', 0.28)
GRIP_BASE_MAX = 1.3
DY = float(os.environ.get('DY', '0'))


def out(**m):
    print(json.dumps(m, default=float), flush=True)
    if not m.get('ok', True):
        raise c3.Fail(m.get('why', 'check failed'))      # stop the chain at the first failed check


def restore(ctx, st):
    ctx.value('g1_crouch', pelvis_drop_m=0, forward_rad=0.1, arms_home=False, secs=3)
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'da90_restore')
    out(ok=ob['tilt_deg'] < 6, tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])


def prep(ctx, st):
    c3.step_prep(ctx, st)
    try:
        c3.step_perceive(ctx, st, 'left')     # its own gate (grasp x < .36) refuses this stance but saves the state
    except c3.Fail:
        pass
    st = c3.load(); b = st['box']; gx = st['far'] - GRASP_BACK
    ok = 0.20 < gx < 0.36 and abs(st['grasp'][1]) < 0.30 and b['x'][0] < 0.40 and b['centre'][1] < -0.2
    out(ok=ok, why='' if ok else 'puppy or box out of the far-stance range: ask the operator', far=st['far'],
        grasp_y=st['grasp'][1], top=st['top'], box_x0=b['x'][0], release=st['release'])
    if not ok:
        raise c3.Fail('range')


def waist(ctx, st):
    c.pps.ramp(ctx, {'waist_yaw_joint': WAIST_YAW, 'waist_roll_joint': WAIST_ROLL}, 5.0)
    tilts = []
    for k in range(2):
        time.sleep(1.5)
        ob, _ = c.observe(ctx, f'da90_waist{k}')
        tilts.append(ob['tilt_deg'])
    j = ob['proprio']['joint_positions_rad']
    st['waist_cmd'] = WAIST_YAW; c3.save(st)
    out(ok=max(tilts) < 7, tilts=tilts, waist_meas=[round(j[2], 3), round(j[5], 3), round(j[8], 3)], frame=ob['camera']['frame_id'])


def hover(ctx, st):
    gx = st['far'] - GRASP_BACK; gy = st['grasp'][1] + DY; top = st['top']
    st['grasp'] = [gx, gy]; c3.save(st)
    c.ppg.set_pinch_shape(ctx, 'right')
    hist = []
    for z, secs in ((top + 0.13, 5.0), (top + 0.06, 3.0)):
        r = c3.move(ctx, (gx, gy, z), c3.GRASP_YAW, secs, max_jump=1.6); r.pop('ob', None)
        hist.append([round(z, 3), r.get('palm'), r.get('tilt')])
        if not r['ok']:
            return out(ok=False, why=r['why'], hist=hist)
    out(ok=True, grasp=[gx, gy], hist=hist, frame=r['frame'])


def grasp(ctx, st):
    gx, gy = st['grasp']; top = st['top']; hist = []; z = None
    for d in (0.02, 0.0, -0.015, -0.03, -PRESS_MAX):
        zc = top + d
        r = c3.move(ctx, (gx, gy, zc), c3.GRASP_YAW, 2.5); r.pop('ob', None)
        if not r['ok']:
            break                                  # deeper level out of reach: close where we are
        z = zc; hist.append([round(zc, 3), [round(v, 3) for v in r['palm']], r['tilt']])
        if r['tilt'] > 7 or r['palm'][2] > zc + 0.03:
            break                                  # palm held up by the plush (stall) or tilting
    if z is None:
        return out(ok=False, why='no press level reachable', hist=hist)
    c.pps.ramp(ctx, {}, 4.0, hands=(0.0, 1.0))
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'da90_grasp')
    f = c.fingers(ob, 'right')
    st['press_z'] = z; c3.save(st)
    out(ok=c3.grip(f), base_idx_mid=[round(f[5], 2), round(f[3], 2)], hist=hist, frame=ob['camera']['frame_id'])


def raise_(ctx, st):
    gx, gy = st['grasp']; z = st['press_z'] + 0.03
    r = c3.move(ctx, (gx, gy, z), c3.GRASP_YAW, 4.0); r.pop('ob', None)
    st['press_z'] = z; c3.save(st)
    f = r.get('fingers', [9] * 7)
    out(ok=r['ok'] and min(f[3], f[5]) < GRIP_BASE_MAX, palm=r.get('palm'), base_idx_mid=[f[5], f[3]], frame=r.get('frame'))


def lift(ctx, st):
    gx, gy = st['grasp']; x = gx + 0.015; z = st['press_z']; hist = []
    while z < LIFT_Z - 1e-3:
        z = min(LIFT_Z, z + 0.03)
        r = c3.move(ctx, (x, gy, z), c3.GRASP_YAW, 4.0); r.pop('ob', None)
        hist.append([round(r.get('palm', [0, 0, 0])[2], 3), r.get('fingers', [])[3:6:2]])
        if not r['ok'] or not c3.grip(r['fingers']):
            return out(ok=False, why=r.get('why') or 'grip lost', hist=hist, frame=r.get('frame'))
    out(ok=True, palm=r['palm'], hist=hist, frame=r['frame'])


STEPS = dict(restore=restore, prep=prep, waist=waist, hover=hover, grasp=grasp, raise_=raise_, lift=lift)
STEPS['raise'] = raise_

if __name__ == '__main__':
    st = c3.load()
    with Context() as ctx:
        for s in sys.argv[1].split(','):
            try:
                STEPS[s](ctx, st)
            except c3.Fail:
                sys.exit(1)
            st = c3.load()
