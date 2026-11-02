"""Box LEFT, puppy RIGHT, right hand only (session 89afad). Steps on top of pb3 (same state file output/pb3_state.json):

  python code/puppy_box/lr.py box x0,x1,y0,y1        # box footprint from colour corners (percep.plane_hit), overrides boxdet
  python code/puppy_box/lr.py plan                   # pick ONE fixed waist (yaw, roll) reaching the pressed grasp AND release
  python code/puppy_box/lr.py waist                  # ramp the waist alone (hand high), settle, observe
  python code/puppy_box/lr.py probe                  # empty hand to the release hover: MEASURED palm vs command
  python code/puppy_box/lr.py hover                  # pinch shape, hand to grasp top+0.13, then top+0.06
  python code/puppy_box/lr.py over                   # arm over the grasp (z .30) via a raised via-config (path checked)
  then: lr.py hover, pb3.py grasp, pb3.py lift, lr.py carry, lr.py swing (palm yaw 15->75, depth check), lr.py release,
  pb3.py home. Run 1 order: pb3 prep, pb3 perceive right, lr box, lr plan (or set waist 1.1/0.15), lr waist, lr over, ...
All IK on the COMMANDED row (incl. commanded waist yaw/roll)."""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pb3
from pb3 import c, move, best, report, load, save, GRASP_YAW, RELEASE_YAW

# (yaw, roll) candidates: waist roll +0.15 lowers the right shoulder (pressed grasp at y -0.2 reachable only with it)
WAISTS = ((0.7, 0.1), (0.7, 0.15), (0.9, 0.15))
REL_IN = 0.11          # palm at least this far inside the box's right (inner) wall
REL_Z = 0.27


def step_box(ctx, st, spec):
    x0, x1, y0, y1 = (float(v) for v in spec.split(','))
    st['box'] = {'x': [x0, x1], 'y': [y0, y1], 'y_inner': y0, 'centre': [(x0 + x1) / 2, (y0 + y1) / 2],
                 'rim_z': st['table_z'] + 0.14}
    st['box_side'] = 'left'
    save(st)
    report(True, box=st['box'])


def step_plan(ctx, st):
    gx, gy = st['grasp']; top = st['top']; b = st['box']
    cmd = pb3.cmd_row(ctx)
    rx = min(b['x'][0] + 0.10, b['centre'][0])
    for w, roll in WAISTS:
        q = list(cmd); q[2] = w; q[5] = roll
        eg = best(q, (gx, gy, top - 0.06), GRASP_YAW)[0]
        if eg > 0.01:
            continue
        for ry in (b['y'][0] + 0.16, b['y'][0] + 0.13, b['y'][0] + REL_IN):
            er = best(q, (rx, ry, REL_Z), RELEASE_YAW)[0]
            if er < 0.01:
                st.update(waist_cmd=w, waist_roll=roll, release=[rx, ry, REL_Z], press_depth=-0.06)
                save(st)
                report(True, waist=w, roll=roll, release=st['release'], grasp=st['grasp'], err=[eg, er])
                return
    report(False, 'no fixed waist reaches both the pressed grasp and the release: ask for a closer stance')


def step_waist(ctx, st):
    c.pps.ramp(ctx, {'waist_yaw_joint': st['waist_cmd'], 'waist_roll_joint': st['waist_roll']}, 4.0)
    time.sleep(3.0)                      # yaw creeps for seconds after the ramp
    ob, path = c.observe(ctx, 'lr_waist')
    j = ob['proprio']['joint_positions_rad']
    report(ob['tilt_deg'] < 6, f"tilt {ob['tilt_deg']}", tilt=ob['tilt_deg'], waist_meas=[j[2], j[5], j[8]],
           frame=ob['camera']['frame_id'])


def step_probe(ctx, st):
    rel = st['release']
    r = move(ctx, (rel[0], rel[1], rel[2] + 0.03), RELEASE_YAW, 5.0, max_jump=1.6); ob = r.pop('ob', None)
    if not r['ok']:
        report(False, r['why'])
    time.sleep(1.0)
    ob, _ = c.observe(ctx, 'lr_probe')
    palm = c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm']
    inner = palm[1] - st['box']['y'][0]
    report(inner > 0.08, f'palm {inner:.3f} inside the right wall', cmd=rel, palm=palm, tilt=ob['tilt_deg'],
           frame=ob['camera']['frame_id'])


def step_hover(ctx, st):
    gx, gy = st['grasp']; top = st['top']
    c.ppg.set_pinch_shape(ctx, 'right')
    r = move(ctx, (gx, gy, top + 0.13), GRASP_YAW, 5.0, max_jump=1.6); r.pop('ob', None)
    if r['ok']:
        r = move(ctx, (gx, gy, top + 0.06), GRASP_YAW, 3.0); r.pop('ob', None)
    report(r['ok'], r.get('why', ''), palm=r.get('palm'), waist=r.get('waist'), fingers=r.get('fingers'),
           tilt=r.get('tilt'), frame=r.get('frame'))


def path_min_z(q0, q1, n=21):
    """Lowest link point (z, link, xyz) on the straight joint path q0 -> q1, commanded-row FK."""
    lo = (9.0, '', None)
    for s in np.linspace(0, 1, n):
        for k, v in c.pps.link_points(list(q0 + (q1 - q0) * s), 'right').items():
            if v[2] < lo[0]:
                lo = (v[2], k, [round(x, 3) for x in v])
    return lo


def step_over(ctx, st):
    """Arm to (grasp, z 0.30) at the turned waist via a raised via-config (shoulder pitch -1.3 halfway). The pb3 prep
    arm cannot reach there in one IK move (jump ~2 rad); best() random seeds gave an overhead branch once (sp -2.97)."""
    gx, gy = st['grasp']
    row, _, _ = c.pps.last_row(ctx); cmd = np.array(row[9:38])
    b = cmd.copy()
    for k, v in c.mirror('right', {'shoulder_pitch': 0.68, 'shoulder_roll': -0.29, 'shoulder_yaw': -1.48, 'elbow': -0.2,
                                   'wrist_roll': -0.74, 'wrist_pitch': -0.51, 'wrist_yaw': 0.26}).items():
        b[c.J.index(k)] = v
    t, e, _ = c.pps.solve(list(b), 'right', (gx, gy, 0.30), c.pps.palm_quat(GRASP_YAW, 'right'))
    if e > 0.01:
        report(False, f'no IK above the grasp (err {e:.3f})')
    q2 = cmd.copy()
    for k, v in t.items():
        q2[c.J.index(k)] = v
    I = lambda n: c.J.index(f'right_{n}_joint')
    q1 = cmd + (q2 - cmd) * 0.5; q1[I('shoulder_pitch')] = -1.3; q1[I('elbow')] = min(q1[I('elbow')], 0.0)
    lows = [path_min_z(cmd, q1), path_min_z(q1, q2)]
    # only the elbow may pass low (z >= 0.09, x < 0.28: in front of the puppy, right of the box)
    for z, link, p in lows:
        if z < 0.15 and not (link == 'right_elbow_link' and z >= 0.09 and p[0] < 0.28):
            report(False, f'path too low: {link} at {p}', lows=lows)
    c.pps.ramp(ctx, {k: float(q1[c.J.index(k)]) for k in t}, 4.0)
    c.pps.ramp(ctx, t, 5.0)
    time.sleep(1.5)
    ob, _ = c.observe(ctx, 'lr_over')
    palm = c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm']
    report(ob['tilt_deg'] < 6, f"tilt {ob['tilt_deg']}", palm=palm, lows=lows, frame=ob['camera']['frame_id'])


def body_points(ob, st):
    path = os.path.join(pb3.ROOT, 'output', f"lr_{ob['step_id']}.json")
    json.dump(ob, open(path, 'w'))
    P = pb3.p.load(path); tz = st['table_z']
    pp = pb3.p.puppy(P, tz)
    if pp is None:
        return None
    pts = pb3.p.cloud(P)[0][pp['mask']]
    return pts[pts[:, 2] > tz + 0.03]


def step_carry(ctx, st):
    pb3.RELEASE_YAW = GRASP_YAW          # keep the palm yaw on the carry (no branch flip with the plush hanging)
    pb3.step_carry(ctx, st)


def step_swing(ctx, st):
    """Palm yaw 15 -> 50 -> 75 at the release point: the plush, held at its head-side middle, hangs forward-right of the
    palm; turning the palm counter-clockwise swings it forward-left over the box interior (run 1: centre y .01 -> all > .12)."""
    rel = st['release']
    for yaw in (50, 75):
        r = move(ctx, tuple(rel), yaw, 4.0, max_jump=0.8); ob = r.pop('ob', None)
        if not r['ok'] or not pb3.grip(r['fingers']):
            report(False, r.get('why') or 'grip lost on the swing', palm=r.get('palm'))
    pts = body_points(ob, st); b = st['box']
    fin = float((pts[:, 1] > b['y'][0] + 0.03).mean()) if pts is not None and len(pts) else 0.0
    report(fin > 0.9, f'{fin:.2f} of the visible plush > 3 cm inside the right wall: LOOK before opening', palm=r['palm'],
           frac_inside=round(fin, 2), y_pct=np.percentile(pts[:, 1], [3, 50, 97]).round(3).tolist() if fin else None,
           frame=r['frame'])


def step_release(ctx, st):
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(1.5)
    ob1, _ = c.observe(ctx, 'lr_released')
    rel = st['release']
    r = move(ctx, (rel[0] - 0.02, rel[1] - 0.03, rel[2] + 0.11), 75, 3.0, max_jump=0.8); r.pop('ob', None)
    report(r['ok'], r.get('why', ''), frame_open=ob1['camera']['frame_id'], frame=r.get('frame'), palm=r.get('palm'))


STEPS = dict(box=step_box, plan=step_plan, waist=step_waist, probe=step_probe, over=step_over, hover=step_hover,
             carry=step_carry, swing=step_swing, release=step_release)

if __name__ == '__main__':
    st = load()
    from api.runtime import Context
    with Context() as ctx:
        try:
            STEPS[sys.argv[1]](ctx, st, *sys.argv[2:])
        except pb3.Fail:
            sys.exit(1)
