"""puppy_box with the clear box BEHIND the puppy (puppy between robot and box), right hand only, built on pb3's
primitives. One step per call so the frames can be looked at:

  python code/puppy_box/pbb.py prep                       # pb3 prep: tuck left arm, right hand high, palm down
  python code/puppy_box/pbb.py lean [0.1]                 # waist pitch in its own ramp, two observations
  python code/puppy_box/pbb.py perceive                   # puppy mid-body + box rim corners -> output/pb3_state.json
  python code/puppy_box/pbb.py reach                      # IK on the commanded row: waist yaw reaching grasp AND release
  python code/puppy_box/pbb.py hover | grasp | lift | carry | release | home

Box behind: the release is straight ahead of the grasp, at the far end of the arm's reach. The waist (lean + yaw) is
set ONCE at hover and kept to the release (a waist change while the plush hangs dropped it in earlier sessions).
Perception of the box: the clear walls are invisible in depth, so the box floor rectangle is found in the colour image
(pale low-saturation region beyond the puppy) and intersected with the table plane (percep.plane_hit).
2026-11-02 session 6a0bfc. Pelvis frame, metres."""
import json, os, sys, time
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pb3                       # noqa: E402
from pb3 import c, p, report, Fail, load, save, waypoint, move, best   # noqa: E402

RIM_H = 0.14
INSIDE = 0.075        # release palm at least this far past the near wall (plush hangs a little behind the palm)
REL_Z = 0.21          # release palm height (pelvis frame, legs straight after a walk: rim at ~+0.05)
CARRY_Z = 0.24        # lift / carry height: the hanging plush's legs must clear the near rim on the way over
YAWS = (0.3, 0.5, 0.7)
GRASP_BACK = 0.07


def box_behind(P, tz):
    """Pale low-saturation pixels on the table plane beyond the puppy -> box floor footprint (level frame)."""
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    _, lev = p.cloud(P)
    m = (hsv[..., 1] < 60) & (hsv[..., 2] > 170) & (np.abs(lev[..., 2] - tz) < 0.03) & (P['depth'] > 0)
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 3000:
        return None
    vs, us = np.where(lab == k)
    hits = np.array([p.plane_hit(P, u, v, tz) for u, v in zip(us[::25], vs[::25])])
    x0, x1 = np.percentile(hits[:, 0], [2, 98]); y0, y1 = np.percentile(hits[:, 1], [2, 98])
    x1 = max(x1, x0 + 0.27)      # the far part shows through the white label / far wall, not as pale table: use the size
    return dict(x=[float(x0), float(x1)], y=[float(y0), float(y1)], rim_z=tz + RIM_H, table_z=tz,
                centre=[float((x0 + x1) / 2), float((y0 + y1) / 2)], y_inner=float(y1))


def step_perceive(ctx, st, box_override=None):
    ob, path = c.observe(ctx, 'pbb_perceive')
    P = p.load(path); tz = p.table_z(P)
    box = box_behind(P, tz) if box_override is None else dict(box_override, rim_z=tz + RIM_H, table_z=tz)
    if box is None:
        report(False, 'no clear box region found: give box x0,x1,y0,y1 by hand', frame=ob['camera']['frame_id'])
    pp = p.puppy(P, tz, exclude=(box['x'][0] + 0.02, 2, -2, 2))
    if pp is None:
        report(False, 'no puppy blob', frame=ob['camera']['frame_id'])
    pel, _ = p.cloud(P)
    pts = pel[pp['mask']]
    pts = pts[(pts[:, 2] < tz + 0.15) & (pts[:, 2] > tz + 0.02)]
    # body = the tall part (head/tail/legs are lower); mid-body y = centre of points above 70 % of the top height
    top = float(np.percentile(pts[:, 2], 97))
    body = pts[pts[:, 2] > tz + 0.7 * (top - tz)]
    yc = float(np.median(body[:, 1]))
    mid = pts[np.abs(pts[:, 1] - yc) < 0.05]
    far = float(np.percentile(mid[:, 0], 97))
    gx = far - GRASP_BACK
    rel = [box['x'][0] + INSIDE + 0.005, float(np.clip(box['centre'][1], box['y'][0] + 0.10, box['y'][1] - 0.10)), REL_Z]
    st.update(box=box, box_side='behind', grasp=[gx, yc], top=top, table_z=tz, release=rel, far=far)
    save(st)
    over = os.path.join(pb3.ROOT, 'output', f"pbb_perceive_{ob['step_id']}.jpg")
    p.draw_box(P, box['x'][0], box['x'][1], box['y'][0], box['y'][1], box['rim_z'], tz, over)
    pb3.cv2_mark(P, [(gx, yc, top), tuple(rel)], over)
    report(True, grasp=[round(gx, 3), round(yc, 3)], top=round(top, 3), far=round(far, 3), box_x=box['x'], box_y=box['y'],
           release=rel, overlay=over, frame=ob['camera']['frame_id'])


def step_reach(ctx, st):
    """Smallest commanded waist yaw (with the commanded lean) reaching the grasp at top-0.03 and the release."""
    cmd = pb3.cmd_row(ctx)
    gx, gy = st['grasp']; rel = st['release']
    res = []
    for w in YAWS:
        q = list(cmd); q[2] = w
        eg = best(q, (gx, gy, st['top'] - 0.03), pb3.GRASP_YAW)[0]
        er = best(q, tuple(rel), -10)[0]
        res.append((w, round(eg, 3), round(er, 3)))
    ok = [r for r in res if r[1] < 0.01 and r[2] < 0.01]
    if ok:
        st['waist_cmd'] = ok[0][0]; st['press_depth'] = -0.03; save(st)
    report(bool(ok), '' if ok else 'grasp or release out of reach at every waist yaw', table=res, waist_cmd=st.get('waist_cmd'))


def step_hover(ctx, st):
    gx, gy = st['grasp']; top = st['top']; w = st['waist_cmd']
    c.ppg.set_pinch_shape(ctx, 'right')
    r = waypoint(ctx, (gx, gy, top + 0.13), pb3.GRASP_YAW, w, 5.0, max_jump=1.6)
    if r['ok']:
        r2 = move(ctx, (gx, gy, top + 0.06), pb3.GRASP_YAW, 3.0); r2.pop('ob', None); r = r2
    report(r['ok'], r.get('why', ''), waist_cmd=w, palm=r.get('palm'), waist=r.get('waist'), frame=r.get('frame'))


def step_release(ctx, st):
    """Open over the box, then raise the hand straight up 8 cm (not back toward the near wall)."""
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(1.5)
    ob1, _ = c.observe(ctx, 'pbb_released')
    row, _, _ = c.pps.last_row(ctx)
    pa = np.array(c.pps.link_points(row[9:38], 'right')['right_palm'])
    r = move(ctx, (pa[0] - 0.08, pa[1], pa[2] + 0.06), -10, 3.0, max_jump=1.6); r.pop('ob', None)
    report(r['ok'], r.get('why', ''), frame_open=ob1['camera']['frame_id'], frame=r.get('frame'), palm=r.get('palm'))


STEPS = dict(prep=pb3.step_prep, lean=pb3.step_lean, perceive=step_perceive, reach=step_reach, hover=step_hover,
             grasp=pb3.step_grasp, lift=pb3.step_lift, carry=pb3.step_carry, release=step_release, home=pb3.step_home)

if __name__ == '__main__':
    pb3.LIFT_Z = pb3.CARRY_Z = CARRY_Z
    pb3.GRASP_YAW, pb3.RELEASE_YAW = 0.0, -10.0
    st = load()
    from api.runtime import Context
    with Context() as ctx:
        try:
            if sys.argv[1] == 'lean':
                pb3.step_lean(ctx, st, *sys.argv[2:3])
            elif sys.argv[1] == 'perceive':
                st = {}
                ov = None
                if len(sys.argv) > 2:
                    x0, x1, y0, y1 = [float(v) for v in sys.argv[2].split(',')]
                    ov = dict(x=[x0, x1], y=[y0, y1], centre=[(x0 + x1) / 2, (y0 + y1) / 2], y_inner=y1)
                    ov['table_z'] = None
                step_perceive(ctx, st, ov)
            else:
                STEPS[sys.argv[1]](ctx, st)
        except Fail:
            sys.exit(1)
