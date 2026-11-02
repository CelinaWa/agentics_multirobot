"""puppy_box controller, box LEFT + puppy RIGHT (any head direction), right hand only. Session 0878bd, sim.
  PYTHONPATH=. python code/puppy_box/run_cr.py look,plan            # LOOK at output/cr_plan.jpg (grasp dot + yaw line)
  PYTHONPATH=. python code/puppy_box/run_cr.py prep,waist,over,hover,grasp,lift   # LOOK at the lift frame / fingers
  PYTHONPATH=. python code/puppy_box/run_cr.py carry                # LOOK: body over the box interior? then
  PYTHONPATH=. python code/puppy_box/run_cr.py release,home
Run after the stance step (same-side skill `run.py left stance`). State in output/pb3_state.json. Each step prints JSON;
the chain stops at the first failed check."""
import sys, os, json, time, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)
import cr, pup, layout2  # noqa
pb3, lr = cr.pb3, cr.lr
WAIST = (0.45, 0.15)          # roll 0.25 gave a standing tilt 5.4 and 7.5 on the press (attempt 1)
PRESS_Z = 0.0                 # reach check for the press (palm stalls ~3.5 cm above the depth top of the plush)


def look(ctx):
    cr.look(ctx, '-0.7')


def plan(ctx):
    path = sorted([os.path.join(ROOT, 'output', f) for f in os.listdir(os.path.join(ROOT, 'output')) if f.startswith('pb_cr_look_')],
                  key=os.path.getmtime)[-1]
    L = layout2.layout(path); b = L['box']
    if b is None:
        pb3.report(False, 'no box rim in the look view', frame=L['frame'])
    cs = np.array(b['corners']); bx = [float(cs[:, 0].min()), float(cs[:, 0].max())]; by = [float(cs[:, 1].min()), float(cs[:, 1].max())]
    right_wall = float(np.sort(cs[:, 1])[1])          # the less negative of the two right corners
    if os.environ.get('BOX'):     # x0,x1,y0,y1,right_wall from a clean view (the waist -0.7 look clips the box's left part)
        x0, x1, y0, y1, right_wall = (float(v) for v in os.environ['BOX'].split(','))
        bx, by = [x0, x1], [y0, y1]
    pts, tz = pup.pts_of(path, exclude_y_above=right_wall - 0.01)
    xy = pts[:, :2]; m = xy.mean(0); u, s, vt = np.linalg.svd(xy - m, full_matrices=False); ax = vt[0]
    if ax[1] < 0:
        ax = -ax                                    # axis pointing left (toward the box)
    yaw = math.degrees(math.atan2(ax[1], ax[0])) - 90.0   # fingers perpendicular to the body, pointing forward-right
    while yaw < -90: yaw += 180
    while yaw > 90: yaw -= 180
    top = float(np.percentile(pts[:, 2], 97))
    q0 = pb3.cmd_row(ctx); q = list(q0); q[2], q[5] = WAIST
    centre = [float((bx[0] + bx[1]) / 2), float((by[0] + by[1]) / 2)]
    rel = [centre[0], centre[1], 0.30]
    er = pb3.best(q, tuple(rel), 15.0)[0]
    grasp = None
    for t in np.arange(0.0, 0.16, 0.02):            # from the body centre toward the box-side end (the head here)
        g = m + ax * t
        e = pb3.best(q, (float(g[0]), float(g[1]), PRESS_Z), yaw)[0]
        if e < 0.006:
            grasp = [float(g[0]), float(g[1])]; break
    st = pb3.load()
    st.update(grasp=grasp, top=top, table_z=tz, gyaw=round(yaw, 1), ryaw=15.0, release=rel, waist_cmd=WAIST[0], waist_roll=WAIST[1],
              box={'x': bx, 'y': by, 'y_inner': right_wall, 'centre': centre, 'rim_z': tz + 0.14}, box_side='left', press_depth=-0.03)
    pb3.save(st)
    P = pb3.p.load(path)
    over = os.path.join(ROOT, 'output', 'cr_plan.jpg')
    pb3.p.draw_box(P, bx[0], bx[1], by[0], by[1], tz + 0.14, tz, over)
    if grasp:
        d = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw))]) * 0.08
        pb3.cv2_mark(P, [(grasp[0], grasp[1], top), (grasp[0] + d[0], grasp[1] + d[1], top), tuple(rel)], over)
    ok = grasp is not None and er < 0.01
    pb3.report(ok, '' if ok else f'no reachable grasp on the axis or release err {er:.3f}', grasp=grasp, yaw=round(yaw, 1),
               centre=m.round(3).tolist(), axis=ax.round(3).tolist(), top=round(top, 3), release=rel, release_err=round(er, 4),
               box_x=bx, box_y=by, overlay=over, frame=L['frame'])


def prep(ctx):
    pb3.c.pps.ramp(ctx, {'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}, 3.0)
    pb3.step_prep(ctx, pb3.load())


def lift(ctx):
    pb3.LIFT_Z = 0.30
    pb3.step_lift(ctx, pb3.load())


def carry(ctx):
    pb3.STALL = 1.45           # the soft plush lets the fingers creep to ~1.3-1.4 while still held (empty fist ~1.52-1.57)
    cr.carry(ctx, '0.06', '0.33')


def release(ctx):
    pb3.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(1.5)
    ob, _ = pb3.c.observe(ctx, 'cr_released')
    rel = pb3.load()['release']
    r = pb3.move(ctx, (rel[0] - 0.03, rel[1] - 0.03, 0.40), 15, 3.0, max_jump=1.0); r.pop('ob', None)
    pb3.report(r['ok'], r.get('why', ''), frame_open=ob['camera']['frame_id'], frame=r.get('frame'), palm=r.get('palm'))


def home(ctx):
    import subprocess
    out = subprocess.run([sys.executable, os.path.join(HERE, 'home_cr.py')], capture_output=True, text=True, timeout=200).stdout
    rep = json.loads(out.strip().splitlines()[-1])
    pb3.report(rep.get('ok', False), rep.get('why', ''), **{k: v for k, v in rep.items() if k not in ('ok', 'why')})


STEPS = dict(look=look, plan=plan, prep=prep, waist=lambda ctx: lr.step_waist(ctx, pb3.load()), over=cr.over,
             hover=lambda ctx: lr.step_hover(ctx, pb3.load()), grasp=lambda ctx: cr.grasp_soft(ctx, '-0.02', '6.5'),
             lift=lift, carry=carry, release=release, home=home)

if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        for s in sys.argv[1].split(','):
            print(f'step {s}', flush=True)
            st = pb3.load()
            if st.get('gyaw') is not None:      # plan may have changed the grasp yaw since cr was imported
                pb3.GRASP_YAW = lr.GRASP_YAW = float(st['gyaw'])
            try:
                STEPS[s](ctx)
            except pb3.Fail:
                sys.exit(1)
