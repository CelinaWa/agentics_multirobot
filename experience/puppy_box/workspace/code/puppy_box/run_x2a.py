"""puppy_box controller, session 2a8660 (sim): puppy RIGHT (head right, legs to the robot), clear box LEFT, right hand only.
Built on the library cross-right-hand skill (pb3/lr + headleft_0878bd run_cr/cr). What made run 1 work (attempt 4 of 4):
grasp at the PCA body centre from a both-arms-tucked look (NOT the pb3 perceive: its head-right view cut the head off and put
the grasp 10 cm toward the tail), fingers across the body axis, palm 3 cm back against the finger direction, landing
corrected once on the measured palm, soft stepwise press, slow 3 cm lift, carry at z 0.35, palm-yaw swing, open.
  PYTHONPATH=. python code/puppy_box/run_x2a.py stance,look,plan        # LOOK at output/cr_plan.jpg
  PYTHONPATH=. python code/puppy_box/run_x2a.py prep,waist,over,hover,grasp,lift   # LOOK at the lift frame
  PYTHONPATH=. python code/puppy_box/run_x2a.py carry,swing               # LOOK: body over the interior? else adjust (LOOK)
  PYTHONPATH=. python code/puppy_box/run_x2a.py release,home
BOX=x0,x1,y0,y1 overrides the box (from colour corners, percep.plane_hit) when plan's box gate fails.
Each step prints one JSON line; the chain stops at the first failed check (exit 1)."""
import sys, os, json, time, math
import numpy as np
ROOT = os.getcwd()
CR = os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd')
SS = os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-same-side/code')
sys.path.insert(0, CR)
import cr, pup, layout2, run_cr  # noqa
pb3, lr = cr.pb3, cr.lr

WAIST = tuple(float(v) for v in os.environ.get('WAIST', '0.7,0.15').split(','))  # 258bd8: 0.5,0.2 reaches mid-body y -0.40; 0.9 gave tilt 6.24 with the arm over a body at y -0.30 (run 1); 0.7 reached grasp + release
BACK = 0.03               # palm offset against the finger direction so the fingertips pass the body's far side
LAND_TOL = 0.025          # hover palm within this of its target before pressing
LIFT_STEP, LIFT_Z = float(os.environ.get('LIFT_STEP', 0.03)), 0.30
LIFT_SECS = float(os.environ.get('LIFT_SECS', 3.0))   # 258bd8: 2 cm / 4 s after a slip at palm z .15 with 3 cm / 3 s
CARRY_Z, REL_Z, REL_IN_Y, REL_IN_X = 0.35, 0.30, 0.13, 0.10
STALL = 1.45              # carry: a held plush reads 0.85-1.19 on the middle finger; empty fist 1.52-1.57
report = pb3.report


def st_load():
    return pb3.load()


def stance(ctx):
    sys.path.insert(0, SS)
    import stance as s
    r = s.stance(ctx, log=lambda *a: None)
    report(r['ok'], r['why'], **r['measured'])


def look(ctx):
    cr.look(ctx, '-0.7')


def box_from(path):
    if os.environ.get('BOX'):
        x0, x1, y0, y1 = (float(v) for v in os.environ['BOX'].split(','))
        return [x0, x1], [y0, y1]
    b = layout2.layout(path)['box']
    if b is None:
        return None
    cs = np.array(b['corners'])
    bx = [float(cs[:, 0].min()), float(cs[:, 0].max())]; by = [float(cs[:, 1].min()), float(cs[:, 1].max())]
    # gate: footprint ~28 (x) x 39-41 (y) cm, on the left; layout2 once returned x 0.48-0.615 (half the box)
    if not (0.24 <= bx[1] - bx[0] <= 0.36 and 0.33 <= by[1] - by[0] <= 0.46 and by[0] > -0.15):
        return None
    return bx, by


def plan(ctx):
    path = sorted([os.path.join(ROOT, 'output', f) for f in os.listdir(os.path.join(ROOT, 'output')) if f.startswith('pb_cr_look_')],
                  key=os.path.getmtime)[-1]
    home = sorted([os.path.join(ROOT, 'output', f) for f in os.listdir(os.path.join(ROOT, 'output'))
                   if f.startswith('pb_stance_') or f.startswith('pb_home_cr_')], key=os.path.getmtime)
    box = box_from(home[-1]) if home else None
    if box is None:
        report(False, 'box not found or implausible in the tucked waist-0 view: measure colour corners and pass BOX=x0,x1,y0,y1',
               view=home[-1] if home else None)
    bx, by = box
    pts, tz = pup.pts_of(path, exclude_y_above=by[0] - 0.01)
    if pts is None or len(pts) < 200:
        report(False, 'no puppy points right of the box', view=path)
    xy = pts[:, :2]; m = xy.mean(0)
    if os.environ.get('GC'):     # 258bd8: grasp centre override (true mid-body between the legs; PCA centre sat on the front legs)
        m = np.array([float(v) for v in os.environ['GC'].split(',')])
    _, _, vt = np.linalg.svd(xy - m, full_matrices=False); ax = vt[0]
    if ax[1] < 0:
        ax = -ax
    yaw = math.degrees(math.atan2(ax[1], ax[0])) - 90.0
    while yaw < -90: yaw += 180
    while yaw > 90: yaw -= 180
    if os.environ.get('GYAW'):   # diagonal fingers held through the lift (-57.6 run 1, -36 0878bd); -12 / +5 slipped
        yaw = float(os.environ['GYAW'])
    top =float(np.percentile(pts[:, 2], 97)) + 0.007
    fd = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw))])
    g = m - BACK * fd
    rel = [bx[0] + REL_IN_X, by[0] + REL_IN_Y, REL_Z]
    q = list(pb3.cmd_row(ctx)); q[2], q[5] = WAIST
    eg = pb3.best(q, (float(g[0]), float(g[1]), 0.0), yaw)[0]; er = pb3.best(q, tuple(rel), 15.0)[0]
    st = st_load()
    st.update(grasp=[float(g[0]), float(g[1])], target=[float(g[0]), float(g[1])], top=top, table_z=tz, gyaw=round(yaw, 1),
              ryaw=15.0, release=rel, waist_cmd=WAIST[0], waist_roll=WAIST[1], box_side='left',
              box={'x': bx, 'y': by, 'y_inner': by[0], 'centre': [sum(bx) / 2, sum(by) / 2], 'rim_z': tz + 0.14})
    pb3.save(st)
    P = pb3.p.load(path); over = os.path.join(ROOT, 'output', 'cr_plan.jpg')
    pb3.p.draw_box(P, bx[0], bx[1], by[0], by[1], tz + 0.14, tz, over)
    pb3.cv2_mark(P, [(g[0], g[1], top), (g[0] + 0.08 * fd[0], g[1] + 0.08 * fd[1], top), tuple(rel)], over)
    ok = eg < 0.006 and er < 0.01 and g[0] < 0.36
    report(ok, '' if ok else f'reach: grasp err {eg:.3f} release err {er:.3f} grasp x {g[0]:.3f}', grasp=g.round(3).tolist(),
           centre=m.round(3).tolist(), yaw=round(yaw, 1), top=round(top, 3), release=rel, box_x=bx, box_y=by, overlay=over)


def waist(ctx):
    lr.step_waist(ctx, st_load())


def hover(ctx):
    """lr hover, then correct the commanded xy once by the measured palm error (the palm lands 1-3 cm off at this reach)."""
    st = st_load(); pb3.GRASP_YAW = lr.GRASP_YAW = st['gyaw']
    c = pb3.c; c.ppg.set_pinch_shape(ctx, 'right')
    tx, ty = st['target']; gx, gy = st['grasp']; z = st['top'] + 0.06
    r = pb3.move(ctx, (gx, gy, st['top'] + 0.13), st['gyaw'], 5.0, max_jump=1.6); r.pop('ob', None)
    for k in range(2):
        if not r['ok']:
            report(False, r['why'])
        r = pb3.move(ctx, (gx, gy, z), st['gyaw'], 3.0); r.pop('ob', None)
        ex, ey = tx - r['palm'][0], ty - r['palm'][1]
        if math.hypot(ex, ey) < 0.012 or k == 1:
            break
        gx, gy = gx + ex, gy + ey
    st['grasp'] = [gx, gy]; pb3.save(st)
    ok = r['ok'] and abs(tx - r['palm'][0]) < LAND_TOL and abs(ty - r['palm'][1]) < LAND_TOL and r['tilt'] < 6
    report(ok, '' if ok else 'palm not over the target', palm=r['palm'], target=[tx, ty], cmd=[gx, gy], tilt=r['tilt'], frame=r['frame'])


def grasp(ctx):
    pb3.GRASP_YAW = lr.GRASP_YAW = st_load()['gyaw']
    cr.grasp_soft(ctx, '-0.02', '6.5')


def lift(ctx):
    """3 cm / 3 s steps from the press command to palm cmd LIFT_Z; stop when both base joints read a fist."""
    st = st_load(); gx, gy = st['grasp']; z = st['press_z']; hist = []
    while z < LIFT_Z - 1e-3:
        z = min(LIFT_Z, z + LIFT_STEP)
        r = pb3.move(ctx, (gx, gy, z), st['gyaw'], LIFT_SECS, max_jump=0.8); r.pop('ob', None)
        if not r['ok']:
            report(False, r['why'], history=hist)
        f = r['fingers']; hist.append([round(z, 3), r['palm'][2], f[3], f[5], r['tilt']])
        if min(f[3], f[5]) > STALL:
            report(False, f'grip lost at palm z {r["palm"][2]:.3f}', history=hist, frame=r['frame'])
    report(True, palm=r['palm'], history=hist, frame=r['frame'])


def carry(ctx):
    """Arm-only, <= 6 cm / 5 s steps from the lifted palm (at CARRY_Z) to the release, palm yaw gyaw -> 15, IK tol 2 cm."""
    st = st_load()
    row, _, _ = pb3.c.pps.last_row(ctx)
    s = np.array(pb3.c.pps.link_points(row[9:38], 'right')['right_palm']); s[2] = CARRY_Z
    e = np.array(st['release'], float)
    n = max(2, int(np.ceil(np.linalg.norm((e - s)[:2]) / 0.06))); hist = []
    for k in range(1, n + 1):
        a = k / n; pos = s + a * (e - s); yaw = st['gyaw'] + a * (15.0 - st['gyaw'])
        r = pb3.move(ctx, tuple(pos), yaw, 5.0, max_err=0.02, max_jump=0.8); r.pop('ob', None)
        if not r['ok']:
            report(False, r['why'], history=hist, at=pos.round(3).tolist())
        hist.append({'cmd': pos.round(3).tolist(), 'palm': r['palm'], 'fingers': r['fingers'][3:6:2], 'tilt': r['tilt']})
        if min(r['fingers'][3], r['fingers'][5]) > STALL or r['tilt'] > 8:
            report(False, 'grip lost or tilt on the carry', history=hist, frame=r['frame'])
    inside = r['palm'][1] - st['box']['y'][0]
    report(inside > 0.08, f'palm {inside:.3f} inside the right wall', palm=r['palm'], history=hist, frame=r['frame'])


def swing(ctx):
    pb3.STALL = STALL
    lr.step_swing(ctx, st_load())


def adjust(ctx):
    """Run 5: after the swing the body stuck out TOWARD THE ROBOT (x .11-.25, in front of the near wall: held near one end).
    Palm forward 8 cm and yaw back to 30 swung it over the interior (looked + depth), then release worked (operator: INSIDE).
    Prints depth points of a pixel grid above the table for the agent to check against the box footprint; LOOK, then release."""
    st = st_load(); rel = st['release']; b = st['box']
    r = pb3.move(ctx, (rel[0] + 0.08, rel[1] - 0.01, rel[2]), 30, 4.0, max_err=0.02, max_jump=0.8); r.pop('ob', None)
    if not r['ok']:
        report(False, r['why'])
    sel = [{'u': u, 'v': v} for u in range(60, 600, 60) for v in range(60, 420, 60)]
    q = ctx.value('g1_pixel_to_position', observation_step_id=r['step_id'], selections=sel)
    pts = [p['position_pelvis_m'] for p in q['selections'] if p.get('valid') and p['position_pelvis_m'][2] > st['table_z'] + 0.03]
    out = [p for p in pts if not (b['x'][0] < p[0] < b['x'][1] and b['y'][0] < p[1] < b['y'][1])]
    report(min(r['fingers'][3], r['fingers'][5]) < STALL, 'LOOK: points outside the footprint may be the arm (white)', palm=r['palm'],
           n_above=len(pts), outside=[[round(v, 3) for v in p] for p in out], frame=r['frame'])


def release(ctx):
    lr.step_release(ctx, st_load())


def lean(ctx):
    """258bd8: waist pitch LEAN (default 0.1) in its own ramp after the waist step; the press at y -0.40 rode 4-5 cm above its
    command without it (palm 0.064-0.079 for cmd 0.01-0.04): reach, not the plush, stopped the hand."""
    pb3.step_lean(ctx, st_load(), float(os.environ.get('LEAN', 0.1)))


STEPS = dict(lean=lean, stance=stance, look=look, plan=plan, prep=run_cr.prep, waist=waist, over=cr.over, hover=hover, grasp=grasp,
             lift=lift, carry=carry, swing=swing, adjust=adjust, release=release, home=run_cr.home)

if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        for s in sys.argv[1].split(','):
            print(f'step {s}', flush=True)
            st = st_load()
            if st.get('gyaw') is not None:
                pb3.GRASP_YAW = lr.GRASP_YAW = float(st['gyaw'])
            try:
                STEPS[s](ctx)
            except pb3.Fail:
                sys.exit(1)
