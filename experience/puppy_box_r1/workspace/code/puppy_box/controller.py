"""puppy_box_r1 step-wise controller (r1sonic sim, session 20261105T222431Z_puppy_box_r1_e0ac0c).

Run ONE step at a time and look at the printed report / frames between steps (the agent is the checkpoint):

    PYTHONPATH=/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace/pylib python controller.py <step> ['{json args}']

Steps (in order):
    look                      head 0.6, observe, print edge / paws / box (pelvis frame)
    approach  {"paw_x":0.33}  walk forward in <=0.12 m commands until the nearest front paw centre is <= paw_x ahead
    pregrasp  {"paw":[x,y,z]} open hand 8 cm in front of the paw, fingers vertical, approach (1,0,0.1)
    grasp     {"paw":[x,y,z]} final move (ball ~1 cm past the paw centre), close, verify width 0.02..0.06
    lift      {"dz":0.12,"dx":0.09}  continuous-IK lift (joint steps, 2.5 s each), width check every step
    carry     {"walk":0.12}  walk forward holding; width check
    box                       measure the box rim from depth (front wall x, interior y range)
    place     {"target":[x,y,z]}  continuous-IK move over the box interior, lower 5 cm, open
    retreat                   hand straight back at its height to x 0.2, then HOME joints, then walk back 0.2

Proven this session (memory/RUNS.md): look, approach, pregrasp, grasp (4/4 held, width .032-.035), lift to
z~0.13 (4/5), carry walk (r1_grasp_paw hold 1/1), box, retreat. NOT proven: lifting the puppy over the 14 cm rim -
the paw-ball pinch slips (friction-limited) at z .12-.29. Do not call r1_grasp_paw (2 falls, 3 knock-offs in 7 calls).
Not run end-to-end as a script (no resets left when it was written); the steps mirror the manual calls that worked.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

WSR = Path('/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/'
           '20261105T232821Z_puppy_box_r1_c4f6c3/agent-workspace')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(WSR))
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'tools'))
from api.runtime import Context  # noqa: E402

HOME = {'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0.0}
HELD_MIN, HELD_MAX = 0.02, 0.06          # measured width range that means "something is held"
STATE = HERE / 'state.json'              # last commanded right-arm joints (seed for continuous IK)


def log(step, **kw):
    print(json.dumps({'t': time.strftime('%H:%M:%S'), 'step': step, **kw}, default=str), flush=True)


def save_q(q):
    STATE.write_text(json.dumps(q))


def load_q():
    return json.loads(STATE.read_text()) if STATE.exists() else dict(HOME)


def observe(ctx):
    o = ctx.value('r1_observe')
    return o['camera']['frame_id'], o


def width(ctx):
    return ctx.value('r1_status')['proprio']['gripper_width_m']['right']


def step_look(ctx, a):
    from scene import brief
    ctx.value('r1_head', pitch=a.get('pitch', 0.6))
    fid, o = observe(ctx)
    b = brief(fid)
    log('look', frame=fid, tilt=o['tilt_deg'], **b)
    return b


def front_paws(b, min_area=500, merge_m=0.03):
    """Front paws = the two paw blobs with the most negative y (head-RIGHT layout). Drops small blobs (the dark nose
    tip, slivers at the image border), merges blobs closer than merge_m (k-means splits), and returns centres
    (visible surface + 1.5 cm x, -1.5 cm z). For a head-LEFT layout take the most positive y instead."""
    blobs = [np.array(q['p']) for q in b.get('paw_blobs', []) if q['area'] >= min_area and q['p'][2] > -0.06]
    merged = []
    for q in sorted(blobs, key=lambda q: q[1]):
        if merged and np.linalg.norm(merged[-1][:2] - q[:2]) < merge_m:
            merged[-1] = (merged[-1] + q) / 2
        else:
            merged.append(q)
    return [(q + np.array([0.015, 0.0, -0.015])).round(3).tolist() for q in merged[:2]]


def step_approach(ctx, a):
    target = a.get('paw_x', 0.33)
    b = {}
    for _ in range(6):
        b = step_look(ctx, {})
        fp = front_paws(b)
        if not fp:
            log('approach', stop='no paws visible (too close? head max 0.6): use visible leg points'); return b
        x = min(q[0] for q in fp)
        if x <= target + 0.01:
            log('approach', done=True, paw_x=x, edge=b.get('edge_x')); return b
        cmd = min(0.12, max(0.06, (x - target) * 1.2))
        w = ctx.value('r1_walk', forward_m=cmd)
        log('approach', paw_x=x, cmd=cmd, walked=w.get('walked_m'), tilt=w.get('tilt_deg'))
        if w.get('fallen'):
            ctx.value('r1_hold'); raise SystemExit('fallen')
    return b


def move_hand(ctx, pos, approach, width_m, secs=2.0):
    r = ctx.value('r1_move_hand', arm='right', position=[float(v) for v in pos], approach=approach,
                  fingers=[0, 0, 1], gripper_width_m=width_m, secs=secs)
    save_q(r['plan']['joints'])
    return r


def step_pregrasp(ctx, a):
    x, y, z = a['paw']
    r = move_hand(ctx, [x - 0.08, y, z], [1, 0, 0.1], 0.07, 2.5)
    log('pregrasp', hand=r['grasp_points_pelvis_m']['right'], err=r.get('grasp_error_m'))


def step_grasp(ctx, a):
    x, y, z = a['paw']
    r = move_hand(ctx, [x + 0.01, y, z + 0.005], [1, 0, -0.1], 0.07, 2.0)
    log('grasp_move', hand=r['grasp_points_pelvis_m']['right'], err=r.get('grasp_error_m'))
    g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)   # also syncs the stored gripper command
    w = g['gripper']['measured_m']
    held = HELD_MIN <= w <= HELD_MAX
    log('grasp', width=w, held=held)
    return held


def joint_steps(ctx, plan, secs=2.5):
    for wp in plan:
        r = ctx.value('r1_move_joints', joints=wp['q'], secs=secs)
        save_q(wp['q'])
        w = r['gripper_width_m']['right']
        log('joint_step', goal=wp['goal'], hand=r['grasp_points_pelvis_m']['right'], width=w, tilt=r['tilt_deg'])
        if w < HELD_MIN:
            log('slip', why='width %.4f < %.3f' % (w, HELD_MIN)); return False
    return True


def step_lift(ctx, a):
    from contik import plan_b, bfk
    q0 = load_q()
    p, _ = bfk(q0)
    n = a.get('n', 4)
    goals = [[p[0] + a.get('dx', 0.09) * k / n, p[1], p[2] + a.get('dz', 0.12) * k / n] for k in range(1, n + 1)]
    plan = plan_b(q0, goals, [1, 0, 0], [0, 0, 1])
    bad = [wp for wp in plan if wp['jump'] > 0.6 or wp['err'] > 0.03]
    if bad:
        log('lift', refused='plan has a joint jump > 0.6 rad or error > 3 cm', bad=bad); return False
    return joint_steps(ctx, plan)


def step_carry(ctx, a):
    w0 = width(ctx)
    r = ctx.value('r1_walk', forward_m=a.get('walk', 0.12))
    w1 = width(ctx)
    log('carry', walked=r.get('walked_m'), tilt=r.get('tilt_deg'), width_before=w0, width_after=w1)
    return w1 >= HELD_MIN


def step_box(ctx, a):
    """Box rim from depth: points 0.125..0.165 above the table; front wall = near x quantile. CHECK the result against
    r1_pixel_to_position on the rim pixels: the puppy's top (~0.12 above the table) can contaminate it."""
    from percep import load_frame, cloud, table
    fid, o = observe(ctx)
    meta, rgb, dep = load_frame(fid)
    t = table(meta, dep) or {}
    zt = t.get('z', -0.035)
    P, ok = cloud(meta, dep, 1)
    h = P[..., 2] - zt
    m = ok & (h > 0.125) & (h < 0.165) & (P[..., 0] > 0.3) & (P[..., 0] < 1.2)
    p = P[m]
    if len(p) < 200:
        log('box', stop='rim not found', frame=fid); return None
    res = {'frame': fid, 'front_x': round(float(np.percentile(p[:, 0], 3)), 3),
           'back_x': round(float(np.percentile(p[:, 0], 97)), 3),
           'y_left': round(float(np.percentile(p[:, 1], 97)), 3), 'y_right': round(float(np.percentile(p[:, 1], 3)), 3),
           'rim_z': round(zt + 0.14, 3)}
    log('box', **res)
    return res


def step_place(ctx, a):
    from contik import plan_b, bfk
    q0 = load_q()
    p, _ = bfk(q0)
    tx, ty, tz = a['target']
    n = 4
    goals = [[p[0] + (tx - p[0]) * k / n, p[1] + (ty - p[1]) * k / n, max(p[2], tz)] for k in range(1, n + 1)]
    plan = plan_b(q0, goals, None, [0, 0, 1])
    if any(wp['jump'] > 0.6 or wp['err'] > 0.03 for wp in plan):
        log('place', refused='plan not continuous', plan=plan); return False
    if not joint_steps(ctx, plan):
        return False
    joint_steps(ctx, plan_b(load_q(), [[tx, ty, tz - 0.05]], None, [0, 0, 1]))
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
    fid, o = observe(ctx)
    log('place', released=True, frame=fid)
    return True


def step_retreat(ctx, a):
    st = ctx.value('r1_status')
    hand = st['proprio']['grasp_points_pelvis_m']['right']
    z = max(hand[2], 0.12)
    ctx.value('r1_move_hand', arm='right', position=[0.2, -0.22, z], gripper_width_m=0.07, secs=2.0)
    ctx.value('r1_move_joints', joints={**HOME, 'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}, secs=2.5)
    save_q(HOME)
    r = ctx.value('r1_walk', forward_m=a.get('back', -0.2))
    log('retreat', walked=r.get('walked_m'), tilt=r.get('tilt_deg'))


STEPS = {'look': step_look, 'approach': step_approach, 'pregrasp': step_pregrasp, 'grasp': step_grasp,
         'lift': step_lift, 'carry': step_carry, 'box': step_box, 'place': step_place, 'retreat': step_retreat}

if __name__ == '__main__':
    name = sys.argv[1]
    args = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    with Context() as ctx:
        try:
            STEPS[name](ctx, args)
        except SystemExit:
            raise
        except Exception as e:
            try:
                ctx.value('r1_hold')
            except Exception:
                pass
            log('error', step=name, why=str(e)[:500])
