"""Backend-primitive chain (session 3d54de): my look (paw sphere fit + measured leg axis, waist look when the paw is
hidden) -> r1_leg_lift(paw, leg_dir) -> r1_carry_release(target, advance_m, lower_m) -> retreat (v4.out).
TASK.md (engineering 2026-11-06 03:20): r1_leg_lift held the lift 6/6 from paw x .36-.37; carries work from paw
x .29-.34. Compare with the own low-tilt lift of lowtilt_v4 (~50%).
    python code/puppy_box/bl.py <tag> [depth=.03] [lift_z=.265] [adv=.06] [lower=.06] [in_m=.10] [close_edge=.40]"""
import sys, json, time, math, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('s', 'code/puppy_box/lowtilt_v4.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

def P(**k): print(json.dumps(k, default=str), flush=True)

CARRY_W = 0.034
BIAS = (0.012, 0.008)


def put_down(ctx, hand, ld=(1.0, 0.0)):
    """lower a (possibly) loaded hand straight down (servo off) to z .05, open there, back off, HOME."""
    try:
        if hand[2] > 0.07:
            ctx.value('r1_move_hand', arm='right', position=[hand[0], hand[1], 0.05], approach=[ld[0], ld[1], 0.1], servo=False, secs=3.0)
    except Exception as ex:
        P(step='put_down_refused', why=str(ex)[:150])
    m.free_arm(ctx, log=lambda *a: None)


def run(ctx, depth=0.03, lift_z=0.265, adv=0.06, lower=0.06, in_m=0.10, step=0.10, close_edge=0.40, bias=0.0):
    m.chain.ctx = ctx; m.CLOSE_EDGE = close_edge
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.5)
    ctx.value('r1_move_joints', joints=m.v4.HOME_BOTH, secs=1.0)
    ctx.value('r1_head', pitch=0.6, secs=1.0)
    meas = {}
    r = m.look(ctx, log=lambda *a: None); meas['look'] = r['measured']
    P(step='look', ok=r['ok'], why=r['why'], bias=bias, sc={k: r['measured'].get(k) for k in ('paw', 'leg_yaw', 'edge', 'wall', 'overhang', 'frame')})
    if not r['ok']:
        return {'ok': False, 'why': 'look: ' + r['why'], 'measured': meas}
    sc = r['measured']; a = math.radians(sc['leg_yaw'])
    # v5d: my sphere fit reads the paw ~(+.011..+.014, +.006..+.010) from the operator's ground truth (90b103, s1):
    # the backend's depth_m is meant from the TRUE paw centre (closes .031-.036 = clean shaft); mine closed .037-.042
    if bias:
        sc['paw_raw'] = list(sc['paw'])
        sc['paw'] = [sc['paw'][0] - BIAS[0], sc['paw'][1] - BIAS[1], sc['paw'][2]]
    ld = [round(math.cos(a), 4), round(math.sin(a), 4)]
    L = ctx.value('r1_leg_lift', paw_position=[round(v, 4) for v in sc['paw']], leg_direction=ld, depth_m=depth, lift_z_m=lift_z)
    meas['lift'] = L
    P(step='leg_lift', reply=L)
    hand = L.get('hand_pelvis_m') or ctx.value('r1_status')['proprio']['grasp_points_pelvis_m']['right']
    w_end = L.get('gripper_width_m') or 0.0
    top = (L.get('stage') == 'lifted') or (hand[2] >= lift_z - 0.05)
    # v5 (R2): the tool reached the top (hand z .228) still holding .0366 but flagged width_falling; free_arm then opened
    # at z .23 and dropped the puppy. A grip still >= CARRY_W at the top goes on to the carry (5600c1: carries entering
    # >= .0337 held 8/8); a falling grip below the top is lowered first and opened low.
    if L.get('held') and top and w_end >= CARRY_W:
        pass
    else:
        put_down(ctx, hand, ld)
        return {'ok': False, 'why': 'lift: %s (w %.4f, hand z %.3f)' % (L.get('stage') or L.get('reason') or 'not held', w_end, hand[2]), 'measured': meas}
    # v5c (R3: torso landed on the front rim): r1_carry_release counts advance_m in the total it compares with
    # target_forward_m (s1: .06 + .093 >= .146 -> released after 1 step; R3 .06 + .118 >= .167). So the target is the
    # TOTAL hand travel (advance + walk) and the edge bound applies to the walk part only.
    tgt = sc['wall'] + in_m - (hand[0] + m.BODY_OFF_X)
    if sc.get('edge') and sc['edge'].get('dist'):
        tgt = min(tgt, sc['edge']['dist'] - m.EDGE_MIN + adv)
    tgt = round(min(0.40, max(0.05 + adv, tgt)), 3)
    walk = max(0.04, tgt - adv)
    n = max(1, int(math.ceil(walk / step - 1e-6))); st = round(min(0.10, max(0.04, walk / n)), 3)
    C = ctx.value('r1_carry_release', target_forward_m=tgt, advance_m=adv, lower_m=lower, max_steps=n, step_m=st)
    o = ctx.value('r1_observe')
    meas['carry'] = {'target': tgt, 'reply': C, 'after_frame': o['camera']['frame_id']}
    P(step='carry', target=tgt, steps=n, step_m=st, reply=C, after_frame=o['camera']['frame_id'])
    ok = bool(C.get('released')) and not C.get('dropped')
    try:
        m.retreat(ctx, 0.2)
    except Exception as ex:
        ctx.value('r1_hold')
        return {'ok': False, 'why': 'retreat failed (HOLDING): %s' % str(ex)[:200], 'measured': meas}
    return {'ok': ok, 'why': 'released over the box; verify inside' if ok else 'carry: dropped=%s released=%s' % (C.get('dropped'), C.get('released')), 'measured': meas}

if __name__ == '__main__':
    tag = sys.argv[1]; kw = {}
    for s in sys.argv[2:]:
        k, v = s.split('='); kw[k] = float(v)
    ctx = Context(); t0 = time.time()
    res = run(ctx, **kw); res['tag'] = tag; res['secs'] = round(time.time() - t0, 1); res['variant'] = 'bl'
    with open('output/skill_runs.jsonl', 'a') as fh:
        fh.write(json.dumps(res, default=str) + '\n')
    P(step='RESULT', tag=tag, ok=res['ok'], why=res['why'], secs=res['secs'])
