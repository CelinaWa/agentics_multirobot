"""Chain-lift variant (session 3d54de): my look + grasp (lowtilt_v4) at depth .045 past my paw fit, then the
7c03aa/cf6e88 chain.py LIFT profile (11 servo-off steps to cmd z ~+.43, approach z 0 -> .5, approach YAW rotated
from the leg yaw to 2 deg from step 2 on, 1.2 s/step) - lifted 21/21 in those sessions - then r1_carry_release (no
advance) and the v4 retreat. Abort: width > 4 mm below its peak in steps 0-5 -> retrace down, open low.
    python code/puppy_box/cl.py <tag> [depth=.045] [secs=1.2] [in_m=.08] [lower=.06] [yaw_end=2] [close_edge=.40]"""
import sys, json, time, math, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('s', 'code/puppy_box/lowtilt_v4.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
chain = m.chain
BODY_OFF = 0.035      # torso landed at hand x + .02..+.05 (R8 .044, R10 .025, s4 .053; operator ground truth)
EDGE_MIN = 0.16      # R10: the .22 bound cut the walk to .147 -> torso on the front rim; R8 ended .147 from the edge, no contact
TOP_MIN_W = 0.031   # carries held at .039-.049 (s1, R3, s4); .0285 dropped (s6)
def P(**k): print(json.dumps(k, default=str), flush=True)


def lift_chain(ctx, gm, secs=1.2, yaw_end=2.0, abort_fall=0.006):
    G, ap, f, ld = gm['G'], gm['ap'], gm['f'], gm['ld']
    yaw0 = math.atan2(ld[1], ld[0]); y_end = math.radians(yaw_end)
    x, y, z = G; prof = chain.LIFT; path = [(tuple(G), (ld[0], ld[1], 0.0))]
    widths = [gm['close_width']]; peak = 0.0; last = tuple(G); fi = f
    for i, (da, dz, az) in enumerate(prof):
        x, y, z = x + da * ld[0], y + da * ld[1], z + dz
        t_ = min(1.0, max(0.0, (i - 2) / (len(prof) - 3)))
        yw = yaw0 + (y_end - yaw0) * t_
        api = (math.cos(yw), math.sin(yw), az); fi = (-math.sin(yw), math.cos(yw), 0.0)
        r = chain.guarded((x, y, z), api, fi, w=0.0, secs=secs, max_jump=0.45, servo=False)
        if r is None and i <= 2:
            # R7: from a far paw (G x .42) lift step 0 was refused (IK) -> let the waist help the first steps
            r = chain.guarded((x, y, z), api, fi, w=0.0, secs=secs, max_jump=0.45, servo=False, use_waist=True)
            P(step='lift_waist_retry', i=i, ok=r is not None)
        if r is None:
            P(step='lift_refused', i=i, cmd=[x, y, z])
            x, y, z = last
            if z < 0.25:
                ok = m._put_back(path, fi)
                if ok:
                    m._back_off(G, ap, f, ld)
                return {'ok': False, 'why': 'lift refused low (z %.3f)%s' % (z, '' if ok else ': HOLDING'), 'widths': widths}
            break
        last = (x, y, z); path.append(((x, y, z), api))
        wi = r['gripper_width_m']['right']; widths.append(wi); peak = max(peak, wi)
        P(step='lift', i=i, cmd=[round(v, 3) for v in (x, y, z)], width=wi, hand=r['grasp_points_pelvis_m']['right'])
        if wi < m.HELD_MIN:
            return {'ok': False, 'why': 'slipped in the lift (step %d)' % i, 'widths': widths, 'slipped': True}
        if i <= 5 and wi < peak - abort_fall:
            ok = m._put_back(path, fi)
            if not ok:
                return {'ok': False, 'why': 'abort high, put-back refused: HOLDING', 'widths': widths, 'holding': True}
            m._back_off(G, ap, f, ld)
            return {'ok': False, 'why': 'lift aborted step %d (%.4f < peak %.4f - %.3f)' % (i, wi, peak, abort_fall), 'widths': widths}
    st = ctx.value('r1_status')['proprio']
    w = st['gripper_width_m']['right']
    if w < TOP_MIN_W:
        # v5 (s6): a grip that ended the lift at .0285 (falling from .039) dropped on carry step 2, outside the box.
        # Retrace the lift path down and put the puppy back on the table instead of carrying a weak grip.
        ok = m._put_back(path, fi)
        if not ok:
            return {'ok': False, 'why': 'weak grip at the top (%.4f), put-back refused: HOLDING' % w, 'widths': widths, 'holding': True}
        m._back_off(G, ap, f, ld)
        return {'ok': False, 'why': 'weak grip at the top (%.4f < %.3f): put back' % (w, TOP_MIN_W), 'widths': widths}
    return {'ok': True, 'hand': st['grasp_points_pelvis_m']['right'], 'width': w,
            'widths': widths, 'cmd': last, 'yaw_end': yaw_end}


def run(ctx, depth=0.045, secs=1.2, in_m=0.09, lower=0.12, yaw_end=2.0, close_edge=0.43, step=0.10, abort_fall=0.006, min_close=0.039, regrasps=0, regrasp_step=0.015):   # v7: regrasp rule OFF (lifted 1/5 after regrasps; s18 regrasp narrowed .0369 -> .0266 and slipped)   # R9: edge read .402 with the paw hidden -> no waist look at .40
    chain.ctx = ctx; m.CLOSE_EDGE = close_edge; m.MAX_REGRASP = 0
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.5)
    ctx.value('r1_move_joints', joints=m.v4.HOME_BOTH, secs=1.0)
    ctx.value('r1_head', pitch=0.6, secs=1.0)
    meas = {}
    r = m.look(ctx, log=lambda *a: None); meas['look'] = r['measured']
    P(step='look', ok=r['ok'], why=r['why'], sc={k: r['measured'].get(k) for k in ('paw', 'leg_yaw', 'edge', 'wall', 'overhang', 'frame')})
    if not r['ok']:
        return {'ok': False, 'why': 'look: ' + r['why'], 'measured': meas}
    sc = r['measured']
    # R7: far paws (x .38) put G at x .42, out of the lift's reach -> shallower grasp, still >= .03 (shaft)
    depth = round(max(0.03, min(depth, 0.405 - sc['paw'][0])), 3)
    if sc['paw'][1] < -0.30 or sc['paw'][1] > -0.12:
        return {'ok': False, 'why': 'stance: paw y %.3f outside the right-arm window (-.30..-.12)' % sc['paw'][1], 'measured': meas}
    g = m.grasp(ctx, sc, depth, log=lambda *a: None); meas['grasp'] = g['measured']
    P(step='grasp', ok=g['ok'], why=g['why'], depth=depth, close=g['measured'].get('close_width'), hand=g['measured'].get('hand'))
    if not g['ok']:
        return {'ok': False, 'why': 'grasp: ' + g['why'], 'measured': meas}
    # v6 (3d54de): chain lifts completed 4/4 with close >= .039 (s4 .0399, R6 .0471, R8b .0417, R10 .0391) and 1/7
    # below (.032-.0387: s5 s7 s8 s9 s9b R8a aborted, s6 weak) -> regrasp deeper (thicker leg) while the close is narrow
    gm = g['measured']
    for k in range(int(regrasps)):
        if gm['close_width'] >= min_close:
            break
        ld = gm['ld']; G = gm['G']
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.8)
        G2 = [G[0] + regrasp_step * ld[0], G[1] + regrasp_step * ld[1], G[2]]
        if chain.guarded(G2, gm['ap'], gm['f'], w=0.07, secs=1.2) is None:
            P(step='regrasp_refused', G=G2); ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
            gm['close_width'] = ctx.value('r1_status')['proprio']['gripper_width_m']['right']; break
        gg = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.0)
        w2 = gg['gripper']['measured_m']
        P(step='regrasp', k=k, G=[round(v, 3) for v in G2], close=w2)
        gm = dict(gm, G=G2, close_width=w2, hand=gg['grasp_points_pelvis_m']['right'])
        if w2 < m.HELD_MIN:
            m._back_off(G2, gm['ap'], gm['f'], ld)
            return {'ok': False, 'why': 'grasp: regrasp missed (%.4f)' % w2, 'measured': meas}
    meas['grasp'] = gm; g = dict(g, measured=gm)
    L = lift_chain(ctx, g['measured'], secs=secs, yaw_end=yaw_end, abort_fall=abort_fall); meas['lift'] = L
    P(step='lifted', **{k: L.get(k) for k in ('ok', 'why', 'hand', 'width', 'widths')})
    if not L['ok']:
        if L.get('slipped'):
            m.free_arm(ctx, log=lambda *a: None)
        return {'ok': False, 'why': 'lift: ' + L['why'], 'measured': meas}
    hand = L['hand']
    tgt = sc['wall'] + in_m - (hand[0] + BODY_OFF)
    if sc.get('edge') and sc['edge'].get('dist'):
        tgt = min(tgt, sc['edge']['dist'] - EDGE_MIN)
    tgt = round(min(0.30, max(0.04, tgt)), 3)
    n = max(1, int(math.ceil(tgt / step - 1e-6))); st = round(min(0.10, max(0.04, tgt / n)), 3)
    C = ctx.value('r1_carry_release', target_forward_m=tgt, lower_m=lower, max_steps=n, step_m=st)
    o = ctx.value('r1_observe')
    meas['carry'] = {'target': tgt, 'reply': C, 'after_frame': o['camera']['frame_id']}
    P(step='carry', target=tgt, steps=n, step_m=st, reply=C, after_frame=o['camera']['frame_id'])
    ok = bool(C.get('released')) and not C.get('dropped')
    try:
        m.retreat(ctx, 0.2)
    except SystemExit:
        # s14: v4.out's walk back moved .024/-.001/-.009 with the hand out at (.14,-.40,.14) (torso rolled .22) and
        # sys.exit(2) killed the batch. Recovery that worked: hand up, HOME both arms + waist, then walk back (-.207).
        try:
            h = ctx.value('r1_status')['proprio']['grasp_points_pelvis_m']['right']
            if h[2] < 0.12:
                chain.guarded([h[0], h[1], h[2] + 0.10], (0.9, -0.42, 0.12), (0.17, -0.09, -0.98), w=0.07, secs=2.0, max_jump=1.5)
            ctx.value('r1_move_joints', joints=m.v4.HOME_BOTH, secs=3.0)
            wb = ctx.value('r1_walk', forward_m=-0.15)
            P(step='retreat_recovery', walked=wb.get('walked_m'))
            if (wb.get('walked_m') or 0) > -0.05:
                ctx.value('r1_hold')
                return {'ok': False, 'why': 'retreat failed twice (HOLDING, ask for help)', 'measured': meas}
        except Exception as ex:
            ctx.value('r1_hold')
            return {'ok': False, 'why': 'retreat recovery failed (HOLDING): %s' % str(ex)[:200], 'measured': meas}
    except Exception as ex:
        ctx.value('r1_hold')
        return {'ok': False, 'why': 'retreat failed (HOLDING): %s' % str(ex)[:200], 'measured': meas}
    return {'ok': ok, 'why': 'released over the box; verify inside' if ok else 'carry: dropped=%s released=%s' % (C.get('dropped'), C.get('released')), 'measured': meas}


if __name__ == '__main__':
    tag = sys.argv[1]; kw = {}
    for s in sys.argv[2:]:
        k, v = s.split('='); kw[k] = float(v)
    ctx = Context(); t0 = time.time()
    res = run(ctx, **kw); res['tag'] = tag; res['secs'] = round(time.time() - t0, 1); res['variant'] = 'cl'
    with open('output/skill_runs.jsonl', 'a') as fh:
        fh.write(json.dumps(res, default=str) + '\n')
    P(step='RESULT', tag=tag, ok=res['ok'], why=res['why'], secs=res['secs'])
