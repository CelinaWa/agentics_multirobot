"""Controller v4 for puppy_box_r1 from a grasp stance: up to N in-place attempts of the v4 skill (look -> grasp ->
low-tilt lift -> [carry precondition] -> advance -> carry/release -> retreat). A lift abort retraces, drags the paw back
2.5 cm (overhang restored) and opens low, so the next attempt re-perceives and re-grasps. Stops on success, on a
carry failure, on 'HOLDING' (ask for help) or when the look finds no clean paw.
    python code/puppy_box/ctl.py <tag> [tries=3] [key=value ...]   (keys of lowtilt_v4.main)"""
import sys, json, time, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('s', 'code/puppy_box/lowtilt_v4.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

if __name__ == '__main__':
    tag = sys.argv[1]; kw = {}; tries = 3
    for a in sys.argv[2:]:
        k, v = a.split('=')
        if k == 'tries':
            tries = int(v)
        else:
            kw[k] = float(v)
    ctx = Context(); t0 = time.time(); hist = []
    # v4 stance check (5600c1 tally: lift completed 1/9 with the paw at x < .32, 5/8 at x >= .32): step back while the
    # paw is closer than X_MIN, forward if it ends beyond X_MAX (backward walks overshoot 1.5-3.5x and drift right)
    X_MIN, X_MAX = kw.pop('x_min', 0.0), kw.pop('x_max', 9.0)   # OFF by default: j20 walks fixed x (.312 -> .368) but drifted y -.19 -> -.14 (lift failed); enable with x_min=0.315 x_max=0.40
    m.chain.ctx = ctx
    if 'close_edge' in kw:
        m.CLOSE_EDGE = kw.pop('close_edge')   # 3d54de: paw cut at the image corner with edge .375 -> waist look
    ctx.value('r1_move_joints', joints=m.v4.HOME_BOTH, secs=1.0); ctx.value('r1_head', pitch=0.6, secs=0.5)
    for k in range(3):
        r = m.look(ctx, log=lambda *a: None)
        px = r['measured'].get('paw', [None])[0] if r['ok'] else None
        print('STANCE', k, json.dumps({'ok': r['ok'], 'paw': r['measured'].get('paw'), 'edge': r['measured'].get('edge')}), flush=True)
        if px is None or X_MIN <= px <= X_MAX:
            break
        w = ctx.value('r1_walk', forward_m=-0.05 if px < X_MIN else 0.07)
        print('WALK', json.dumps({'walked': w.get('walked_m'), 'tilt': w.get('tilt_deg')}), flush=True)
        time.sleep(1.5)
    for i in range(tries):
        res = m.main(ctx, **kw)
        why = res['why']; hist.append(why)
        res['tag'] = '%s.%d' % (tag, i); res['secs'] = round(time.time() - t0, 1)
        with open('output/skill_runs.jsonl', 'a') as fh:
            fh.write(json.dumps(res, default=str) + '\n')
        print('ATTEMPT', i, json.dumps({'ok': res['ok'], 'why': why, 'secs': res['secs']}), flush=True)
        if res['ok'] or 'HOLDING' in why or why.startswith(('carry', 'look', 'retreat')):
            break
        if not (why.startswith('lift: lift aborted') or why.startswith('grasp:')):
            break
    print('RESULT', json.dumps({'tag': tag, 'ok': res['ok'], 'attempts': len(hist), 'whys': hist, 'secs': round(time.time() - t0, 1)}))
