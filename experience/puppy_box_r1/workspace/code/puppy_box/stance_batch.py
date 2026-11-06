"""Unattended stance-placement batch (sim accelerator): for each run ask the operator for a grasp stance, run
cl.run (+ one in-place retry after a safe lift abort / grasp miss), ask the ground-truth question, log a line.
    python code/puppy_box/stance_batch.py <first_tag_number> <n_runs>      -> output/stance_batch.jsonl"""
import sys, json, time, re, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('cl', 'code/puppy_box/cl.py'); cl = importlib.util.module_from_spec(spec); spec.loader.exec_module(cl)
ctx = Context()
k0, n = int(sys.argv[1]), int(sys.argv[2])
KW = {a.split('=')[0]: float(a.split('=')[1]) for a in sys.argv[3:]}   # passed to cl.pick/run
for k in range(k0, k0 + n):
    tag = 's%d' % k; t0 = time.time()
    a = ctx.value('request_operator', kind='help', message='Please place me at the grasp stance.', timeout_s=300)
    rep = a.get('reply', '') if isinstance(a, dict) else str(a)
    mm = re.search(r'paw is at \(([-\d.]+), ([-\d.]+)', rep)
    op_paw = [float(mm.group(1)), float(mm.group(2))] if mm else None
    if 'stance' not in rep:
        print(json.dumps({'tag': tag, 'stop': 'no stance placement', 'reply': rep[:200]}), flush=True); break
    tries = []
    for t in range(2):
        try:
            res = cl.run(ctx, **KW)
        except Exception as ex:
            ctx.value('r1_hold'); res = {'ok': False, 'why': 'EXCEPTION (holding): %s' % str(ex)[:200], 'measured': {}}
        L = res['measured'].get('lift') or {}
        C = (res['measured'].get('carry') or {}).get('reply') or {}
        tries.append({'why': res['why'], 'paw': (res['measured'].get('look') or {}).get('paw'),
                      'close': (res['measured'].get('grasp') or {}).get('close_width'), 'widths': L.get('widths'),
                      'carry': {k_: C.get(k_) for k_ in ('walked_total_m', 'dropped', 'released', 'width_before_release_m')} if C else None})
        if res['ok'] or not re.match(r'(lift: lift aborted|lift: weak grip|grasp: )', res['why']):
            break
    inside = None
    if res['ok'] or 'carry' in res['why']:
        q = ctx.value('request_operator', kind='help', message='Question only, please do not move anything: is the puppy inside the box?', timeout_s=120)
        qr = q.get('reply', '') if isinstance(q, dict) else str(q)
        inside = 'INSIDE' in qr and 'NOT inside' not in qr
        tries[-1]['operator'] = qr[:260]
    line = {'tag': tag, 'kw': KW, 'op_paw': op_paw, 'inside': inside, 'tries': tries, 'secs': round(time.time() - t0, 1)}
    with open('output/stance_batch.jsonl', 'a') as fh:
        fh.write(json.dumps(line, default=str) + '\n')
    print(json.dumps({'tag': tag, 'op_paw': op_paw, 'inside': inside, 'whys': [t['why'][:80] for t in tries], 'secs': line['secs']}), flush=True)
    if any('HOLDING' in t['why'] or 'EXCEPTION' in t['why'] for t in tries):
        print(json.dumps({'stop': 'holding/exception'}), flush=True); break
