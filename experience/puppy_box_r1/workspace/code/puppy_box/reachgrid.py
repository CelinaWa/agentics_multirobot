import sys, json, itertools
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T060138Z_puppy_box_r1_2fbfb7/agent-workspace')
from api.runtime import Context
pts = json.loads(sys.argv[1]); appr = json.loads(sys.argv[2]) if len(sys.argv) > 2 else None
waist = len(sys.argv) > 3 and sys.argv[3] == 'waist'
with Context() as ctx:
    for p in pts:
        kw = dict(arm='right', position=p, preview_only=True, use_waist=waist)
        if appr: kw['approach'] = appr
        try:
            r = ctx.value('r1_move_hand', **kw)
            print(p, {k: r.get(k) for k in ('ik_error_m', 'approach_error_deg', 'reachable', 'ok', 'refused') if k in r} or str(r)[:300])
        except Exception as e:
            print(p, 'ERR', str(e)[:250])
