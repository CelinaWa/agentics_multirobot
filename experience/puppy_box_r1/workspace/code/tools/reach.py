"""Preview-only reach grid for the right hand. python reach.py '<json list of [x,y,z]>' '[ax,ay,az]|null' '[fx,fy,fz]|null' [waist]"""
import sys, json
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T232821Z_puppy_box_r1_c4f6c3/agent-workspace')
from api.runtime import Context
pts = json.loads(sys.argv[1]); appr = json.loads(sys.argv[2]) if len(sys.argv) > 2 else None
fing = json.loads(sys.argv[3]) if len(sys.argv) > 3 else None
waist = len(sys.argv) > 4 and sys.argv[4] == 'waist'
with Context() as ctx:
    for p in pts:
        kw = dict(arm='right', position=p, preview_only=True, use_waist=waist)
        if appr: kw['approach'] = appr
        if fing: kw['fingers'] = fing
        try:
            r = ctx.value('r1_move_hand', **kw)
            pv = r.get('preview', r)
            j = pv.get('joints', {})
            print(p, 'ik %s appr %s appr_ach %s fing %s waist %s' % (pv.get('ik_error_m'), pv.get('approach_error_deg'),
                  pv.get('approach_achieved'), pv.get('fingers_axis_achieved'),
                  {k: v for k, v in j.items() if 'waist' in k}), r.get('refused', ''), r.get('reason', ''))
        except Exception as e:
            print(p, 'ERR', str(e)[:300])
