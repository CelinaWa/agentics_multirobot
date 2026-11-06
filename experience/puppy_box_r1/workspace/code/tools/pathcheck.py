"""Preview a sequence of right-hand waypoints with a fixed approach/fingers; report ik error and joint jumps.
python pathcheck.py '<json list of [x,y,z]>' '[ax,ay,az]' '[fx,fy,fz]'"""
import sys, json
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T232821Z_puppy_box_r1_c4f6c3/agent-workspace')
from api.runtime import Context
pts = json.loads(sys.argv[1]); appr = json.loads(sys.argv[2]); fing = json.loads(sys.argv[3])
prev = None
with Context() as ctx:
    for p in pts:
        r = ctx.value('r1_move_hand', arm='right', position=p, approach=appr, fingers=fing, preview_only=True)
        pv = r.get('preview', r)
        j = pv.get('joints', {})
        jv = [j.get(k, 0) for k in sorted(j)]
        jump = max(abs(a - b) for a, b in zip(jv, prev)) if prev else 0
        prev = jv
        print(p, 'ik %.4f appr_err %s maxjump %.2f' % (pv.get('ik_error_m', -1), pv.get('approach_error_deg'), jump),
              {k.replace('right_', '').replace('_joint', ''): round(v, 2) for k, v in j.items()})
