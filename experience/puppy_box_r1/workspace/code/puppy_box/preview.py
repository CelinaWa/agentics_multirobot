"""Preview right-hand goals: python preview.py x y z ax ay az fx fy fz [x y z ...]"""
import sys, json
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace')
from api.runtime import Context
ctx = Context()
a = [float(v) for v in sys.argv[1:]]
for i in range(0, len(a), 9):
    p, ap, f = a[i:i+3], a[i+3:i+6], a[i+6:i+9]
    r = ctx.value('r1_move_hand', arm='right', position=p, approach=ap, fingers=f, gripper_width_m=0.07, preview_only=True)
    pv = r.get('preview', r)
    print(json.dumps({'goal': p, 'reach': r.get('reachable', pv.get('reachable')), 'ik_err': pv.get('ik_error_m'),
                      'appr': pv.get('approach_achieved'), 'fing': pv.get('fingers_axis_achieved'),
                      'j': {k[6:-6]: round(v, 2) for k, v in pv.get('joints', {}).items() if k.startswith('right')}}))
