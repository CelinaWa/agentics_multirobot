"""Preview right-hand goals. python pv.py x,y,z ax,ay,az fx,fy,fz [waist]"""
import sys, json
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace')
from api.runtime import Context
ctx = Context()
p = [float(v) for v in sys.argv[1].split(',')]
a = [float(v) for v in sys.argv[2].split(',')]
f = [float(v) for v in sys.argv[3].split(',')]
w = len(sys.argv) > 4 and sys.argv[4] == '1'
r = ctx.value('r1_move_hand', arm='right', position=p, approach=a, fingers=f, preview_only=True, use_waist=w, gripper_width_m=0.07)
pv = r.get('preview', r)
print(json.dumps({k: pv.get(k) for k in ('reachable', 'ik_error_m', 'approach_error_deg', 'approach_axis_achieved', 'fingers_axis_achieved', 'joints', 'reason')}, default=str))
