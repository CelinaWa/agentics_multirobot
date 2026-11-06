"""Preview r1_move_hand (no motion). python pv2.py x y z ax ay az fx fy fz [use_waist]"""
import sys, json
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T060138Z_puppy_box_r1_2fbfb7/agent-workspace'
sys.path.insert(0, WS)
from api.runtime import Context
ctx = Context()
a = [float(v) for v in sys.argv[1:10]]
uw = len(sys.argv) > 10 and sys.argv[10] == '1'
r = ctx.value('r1_move_hand', arm='right', position=a[0:3], approach=a[3:6], fingers=a[6:9], gripper_width_m=0.07,
              preview_only=True, use_waist=uw)
pv = r.get('preview', r)
print(json.dumps({k: pv.get(k) for k in ('reachable', 'ik_error_m', 'approach_error_deg', 'approach_achieved',
                                          'fingers_axis_achieved', 'joints')}, default=str))
