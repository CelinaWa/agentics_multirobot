"""Preview-only IK grid: right hand at table height with approach yawed psi (deg) in the pelvis frame, fingers
horizontal. No motion. python yawreach.py"""
import sys, json, math
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace'
sys.path.insert(0, WS)
from api.runtime import Context
ctx = Context()
out = []
for psi in (-45, -30, -15, 0):
    a = (math.cos(math.radians(psi)), math.sin(math.radians(psi)), 0.0)
    f = (-a[1], a[0], 0.0)
    for x in (0.30, 0.36, 0.42):
        for y in (-0.15, -0.22, -0.30, -0.38):
            r = ctx.value('r1_move_hand', arm='right', position=[x, y, 0.0], approach=list(a), fingers=list(f),
                          gripper_width_m=0.07, preview_only=True)
            pv = r.get('preview', r)
            out.append((psi, x, y, round(pv.get('ik_error_m', 9), 3), round(pv.get('approach_error_deg', 99), 1),
                        round(abs((pv.get('fingers_axis_achieved') or [0, 0, 0])[2]), 2)))
for o in out:
    print(o)
