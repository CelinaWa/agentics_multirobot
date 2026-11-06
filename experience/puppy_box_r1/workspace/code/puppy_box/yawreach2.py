import sys, json, math
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace'
sys.path.insert(0, WS)
from api.runtime import Context
ctx = Context()
psis = [float(v) for v in sys.argv[1].split(',')]
xs = [float(v) for v in sys.argv[2].split(',')]
ys = [float(v) for v in sys.argv[3].split(',')]
z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
for psi in psis:
    a = (math.cos(math.radians(psi)), math.sin(math.radians(psi)), 0.0)
    f = (-a[1], a[0], 0.0)
    row = []
    for x in xs:
        for y in ys:
            r = ctx.value('r1_move_hand', arm='right', position=[x, y, z], approach=list(a), fingers=list(f),
                          gripper_width_m=0.07, preview_only=True)
            pv = r.get('preview', r)
            row.append((x, y, round(pv.get('ik_error_m', 9), 3), round(pv.get('approach_error_deg', 99), 1),
                        round(abs((pv.get('fingers_axis_achieved') or [0, 0, 0])[2]), 2)))
    print(psi, row)
