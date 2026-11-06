import sys, json, math
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T043952Z_puppy_box_r1_cf6e88/agent-workspace')
from api.runtime import Context
ctx = Context()
z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.01
for a in (0, -15, -30, -45):
    row = []
    for x in (0.33, 0.38, 0.43):
        for y in (-0.10, -0.15, -0.20, -0.25, -0.30):
            ar = math.radians(a)
            ap = [math.cos(ar), math.sin(ar), 0]
            f = [-math.sin(ar), math.cos(ar), 0]
            r = ctx.value('r1_move_hand', arm='right', position=[x, y, z], approach=ap, fingers=f, gripper_width_m=0.07, preview_only=True)
            pv = r.get('preview', r)
            fa = pv.get('fingers_axis_achieved')
            ok = abs(fa[2]) < 0.3 if fa else False
            row.append(f"{x:.2f},{y:+.2f}:{pv.get('ik_error_m', 9):.3f}{'' if ok else 'V'}")
    print(a, ' '.join(row), flush=True)
