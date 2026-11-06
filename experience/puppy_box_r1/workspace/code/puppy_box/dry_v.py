"""Dry IK preview of the v4 grasp + lift path with vertical or horizontal fingers. python code/puppy_box/dry_v.py [vertical 1/0] [depth]"""
import sys, math, json, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('s', 'code/puppy_box/lowtilt_v4.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ctx = Context(); m.chain.ctx = ctx
vert = int(sys.argv[1]) if len(sys.argv) > 1 else 1; dep = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03
ctx.value('r1_head', pitch=0.6, secs=0.5)
r = m.look(ctx, log=lambda *a: None); sc = r['measured']; print('look', r['ok'], sc.get('paw'), sc.get('leg_yaw'), sc.get('wall'))
yaw = math.radians(sc['leg_yaw']); ld = (math.cos(yaw), math.sin(yaw))
f = [0, 0, 1] if vert else [-ld[1], ld[0], 0]
P = sc['paw']; G = [P[0] + dep * ld[0], P[1] + dep * ld[1], P[2]]
pre = [G[0] - 0.08 * ld[0], G[1] - 0.08 * ld[1], G[2]]
ap = m.chain.choose_approach(pre, sc['leg_yaw'], f)
prev = None
def pv(pos, a):
    global prev
    p = ctx.value('r1_move_hand', arm='right', position=[float(v) for v in pos], approach=[float(v) for v in a], fingers=f,
                  gripper_width_m=0.0, preview_only=True)['preview']
    q = p['joints']; jump = max(abs(q[k] - prev[k]) for k in q if 'right' in k) if prev else 0; prev = q
    print([round(v, 3) for v in pos], 'ik %.4f' % p['ik_error_m'], 'jump %.2f' % jump, 'wr %.2f' % q['right_wrist_roll_joint'],
          'fing', [round(v, 2) for v in p.get('fingers_axis_achieved', [])], 'ap', [round(v, 2) for v in p['approach_achieved']])
pv(pre, ap); pv(G, ap)
hz = math.hypot(ap[0], ap[1]); x, y, z = G
for da, dz, k in m.PROFILE:
    if z + dz > 0.29: break
    x, y, z = x + da * ld[0], y + da * ld[1], z + dz
    pv((x, y, z), (ap[0] / hz, ap[1] / hz, 0.15 * k))
for j in range(2):
    x, y = x + 0.03 * ld[0], y + 0.03 * ld[1]; pv((x, y, z), (ap[0] / hz, ap[1] / hz, 0.15))
