"""Slower lift (session 996d97): 3 cm / 4 s steps from the press command to LIFT_Z, grip check after each.
python code/puppy_box/lift2.py [step=0.03] [secs=4]"""
import sys, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.runtime import Context
dz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03; secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
st = c3.load(); gx, gy = st['grasp']; x = gx + 0.015; z = st['press_z']; hist = []; ok = True
with Context() as ctx:
    while z < c3.LIFT_Z - 1e-3:
        z = min(c3.LIFT_Z, z + dz)
        r = c3.move(ctx, (x, gy, z), c3.GRASP_YAW, secs); r.pop('ob', None)
        hist.append([round(r.get('palm', [0, 0, 0])[2], 3), [round(v, 2) for v in r.get('fingers', [])]])
        if not r['ok'] or not c3.grip(r['fingers']):
            ok = False; break
print(json.dumps({'ok': ok, 'why': '' if ok else (r.get('why') or 'grip lost'), 'palm': r.get('palm'), 'hist': hist, 'frame': r.get('frame')}, default=float))
