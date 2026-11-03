"""Open in place, press the open hand down in 1.5 cm / 2 s steps to command z ZMIN (stop on tilt > TILT), close 3 s, report fingers.
  ZMIN=-0.01 PYTHONPATH=. python code/p258/regrasp.py"""
import sys, os, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd')
import cr  # noqa
pb3 = cr.pb3
from api.runtime import Context
st = pb3.load(); pb3.GRASP_YAW = cr.lr.GRASP_YAW = st['gyaw']
zmin = float(os.environ.get('ZMIN', -0.01)); tmax = float(os.environ.get('TILT', 6.0))
with Context() as ctx:
    pb3.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0)); time.sleep(0.5)
    gx, gy = st['grasp']; z = st['press_z']; hist = []
    while z > zmin + 1e-3:
        z = max(zmin, z - 0.015)
        r = pb3.move(ctx, (gx, gy, z), st['gyaw'], 2.0); r.pop('ob', None)
        if not r['ok']:
            z += 0.015; break
        hist.append([round(z, 3), round(r['palm'][2], 3), r['tilt']])
        if r['tilt'] > tmax:
            break
    pb3.c.pps.ramp(ctx, {}, 3.0, hands=(0.0, 1.0)); time.sleep(1.0)
    ob, _ = pb3.c.observe(ctx, 'regrasp')
    f = pb3.c.fingers(ob, 'right')
    st['press_z'] = z; pb3.save(st)
    print(json.dumps(dict(ok=pb3.grip(f), hist=hist, fingers=[round(v, 2) for v in f], tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])))
