"""Right palm to x,y,z (commanded-row IK) keeping the hand fraction: up.py x y z yaw secs. Prints palm, fingers, tilt."""
import sys, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
from api.runtime import Context
x, y, z, yaw, secs = [float(v) for v in sys.argv[1:6]]
with Context() as ctx:
    r = pbb.move(ctx, (x, y, z), yaw, secs, max_jump=1.6)
    ob = r.pop('ob', None)
    f = pbb.c.fingers(ob, 'right') if ob else None
    print(json.dumps(dict(r, fingers=[round(v, 2) for v in f] if f else None, frame=ob['camera']['frame_id'] if ob else None), default=float))
