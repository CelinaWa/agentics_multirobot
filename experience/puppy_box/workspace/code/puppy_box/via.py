"""Waypoints for the right palm: via.py yaw waist secs x,y,z [x,y,z ...] (commanded-row IK, max_jump 1.6). Stops on refusal."""
import sys, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
from api.runtime import Context
yaw, w, secs = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
with Context() as ctx:
    for s in sys.argv[4:]:
        pos = tuple(float(v) for v in s.split(','))
        r = pbb.waypoint(ctx, pos, yaw, w, secs, max_jump=1.6); r.pop('ob', None)
        print(json.dumps(r, default=float), flush=True)
        if not r['ok']:
            sys.exit(1)
