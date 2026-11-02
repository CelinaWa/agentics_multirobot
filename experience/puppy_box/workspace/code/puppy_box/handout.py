"""Move the right hand up-left out of the camera view (box-behind skill step 3), waist yaw cmd 0."""
import sys, os, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
from api.runtime import Context
with Context() as ctx:
    r = pbb.waypoint(ctx, (0.22, 0.06, 0.32), 20, 0.0, 5.0, max_jump=1.6)
    r.pop('ob', None); print(json.dumps(r, default=float))
