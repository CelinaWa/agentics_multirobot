"""Like goto.py but with per-waypoint finger yaw: goto2.py x,y,z,secs,yaw_deg ... (ctl.move, max jump 1.0, path check)."""
import sys, os, json
sys.path.insert(0, os.path.join(os.getcwd(), 'code/e578'))
import ctl
from api.runtime import Context
st = ctl.F.load()
with Context() as ctx:
    for a in sys.argv[1:]:
        x, y, z, s, yaw = map(float, a.split(','))
        ctl.YAW = yaw
        r = ctl.move(ctx, st, (x, y, z), s, max_jump=1.0)
        print(json.dumps(dict(cmd=[x, y, z, yaw], **{k: r.get(k) for k in ('ok', 'why', 'palm', 'tips_x', 'jump', 'tilt', 'frame')})), flush=True)
        if not r['ok']:
            break
