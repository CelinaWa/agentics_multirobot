"""Palm-down (yaw 0) moves through waypoints with run_f0.move (commanded-row IK + FK path check vs puppy and wall).
Usage: goto.py [nocheck] x,y,z,secs [x,y,z,secs ...]   (state: output/f0_state.json). Prints the measured result of each."""
import sys, os, json
W = os.getcwd(); sys.path.insert(0, os.path.join(W, 'skill-library/skills/plush-pick-place-box-behind/code'))
import run_f0 as F
from api.runtime import Context
args = sys.argv[1:]; check = True
if args and args[0] == 'nocheck':
    check = False; args = args[1:]
st = F.load()
with Context() as ctx:
    for a in args:
        x, y, z, s = map(float, a.split(','))
        r = F.move(ctx, st, (x, y, z), s, check=check)
        print(json.dumps(dict(cmd=[x, y, z], **{k: r.get(k) for k in ('ok', 'why', 'palm', 'tips_x', 'fingers', 'tilt', 'frame', 'left_palm')})), flush=True)
        if not r['ok']:
            break
