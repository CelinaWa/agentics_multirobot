"""Holding move(s) with ctl.move at a given max jump, grip-checked: nudge.py max_jump x,y,z[,secs] ..."""
import sys, os, json
sys.path.insert(0, os.path.join(os.getcwd(), 'code/e578'))
import ctl
from api.runtime import Context
st = ctl.F.load(); mj = float(sys.argv[1])
with Context() as ctx:
    for a in sys.argv[2:]:
        v = [float(x) for x in a.split(',')]
        r = ctl.move(ctx, st, tuple(v[:3]), v[3] if len(v) > 3 else 5.0, check=False, max_jump=mj)
        print(json.dumps(dict(cmd=v, held=bool(ctl.F.held(r)), **{k: r.get(k) for k in ('ok', 'why', 'palm', 'fingers', 'tilt', 'jump', 'json')})), flush=True)
        if not ctl.F.held(r):
            break
        st['cmd'] = [v[0], v[1]]; ctl.F.save(st)
