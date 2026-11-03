"""Closed raise: move the palm to (cmd x, cmd y, measured z + DZ) with ctl.move (max jump arg), grip-checked."""
import sys, os, json
sys.path.insert(0, os.path.join(os.getcwd(), 'code/e578'))
import ctl
from api.runtime import Context
dz, mj = float(sys.argv[1]), float(sys.argv[2])
st = ctl.F.load(); cx, cy = st['cmd']
with Context() as ctx:
    m = ctl.F.measure(ctx)
    r = ctl.move(ctx, st, (cx, cy, m['palm'][2] + dz), 3.0, check=False, max_jump=mj)
    print(json.dumps(dict(held=bool(ctl.F.held(r)), **{k: r.get(k) for k in ('ok', 'why', 'palm', 'fingers', 'tilt', 'jump', 'frame')})))
