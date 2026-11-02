"""Open the right hand, then raise the palm straight up to z (default 0.30) at the current waist cmd."""
import sys, json, numpy as np
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
from api.runtime import Context
z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
with Context() as ctx:
    pbb.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    row, _, _ = pbb.c.pps.last_row(ctx)
    pa = np.array(pbb.c.pps.link_points(row[9:38], 'right')['right_palm'])
    r = pbb.move(ctx, (pa[0] - 0.03, pa[1], z), 0.0, 3.0, max_jump=1.6); r.pop('ob', None)
    print(json.dumps(r, default=float))
