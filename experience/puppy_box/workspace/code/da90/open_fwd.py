"""After an empty close: open the hand where it is, then shift the grasp DX forward (state 'far' += DX so hover's
far - GRASP_BACK lands DX further). Usage: PYTHONPATH=. python code/da90/open_fwd.py DX ; then ctl.py hover"""
import sys, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.runtime import Context
dx = float(sys.argv[1])
with Context() as ctx:
    c3.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(1.0)
st = c3.load(); st['far'] += dx; c3.save(st)
print(json.dumps(dict(ok=True, far=st['far'])))
