"""Carry the held plush forward in 4 cm / 5 s steps at palm z ZC to x XR (args: XR ZC), grip check (run_f0.held) each step."""
import sys, os, json
import numpy as np
sys.path.insert(0, os.path.join(os.getcwd(), 'skill-library/skills/plush-pick-place-box-behind/code'))
import run_f0 as F
from api.runtime import Context
xr, zc = float(sys.argv[1]), float(sys.argv[2])
st = F.load(); cx, cy = st['cmd']
with Context() as ctx:
    xs = list(np.arange(cx + 0.04, xr, 0.04)) + [xr]
    for x in xs:
        r = F.move(ctx, st, (float(x), cy, zc), 5.0, check=False)
        print(json.dumps(dict(x=round(float(x), 3), ok=r.get('ok'), why=r.get('why'), palm=r.get('palm'), fingers=r.get('fingers'), tilt=r.get('tilt'), frame=r.get('frame'))), flush=True)
        if not F.held(r):
            print('STOP: grip lost or refused'); break
        st['cmd'] = [float(x), cy]; F.save(st)
