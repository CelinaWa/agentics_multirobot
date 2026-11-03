"""No-motion measure (run_f0.measure): palm, finger links, fingers, tilt, frame."""
import sys, os, json
sys.path.insert(0, os.path.join(os.getcwd(), 'skill-library/skills/plush-pick-place-box-behind/code'))
import run_f0 as F
from api.runtime import Context
with Context() as ctx:
    r = F.measure(ctx)
    ob = json.load(open(r['json']))
    r['waist'] = [round(ob['proprio']['joint_positions_rad'][i], 3) for i in (2, 5, 8)]
    print(json.dumps(r))
