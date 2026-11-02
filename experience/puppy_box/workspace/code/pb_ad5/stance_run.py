import sys, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
from api.runtime import Context
import stance
stance.EDGE_OK, stance.EDGE_GOAL = 0.205, 0.175
ctx = Context()
print(json.dumps(stance.stance(ctx), default=float))
