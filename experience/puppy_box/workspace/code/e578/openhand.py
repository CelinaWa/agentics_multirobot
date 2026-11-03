"""Open the right hand in place (pinch-open shape) over 1.5 s, then measure."""
import sys, os, json, time
sys.path.insert(0, os.path.join(os.getcwd(), 'skill-library/skills/plush-pick-place-box-behind/code'))
import run_f0 as F
from api.runtime import Context
with Context() as ctx:
    F.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0)); time.sleep(1.0)
    print(json.dumps(F.measure(ctx)))
