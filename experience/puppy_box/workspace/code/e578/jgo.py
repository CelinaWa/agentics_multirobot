"""Joint-space ramp of the right arm to given joints (json dict of short names) over secs. Usage: jgo.py '<json>' secs"""
import sys, os, json, time
W = os.getcwd()
for d in ('skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import controller as c
from api.runtime import Context
t = {f'right_{k}_joint': float(v) for k, v in json.loads(sys.argv[1]).items()}
with Context() as ctx:
    c.pps.ramp(ctx, t, float(sys.argv[2])); time.sleep(1.0)
