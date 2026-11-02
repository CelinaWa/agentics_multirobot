"""Prep that keeps the right hand at the side (y <= -0.30) while raising it, so it never passes over a puppy lying
straight ahead (37fe31 incident: pb3 prep swung the hand forward through the puppy). Left arm stays tucked."""
import sys, os, json, time
sys.path.insert(0, 'code/puppy_box')
import controller as c
from api.runtime import Context
R = lambda d: c.mirror('right', d)
with Context() as ctx:
    c.pps.ramp(ctx, c.mirror('left', {'shoulder_pitch': 0.45, 'elbow': -0.5}), 2.5, hands=(0.0, 0.0))
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.9}), 2.5)
    c.pps.ramp(ctx, R({'shoulder_roll': -0.45}), 2.5)
    c.pps.ramp(ctx, R({'elbow': -0.9}), 3.0)
    c.pps.ramp(ctx, R({'shoulder_pitch': 0.1}), 3.0)
    c.pps.ramp(ctx, R({'wrist_roll': -1.3}), 2.5)
    time.sleep(1.0)
    ob, path = c.observe(ctx, 'prep_side')
    print(json.dumps(dict(tilt=ob['tilt_deg'], path=path)))
