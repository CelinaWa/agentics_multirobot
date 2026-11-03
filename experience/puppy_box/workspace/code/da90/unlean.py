"""Waist pitch back to 0 (hand stays where it is), observe. Usage: PYTHONPATH=. python code/da90/unlean.py"""
import sys, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.runtime import Context
with Context() as ctx:
    c3.c.pps.ramp(ctx, {'waist_pitch_joint': 0.0}, 3.0)
    time.sleep(1.0)
    ob, path = c3.c.observe(ctx, 'unlean')
    j = ob['proprio']['joint_positions_rad']
    print(json.dumps(dict(tilt=ob['tilt_deg'], waist=[round(j[2], 3), round(j[5], 3), round(j[8], 3)], frame=ob['camera']['frame_id'], path=path)))
