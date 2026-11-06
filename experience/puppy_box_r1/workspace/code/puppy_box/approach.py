"""Deterministic approach from the reset start pose (puppy_box_r1 sim layout: box behind, overhang hind paw ~(.546,-.023)).
Steps: waist 0 + head 0.5, back -0.1 x3, turn 70, turn -70; prints each walk reply. Then observe twice (settle check).
python approach.py"""
import sys, json, time
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T060138Z_puppy_box_r1_2fbfb7/agent-workspace')
from api.runtime import Context
ctx = Context()
ctx.value('r1_move_joints', joints={'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}, secs=1.0)
ctx.value('r1_head', pitch=0.5, secs=1.0)
for kw in [dict(forward_m=-0.1)] * 3 + [dict(turn_deg=70), dict(turn_deg=-70)]:
    r = ctx.value('r1_walk', **kw)
    print(json.dumps({'cmd': kw, **{k: r.get(k) for k in ('walked_m', 'pelvis', 'tilt_deg', 'fallen')}}), flush=True)
    if r.get('fallen') or (r.get('tilt_deg') or 0) > 8:
        print('STOP: tilt/fall'); sys.exit(2)
