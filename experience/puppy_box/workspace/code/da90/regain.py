"""After a failed close at the far stance: hand straight up to top+0.15 (open), waist yaw/roll back to 0 with the hand high,
crouch(0, forward_rad FWD, arms_home False) to bring the pelvis forward, observe. Usage: PYTHONPATH=. python code/da90/regain.py [FWD]"""
import sys, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.runtime import Context
fwd = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
st = c3.load(); gx, gy = st['grasp']
with Context() as ctx:
    c3.c.pps.ramp(ctx, {}, 1.0, hands=(0.0, 0.0))
    r = c3.move(ctx, (gx - 0.02, gy, st['top'] + 0.15), c3.GRASP_YAW, 3.0, max_jump=1.6); r.pop('ob', None)
    if not r['ok']:
        print(json.dumps(dict(ok=False, why=r['why']))); sys.exit(1)
    c3.waypoint(ctx, (0.24, -0.10, 0.30), 0, 0.0, 5.0, max_jump=1.6)
    c3.c.pps.ramp(ctx, {'waist_roll_joint': 0.0, 'waist_pitch_joint': 0.0}, 3.0)
    ctx.value('g1_crouch', pelvis_drop_m=0, forward_rad=fwd, arms_home=False, secs=3)
    time.sleep(1.0)
    ob, _ = c3.c.observe(ctx, 'da90_regain')
    print(json.dumps(dict(ok=ob['tilt_deg'] < 6, tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])))
