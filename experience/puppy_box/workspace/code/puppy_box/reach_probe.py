"""Empty-hand reach probe at the far reach limit (box behind the puppy): waist yaw cmd + arm to a high hover, lean in its
own ramp, palm commanded to the target, measured palm reported, then hand back and unlean at once.
python code/puppy_box/reach_probe.py LEAN WAIST_YAW x y z"""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from mv import move
from carry import waypoint

if __name__ == '__main__':
    lean, wy, x, y, z = [float(v) for v in sys.argv[1:6]]
    from api.runtime import Context
    out = {}
    with Context() as ctx:
        r = waypoint(ctx, (0.30, y, z + 0.04), 0, wy, 5.0, max_jump=1.6); r.pop('ob', None); out['hover'] = r.get('palm') or r.get('why')
        c.pps.ramp(ctx, {'waist_pitch_joint': lean}, 4.0)
        ob = ctx.value('g1_observe'); out['tilt_lean'] = ob['tilt_deg']
        if ob['tilt_deg'] < 6:
            r = move(ctx, (x, y, z), -10, 4.0, max_jump=1.6); ob2 = r.pop('ob', None)
            out['reach'] = {k: r.get(k) for k in ('ok', 'why', 'palm', 'tilt', 'waist', 'frame')}
        r = move(ctx, (0.30, y, z + 0.04), 0, 3.0, max_jump=1.6); r.pop('ob', None); out['back'] = r.get('palm') or r.get('why')
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.0}, 4.0)
        ob = ctx.value('g1_observe'); out['tilt_end'] = ob['tilt_deg']
    print(json.dumps(out, default=float))
