"""Ramp named joints: python ramp.py secs 'left:shoulder_pitch=0.7,left:elbow=-0.9,waist_yaw_joint=-0.5' -> prints palms."""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c, palms
from api.runtime import Context
if __name__ == '__main__':
    secs = float(sys.argv[1]); t = {}
    for kv in sys.argv[2].split(','):
        k, v = kv.split('=')
        if ':' in k:
            s, n = k.split(':'); t.update(c.mirror(s, {n: float(v)}))
        else:
            t[k] = float(v)
    with Context() as ctx:
        rep = c.pps.ramp(ctx, t, secs)
        time.sleep(1.0)
        ob, path = c.observe(ctx, 'ramp')
        j = ob['proprio']['joint_positions_rad']
        fb = rep['transition']['feedback']
        print(json.dumps({'palms': palms.palms(j), 'waist_ypr': [round(j[2], 3), round(j[8], 3), round(j[5], 3)],
                          'tilt': ob['tilt_deg'], 'max_tilt': fb.get('max_root_orientation_projection_deg'),
                          'track': fb.get('max_joint_tracking_error_rad'), 'frame': ob['camera']['frame_id'], 'json': path}))
