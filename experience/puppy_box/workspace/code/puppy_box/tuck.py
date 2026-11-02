"""Tuck both arms back and up (shoulder pitch first, then elbow): sp 0.45, el -0.5; hands open. Then observe."""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from api.runtime import Context

if __name__ == '__main__':
    sides = sys.argv[1:] or ['left', 'right']
    with Context() as ctx:
        m = lambda d: {k: v for s in sides for k, v in c.mirror(s, d).items()}
        c.pps.ramp(ctx, m({'shoulder_pitch': 0.45}), 2.5, hands=(0.0, 0.0))
        c.pps.ramp(ctx, m({'elbow': -0.5}), 3.0, hands=(0.0, 0.0))
        time.sleep(1.0)
        ob, path = c.observe(ctx, 'tuck')
        print(json.dumps({'tilt': ob['tilt_deg'], 'frame': ob['camera']['frame_id'], 'json': path}))
