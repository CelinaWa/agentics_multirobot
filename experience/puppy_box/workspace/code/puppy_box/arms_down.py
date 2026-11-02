"""Both arms down beside the hips (shoulder pitch back first, then elbows straighter) before a walk; observe."""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from api.runtime import Context

if __name__ == '__main__':
    with Context() as ctx:
        both = lambda d: dict(c.mirror('right', d), **c.mirror('left', d))
        c.pps.ramp(ctx, both({'shoulder_pitch': 0.4}), 2.5, hands=(0.0, 0.0))
        c.pps.ramp(ctx, both({'elbow': 0.3}), 2.5, hands=(0.0, 0.0))
        time.sleep(1.0)
        ob, path = c.observe(ctx, 'armsdown')
        print(json.dumps({'tilt': ob['tilt_deg'], 'frame': ob['camera']['frame_id'], 'json': path}))
