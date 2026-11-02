"""Slow lift / carry step with the closed hand: python code/puppy_box/lift.py x y z secs bias
Prints palm and right-hand finger state; grip = index_0 measured (< 1.3 means something is in the hand)."""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library/skills/palm-pose-servo/code'))
from api.runtime import Context
import run as pps

if __name__ == '__main__':
    x, y, z, secs = map(float, sys.argv[1:5])
    bias = [float(v) for v in sys.argv[5].split(',')]
    with Context() as ctx:
        r = pps.main(ctx, os.environ.get('SIDE', 'right'), (x, y, z), yaw_deg=float(os.environ.get('YAW', '0')), iters=1, bias=bias, secs=secs, log=lambda *_: None)
        m = r['measured']
        hs = m.get('hand_state', []); h = [abs(v) for v in (hs[:7] if os.environ.get('SIDE', 'right') == 'left' else hs[7:])]
        print(json.dumps({'palm': m.get('palm'), 'tilt': m.get('tilt_deg'), 'frame': m.get('frame_id'),
                          'hand_R': [round(v, 2) for v in h], 'grip': bool(h and min(h[3], h[5]) < 1.3), 'why': r.get('why')}))
