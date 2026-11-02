"""Palm servo with an unclipped (|c|<=0.15) measured-FK correction:
python code/puppy_box/servo2.py x y z iters bx,by,bz [yaw_deg]   (SIDE, SECS env). Prints the final bias."""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library/skills/palm-pose-servo/code'))
from api.runtime import Context
import run as pps

if __name__ == '__main__':
    x, y, z = map(float, sys.argv[1:4]); iters = int(sys.argv[4])
    b = np.array([float(v) for v in sys.argv[5].split(',')])
    yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    side = os.environ.get('SIDE', 'left'); secs = float(os.environ.get('SECS', '2.5'))
    tgt = np.array([x, y, z])
    with Context() as ctx:
        for it in range(iters):
            r = pps.main(ctx, side, tuple(tgt), yaw_deg=yaw, bias=list(b), secs=secs, log=lambda *_: None)
            m = r.get('measured', {})
            if 'palm' not in m:
                print(json.dumps({'it': it, 'why': r.get('why')})); break
            e = tgt - np.array(m['palm'])
            hs = m['hand_state']; h = [abs(v) for v in (hs[:7] if side == 'left' else hs[7:])]
            print(json.dumps({'it': it, 'palm': m['palm'], 'err': round(float(np.linalg.norm(e)), 3), 'bias': [round(v, 4) for v in b],
                              'fingers': [round(v, 2) for v in h], 'tilt': m['tilt_deg'], 'frame': m['frame_id']}))
            if it < iters - 1:
                b = np.clip(b + 0.8 * e, -0.15, 0.15)
        print('BIAS', ','.join(f'{v:.4f}' for v in b))
