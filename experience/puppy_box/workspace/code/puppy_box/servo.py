"""CLI wrapper for palm-pose-servo main: python code/puppy_box/servo.py x y z [yaw_deg] [iters] [bias_x,bias_y,bias_z]"""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library/skills/palm-pose-servo/code'))
from api.runtime import Context
import run as pps

if __name__ == '__main__':
    x, y, z = map(float, sys.argv[1:4])
    yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    iters = int(sys.argv[5]) if len(sys.argv) > 5 else 2
    bias = [float(v) for v in sys.argv[6].split(',')] if len(sys.argv) > 6 else None
    secs = float(os.environ.get('SECS', '2.0'))
    with Context() as ctx:
        r = pps.main(ctx, os.environ.get('SIDE', 'right'), (x, y, z), yaw_deg=yaw, iters=iters, bias=bias, secs=secs)
        m = r.get('measured', {})
        print(json.dumps({'ok': r['ok'], 'why': r.get('why'), 'palm': m.get('palm'), 'err': m.get('err_m'),
                          'bias': m.get('bias'), 'frame': m.get('frame_id'), 'tilt': m.get('tilt_deg'),
                          'links': m.get('links'), 'hand': [round(v, 2) for v in m.get('hand_state', [])]}))
