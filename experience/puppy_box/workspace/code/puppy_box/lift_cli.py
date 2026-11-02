"""python code/puppy_box/lift_cli.py SIDE 'x,y,z;...' bx,by,bz [secs]: plush-pinch-grasp lift() through waypoints."""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library/skills/plush-pinch-grasp/code'))
from api.runtime import Context
import run as ppg

if __name__ == '__main__':
    side = sys.argv[1]; pts = [tuple(map(float, s.split(','))) for s in sys.argv[2].split(';')]
    bias = [float(v) for v in sys.argv[3].split(',')]
    secs = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
    with Context() as ctx:
        r = ppg.lift(ctx, side, [p[:2] for p in pts], [p[2] for p in pts], bias, secs=secs, log=lambda *a: print(*a, flush=True))
        print('RESULT', r['ok'], r['why'])
