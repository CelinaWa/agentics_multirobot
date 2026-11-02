"""python code/puppy_box/grasp_cli.py SIDE x y [deep]: settle waist, pinch shape, plush-pinch-grasp grasp(); prints result."""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library/skills/plush-pinch-grasp/code'))
from api.runtime import Context
import run as ppg

if __name__ == '__main__':
    side = sys.argv[1]; xy = (float(sys.argv[2]), float(sys.argv[3]))
    with Context() as ctx:
        ppg.settle_waist(ctx)
        ppg.set_pinch_shape(ctx, side, deep=len(sys.argv) > 4)
        r = ppg.grasp(ctx, side, xy, log=lambda *a: print(*a, flush=True))
        print('RESULT', json.dumps(r, default=str))
