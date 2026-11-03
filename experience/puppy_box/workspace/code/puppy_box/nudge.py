"""Nudge the held plush: run.py mv (commanded-row IK + arm sync) to an absolute palm target, keep the release yaw.
  PYTHONPATH=. python code/puppy_box/nudge.py X Y Z [secs]"""
import json, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run as R

if __name__ == '__main__':
    from api.runtime import Context
    a = [float(v) for v in sys.argv[1:]]
    st = R.load()
    with Context() as ctx:
        r = R.mv(ctx, tuple(a[:3]), st['ryaw'], a[3] if len(a) > 3 else 4.0, max_jump=0.8)
        ob = r.pop('ob', None)
        if ob is not None:
            frac, over = R.hanging_check(ob, st)
            r['hanging_inside_frac'] = frac; r['overlay'] = over
        print(json.dumps(r, default=float))
