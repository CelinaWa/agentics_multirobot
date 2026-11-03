"""Raise the right hand at the side (prep_side sequence) with an FK check of every stage against the puppy footprint;
the left arm is not moved (already tucked back). prep.py [dry]"""
import sys, os, json, time
sys.path[:0] = [os.path.join(os.getcwd(), d) for d in ('code/puppy_box', '.')]
import controller as c
from api.runtime import Context
R = lambda d: c.mirror('right', d)
STAGES = [({'shoulder_pitch': 1.4}, 3.0), ({'shoulder_roll': -1.1}, 3.0), ({'elbow': -0.9}, 3.0), ({'wrist_roll': -1.3}, 2.5)]
st = json.load(open('output/f0_state.json'))
x0, x1, y0, y1 = st['foot']
with Context() as ctx:
    row, meas, _ = c.pps.last_row(ctx); j = list(row[9:38]) if row else list(meas)
    for d, _ in STAGES:
        a = list(j)
        for k, v in R(d).items():
            j[c.J.index(k)] = v
        for s in [i / 8 for i in range(1, 9)]:
            q = [p + (b - p) * s for p, b in zip(a, j)]
            for n, p in c.pps.link_points(q, 'right').items():
                if x0 - 0.04 < p[0] < x1 + 0.04 and y0 - 0.04 < p[1] < y1 + 0.04 and p[2] < st['top'] + 0.06:
                    sys.exit(f'stage {d} s={s}: {n} at {[round(v, 3) for v in p]} over the puppy')
        print(d, {n: [round(v, 3) for v in p] for n, p in c.pps.link_points(j, 'right').items() if n == 'right_palm'})
    if 'dry' in sys.argv:
        sys.exit(0)
    for d, secs in STAGES:
        c.pps.ramp(ctx, R(d), secs)
    time.sleep(1.0)
    ob, path = c.observe(ctx, 'prep4')
    print(json.dumps(dict(tilt=ob['tilt_deg'], path=path, frame=ob['camera']['frame_id'])))
