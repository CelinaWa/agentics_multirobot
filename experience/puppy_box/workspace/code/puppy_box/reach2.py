"""Multi-seed palm-down IK reach check on a saved observation: python reach2.py obs.json x y z [palm_yaw_deg] [waist_yaw] [waist_pitch]"""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
J = c.J
SEEDS = [{}, {'shoulder_pitch': -0.5, 'elbow': 0.3}, {'shoulder_pitch': -0.6, 'elbow': 0.6, 'wrist_roll': -1.3},
         {'shoulder_pitch': -0.3, 'shoulder_roll': 0.2, 'shoulder_yaw': 0.3, 'elbow': 0.8, 'wrist_roll': -1.3},
         {'shoulder_pitch': -0.8, 'elbow': 0.0, 'wrist_roll': -1.3}]


RANDOM_SEEDS = 20
RANGES = dict(shoulder_pitch=(-1.5, 0.3), shoulder_roll=(-0.8, 0.1), shoulder_yaw=(-0.8, 0.8), elbow=(-0.6, 1.2),
              wrist_roll=(-1.8, 0.0), wrist_pitch=(-0.8, 0.8), wrist_yaw=(-0.8, 0.8))   # right-hand values
FLIP = ('shoulder_roll', 'shoulder_yaw', 'wrist_roll', 'wrist_yaw')


def best(j29, pos, yaw_deg=0.0, side='right'):
    q = c.pps.palm_quat(yaw_deg, side)
    out = []
    for s in [None] + SEEDS:
        b = np.array(j29, float)
        if s is not None:
            for k, v in c.mirror(side, s).items():
                b[J.index(k)] = v
        t, e, _ = c.pps.solve(list(b), side, pos, q)
        out.append((e, t))
    if min(o[0] for o in out) > 0.004:      # near the reach limit the 6 fixed seeds hit local minima (2026-11-02 6a0bfc)
        rng = np.random.default_rng(0)
        for _ in range(RANDOM_SEEDS):
            b = np.array(j29, float)
            for k, (lo, hi) in RANGES.items():
                b[J.index(f'{side}_{k}_joint')] = rng.uniform(lo, hi) * (-1 if side == 'left' and k in FLIP else 1)
            t, e, _ = c.pps.solve(list(b), side, pos, q)
            out.append((e, t))
    return min(out, key=lambda a: a[0])


if __name__ == '__main__':
    ob = json.load(open(sys.argv[1]))
    j = np.array(ob['proprio']['joint_positions_rad'])
    a = [float(v) for v in sys.argv[2:]]
    if len(a) > 4: j[2] = a[4]
    if len(a) > 5: j[8] = a[5]
    e, t = best(j, a[:3], a[3] if len(a) > 3 else 0.0)
    print(round(e, 4), {k[6:-6]: round(v, 2) for k, v in t.items()})
