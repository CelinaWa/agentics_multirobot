"""Plan r1_walk calls with the measured sim walk model (session c4f6c3) - MPC style: execute the first action,
re-measure (odometry pelvis + edges heading), re-plan.
Model: turn cmd c -> theta = 0.85 c, the robot pivots about a centre at robot (0.05, +-0.32) (inside of the turn);
straight walk cmd c -> distance c (fwd, noisy 0.3-1.8x), back cmd -> 0.23 m; every call drifts 0.05 m to the
robot's right.
python walkplan.py x y psi_deg  tx ty tpsi_deg  edge_x  [min_clear]
  (world frame = odometry; world x = table normal; edge_x = world x of the front edge)"""
import sys, math, itertools, json
import numpy as np

TURNS = [c for c in range(-100, 101, 10) if c != 0]
WALKS = [-0.1, 0.1, 0.15, 0.2, 0.3, 0.4]


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def step(pose, act):
    x, y, psi = pose
    kind, c = act
    p = np.array([x, y])
    if kind == 'turn':
        th = math.radians((0.87 if c > 0 else 0.86) * c)
        cen = np.array([0.05, 0.31]) if th > 0 else np.array([0.02, -0.52])
        d = (np.eye(2) - rot(th)) @ cen
        p = p + rot(psi) @ d
        psi = psi + th
    else:
        d = 0.8 * c if c > 0 else -0.23
        p = p + rot(psi) @ np.array([d, 0.0])
    p = p + rot(psi) @ np.array([0.0, -0.04])
    return (p[0], p[1], psi)


def plan(pose, target, edge_x, min_clear=0.38, depth=3):
    acts = [('turn', c) for c in TURNS] + [('walk', c) for c in WALKS]
    best = None
    for n in range(1, depth + 1):
        for seq in itertools.product(acts, repeat=n):
            if any(seq[i][0] == seq[i + 1][0] == 'turn' and seq[i][1] * seq[i + 1][1] > 0 for i in range(n - 1)):
                continue
            q, ok = pose, True
            for i, a in enumerate(seq):
                q = step(q, a)
                if edge_x - q[0] < (min_clear if i < n - 1 else min(min_clear, 0.40)):
                    ok = False
                    break
            if not ok:
                continue
            e = math.hypot(q[0] - target[0], q[1] - target[1]) + 0.25 * abs(q[2] - target[2]) + 0.01 * n
            if best is None or e < best[0]:
                best = (e, seq, q)
    return best


if __name__ == '__main__':
    a = [float(v) for v in sys.argv[1:]]
    pose = (a[0], a[1], math.radians(a[2]))
    target = (a[3], a[4], math.radians(a[5]))
    mc = a[7] if len(a) > 7 else 0.38
    e, seq, q = plan(pose, target, a[6], mc)
    print(json.dumps({'cost': round(e, 3), 'seq': seq, 'end': [round(q[0], 3), round(q[1], 3), round(math.degrees(q[2]), 1)]}))
