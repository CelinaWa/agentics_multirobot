import numpy as np, sys, os
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as Rr
sys.path.insert(0, os.path.dirname(__file__))
from fk import M
ARM = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw']
LO = dict(zip(M.canonical29, [-2.5307, -2.5307, -2.618, -0.5236, -2.9671, -0.52, -2.7576, -2.7576, -0.52, -0.0873, -0.0873, -3.0892, -3.0892, -0.8727, -0.8727, -1.5882, -2.2515, -0.2618, -0.2618, -2.618, -2.618, -1.0472, -1.0472, -1.9722, -1.9722, -1.6144, -1.6144, -1.6144, -1.6144]))
HI = dict(zip(M.canonical29, [2.8798, 2.8798, 2.618, 2.9671, 0.5236, 0.52, 2.7576, 2.7576, 0.52, 2.8798, 2.8798, 2.6704, 2.6704, 0.5236, 0.5236, 2.2515, 1.5882, 0.2618, 0.2618, 2.618, 2.618, 2.0944, 2.0944, 1.9722, 1.9722, 1.6144, 1.6144, 1.6144, 1.6144]))

def palm(j29, side):
    R, p = M.fk(list(j29)).frame_pose(f'{side}_palm')
    return np.array(p), np.array(R)

def solve(j29, side, pos, quat_wxyz=None, w_rot=0.05):
    names = [f'{side}_{a}_joint' for a in ARM]
    idx = [M.canonical29.index(n) for n in names]
    Rt = None if quat_wxyz is None else Rr.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]).as_matrix()
    base = np.array(j29, float)
    def f(x):
        j = base.copy(); j[idx] = x
        p, R = palm(j, side)
        c = np.sum((p - pos) ** 2)
        if Rt is not None: c += w_rot * np.sum((R - Rt) ** 2) * 0.01
        return 1e4 * (c + 1e-4 * np.sum((x - base[idx]) ** 2))
    b = [(LO[n] + 0.06, HI[n] - 0.06) for n in names]
    r = minimize(f, base[idx], bounds=b, method='L-BFGS-B', options=dict(eps=1e-5))
    j = base.copy(); j[idx] = r.x
    p, R = palm(j, side)
    return dict(zip(names, r.x)), float(np.linalg.norm(p - pos)), j
