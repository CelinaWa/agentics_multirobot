"""Numerical IK for one arm on the robot model (pelvis frame), waist fixed at the given values."""
import math
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, '.')
import g1  # noqa
import g1_model

M = g1_model.MODEL
ARM = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw']


def limits(side):
    st = g1_model.MODEL.data
    lo, hi = [], []
    for j in ARM:
        name = '%s_%s_joint' % (side, j)
        jj = M._joint_index[name]
        lim = jj.get('limit') or jj.get('limits_rad') or [-3, 3]
        if isinstance(lim, dict):
            lim = [lim.get('lower', -3), lim.get('upper', 3)]
        lo.append(lim[0] + 0.08)
        hi.append(lim[1] - 0.08)
    return lo, hi


def claw_rotation(side, tilt=math.pi / 4, yaw=0.0):
    """Palm frame for a claw grasp: fingers forward and `tilt` down, palm facing down-back, closing along x.
    Columns = palm x, y, z axes in pelvis frame. Left palm: thumb along -y_p."""
    c, s = math.cos(tilt), math.sin(tilt)
    xp = np.array([c, 0, -s])
    if side == 'left':
        yp = np.array([s, 0, c])
    else:  # right palm: thumb along +y_p (mirror)
        yp = -np.array([s, 0, c])
    zp = np.cross(xp, yp)
    R = np.stack([xp, yp, zp], 1)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ R


def fk_point(joints, name):
    p = M.fk(joints)
    r, t = p.frame_pose(name)
    return np.array(r), np.array(t)


def solve(side, base_joints, pos, R=None, link=None, seed=None, w_rot=0.3):
    """base_joints: dict of all 29 joints (name->rad). Returns (arm dict, pos err m, rot err rad)."""
    link = link or '%s_hand_palm_link' % side
    names = ['%s_%s_joint' % (side, j) for j in ARM]
    lo, hi = limits(side)
    x0 = np.array(seed if seed is not None else [base_joints[n] for n in names])

    def err(x):
        j = dict(base_joints)
        j.update(zip(names, x))
        r, t = fk_point(j, link)
        e = np.sum((t - pos) ** 2) * 100
        if R is not None:
            e += w_rot * np.sum((r - R) ** 2)
        return e

    res = minimize(err, np.clip(x0, lo, hi), bounds=list(zip(lo, hi)), method='L-BFGS-B')
    j = dict(base_joints)
    j.update(zip(names, res.x))
    r, t = fk_point(j, link)
    rot_err = 0.0 if R is None else float(np.arccos(np.clip((np.trace(r.T @ R) - 1) / 2, -1, 1)))
    return dict(zip(names, res.x)), float(np.linalg.norm(t - pos)), rot_err


def tips(joints, side, hand_frac=0.0):
    hand = M.hand_pose(side, hand_frac)
    p = M.fk(joints, hand)
    out = {}
    for nm in ('palm', 'thumb_tip', 'index_tip', 'middle_tip', 'hand_center'):
        key = '%s_%s' % (side, nm)
        try:
            out[nm] = np.round(p.point(key, 'pelvis')['position_m'], 3).tolist()
        except Exception:
            pass
    return out


def solve_multi(side, base, pos, R, link=None, w_rot=0.3):
    best = None
    sgn = 1 if side == 'left' else -1
    for sp in (-1.2, -0.6, 0.0):
        for wr in (-1.5, 0.0, 1.5):
            for sy in (-0.5, 0.5):
                seed = [sp, sgn * 0.2, sy, 0.3, wr, 0.0, 0.0]
                a, pe, re = solve(side, base, pos, R, link=link, seed=seed, w_rot=w_rot)
                score = pe * 10 + re
                if best is None or score < best[0]:
                    best = (score, a, pe, re)
    return best[1:]
