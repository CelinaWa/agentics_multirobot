"""Session robot helpers: reuse palm-pose-servo (ramp, IK, FK)."""
import os, sys, json
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'skill-library', 'skills', 'palm-pose-servo', 'code'))
from api.runtime import Context  # noqa
from common import ramp, last_row, link_points, M  # noqa
from ik import solve, palm  # noqa
from scipy.spatial.transform import Rotation as Rr

def quat_from_axes(f, n):
    """Palm frame x = fingers f, y = palm normal n (world/pelvis vectors) -> wxyz."""
    f = np.asarray(f, float); f /= np.linalg.norm(f)
    n = np.asarray(n, float); n -= f * f.dot(n); n /= np.linalg.norm(n)
    R = np.stack([f, n, np.cross(f, n)], 1)
    q = Rr.from_matrix(R).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]

def obs(ctx):
    o = ctx.value('g1_observe')
    return o

def summary(o, side='right'):
    pts = link_points(o['proprio']['joint_positions_rad'], side)
    return {'frame': o['camera']['frame_id'], 'tilt': o['tilt_deg'], 'step': o['step_id'],
            'hand': [round(v, 2) for v in o['proprio']['hand_state']['positions_rad'][7:]], **pts}
