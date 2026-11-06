"""Leg axis (paw -> hip) of one paw from a frame: cream (leg) points in a cone around a rough direction, 3.5-10 cm from
the paw centre, line fit through the paw centre. python legaxis.py <frame> <u> <v> [rough_yaw_deg]"""
import sys, json, math
import numpy as np, cv2
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from percep import load_frame, cloud
import pawfit


def axis(fid, u=None, v=None, rough_yaw=12.0, centre=None, rmin=0.035, rmax=0.10, cone=35.0):
    if centre is None:
        ft = pawfit.fit(fid, u, v)
        c = np.array(ft['centre'])
    else:
        c = np.array(centre)
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 2] > 110) & (hsv[..., 1] < 140)
    dark = (hsv[..., 2] < 110)
    P, ok = cloud(meta, dep, stride=1)
    p = P[ok & cream & ~dark]
    q = p - c
    d = np.linalg.norm(q, axis=1)
    sel = (d > rmin) & (d < rmax)
    q = q[sel]; d = d[sel]
    ry = math.radians(rough_yaw)
    hd = np.array([math.cos(ry), math.sin(ry)])
    qh = q[:, :2] / np.maximum(np.linalg.norm(q[:, :2], axis=1, keepdims=True), 1e-6)
    ang = np.degrees(np.arccos(np.clip(qh @ hd, -1, 1)))
    q = q[ang < cone]
    if len(q) < 50:
        return {'paw': c.round(4).tolist(), 'n': int(len(q))}
    # iterate: direction = mean unit vector, then keep points within 2.2 cm of the line
    u_ = q.mean(0); u_ /= np.linalg.norm(u_)
    for _ in range(5):
        t = q @ u_
        perp = np.linalg.norm(q - np.outer(t, u_), axis=1)
        k = perp < 0.022
        if k.sum() < 30:
            break
        # least squares line through the origin (paw centre): principal direction of the kept points
        w = q[k]
        _, _, vt = np.linalg.svd(w, full_matrices=False)
        nu = vt[0] if vt[0] @ u_ > 0 else -vt[0]
        u_ = nu
    yaw = math.degrees(math.atan2(u_[1], u_[0]))
    pitch = math.degrees(math.atan2(u_[2], math.hypot(u_[0], u_[1])))
    return {'paw': c.round(4).tolist(), 'n': int(k.sum()), 'yaw': round(yaw, 1), 'pitch': round(pitch, 1),
            'dir': u_.round(3).tolist()}


if __name__ == '__main__':
    a = sys.argv
    print(json.dumps(axis(a[1], int(a[2]), int(a[3]), float(a[4]) if len(a) > 4 else 12.0)))
