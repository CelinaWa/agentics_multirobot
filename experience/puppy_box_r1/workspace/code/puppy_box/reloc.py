"""Re-localize: rigid 4-DOF (x, y, z, yaw) transform from a reference frame's pelvis to the current frame's pelvis by
ICP on the scene points above the table (puppy, box) - the pelvis shifts when the arm moves, so positions measured
in the reference frame are mapped into the current one.
python reloc.py ref_frame cur_frame [x,y,z ...]   -> transform + mapped points
Excludes points within 0.15 m of either grasp point (the arm)."""
import sys, json, math
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace/code/puppy_box')
from percep import load_frame, cloud
HANDS = {}   # frame -> list of hand positions to exclude


def pts(fid, stride=2):
    meta, rgb, dep = load_frame(fid)
    P, ok = cloud(meta, dep, stride=stride)
    p = P[ok]
    z = p[:, 2]
    zz = z[(z > -0.10) & (z < 0.08)]
    hist, e = np.histogram(zz, bins=90, range=(-0.10, 0.08))
    zt = e[np.argmax(hist)] + 0.001
    sel = (p[:, 0] > 0.25) & (p[:, 0] < 1.3) & (np.abs(p[:, 1]) < 0.7) & (p[:, 2] > zt + 0.008) & (p[:, 2] < zt + 0.30)
    p = p[sel]
    for g in HANDS.get(fid, []):
        p = p[np.linalg.norm(p - np.asarray(g), axis=1) > 0.15]
    return p, zt


def coarse(src, dst, tree, span=0.16, step=0.01, yaws=np.radians(np.array([0.0]))):
    sub = src[::max(1, len(src) // 2500)]
    best = None
    for th in yaws:
        c, s = math.cos(th), math.sin(th)
        q0 = sub @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T
        for dx in np.arange(-span, span + 1e-9, step):
            for dy in np.arange(-span, span + 1e-9, step):
                d, _ = tree.query(q0 + np.array([dx, dy, 0.0]))
                sc = np.minimum(d, 0.02).mean()
                if best is None or sc < best[0]:
                    best = (sc, th, dx, dy)
    return best[1], np.array([best[2], best[3], 0.0])


def icp(src, dst, iters=40):
    """find yaw th, t with dst ~ R(th) src + t (point-to-point, trimmed; coarse x/y grid init)"""
    tree = cKDTree(dst)
    th, t = coarse(src, dst, tree)
    for it in range(iters):
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        q = src @ R.T + t
        d, i = tree.query(q)
        thr = max(0.008, np.percentile(d, 70))
        m = d < thr
        a, b = src[m], dst[i[m]]
        ca, cb = a.mean(0), b.mean(0)
        A, B = a - ca, b - cb
        H = A[:, :2].T @ B[:, :2]
        th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        t = cb - R @ ca
    return th, t, float(np.median(d[m])), int(m.sum())


def transform(ref, cur):
    a, _ = pts(ref)
    b, _ = pts(cur)
    th, t, res, n = icp(a, b)
    return th, t, res, n


if __name__ == '__main__':
    import os
    if os.environ.get('HAND'):
        HANDS[sys.argv[2]] = [[float(v) for v in os.environ['HAND'].split(',')]]
    th, t, res, n = transform(sys.argv[1], sys.argv[2])
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    print(json.dumps({'yaw_deg': round(math.degrees(th), 2), 't': np.round(t, 4).tolist(), 'median_resid': round(res, 4), 'n': n}))
    for a in sys.argv[3:]:
        p = np.array([float(v) for v in a.split(',')])
        print(a, '->', np.round(R @ p + t, 4).tolist())
