"""Table edges from one frame: drop-off boundary of the table top (pixels whose 3-D point is on the table plane
and whose pixel 4 rows below is >= 5 cm lower), RANSAC lines in the pelvis xy plane.
python edges.py <frame_id> [table_z]  -> JSON lines: point nearest the pelvis, unit direction, normal-distance,
yaw of the line normal (deg, 0 = edge square in front), inliers."""
import json, sys
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from percep import load_frame, cloud, table


def lines(fid, zt=None):
    meta, rgb, dep = load_frame(fid)
    if zt is None:
        zt = (table(meta, dep) or {}).get('z')
    P, ok = cloud(meta, dep, stride=1)
    H = P.shape[0]
    on = ok & (np.abs(P[..., 2] - zt) < 0.012)
    below = np.zeros_like(on)
    below[:-4] = ok[4:] & (P[4:, :, 2] < zt - 0.009)
    b = on & below
    b[H - 6:] = False
    pts = P[b][:, :2]
    pts = pts[np.linalg.norm(pts, axis=1) < 2.0]
    out, rng = [], np.random.default_rng(0)
    for _ in range(3):
        if len(pts) < 30:
            break
        best = None
        for _ in range(400):
            i, j = rng.choice(len(pts), 2, replace=False)
            d = pts[j] - pts[i]
            if np.linalg.norm(d) < 0.05:
                continue
            d /= np.linalg.norm(d)
            n = np.array([-d[1], d[0]])
            r = np.abs((pts - pts[i]) @ n)
            inl = r < 0.008
            if best is None or inl.sum() > best[0].sum():
                best = (inl, pts[i], d)
        if best is None:
            break
        inl, p0, d = best
        if inl.sum() < 30:
            break
        q = pts[inl]
        c = q.mean(0)
        u, s, vt = np.linalg.svd(q - c)
        d = vt[0]
        n = np.array([-d[1], d[0]])
        if n @ c < 0:
            n = -n
        dist = float(n @ c)
        t = (q - c) @ d
        out.append({'n_in': int(inl.sum()), 'dist': round(dist, 3), 'normal_yaw_deg': round(float(np.degrees(np.arctan2(n[1], n[0]))), 1),
                    'foot': np.round(n * dist, 3).tolist(), 'extent': [np.round(c + t.min() * d, 3).tolist(), np.round(c + t.max() * d, 3).tolist()]})
        pts = pts[~inl]
    return {'table_z': zt, 'lines': out}


if __name__ == '__main__':
    print(json.dumps(lines(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else None)))
