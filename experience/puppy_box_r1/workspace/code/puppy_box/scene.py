"""Scene geometry from one frame: table plane/edge (percep.table), paws (dark-brown blobs), box rim.
All in the pelvis frame of the observation. python scene.py <frame_id>"""
import json, sys
import numpy as np, cv2
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from percep import load_frame, cloud, table


def paws(meta, rgb, dep, zt=None):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    # dark brown spheres: low value, some saturation, hue orange/red
    m = ((hsv[..., 2] < 90) & (hsv[..., 2] > 15) & (hsv[..., 1] > 60) &
         ((hsv[..., 0] < 25) | (hsv[..., 0] > 160))).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    P, ok = cloud(meta, dep, stride=1)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    out = []
    f = float(np.asarray(meta['intrinsics_full'])[0, 0])
    cam = np.asarray(meta['camera_pose_pelvis']['position_m'])
    PAW_R = 0.025
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < 60:
            continue
        sel = (lab == i) & ok
        if sel.sum() < 20:
            continue
        vs, us = np.nonzero(sel)
        pc = np.median(P[sel], 0)
        single = np.pi * (PAW_R * f / max(np.linalg.norm(pc - cam), 0.2)) ** 2
        k = int(round(st[i, 4] / single)) if st[i, 4] > 1.5 * single else 1
        k = max(1, min(k, 4))
        if k > 1:
            pts = np.stack([us, vs], 1).astype(np.float32)
            _, lbl, cc = cv2.kmeans(pts, k, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5), 5,
                                    cv2.KMEANS_PP_CENTERS)
            groups = [(lbl.ravel() == j, cc[j]) for j in range(k)]
        else:
            groups = [(np.ones(len(us), bool), (us.mean(), vs.mean()))]
        for g, c2 in groups:
            p = P[vs[g], us[g]]
            # depth-median keeps the near surface of the paw sphere; drop floor pixels behind it
            p = p[p[:, 2] > np.median(p[:, 2]) - 0.08]
            c = np.median(p, 0)
            if zt is not None and (c[2] < zt - 0.25 or c[2] > zt + 0.15):
                continue
            out.append({'pix': [round(float(c2[0])), round(float(c2[1]))], 'area': int(g.sum()),
                        'p': np.round(c, 3).tolist(), 'zmin': round(float(p[:, 2].min()), 3)})
    return sorted(out, key=lambda d: d['pix'][0])


def box_rim(meta, dep, zt, rim_h=0.14, tol=0.02):
    """Points at the rim height (table + rim_h) over the table: xy extent of the box rim."""
    P, ok = cloud(meta, dep, stride=2)
    h = P[..., 2] - zt
    m = ok & (np.abs(h - rim_h) < tol)
    p = P[m]
    if len(p) < 30:
        return None
    # keep the main cluster (drop the wall etc.): x within table depth
    p = p[(p[:, 0] < 2.0)]
    lo, hi = np.percentile(p[:, :2], 2, 0), np.percentile(p[:, :2], 98, 0)
    return {'n': int(len(p)), 'x': [round(float(lo[0]), 3), round(float(hi[0]), 3)],
            'y': [round(float(lo[1]), 3), round(float(hi[1]), 3)],
            'centre': [round(float((lo[0] + hi[0]) / 2), 3), round(float((lo[1] + hi[1]) / 2), 3)]}


def summary(fid):
    meta, rgb, dep = load_frame(fid)
    tb = table(meta, dep) or {}
    zt = tb.get('z')
    res = {'frame': fid, 'table_z': zt, 'edge_x': tb.get('edge_x'), 'edge_yaw_deg': tb.get('edge_yaw_deg'),
           'view_limited': tb.get('view_limited')}
    res['paws'] = paws(meta, rgb, dep, zt)
    if zt is not None:
        res['box_rim'] = box_rim(meta, dep, zt)
    return res



def box_extent(meta, dep, zt, edge_x, rim_h=0.14):
    """Box from depth: points 0.11..0.16 above the table, beyond the edge and before the back wall
    (edge+0.58). Returns x/y extents (2-98 pct) and the centre, pelvis frame."""
    P, ok = cloud(meta, dep, stride=1)
    h = P[..., 2] - zt
    m = ok & (h > rim_h - 0.015) & (h < rim_h + 0.02) & (P[..., 0] > edge_x + 0.17) & (P[..., 0] < edge_x + 0.58)
    p = P[m]
    if len(p) < 200:
        return None
    lo, hi = np.percentile(p[:, :2], 1, 0), np.percentile(p[:, :2], 99, 0)
    return {'n': int(len(p)), 'x': [round(float(lo[0]), 3), round(float(hi[0]), 3)],
            'y': [round(float(lo[1]), 3), round(float(hi[1]), 3)],
            'centre': [round(float((lo[0] + hi[0]) / 2), 3), round(float((lo[1] + hi[1]) / 2), 3)]}


def brief(fid):
    meta, rgb, dep = load_frame(fid)
    tb = table(meta, dep) or {}
    zt, ex = tb.get('z'), tb.get('edge_x')
    out = {'edge_x': round(ex, 3) if ex else None, 'edge_yaw_deg': round(tb.get('edge_yaw_deg', 0), 1), 'table_z': zt,
           'paws': [p['p'] for p in paws(meta, rgb, dep, zt)],
           'paw_blobs': [{'p': p['p'], 'area': p['area'], 'pix': p['pix']} for p in paws(meta, rgb, dep, zt)]}
    if zt is not None and ex is not None:
        out['box'] = box_extent(meta, dep, zt, ex)
    return out


if __name__ == '__main__':
    print(json.dumps(brief(sys.argv[1])))
