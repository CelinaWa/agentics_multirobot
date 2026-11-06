"""Geometry-only perception for puppy_box_r1 from one r1_observe frame (no robot motion).

All outputs are in the pelvis frame of the observation. Semantic checks (is it the puppy? is it in
the box?) are left to the agent / a perception subagent at the checkpoints."""
import json
from pathlib import Path
import numpy as np
import cv2

WS = Path('/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace')


def quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_frame(frame_id):
    d = WS / 'runtime' / 'frames' / frame_id
    meta = json.loads((d / 'meta.json').read_text())
    rgb = cv2.imread(str(d / 'cam_head.jpg'))
    dep = cv2.imread(str(d / 'cam_head_depth.png'), -1).astype(np.float32) / 1000.0
    return meta, rgb, dep


def cloud(meta, dep, stride=2):
    """Pelvis-frame points (H/stride, W/stride, 3) and validity mask."""
    K = np.asarray(meta['intrinsics_full'])
    f, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    pose = meta['camera_pose_pelvis']
    R = quat_wxyz_to_R(pose['quaternion_wxyz'])
    t = np.asarray(pose['position_m'])
    d = dep[::stride, ::stride]
    v, u = np.mgrid[0:dep.shape[0]:stride, 0:dep.shape[1]:stride]
    pc = np.stack([d, -(u - cx) / f * d, -(v - cy) / f * d], -1)   # camera: X fwd, Y left, Z up
    P = pc @ R.T + t
    return P, d > 0.05


def table(meta, dep):
    """Table plane height and front edge: {z, edge_x (median near edge), edge_yaw_deg, y_range}."""
    P, ok = cloud(meta, dep)
    z = P[..., 2][ok]
    # table top is the dominant horizontal level between -0.10 and +0.08 (pelvis ~0.73 m above floor)
    zz = z[(z > -0.10) & (z < 0.08)]
    if zz.size < 500:
        return None
    hist, edges = np.histogram(zz, bins=90, range=(-0.10, 0.08))
    zt = edges[np.argmax(hist)] + 0.001
    on = ok & (np.abs(P[..., 2] - zt) < 0.012)
    pts = P[on]
    # near edge: per 4 cm y-bin, the smallest x of table points
    ys = np.arange(pts[:, 1].min(), pts[:, 1].max(), 0.04)
    near = []
    for y0 in ys:
        s = pts[(pts[:, 1] >= y0) & (pts[:, 1] < y0 + 0.04)]
        if len(s) > 20:
            near.append((y0 + 0.02, np.percentile(s[:, 0], 2)))
    near = np.array(near)
    if len(near) < 3:
        return {'z': float(zt), 'edge_x': None}
    # robust line fit x = a*y + b: the view cone inflates near-x at the image sides, so keep the
    # bins close to the low quantile (the true edge) and fit those
    q = np.percentile(near[:, 1], 30)
    keep = near[:, 1] < q + 0.02
    a, b = np.polyfit(near[keep, 0], near[keep, 1], 1) if keep.sum() >= 3 else (0.0, float(np.median(near[:, 1])))
    res = np.abs(near[:, 1] - (a * near[:, 0] + b))
    keep = res < 0.012
    if keep.sum() >= 3:
        a, b = np.polyfit(near[keep, 0], near[keep, 1], 1)
    # if the image bottom cuts the table, the "edge" is just the view limit
    bottom = P[-1][ok[-1]]
    view_limited = bottom.size > 0 and np.median(np.abs(bottom[:, 2] - zt)) < 0.015
    return {'z': float(zt), 'edge_x': float(b), 'edge_yaw_deg': float(np.degrees(np.arctan(a))),
            'near': near.round(3).tolist(), 'view_limited': bool(view_limited)}


def above_table_blobs(meta, dep, zt, edge_x, lo=0.015, hi=0.20, depth_x=0.65):
    """Connected regions of points standing lo..hi above the table, over the table top.
    Returns list of {n, centroid, xmin..zmax, pix_bbox}."""
    stride = 2
    P, ok = cloud(meta, dep, stride)
    h = P[..., 2] - zt
    m = ok & (h > lo) & (h < hi)
    if edge_x is not None:
        m &= (P[..., 0] > edge_x - 0.12) & (P[..., 0] < edge_x + depth_x)
    m = m.astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 40:
            continue
        p = P[lab == i]
        x0, y0, w, hh = stats[i, :4]
        out.append({'n': int(len(p)), 'centroid': p.mean(0).round(3).tolist(),
                    'min': p.min(0).round(3).tolist(), 'max': p.max(0).round(3).tolist(),
                    'pix_bbox': [int(x0 * stride), int(y0 * stride), int(w * stride), int(hh * stride)]})
    return sorted(out, key=lambda b: -b['n'])


def summary(frame_id):
    meta, rgb, dep = load_frame(frame_id)
    tb = table(meta, dep)
    res = {'frame_id': frame_id, 'table': tb}
    if tb and tb.get('edge_x') is not None:
        res['blobs'] = above_table_blobs(meta, dep, tb['z'], tb['edge_x'])[:6]
    return res


if __name__ == '__main__':
    import sys
    print(json.dumps(summary(sys.argv[1]), indent=1))
