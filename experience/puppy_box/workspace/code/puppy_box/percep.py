"""Perception from a saved observation: depth -> level-frame point cloud, table height, puppy blob,
ray/plane intersection for the clear box rim (depth sees through the walls)."""
import json, numpy as np, cv2
from scipy.spatial.transform import Rotation as Rr

def load(path):
    ob = json.load(open(path))
    fid = ob['camera']['frame_id']
    d = cv2.imread(f'runtime/frames/{fid}/cam_front_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    img = cv2.imread(f'runtime/frames/{fid}/cam_front.jpg')
    K = np.array(ob['camera_intrinsics']['cam_front']['matrix'])
    cp = ob['camera_poses']['cam_front']
    q = cp['quaternion_wxyz']
    Rc = Rr.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()   # camera -> pelvis
    tc = np.array(cp['position_m'])
    w = ob['proprio']['root_orientation_world_wxyz']
    Rw = Rr.from_quat([w[1], w[2], w[3], w[0]])
    yaw = Rw.as_euler('ZYX')[0]
    Rl = (Rr.from_euler('Z', -yaw) * Rw).as_matrix()            # pelvis -> level
    return dict(ob=ob, depth=d, img=img, K=K, Rc=Rc, tc=tc, Rl=Rl, fid=fid)

def rays(P, u, v):
    """camera-frame (x fwd, y left, z up) unit-depth direction for pixels."""
    K = P['K']
    return np.stack([np.ones_like(u, dtype=float), -(u - K[0, 2]) / K[0, 0], -(v - K[1, 2]) / K[1, 1]], -1)

def cloud(P):
    h, w = P['depth'].shape
    v, u = np.mgrid[0:h, 0:w]
    r = rays(P, u, v) * P['depth'][..., None]
    pel = r @ P['Rc'].T + P['tc']
    return pel, pel @ P['Rl'].T            # pelvis, level

def to_pelvis(P, lev):
    return np.asarray(lev) @ P['Rl']

def plane_hit(P, u, v, z_level):
    """Intersect the pixel ray with the level-frame plane z = z_level; returns level point."""
    d = (rays(P, np.array(float(u)), np.array(float(v))) @ P['Rc'].T) @ P['Rl'].T
    o = P['tc'] @ P['Rl'].T
    t = (z_level - o[2]) / d[2]
    return o + t * d

def table_z(P):
    import os
    if os.environ.get('TABLE_Z'):   # override when the histogram locks onto an arm (258bd8: tucked left arm in a turned view)
        return float(os.environ['TABLE_Z'])
    _, lev = cloud(P)
    z = lev[..., 2][P['depth'] > 0]
    hist, edges = np.histogram(z, bins=np.arange(-0.3, 0.3, 0.005))
    i = np.argmax(hist)
    return float(edges[i] + 0.0025)

def puppy(P, tz, min_h=0.03, exclude=None):
    """Cream blob above the table (excluding white robot hand by colour). Returns stats in level frame."""
    _, lev = cloud(P)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 40) & (hsv[..., 1] < 140) & (hsv[..., 2] > 120)
    m = cream & (lev[..., 2] > tz + min_h) & (P['depth'] > 0)
    if exclude is not None:   # (xmin, xmax, ymin, ymax) level-frame box footprint to ignore
        x0, x1, y0, y1 = exclude
        m &= ~((lev[..., 0] > x0) & (lev[..., 0] < x1) & (lev[..., 1] > y0) & (lev[..., 1] < y1))
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
    pts = lev[lab == k]
    return dict(mask=lab == k, n=int(len(pts)), centroid=pts.mean(0).tolist(), top=float(pts[:, 2].max()),
                xmin=float(pts[:, 0].min()), xmax=float(pts[:, 0].max()), ymin=float(pts[:, 1].min()),
                ymax=float(pts[:, 1].max()), bbox_px=st[k, :4].tolist(), pts=pts)

def project(P, pel_pts):
    """pelvis-frame points -> pixel (u, v)."""
    c = (np.atleast_2d(pel_pts) - P['tc']) @ P['Rc']
    K = P['K']
    return np.stack([K[0, 2] - K[0, 0] * c[:, 1] / c[:, 0], K[1, 2] - K[1, 1] * c[:, 2] / c[:, 0]], -1)

def draw_box(P, x0, x1, y0, y1, z_top, z_bot, path):
    img = P['img'].copy()
    for z, col in ((z_top, (0, 0, 255)), (z_bot, (255, 0, 0))):
        c = np.array([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]])
        uv = project(P, to_pelvis(P, c)).astype(int)
        cv2.polylines(img, [uv.reshape(-1, 1, 2)], True, col, 2)
    cv2.imwrite(path, img)
