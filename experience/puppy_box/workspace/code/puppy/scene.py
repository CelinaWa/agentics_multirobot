"""Scene measurement from one observation: table height, puppy points above the table."""
import json, sys
import numpy as np
sys.path.insert(0, '.')
import g1  # noqa
import g1_model

FR = '../../runtime/frames/'


def cloud(o, stride=2):
    d = g1_model.depth_from_png(FR + o['camera']['frame_id'] + '/cam_front_depth.png')[::stride, ::stride]
    K = np.array(o['camera_intrinsics']['cam_front']['matrix'])
    cp = o['camera_poses']['cam_front']
    R = quat_matrix(cp['quaternion_wxyz'])
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W] * stride
    cam = np.stack([d, -(uu - K[0, 2]) * d / K[0, 0], -(vv - K[1, 2]) * d / K[1, 1]], -1)
    return cam @ R.T + np.array(cp['position_m'])  # (H/stride, W/stride, 3) pelvis frame


def quat_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def puppy(o, stride=2, xmax=0.65, exclude_hand=True):
    pts = cloud(o, stride)
    flat = pts.reshape(-1, 3)
    ok = np.isfinite(flat).all(1) & (np.abs(flat).sum(1) > 0)
    z = flat[ok, 2]
    table = float(np.percentile(z[(flat[ok, 0] < xmax)], 30))
    H, W, _ = pts.shape
    vv, uu = np.mgrid[0:H, 0:W]
    m = ok.reshape(H, W) & (pts[..., 2] > table + 0.03) & (pts[..., 0] < xmax) & (pts[..., 2] < table + 0.2)
    if exclude_hand:
        # drop the robot's own hands: points close to measured hand links
        P = g1_model.pose_from_observation(o)
        for nm in P.names():
            if 'hand' in nm or 'wrist' in nm:
                c = np.array(P.point(nm, 'pelvis')['position_m'])
                m &= np.linalg.norm(pts - c, axis=-1) > 0.07
    sel = pts[m]
    return table, sel, (uu[m] * stride, vv[m] * stride)


if __name__ == '__main__':
    o = json.load(open('../../output/obs/last.json'))
    table, sel, (u, v) = puppy(o)
    print('table z %.3f  n=%d' % (table, len(sel)))
    if len(sel):
        print('x %.3f..%.3f  y %.3f..%.3f  ztop %.3f' % (*np.percentile(sel[:, 0], [2, 98]), *np.percentile(sel[:, 1], [2, 98]), np.percentile(sel[:, 2], 98)))
        # slices along y
        for y0 in np.arange(np.percentile(sel[:, 1], 2), np.percentile(sel[:, 1], 98), 0.04):
            s = sel[(sel[:, 1] >= y0) & (sel[:, 1] < y0 + 0.04)]
            if len(s) > 5:
                print('y %.2f: x %.3f..%.3f ztop %.3f  n=%d' % (y0, *np.percentile(s[:, 0], [3, 97]), s[:, 2].max(), len(s)))
