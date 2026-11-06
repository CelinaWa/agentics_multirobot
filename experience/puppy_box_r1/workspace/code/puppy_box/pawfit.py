"""Paw-ball centre by sphere fit (r = 0.025) to the dark-brown blob containing/near a seed pixel.
python pawfit.py <frame_id> u v  -> centre (pelvis), rms, n"""
import sys, json
import numpy as np, cv2
from scipy.optimize import least_squares
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace/code/puppy_box')
from percep import load_frame, cloud

R = 0.025


def fit(fid, u, v, r=R):
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 2] < 110) & (hsv[..., 2] > 15) & (hsv[..., 1] > 50) &
         ((hsv[..., 0] < 25) | (hsv[..., 0] > 160))).astype(np.uint8)
    n, lab = cv2.connectedComponents(m, 8)
    P, ok = cloud(meta, dep, stride=1)
    ys, xs = np.nonzero(lab > 0)
    if len(xs) == 0:
        return None
    k = np.argmin((xs - u) ** 2 + (ys - v) ** 2)
    L = lab[ys[k], xs[k]]
    sel = (lab == L) & ok
    # erode to drop the silhouette rim (mixed depth)
    sel = cv2.erode(sel.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool) & ok
    seed = np.median(P[max(v-2,0):v+3, max(u-2,0):u+3].reshape(-1, 3), 0)
    p = P[sel]
    p = p[np.linalg.norm(p - seed, axis=1) < 0.045]
    cam = np.asarray(meta['camera_pose_pelvis']['position_m'])
    c0 = p.mean(0) + R * (p.mean(0) - cam) / np.linalg.norm(p.mean(0) - cam)
    res = least_squares(lambda c: np.linalg.norm(p - c, axis=1) - r, c0)
    c = res.x
    rms = float(np.sqrt(np.mean(res.fun ** 2)))
    return {'centre': np.round(c, 3).tolist(), 'rms': round(rms, 4), 'n': int(len(p)),
            'seed': np.round(seed, 3).tolist(), 'zmin_pts': round(float(p[:, 2].min()), 3),
            'zmax_pts': round(float(p[:, 2].max()), 3)}


if __name__ == '__main__':
    print(json.dumps(fit(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))))
