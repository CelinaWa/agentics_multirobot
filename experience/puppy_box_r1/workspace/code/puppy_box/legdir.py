"""Leg direction (paw -> hip) of a paw from one frame: points on the puppy (not dark brown) within 4-12 cm of the
paw-ball centre, above table-0.06, PCA in xy. python legdir.py <frame> <u> <v>  -> paw centre, leg yaw (pelvis frame
deg), pitch, n. Also returns leg yaw relative to the table normal if an edge yaw is given (4th arg)."""
import sys, json, math
import numpy as np, cv2
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T033337Z_puppy_box_r1_7c03aa/agent-workspace'
sys.path.insert(0, WS + '/code/puppy_box')
from percep import load_frame, cloud
import pawfit


def leg(fid, u, v, rmin=0.035, rmax=0.10):
    ft = pawfit.fit(fid, u, v)
    c = np.array(ft['centre'])
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 2] > 120) & (hsv[..., 1] < 120)
    P, ok = cloud(meta, dep, stride=1)
    m = ok & cream
    p = P[m]
    d = np.linalg.norm(p - c, axis=1)
    p = p[(d > rmin) & (d < rmax) & (p[:, 2] > c[2] - 0.03)]
    if len(p) < 30:
        return {'paw': ft, 'n': int(len(p))}
    q = p - c
    # direction from the paw to the mean of the shaft points, refined by PCA
    mu = q.mean(0)
    u_, s, vt = np.linalg.svd(q - mu)
    dv = vt[0] if vt[0] @ mu > 0 else -vt[0]
    yaw = math.degrees(math.atan2(dv[1], dv[0]))
    pitch = math.degrees(math.atan2(dv[2], math.hypot(dv[0], dv[1])))
    myaw = math.degrees(math.atan2(mu[1], mu[0]))
    return {'paw': ft['centre'], 'paw_n': ft['n'], 'n': int(len(p)), 'yaw_pca': round(yaw, 1), 'yaw_mean': round(myaw, 1),
            'pitch': round(pitch, 1), 'mean_dist': round(float(np.linalg.norm(mu)), 3)}


if __name__ == '__main__':
    r = leg(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
    if len(sys.argv) > 4 and 'yaw_pca' in r:
        e = float(sys.argv[4])   # edge normal yaw in the pelvis frame
        r['yaw_vs_table'] = round(r['yaw_pca'] - e, 1)
        r['yaw_mean_vs_table'] = round(r['yaw_mean'] - e, 1)
    print(json.dumps(r))
