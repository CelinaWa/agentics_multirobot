"""Depth cloud in the pelvis frame from a saved frame + puppy/box measurement."""
import json, os, sys
import numpy as np, cv2
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FR = os.path.join(ROOT, 'runtime', 'frames')

def qm(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def cloud(fid):
    m = json.load(open(f'{FR}/{fid}/meta.json'))
    d = cv2.imread(f'{FR}/{fid}/cam_front_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    if d.shape[0] != 480: d = d.T
    K = np.array(m['intrinsics_full']); cp = m['camera_pose_pelvis']
    H, W = d.shape; vv, uu = np.mgrid[0:H, 0:W]
    cam = np.stack([d, -(uu-K[0,2])*d/K[0,0], -(vv-K[1,2])*d/K[1,1]], -1)
    P = cam @ qm(cp['quaternion_wxyz']).T + np.array(cp['position_m'])
    P[d <= 0] = np.nan
    return P

def measure(fid):
    P = cloud(fid); z = P[..., 2]
    table = float(np.nanpercentile(z[P[..., 0] < 0.8], 30))
    obj = (z > table + 0.03) & (z < table + 0.25)
    img = cv2.imread(f'{FR}/{fid}/cam_front.jpg')
    return P, table, obj, img

if __name__ == '__main__':
    fid = sys.argv[1]
    P, t, obj, img = measure(fid)
    print('table z', round(t, 3))
    q = P[obj & (P[...,1] < 0.05)]
    print('puppy pts', len(q), 'x', np.nanpercentile(q[:,0],[2,50,98]).round(3), 'y', np.nanpercentile(q[:,1],[2,50,98]).round(3), 'ztop', np.nanmax(q[:,2]).round(3))
    for y0 in np.arange(-0.45, 0.1, 0.03):
        s = q[(q[:,1]>=y0)&(q[:,1]<y0+0.03)]
        if len(s) > 20:
            print(f'y {y0+0.015:+.3f} n {len(s):5d} x {np.percentile(s[:,0],5):.3f}-{np.percentile(s[:,0],95):.3f} ztop {s[:,2].max()-t:.3f}')
