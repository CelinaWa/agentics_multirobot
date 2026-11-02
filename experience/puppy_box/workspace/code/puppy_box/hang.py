"""Where does the held plush hang? Cream/dark-paw pixels whose depth puts them 4-30 cm above the table, in the pelvis frame.
hang(frame_id, table_z) -> {n, x_med, x_p20}. Uses runtime/frames/<id>/meta.json (camera pose in the pelvis frame)."""
import json, sys, numpy as np, cv2
from scipy.spatial.transform import Rotation as Rr

def hang(fid, tz):
    m = json.load(open(f'runtime/frames/{fid}/meta.json'))
    d = cv2.imread(f'runtime/frames/{fid}/cam_front_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    img = cv2.imread(f'runtime/frames/{fid}/cam_front.jpg')
    K = np.array(m['intrinsics_full']['matrix'] if isinstance(m['intrinsics_full'], dict) and 'matrix' in m['intrinsics_full'] else m['intrinsics_full'])
    cp = m['camera_pose_pelvis']; q = cp['quaternion_wxyz']
    R = Rr.from_quat([q[1], q[2], q[3], q[0]]).as_matrix(); t = np.array(cp['position_m'])
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    # camera axes: X forward (optical), Y left, Z up
    pc = np.stack([d, -(u - K[0][2]) * d / K[0][0], -(v - K[1][2]) * d / K[1][1]], -1)
    pts = pc @ R.T + t
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = pts[..., 2] - tz
    m_ = (d > 0) & (h > 0.04) & (h < 0.30) & (((hsv[..., 1] > 40) & (hsv[..., 0] < 30)) | (hsv[..., 2] < 70))
    p = pts[m_]
    if len(p) < 200:
        return dict(n=int(len(p)))
    return dict(n=int(len(p)), x_med=round(float(np.median(p[:, 0])), 3), x_p20=round(float(np.percentile(p[:, 0], 20)), 3),
                y_med=round(float(np.median(p[:, 1])), 3))

if __name__ == '__main__':
    print(json.dumps(hang(sys.argv[1], float(sys.argv[2]))))
