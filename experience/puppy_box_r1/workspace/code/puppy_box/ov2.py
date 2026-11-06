"""Overlay paw centre + leg axis + grasp point on a frame. python code/puppy_box/ov2.py frame px py pz yaw [depth] [out]"""
import sys, math
sys.path.insert(0, '.'); sys.path.insert(0, 'skill-library/skills/r1-leg-axis-grasp-short-carry/code')
import numpy as np, cv2
from chain import load_frame
fid = sys.argv[1]; paw = np.array([float(v) for v in sys.argv[2:5]]); yaw = float(sys.argv[5])
dep = float(sys.argv[6]) if len(sys.argv) > 6 else 0.03
out = sys.argv[7] if len(sys.argv) > 7 else 'output/overlay/%s.png' % fid
meta, rgb, D = load_frame(fid)
cp = meta['camera_pose_pelvis']; w, x, y, z = cp['quaternion_wxyz']
R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
t = np.array(cp['position_m']); K = np.array(meta['intrinsics_full'])
def proj(p):
    c = R.T @ (np.array(p) - t)
    return (int(K[0, 2] - K[0, 0] * c[1] / c[0]), int(K[1, 2] - K[1, 1] * c[2] / c[0]))
img = rgb.copy()
d = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0])
cv2.line(img, proj(paw - 0.03 * d), proj(paw + 0.12 * d), (0, 255, 0), 1)
cv2.circle(img, proj(paw), 3, (0, 0, 255), -1)
cv2.circle(img, proj(paw + dep * d), 3, (255, 0, 0), -1)
x0, y0 = proj(paw)
crop = img[max(0, y0 - 150):y0 + 60, max(0, x0 - 160):x0 + 160]
cv2.imwrite(out, cv2.resize(crop, None, fx=2, fy=2)); print(out, proj(paw), img.shape)
