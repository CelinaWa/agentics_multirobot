"""Draw the stored v6 look (paw centre, leg axis, alt yaws) onto its frame. python code/puppy_box/overlay.py [out.png] [yaw2 ...]"""
import sys, json, math
sys.path.insert(0, '.'); sys.path.insert(0, 'code/puppy_box')
import numpy as np, cv2
from chain import load_frame
s = json.load(open('output/v6_state.json'))
meta, rgb, dep = load_frame(s['frame'])
cp = meta['camera_pose_pelvis']
w, x, y, z = cp['quaternion_wxyz']
R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
t = np.array(cp['position_m']); K = np.array(meta['intrinsics_full'])
def proj(p):
    c = R.T @ (np.array(p) - t)          # camera X fwd, Y left, Z up
    return (int(K[0, 2] - K[0, 0] * c[1] / c[0]), int(K[1, 2] - K[1, 1] * c[2] / c[0]))
img = rgb.copy()
paw = np.array(s['paw']); ap = np.array(s['axis_pt'])
cv2.circle(img, proj(paw), 4, (0, 0, 255), -1)
yaws = [s['leg_yaw']] + [float(v) for v in sys.argv[2:]]
cols = [(0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255)]
for k, yw in enumerate(yaws):
    d = np.array([math.cos(math.radians(yw)), math.sin(math.radians(yw)), 0])
    cv2.line(img, proj(ap - 0.03 * d), proj(ap + 0.12 * d), cols[k % 4], 1)
for e in s.get('axis_edge', []):
    pass
out = sys.argv[1] if len(sys.argv) > 1 else 'output/overlay/look.png'
x0, y0 = proj(paw)
crop = img[max(0, y0 - 160):y0 + 80, max(0, x0 - 200):x0 + 200]
cv2.imwrite(out, cv2.resize(crop, None, fx=2, fy=2))
print(out, proj(paw))
