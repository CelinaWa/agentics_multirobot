"""Render the office scene from the viewpoints of the reference video frames, for a side-by-side comparison.
  compare_views.py <scene dir>   -> <scene dir>/cmp_<name>.jpg (450 x 600 each)"""
import sys, math
from pathlib import Path
import numpy as np, mujoco, cv2
out = Path(sys.argv[1])
m = mujoco.MjModel.from_xml_path(str(out / 'office_g1_r1.xml')); d = mujoco.MjData(m)
for prefix in ('', 'r1_'):                 # the robots are not in the video frames: park them out of view
    j = m.body(prefix + 'pelvis').jntadr[0]; d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 2] = [-6.0, 6.0 if prefix == '' else 7.0]
j = m.body('box').jntadr[0]; d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 2] = [-6.0, 5.0]
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 600, 450)
# name: (camera position, point it looks at) in the room frame; each matches one video frame
VIEWS = {'a_t0.4_from_near_right': ([-0.1, -1.15, 1.45], [3.4, -0.5, 0.55]),
         'b_t4.6_fridge_corner': ([2.0, -0.55, 1.5], [3.7, -0.75, 0.6]),
         'c_t7.0_along_back_wall': ([3.1, -1.0, 1.5], [3.3, 1.2, 0.7]),
         'd_t9.3_corner_shelves': ([2.3, -0.4, 1.5], [3.0, 1.4, 0.9]),
         'e_t11_left_wall': ([2.3, 0.0, 1.45], [2.0, 1.5, 0.8]),
         'f_t14.5_wire_shelf_pillar': ([2.1, 0.5, 1.5], [0.9, 1.4, 1.0]),
         'g_t19.5_from_near_left': ([-0.2, 0.75, 1.55], [3.0, -1.0, 0.7]),
         'h_t24.5_overview': ([-1.3, 0.55, 1.95], [3.2, 0.2, 0.7])}
for name, (pos, tgt) in VIEWS.items():
    v = np.array(tgt, float) - np.array(pos, float); dist = float(np.linalg.norm(v))
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = tgt; cam.distance = dist
    cam.azimuth = math.degrees(math.atan2(v[1], v[0])); cam.elevation = math.degrees(math.asin(v[2] / dist))
    r.update_scene(d, cam); img = np.ascontiguousarray(r.render()[:, :, ::-1])
    cv2.putText(img, name[2:], (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2); cv2.imwrite(str(out / f'cmp_{name}.jpg'), img)
print('rendered', len(VIEWS))
