"""Stills of the office scene and a standing-reach check: how low each robot's hand gets without bending its legs.
  view_scene.py <scene dir>"""
import sys
from pathlib import Path
import numpy as np, mujoco, cv2
out = Path(sys.argv[1])
m = mujoco.MjModel.from_xml_path(str(out / 'office_g1_r1.xml')); d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 600, 800)
box = d.xpos[m.body('box').id]
views = {'like_video': (-4, -22, 4.7, [2.6, 0.1, 0.8]), 'top': (90, -89, 6.0, [1.9, 0.0, 0.0]),
         'from_right': (55, -30, 3.6, [2.4, 0.3, 0.6]), 'toward_left_wall': (90, -16, 2.1, [2.4, 0.9, 0.9])}
for name, (az, el, dist, look) in views.items():
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth, cam.elevation, cam.distance = az, el, dist; cam.lookat[:] = look
    r.update_scene(d, cam); img = np.ascontiguousarray(r.render()[:, :, ::-1])
    cv2.putText(img, name, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2); cv2.imwrite(str(out / f'{name}.jpg'), img)
print('robots:', {p: (np.round(d.xpos[m.body(('' if p == 'g1' else 'r1_') + 'pelvis').id], 3).tolist()) for p in ('g1', 'r1')}, 'box', np.round(box, 3).tolist())
rim = {n: d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)].copy() for n in ('box_rim_a', 'box_rim_b')}
print('box rim height %.3f m' % rim['box_rim_a'][2])
# standing reach: sample each robot's right arm (and waist) joints, legs as they stand; lowest hand point and its forward distance
rng = np.random.default_rng(0)
def joints(prefix, words):
    return [j for j in range(m.njnt) if m.joint(j).name.startswith(prefix) and any(w in m.joint(j).name for w in words)]
hands = {'g1': ('right_wrist_yaw_link', ('right_shoulder', 'right_elbow', 'right_wrist', 'waist_pitch')),
         'r1': ('r1_right_wrist_roll_link', ('right_shoulder', 'right_elbow', 'right_wrist'))}
q0 = d.qpos.copy()
for robot, (hand, words) in hands.items():
    js = joints('' if robot == 'g1' else 'r1_', words); pel = d.xpos[m.body(('' if robot == 'g1' else 'r1_') + 'pelvis').id].copy()
    R = d.xmat[m.body(('' if robot == 'g1' else 'r1_') + 'pelvis').id].reshape(3, 3); best = None
    for _ in range(40000):
        d.qpos[:] = q0
        for j in js:
            lo, hi = m.jnt_range[j]; d.qpos[m.jnt_qposadr[j]] = rng.uniform(lo, hi)
        mujoco.mj_kinematics(m, d)
        p = d.xpos[m.body(hand).id]; local = R.T @ (p - pel)
        if 0.25 <= local[0] <= 0.60 and abs(local[1]) < 0.30 and (best is None or p[2] < best[0]):
            best = (float(p[2]), local.round(3).tolist())
    print('%s standing, legs straight: lowest wrist 0.25-0.60 m ahead = %.3f m above the floor (pelvis frame %s); joints sampled: %s'
          % (robot, best[0], best[1], [m.joint(j).name for j in js]))
