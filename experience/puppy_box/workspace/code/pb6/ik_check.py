import sys
sys.path.insert(0, 'skill-library/skills/palm-pose-servo/code')
from common import M, link_points
from ik import solve, palm_quat
j = [-0.16516, -0.15347, 0.00982, 0.00884, -0.19727, -0.0311, -0.04702, -0.16484, 0.01887, 0.41763, 0.49987, 0.13381, 0.20995, -0.1669, -0.23881, 0.17675, -0.16561, -0.05996, 0.08767, 0.1887, -0.05429, 0.37711, 0.58982, 0.05707, -0.09369, -0.24515, 0.00885, -0.0252, 0.05787]
for lean in [0.0, 0.15]:
  for yaw in [-90, -60]:
    for z in [0.16, 0.08, 0.05, 0.03]:
        jj = list(j); jj[8] = lean
        t, e, full = solve(jj, 'left', (0.40, 0.23, z), palm_quat(yaw, 'left'))
        lp = link_points(full, 'left')
        print(lean, yaw, z, round(e, 3), {k.split('_', 1)[1][:3]: round(v, 2) for k, v in t.items()}, 'thumb', lp['left_hand_thumb_2_link'], 'idx', lp['left_hand_index_1_link'], 'elb', lp['left_elbow_link'])
