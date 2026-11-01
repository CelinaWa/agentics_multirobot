import sys
sys.path.insert(0, 'code/pb6'); sys.path.insert(0, 'skill-library/skills/palm-pose-servo/code')
from ik_seed import seeded
from ik import palm_quat
from common import link_points
for x in [0.33, 0.36]:
    for yaw in [-90, -45, 0]:
        for z in [0.14, 0.05]:
            t, e, full = seeded(0.0, (x, 0.25, z), palm_quat(yaw, 'left'))
            lp = link_points(full, 'left')
            print(x, yaw, z, round(e, 3), [round(v, 2) for v in t.values()], 'th', lp['left_hand_thumb_2_link'], 'idx', lp['left_hand_index_1_link'])
