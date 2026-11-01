import sys
import numpy as np
sys.path.insert(0, 'skill-library/skills/palm-pose-servo/code')
from common import M, link_points
from ik import solve, palm_quat
J = M.canonical29
base = [-0.16516, -0.15347, 0.00982, 0.00884, -0.19727, -0.0311, -0.04702, -0.16484, 0.01887, 0.41763, 0.49987, 0.13381, 0.20995, -0.1669, -0.23881, 0.17675, -0.16561, -0.05996, 0.08767, 0.1887, -0.05429, 0.37711, 0.58982, 0.05707, -0.09369, -0.24515, 0.00885, -0.0252, 0.05787]
def seeded(lean, pos, quat, side='left'):
    best = None
    for sp, sr, sy, el, wr in [(-1.0, 0.3, 0, 0.5, 0), (-0.8, 0.1, -0.5, 0.8, 1.0), (-1.2, 0.5, 0.5, 0.3, 1.5), (-0.6, 0.2, -1.0, 1.2, -1.0), (-1.0, 0.0, 0.0, 1.0, 1.5), (-1.0, 0.4, -0.3, 0.6, 2.0)]:
        j = list(base); j[8] = lean
        for n, v in zip(['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll'], [sp, sr, sy, el, wr]):
            j[J.index(f'{side}_{n}_joint')] = v
        t, e, full = solve(j, side, pos, quat)
        if best is None or e < best[1]:
            best = (t, e, full)
    return best
if __name__ == '__main__':
    for lean in [0.0, 0.15]:
        for yaw in [-90, -60]:
            for z in [0.16, 0.06, 0.04]:
                t, e, full = seeded(lean, (0.40, 0.23, z), palm_quat(yaw, 'left'))
                lp = link_points(full, 'left')
                print(lean, yaw, z, round(e, 3), [round(v, 2) for v in t.values()], 'th', lp['left_hand_thumb_2_link'], 'idx', lp['left_hand_index_1_link'])
