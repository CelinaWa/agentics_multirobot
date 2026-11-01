import sys, os, json
sys.path.insert(0, 'skill-library/skills/palm-pose-servo/code')
from common import M
from ik import solve, palm_quat
import numpy as np
j = [-0.18294, -0.15412, 0.01559, 0.0403, -0.10807, -0.02037, 0.03994, -0.11716, -0.00303, 0.35536, 0.42922, 0.09473, 0.23558, -0.15138, -0.2533, 0.16187, -0.11581, -0.04656, 0.04405, 0.11292, -0.13655, 0.43651, 0.54714, 0.04205, -0.1406, -0.34631, 0.00676, -0.01371, 0.1066]
W = {'yaw': 2, 'roll': 5, 'pitch': 8}
for side in ['left', 'right']:
    for (p, y) in [(0, 0), (0.2, 0), (0.2, -0.3), (0.2, 0.3)]:
        jj = list(j); jj[8] = p; jj[2] = y
        for tgt in [(0.45, 0.30, 0.10), (0.38, 0.27, 0.06), (0.43, -0.33, 0.20), (0.40, -0.25, 0.20)]:
            _, e, _ = solve(jj, side, tgt)
            print(side, p, y, tgt, round(e, 3))
