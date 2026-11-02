"""Puppy far edge / top per 4 cm y slice from a saved observation (pelvis frame). python slices.py obs.json"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p
P = p.load(sys.argv[1]); tz = p.table_z(P); pp = p.puppy(P, tz, exclude=(-1, 2, -2, -0.08)); pts = pp['pts']
print('table z', round(tz, 4))
for y0 in np.arange(-0.10, 0.40, 0.04):
    s = pts[(pts[:, 1] > y0) & (pts[:, 1] < y0 + 0.04) & (pts[:, 2] < tz + 0.15)]
    if len(s) > 30:
        print(' y %.2f n %5d x %.3f..%.3f top %.3f' % (y0 + 0.02, len(s), np.percentile(s[:, 0], 3), np.percentile(s[:, 0], 97), np.percentile(s[:, 2], 97) - tz))
