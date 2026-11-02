"""Puppy body from several obs jsons (robot not moved between them): merged cream points above the table, excluding
x > box near wall. Prints mid-body y (tall part), far edge near mid-body, top, y extent. Usage: pup2.py BOXX0 obs1 obs2..."""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p
bx0 = float(sys.argv[1]); allp = []
for f in sys.argv[2:]:
    P = p.load(f); tz = p.table_z(P)
    pp = p.puppy(P, tz, exclude=(bx0 + 0.02, 2, -2, 2))
    pel, _ = p.cloud(P); pts = pel[pp['mask']]
    pts = pts[(pts[:, 2] < tz + 0.15) & (pts[:, 2] > tz + 0.02)]; allp.append(pts)
pts = np.vstack(allp)
top = float(np.percentile(pts[:, 2], 97))
body = pts[pts[:, 2] > tz + 0.7 * (top - tz)]
yc = float(np.median(body[:, 1]))
mid = pts[np.abs(pts[:, 1] - yc) < 0.05]
print(json.dumps(dict(tz=tz, top=round(top, 3), yc=round(yc, 3), body_y=np.percentile(body[:, 1], [2, 98]).round(3).tolist(),
      all_y=np.percentile(pts[:, 1], [1, 99]).round(3).tolist(), far=round(float(np.percentile(mid[:, 0], 97)), 3),
      near=round(float(np.percentile(mid[:, 0], 3)), 3))))
