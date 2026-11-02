"""Left-hand clearance check: left palm/finger points vs the puppy cloud (no motion). python lcheck.py"""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c, percep as p
from api.runtime import Context
with Context() as ctx:
    ob, path = c.observe(ctx, 'lcheck')
j = ob['proprio']['joint_positions_rad']
lp = c.pps.link_points(j, 'left')
P = p.load(path); tz = p.table_z(P)
pel, _ = p.cloud(P)
pts = pel.reshape(-1, 3)
pts = pts[np.isfinite(pts).all(1) & (pts[:, 2] > tz + 0.015) & (pts[:, 2] < tz + 0.2)]
out = {'tilt': ob['tilt_deg'], 'table_z': round(tz, 3), 'frame': ob['camera']['frame_id'], 'path': path,
       'left_joints': {k: round(j[c.J.index('left_' + k + '_joint')], 2) for k in ('shoulder_pitch', 'elbow')},
       'fingersL': [round(v, 2) for v in c.fingers(ob, 'left')], 'fingersR': [round(v, 2) for v in c.fingers(ob, 'right')]}
for k, v in lp.items():
    v = np.array(v)
    d = np.linalg.norm(pts - v, axis=1)
    out[k] = [round(x, 3) for x in v] + [round(float(d.min()), 3) if len(d) else None]
print(json.dumps(out))
