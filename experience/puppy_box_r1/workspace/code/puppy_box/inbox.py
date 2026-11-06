"""Judge 'puppy inside the box' from depth: cream puppy points vs the box rim volume (session 2fbfb7).
python code/puppy_box/inbox.py [frame_id]   (default: new observation at the current head pitch)
Prints table z, box rim pose, and the fraction of puppy points inside the box footprint / below the rim."""
import sys, json
from pathlib import Path
import numpy as np, cv2
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
from api.runtime import Context
import scene
from percep import load_frame, cloud
import chain


def judge(fid):
    b = scene.brief(fid); zt = b['table_z']
    bp = chain.box_pose(fid, zt)
    meta, rgb, dep = load_frame(fid)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 2] > 110) & (hsv[..., 1] > 35) & (hsv[..., 1] < 140) & (hsv[..., 0] > 8) & (hsv[..., 0] < 35)
    brown = (hsv[..., 2] < 90) & (hsv[..., 2] > 15) & (hsv[..., 1] > 60)
    P, ok = cloud(meta, dep, stride=1)
    pts = P[ok & (cream | brown)]
    pts = pts[pts[:, 2] > zt + 0.01]
    out = {'frame': fid, 'table_z': zt, 'box_pose': bp, 'n': int(len(pts))}
    if bp and len(pts):
        x0 = bp['x0']; ylo, yhi = bp['ylo'], bp['yhi']
        if abs((yhi - ylo) - 0.395) > 0.03:
            yhi = ylo + 0.395
        inxy = (pts[:, 0] > x0 + 0.01) & (pts[:, 0] < x0 + 0.27) & (pts[:, 1] > ylo + 0.01) & (pts[:, 1] < yhi - 0.01)
        rim = zt + chain.RIM_H
        out.update(rim_z=round(rim, 3), frac_in_footprint=round(float(inxy.mean()), 3),
                   n_in_below_rim=int((inxy & (pts[:, 2] < rim - 0.01)).sum()),
                   n_in_above_rim=int((inxy & (pts[:, 2] >= rim - 0.01)).sum()),
                   n_front_of_box=int((pts[:, 0] < x0 - 0.01).sum()),
                   z_in=[round(float(v), 3) for v in np.percentile(pts[inxy][:, 2], [5, 50, 95])] if inxy.any() else None)
    return out


if __name__ == '__main__':
    if len(sys.argv) > 1:
        fid = sys.argv[1]
    else:
        fid = Context().value('r1_observe')['camera']['frame_id']
    print(json.dumps(judge(fid), default=str))
