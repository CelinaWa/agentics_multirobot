"""Layout from one obs json: table z, edge x, puppy (cream, right/left of a split pixel column), box rim footprint
(sim depth sees the rim plane at table + ~0.145). Usage: python code/puppy_box/layout2.py obs.json"""
import sys, os, json, numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p

def layout(path):
    P = p.load(path); pel, lev = p.cloud(P); tz = p.table_z(P)
    h = lev[..., 2] - tz
    ok = P['depth'] > 0
    m = (np.abs(h) < 0.01) & ok & (np.abs(lev[..., 1]) < 0.15)
    edge = float(np.percentile(lev[..., 0][m], 1))
    rim = (np.abs(h - 0.145) < 0.01) & ok
    rim = cv2.morphologyEx(rim.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(rim)
    box = None
    if n > 1:
        k = 1 + np.argmax(st[1:, 4]); pts = lev[lab == k]
        r = cv2.minAreaRect(pts[:, :2].astype(np.float32))
        box = dict(px=int(st[k, 4]), x=np.percentile(pts[:, 0], [1, 99]).round(3).tolist(),
                   y=np.percentile(pts[:, 1], [1, 99]).round(3).tolist(), rect=[list(r[0]), list(r[1]), r[2]],
                   corners=cv2.boxPoints(r).round(3).tolist())
        rimmask = lab == k
    else:
        rimmask = np.zeros_like(ok)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 1] > 40) & (h > 0.015) & (h < 0.2) & ok & ~rimmask.astype(bool)
    cream = cv2.morphologyEx(cream.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(cream)
    pup = None
    if n > 1:
        k = 1 + np.argmax(st[1:, 4]); pts = lev[lab == k]
        pup = dict(px=int(st[k, 4]), bbox_px=st[k, :4].tolist(), x=np.percentile(pts[:, 0], [2, 50, 98]).round(3).tolist(),
                   y=np.percentile(pts[:, 1], [2, 50, 98]).round(3).tolist(), top=round(float(np.percentile(pts[:, 2], 98)), 3))
    return dict(frame=P['fid'], table_z=round(tz, 4), edge=round(edge, 3), box=box, puppy=pup)

if __name__ == '__main__':
    print(json.dumps(layout(sys.argv[1]), indent=1, default=float))
