"""Left-hand clearance: min distance between cream (plush) points in a fresh frame and the left hand links (FK on the measured joints)."""
import json, os, sys
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'skill-library', 'skills', 'plush-pick-place-box-same-side', 'code'))
import controller3 as c3
c, p = c3.c, c3.p


def left_clearance(ob, path):
    P = p.load(path); pel, lev = p.cloud(P)
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    cream = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 30) & (hsv[..., 1] > 40) & (hsv[..., 1] < 140) & (hsv[..., 2] > 120) & (P['depth'] > 0)
    q = pel[cream]
    lp = c.pps.link_points(ob['proprio']['joint_positions_rad'], 'left')
    pts = np.array([v for k, v in lp.items()])
    if len(q) == 0:
        return None, lp
    d = np.min(np.linalg.norm(q[None, ::3, :] - pts[:, None, :], axis=2))
    return float(d), {k: np.round(v, 3).tolist() for k, v in lp.items()}


if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        ob, path = c.observe(ctx, 'lclear')
        d, lp = left_clearance(ob, path)
        print(json.dumps(dict(min_dist=d, frame=ob['camera']['frame_id'], left=lp)))
