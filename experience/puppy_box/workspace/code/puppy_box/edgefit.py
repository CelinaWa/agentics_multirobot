"""Fit the table front edge as a line in the level frame (1st percentile x of table points per 5 cm y-bin, robust fit
over |y| <= 0.25). Prints edge x at y=0 and the edge angle (deg, + = left side farther). Usage: edgefit.py obs.json"""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p


def fit(path):
    P = p.load(path); pel, lev = p.cloud(P); tz = p.table_z(P)
    m = (np.abs(lev[..., 2] - tz) < 0.01) & (P['depth'] > 0)
    xs, ys = lev[..., 0][m], lev[..., 1][m]
    pts = []
    for y0 in np.arange(-0.25, 0.25, 0.05):
        s = (ys >= y0) & (ys < y0 + 0.05)
        if s.sum() > 50:
            pts.append((y0 + 0.025, np.percentile(xs[s], 1)))
    pts = np.array(pts)
    med = np.median(pts[:, 1]); keep = np.abs(pts[:, 1] - med) < 0.06   # drop bins where a hand or the puppy hides the edge
    a, b = np.polyfit(pts[keep, 0], pts[keep, 1], 1)
    return dict(edge0=round(float(b), 3), angle_deg=round(float(np.degrees(np.arctan(a))), 1), table_z=round(float(tz), 4),
                bins=pts.round(3).tolist())


if __name__ == '__main__':
    print(json.dumps(fit(sys.argv[1])))
