"""Puppy points from one or more obs jsons (pelvis frame): y range, far edge per y-slice, top. Usage: pup.py obs.json..."""
import sys, os, json, numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code'))
import percep as p

def pts_of(path, exclude_y_above=None):
    P = p.load(path); tz = p.table_z(P)
    pp = p.puppy(P, tz, exclude=(-1, 2, -0.10, 2))
    pel = p.cloud(P)[0][pp['mask']]
    pel = pel[(pel[:, 2] > tz + 0.015) & (pel[:, 2] < tz + 0.15)]
    if exclude_y_above is not None:
        pel = pel[pel[:, 1] < exclude_y_above]
    return pel, tz

if __name__ == '__main__':
    allp = []
    for f in sys.argv[1:]:
        pts, tz = pts_of(f); allp.append(pts); print(f, 'table_z', round(tz, 4), 'n', len(pts))
    pts = np.vstack(allp)
    print('y pct 2/50/98', np.percentile(pts[:, 1], [2, 50, 98]).round(3), 'x pct', np.percentile(pts[:, 0], [2, 50, 98]).round(3))
    for y0 in np.arange(-0.6, 0.0, 0.04):
        m = (pts[:, 1] >= y0) & (pts[:, 1] < y0 + 0.04)
        if m.sum() > 30:
            s = pts[m]
            print(f'y {y0:+.2f}: n {m.sum():5d} x {np.percentile(s[:,0],3):.3f}-{np.percentile(s[:,0],97):.3f} top {np.percentile(s[:,2],97):+.3f}')
