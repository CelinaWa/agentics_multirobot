"""Locate the puppy torso (upper part of the plush) in the latest observation."""
import json, sys
import numpy as np
sys.path.insert(0, '.')
import scene


def torso(o, xmax=0.62, ymin=-0.15, ymax=0.40):
    t, sel, (u, v) = scene.puppy(o, xmax=xmax)
    keep = (sel[:, 1] > ymin) & (sel[:, 1] < ymax)
    sel, u, v = sel[keep], u[keep], v[keep]
    top = sel[sel[:, 2] > t + 0.08]
    # torso = biggest high blob: take y-histogram of high points, the widest x-extent slice band
    res = {'table_z': round(t, 3), 'n_high': int(len(top))}
    if len(top) < 30:
        return res
    ys = np.arange(ymin, ymax, 0.02)
    best = None
    for y0 in ys:
        q = top[(top[:, 1] >= y0) & (top[:, 1] < y0 + 0.02)]
        if len(q) > 10:
            row = (y0 + 0.01, *np.percentile(q[:, 0], [5, 95]), q[:, 2].max(), len(q))
            res.setdefault('slices', []).append([round(float(x), 3) for x in row])
    return res


if __name__ == '__main__':
    o = json.load(open('../../output/obs/last.json'))
    print(json.dumps(torso(o)))
