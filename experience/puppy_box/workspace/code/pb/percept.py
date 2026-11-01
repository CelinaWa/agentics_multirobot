"""Puppy perception with a local table estimate (the floor is in view when the table edge is close)."""
import numpy as np
from sh import P


def table_z(pts):
    z = pts[:, 2]
    z = z[(z > -0.4) & (z < 0.3)]
    h, e = np.histogram(z, bins=np.arange(-0.4, 0.3, 0.005))
    return float(e[h.argmax()] + 0.0025)


def puppy(o, ymin=-0.25, ymax=0.5, xmin=0.25, xmax=0.75):
    pts = P.cloud(o, 2).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(1) & (np.abs(pts).sum(1) > 0)]
    reg = pts[(pts[:, 0] > xmin) & (pts[:, 0] < xmax) & (pts[:, 1] > ymin) & (pts[:, 1] < ymax)]
    t = table_z(reg)
    obj = reg[(reg[:, 2] > t + 0.02) & (reg[:, 2] < t + 0.2)]
    sl = []
    for y0 in np.arange(ymin, ymax, 0.02):
        q = obj[(obj[:, 1] >= y0) & (obj[:, 1] < y0 + 0.02)]
        hi = q[q[:, 2] > t + 0.07]
        if len(q) > 10:
            sl.append([round(float(y0 + 0.01), 2), len(q), len(hi),
                       round(float(np.percentile(q[:, 0], 5)), 3), round(float(np.percentile(q[:, 0], 95)), 3),
                       round(float(np.percentile(hi[:, 0], 5)), 3) if len(hi) > 5 else None,
                       round(float(np.percentile(hi[:, 0], 95)), 3) if len(hi) > 5 else None,
                       round(float(q[:, 2].max() - t), 3)])
    return t, obj, sl
