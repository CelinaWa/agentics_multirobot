"""Robust table z (histogram peak below +0.0 in the level frame) and front edge (median over y-bins of each bin's
nearest table point, ignoring points within 0.12 m of the hands' rough image region). Usage: edge.py obs.json"""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p

def edge(path):
    P = p.load(path); _, lev = p.cloud(P); ok = P['depth'] > 0
    z = lev[..., 2][ok]
    hist, e = np.histogram(z, bins=np.arange(-0.2, 0.0, 0.005)); tz = float(e[np.argmax(hist)] + 0.0025)
    m = ok & (np.abs(lev[..., 2] - tz) < 0.008)
    xs, ys = lev[..., 0][m], lev[..., 1][m]
    near = []
    for y0 in np.arange(-0.4, 0.4, 0.05):
        s = (ys >= y0) & (ys < y0 + 0.05)
        if s.sum() > 50: near.append(float(np.percentile(xs[s], 2)))
    return dict(table_z=round(tz, 4), edge_bins=[round(v, 3) for v in near], edge=round(float(np.median(near)), 3))

if __name__ == '__main__':
    print(json.dumps(edge(sys.argv[1])))
