"""Observe and summarise the scene: frame path, table z, object slices, rim points (table+0.14)."""
import json, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'skill-library/skills/hand-claw-pregrasp/code'))
import g1 as g1mod, perception as P  # noqa


def look(g, ymin=-0.7, ymax=0.7, xmax=0.85):
    o = g.observe()
    fid = o['camera']['frame_id']
    t, sl = P.slices(o, ymin=ymin, ymax=ymax, xmax=xmax, dz=0.05)
    tr, rim = P.rim_points(o, 0.14, xmax=xmax)
    pts = g.points('pelvis')
    out = {'frame': 'runtime/frames/%s/cam_front.jpg' % fid, 'table_z': round(t, 3), 'tilt': o.get('tilt_deg'),
           'slices': sl, 'n_rim': len(rim),
           'palms': {s: pts.get('%s_hand_palm_link' % s) for s in ('left', 'right')}}
    if len(rim):
        # rim points binned in y
        hist = {}
        for p in rim:
            k = round(float(p[1]) * 20) / 20
            hist.setdefault(k, []).append(p[0])
        out['rim_y_bins'] = {k: [len(v), round(float(np.min(v)), 2), round(float(np.max(v)), 2)] for k, v in sorted(hist.items()) if len(v) > 5}
    return o, out


if __name__ == '__main__':
    g = g1mod.G1()
    o, out = look(g)
    for k, v in out.items():
        if k == 'slices':
            print('slices [y, x5, x95, ztop, n]:')
            for s in v: print('  ', s)
        else:
            print(k, v)
