"""Observe and measure the table front edge per y-bin (pelvis frame) from depth, ignoring the hands' pixels.
  PYTHONPATH=.:code/puppy_box python code/p258/edge.py   -> prints table z and edge x per y bin, saves output/p258_edge_<step>.json"""
import sys, os, json, numpy as np
sys.path.insert(0, 'code/puppy_box')
import percep as p
from api.runtime import Context


def edge(path):
    P = p.load(path)
    pel, lev = p.cloud(P)
    tz = p.table_z(P)
    m = (P['depth'] > 0) & (np.abs(lev[..., 2] - tz) < 0.008)
    out = {}
    for y0 in np.arange(-0.30, 0.40, 0.05):
        s = m & (pel[..., 1] >= y0) & (pel[..., 1] < y0 + 0.05)
        if s.sum() > 30:
            out[round(float(y0 + 0.025), 3)] = round(float(np.percentile(pel[..., 0][s], 1)), 3)
    return tz, out


if __name__ == '__main__':
    with Context() as ctx:
        ob = ctx.value('g1_observe')
    path = f'output/p258_edge_{ob["step_id"]}.json'
    json.dump(ob, open(path, 'w'))
    tz, out = edge(path)
    print(json.dumps(dict(table_z=round(tz, 4), edge_x_by_y=out, frame=ob['camera']['frame_id'], obs=path, tilt=ob['tilt_deg'])))
