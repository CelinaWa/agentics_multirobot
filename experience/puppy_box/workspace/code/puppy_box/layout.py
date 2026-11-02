"""python code/puppy_box/layout.py obs.json : table height, table front edge, puppy blob (pelvis frame)."""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p
from find_puppy import find

def table_edge(P, tz):
    pel, lev = p.cloud(P)
    m = (np.abs(lev[..., 2] - tz) < 0.01) & (P['depth'] > 0)
    xs = lev[m][:, 0]
    return float(np.percentile(xs, 1))

if __name__ == '__main__':
    P = p.load(sys.argv[1]); tz = p.table_z(P)
    ex = tuple(float(v) for v in sys.argv[2].split(',')) if len(sys.argv) > 2 else None
    print(json.dumps({'table_z_level': tz, 'edge_x': table_edge(P, tz), 'tilt': P['ob']['tilt_deg'], 'puppy': find(sys.argv[1], ex)}))
