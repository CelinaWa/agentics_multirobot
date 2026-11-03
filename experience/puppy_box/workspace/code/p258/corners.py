"""Box footprint from colour-image corner pixels: rim corners hit the plane table_z+0.14, floor corners table_z.
  python code/p258/corners.py <obs.json> rim u,v u,v ... floor u,v ...  -> level-frame xy of each corner"""
import sys, json, numpy as np
sys.path.insert(0, 'code/puppy_box')
import percep as p
P = p.load(sys.argv[1]); tz = p.table_z(P); mode = None; out = []
for a in sys.argv[2:]:
    if a in ('rim', 'floor'):
        mode = a; continue
    u, v = (float(t) for t in a.split(','))
    h = p.plane_hit(P, u, v, tz + (0.14 if mode == 'rim' else 0.0))
    out.append((mode, u, v, round(float(h[0]), 3), round(float(h[1]), 3)))
print('table_z', round(tz, 4))
for o in out: print(o)
