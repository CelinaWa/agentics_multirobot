"""Pixel(s) -> level-frame point on a horizontal plane (box walls are invisible in depth). Usage: rimhit.py obs.json dz u,v [u,v ...]
dz is height above the table plane (0 = floor line, 0.14 = rim)."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'puppy_box'))
import percep as p
P = p.load(sys.argv[1]); tz = p.table_z(P); dz = float(sys.argv[2])
for a in sys.argv[3:]:
    u, v = map(float, a.split(','))
    print(a, [round(float(x), 3) for x in p.plane_hit(P, u, v, tz + dz)])
