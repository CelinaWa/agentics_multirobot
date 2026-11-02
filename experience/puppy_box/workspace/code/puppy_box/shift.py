"""Shift the stored grasp point by dx, dy and hover there at top + dz (default 0.06). python code/puppy_box/shift.py dx dy [dz]"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pb3
from api.runtime import Context
dx, dy = float(sys.argv[1]), float(sys.argv[2]); dz = float(sys.argv[3]) if len(sys.argv) > 3 else 0.06
st = pb3.load(); st['grasp'] = [st['grasp'][0] + dx, st['grasp'][1] + dy]; pb3.save(st)
with Context() as ctx:
    r = pb3.move(ctx, (st['grasp'][0], st['grasp'][1], st['top'] + dz), pb3.GRASP_YAW, 3.0); r.pop('ob', None)
print(json.dumps({'grasp': st['grasp'], 'ok': r['ok'], 'why': r.get('why'), 'palm': r.get('palm'), 'frame': r.get('frame')}, default=float))
