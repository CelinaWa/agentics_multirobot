"""Observe, save obs json + png, print layout2. Usage: python code/puppy_box/look.py TAG"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c, percep as p, cv2
from layout2 import layout
from api.runtime import Context
if __name__ == '__main__':
    tag = sys.argv[1] if len(sys.argv) > 1 else 'look'
    with Context() as ctx:
        ob, path = c.observe(ctx, tag)
    P = p.load(path); png = path.replace('.json', '.png'); cv2.imwrite(png, P['img'])
    L = layout(path); L['tilt'] = ob['tilt_deg']; L['png'] = png; L['json'] = path
    if L['box']: L['box'].pop('rect', None)
    print(json.dumps(L, default=float))
