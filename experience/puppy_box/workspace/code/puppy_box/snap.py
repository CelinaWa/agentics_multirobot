"""Observe (no motion), save obs json + rgb jpg under output/, print layout2 summary. Usage: python code/puppy_box/snap.py tag"""
import sys, os, json, base64
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c, layout2
from api.runtime import Context
if __name__ == '__main__':
    with Context() as ctx:
        ob, path = c.observe(ctx, sys.argv[1] if len(sys.argv) > 1 else 'snap')
    print(path, 'tilt', ob.get('tilt_deg'))
    try:
        print(json.dumps(layout2.layout(path), default=float))
    except Exception as e:
        print('layout failed', e)
