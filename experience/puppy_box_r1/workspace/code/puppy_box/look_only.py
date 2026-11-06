"""look only (no motion except head .6): python code/puppy_box/look_only.py"""
import sys, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('s', 'code/puppy_box/lowtilt_v4.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ctx = Context(); m.chain.ctx = ctx
ctx.value('r1_head', pitch=0.6, secs=0.5)
r = m.look(ctx, log=lambda *a: None)
print(r['ok'], r['why'], {k: r['measured'].get(k) for k in ('frame', 'paw', 'leg_yaw', 'wall', 'overhang', 'edge')})
if not r['ok']:
    print(r['measured'])
