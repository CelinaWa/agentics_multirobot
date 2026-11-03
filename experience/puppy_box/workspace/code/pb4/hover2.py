"""run_f0 hover from the side-raised pose (session 4ddb74): short path-checked hops toward the grasp, then the same
one-shot correction on the measured palm (stores st['cmd'])."""
import sys, os, json
sys.path[:0] = [os.path.join(os.getcwd(), d) for d in ('skill-library/skills/plush-pick-place-box-behind/code', 'code/puppy_box', '.')]
import run_f0 as f
from api.runtime import Context
with Context() as ctx:
    st = f.load(); st['contact_ok'] = False
    ctx.value('g1_hand_shape', side='right', open=[0, 0.4, 0, 0.5, 0, 0.5, 0], closed=[0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74])
    tx, ty, top = st['far'] - f.BACK, st['gy'], st['top']
    for pos in [(0.08, -0.28, top + 0.25), (0.15, -0.20, top + 0.23), (tx, ty, top + 0.20), (tx, ty, top + 0.13)]:
        r = f.move(ctx, st, pos, 4.0)
        print(json.dumps(dict(pos=[round(v, 3) for v in pos], ok=r['ok'], palm=r.get('palm'), why=r.get('why'), tilt=r.get('tilt'))), flush=True)
        if not r['ok']:
            sys.exit(1)
    st['cmd'] = [tx + (tx - r['palm'][0]) - f.PRESS_BACK, ty + (ty - r['palm'][1])]
    f.save(st)
    print(json.dumps(dict(r, target=[round(tx, 3), round(ty, 3)], cmd=[round(v, 3) for v in st['cmd']]), default=float))
