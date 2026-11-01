"""Left-hand free-space hover test: unfold, palm-down fingers-forward over the front paws, measure, retract."""
import sys, json
sys.path.insert(0, '.'); sys.path.insert(0, 'skill-library/skills/palm-pose-servo/code')
from api.runtime import Context
from run import unfold, main, retract_home
from ik import palm_quat
ctx = Context()
step = sys.argv[1]
if step == 'unfold':
    unfold(ctx, 'left')
    o = ctx.value('g1_observe'); print(o['camera']['frame_id'], o['tilt_deg'])
elif step == 'hover':
    tgt = tuple(float(v) for v in sys.argv[2:5])
    r = main(ctx, 'left', tgt, quat_wxyz=palm_quat(0, 'left'), iters=2)
    print(json.dumps({k: r[k] for k in ('ok', 'why')}), json.dumps({k: r['measured'].get(k) for k in ('palm', 'err_m', 'bias', 'frame_id', 'tilt_deg', 'ik_err_m', 'jump_rad')}))
elif step == 'retract':
    print(retract_home(ctx, 'left'))
