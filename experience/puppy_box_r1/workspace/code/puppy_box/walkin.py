"""Walk in from the reset start until the target paw (the clean overhanging FRONT paw nearest the head, rightmost) is
at the grasp stance ~(.33, -.20) (session 2fbfb7, sim). Straight walks + small right turns (-8 deg moves the scene
~+4.4 cm in y); the target is TRACKED from the start view (predicted = previous - walked), so it may be cut by the
image border later.  python code/puppy_box/walkin.py [x_goal=.33] [max_moves=7]
Exit 0 at the stance, 2 when the target is lost."""
import sys, json, math
from pathlib import Path
import numpy as np
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
import chain
from chain import ctx, log, perceive, select

x_goal = float(sys.argv[1]) if len(sys.argv) > 1 else 0.33
max_moves = int(sys.argv[2]) if len(sys.argv) > 2 else 7
Y_LO, Y_HI = -0.25, -0.16
X_OK = float(sys.argv[3]) if len(sys.argv) > 3 else 0.37   # R2: stop at <= .37 (one more .08 walk overshot to .29)
ctx.value('r1_head', pitch=0.6, secs=0.8)
o = perceive('walkin0')
T, R = select(o)
if T is None:
    log(step='STOP', why='no clean overhanging paw at the start'); sys.exit(2)
tp = np.array(T['c'][:2])
for k in range(1, max_moves + 1):
    dx = tp[0] - x_goal
    log(step='target', k=k, paw=tp.round(3).tolist(), dx=round(float(dx), 3))
    if tp[0] <= X_OK and Y_LO <= tp[1] <= Y_HI:
        log(step='at_stance', paw=tp.round(3).tolist()); sys.exit(0)
    if dx < -0.05:
        log(step='STOP', why='too close'); sys.exit(2)
    if tp[1] < Y_LO and dx < 0.12:       # too far right: small right turn (scene shifts left ~4.4 cm)
        wr = ctx.value('r1_walk', forward_m=0.0, turn_deg=-8.0)
        pred = tp + np.array([0.0, 0.044])
    elif tp[1] > Y_HI and dx < 0.12:
        wr = ctx.value('r1_walk', forward_m=0.0, turn_deg=8.0)
        pred = tp - np.array([0.0, 0.044])
    else:
        cmd = round(min(0.20, max(0.08, float(dx) * 0.8)), 3)
        wr = ctx.value('r1_walk', forward_m=cmd)
        pred = tp - np.array([float(wr.get('walked_m') or cmd), 0.0])
    log(step='move', walked=wr.get('walked_m'), turned=wr.get('turn_deg'), pelvis=wr.get('pelvis'), tilt=wr.get('tilt_deg'))
    o = perceive('walkin%d' % k)
    cands = [p for p in o['paws'] if p['n'] >= 400 and np.linalg.norm(np.array(p['c'][:2]) - pred) < 0.08]
    if not cands:
        log(step='STOP', why='target not re-found near %s' % pred.round(3).tolist(), paws=[p['c'] for p in o['paws']])
        sys.exit(2)
    tp = np.array(min(cands, key=lambda p: np.linalg.norm(np.array(p['c'][:2]) - pred))['c'][:2])
log(step='STOP', why='max moves'); sys.exit(2)
