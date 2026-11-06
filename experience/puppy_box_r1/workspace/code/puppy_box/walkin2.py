"""Walk in from the reset start to a (possibly ROTATED) grasp stance (session 90b103, sim).
W0 showed lateral shifts by turn pairs do not work (forward >> lateral). Instead: ONE left/right turn far from the
table rotates the target paw to y ~ -.20 in the pelvis frame (the robot then stands up to ~25 deg off square, the
leg yaw in the pelvis frame drops by the same angle), then straight walks until the paw is at x <= X_OK.
    python code/puppy_box/walkin2.py [x_ok .37] [y_goal -.20] [max_moves 8]
Exit 0 at the stance (prints paw, edge), 2 lost/too close/max moves."""
import sys, math, json
sys.path.insert(0, '.'); sys.path.insert(0, 'code/puppy_box')
import numpy as np
import chain
from chain import ctx, log, perceive, select

X_OK = float(sys.argv[1]) if len(sys.argv) > 1 else 0.37
Y_GOAL = float(sys.argv[2]) if len(sys.argv) > 2 else -0.20
MAXM = int(sys.argv[3]) if len(sys.argv) > 3 else 8
Y_TOL = 0.04
ctx.value('r1_head', pitch=0.6, secs=0.8)


def target(o, pred=None):
    T, R = select(o)
    if pred is None:
        return T
    c = [p for p in o['paws'] if p['n'] >= 400 and np.linalg.norm(np.array(p['c'][:2]) - pred) < 0.09]
    if c:
        return min(c, key=lambda p: np.linalg.norm(np.array(p['c'][:2]) - pred))
    return T          # the reported turn angle is unreliable (W0: -45 reported -6.8): fall back to the clean overhang


o = perceive('wi2_0'); T = target(o)
if T is None:
    log(step='STOP', why='no clean overhanging paw at the start'); sys.exit(2)
tp = np.array(T['c'][:2]); turned = 0.0
for k in range(1, MAXM + 1):
    log(step='target', k=k, paw=tp.round(3).tolist(), edge=o['edge'])
    if tp[0] <= X_OK:
        if abs(tp[1] - Y_GOAL) <= 0.05 and tp[0] >= 0.28:
            log(step='at_stance', paw=tp.round(3).tolist(), edge=o['edge'], turned=turned); sys.exit(0)
        log(step='STOP', why='too close / off sideways', paw=tp.round(3).tolist()); sys.exit(2)
    if abs(tp[1] - Y_GOAL) > Y_TOL and tp[0] > 0.45 and abs(turned) < 35:
        # rotation that brings the paw to Y_GOAL: solve -x sin(th) + y cos(th) = Y_GOAL
        r = math.hypot(*tp); a = math.atan2(tp[1], tp[0])
        th = a - math.asin(max(-1, min(1, Y_GOAL / r)))
        cmd = round(math.degrees(th) * 1.15, 1)               # turns came out ~.75-.87 of the command
        cmd = max(-30.0, min(30.0, cmd))
        wr = ctx.value('r1_walk', forward_m=0.0, turn_deg=cmd)
        tt = float(wr.get('turn_deg') or 0.0); turned += tt
        c, s_ = math.cos(math.radians(tt)), math.sin(math.radians(tt))
        pred = np.array([tp[0] * c + tp[1] * s_, -tp[0] * s_ + tp[1] * c]) - np.array([float(wr.get('walked_m') or 0), 0])
    else:
        cmd = round(min(0.20, max(0.07, float(tp[0] - (X_OK - 0.03)) * 0.8)), 3)
        wr = ctx.value('r1_walk', forward_m=cmd)
        pred = tp - np.array([float(wr.get('walked_m') or cmd), 0.0])
    log(step='move', cmd=cmd, walked=wr.get('walked_m'), turned=wr.get('turn_deg'), pelvis=wr.get('pelvis'), tilt=wr.get('tilt_deg'))
    o = perceive('wi2_%d' % k)
    T = target(o, pred)
    if T is None:
        log(step='STOP', why='target not re-found near %s' % pred.round(3).tolist(), paws=[p['c'] for p in o['paws']]); sys.exit(2)
    tp = np.array(T['c'][:2])
log(step='STOP', why='max moves'); sys.exit(2)
