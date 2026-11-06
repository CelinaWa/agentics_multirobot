"""Straight walk-in from the reset start to the grasp stance (session 90b103 R1 lessons, sim).
No turns (forward walks drift the robot right: the paw moves ~+3.5 cm in y per walk, which brings the reset layout's
paw y -.25 to ~-.20 in 2 walks). Re-SELECT the clean overhanging paw after every walk (no tracking: R1's tracker jumped
to a non-overhanging paw once the real one dropped below the view). Stop at x <= X_OK (paw still visible at x >= .33).
    python code/puppy_box/walkin3.py [x_ok .38] [x_goal .35] [max_moves 6]
Exit 0 at the stance, 2 STOP."""
import sys
sys.path.insert(0, '.'); sys.path.insert(0, 'code/puppy_box')
import chain
from chain import ctx, log, perceive, select
X_OK = float(sys.argv[1]) if len(sys.argv) > 1 else 0.38
X_GOAL = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35
MAXM = int(sys.argv[3]) if len(sys.argv) > 3 else 6
ctx.value('r1_head', pitch=0.6, secs=0.8)
backed = False
for k in range(MAXM + 1):
    o = perceive('wi3_%d' % k); T, R = select(o)
    if T is None:
        if backed:
            log(step='STOP', why='no clean overhanging paw (after a step back)'); sys.exit(2)
        wr = ctx.value('r1_walk', forward_m=-0.05); backed = True
        log(step='no_target_step_back', walked=wr.get('walked_m')); continue
    x, y = T['c'][0], T['c'][1]
    log(step='target', k=k, paw=[round(x, 3), round(y, 3)], ovh=T.get('overhang'), edge=o['edge'])
    if x <= X_OK:
        ok = -0.25 <= y <= -0.15 and abs((o['edge'] or {}).get('yaw', 0)) < 8
        log(step='at_stance' if ok else 'STOP', paw=[round(x, 3), round(y, 3)], edge=o['edge'])
        sys.exit(0 if ok else 2)
    cmd = round(min(0.15, max(0.07, (x - X_GOAL) * 0.9)), 3)
    wr = ctx.value('r1_walk', forward_m=cmd)
    log(step='walk', cmd=cmd, walked=wr.get('walked_m'), pelvis=wr.get('pelvis'), tilt=wr.get('tilt_deg'))
log(step='STOP', why='max moves'); sys.exit(2)
