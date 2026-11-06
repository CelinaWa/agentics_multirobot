"""Walk-in v5 (session 3d54de): fewest walk calls, never step back, stance gate.
Lessons: every r1_walk call drifts the robot 2-4 cm right (paw +y) and back steps overshoot 1.5-3x and drift 9-11 cm
(5600c1 walkin3 landed paw y -.14..-.16 -> lifts 1/4). Lift needs paw (my fit) x >= .32, y <= -.18 (5/6).
    python code/puppy_box/walkin5.py [x_goal .35] [x_ok .375] [max_walks 4]
Prints JSON lines; exit 0 = paw in the gate (x .32-.375, y -.26..-.17), 3 = landed outside the gate (report), 2 = STOP."""
import sys, json, time, math
sys.path.insert(0, '.'); sys.path.insert(0, 'skill-library/skills/r1-leg-axis-grasp-short-carry/code')
from api.runtime import Context
import chain
ctx = Context(); chain.ctx = ctx
chain.good = lambda p: p['n'] >= 300 and p['rms'] <= 0.001
X_GOAL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.35
X_OK = float(sys.argv[2]) if len(sys.argv) > 2 else 0.395   # R4: a .06 cmd from paw .388 walked .108 + 6 cm right -> paw (.286,-.161), lift failed
MAXW = int(sys.argv[3]) if len(sys.argv) > 3 else 4
K = 0.85            # walks .15-.2 cover ~.75-1.2 x the command; undershoot on purpose
CMD_MIN, CMD_MAX = 0.07, 0.20   # commands < .06 are unreliable (.04 walked backwards once)
GATE = dict(x=(0.31, 0.395), y=(-0.26, -0.17))
def P(**k): print(json.dumps(k), flush=True)
ctx.value('r1_head', pitch=0.6, secs=0.6)
last = None; turns = 0
Y_GOAL = -0.215
for k in range(MAXW + 3):
    o = chain.perceive('wi5_%d' % k); T, R = chain.select(o)
    if T is None:
        P(step='no_target', k=k, edge=o['edge'], last=last); break
    x, y = T['c'][0], T['c'][1]; last = [round(x, 3), round(y, 3)]
    P(step='target', k=k, paw=last, ovh=T.get('overhang'), n=T['n'], edge=o['edge'], tilt=o['tilt'])
    if x <= X_OK:
        break
    if k - turns >= MAXW:
        P(step='STOP', why='max walks'); sys.exit(2)
    # v5b (R2 drifted LEFT: paw y -.25 -> -.29 -> paw out of reach): correct y with a small turn while the paw is
    # still >= .40 ahead (turn right = paw moves left/+y by ~x*theta; small turn commands achieve ~0.6x)
    if x >= 0.40 and abs(y - Y_GOAL) > 0.045 and turns < 2:   # R3: .033 triggered an unneeded turn (edge yaw +4)
        th = math.degrees(-(Y_GOAL - y) / x) * 1.5
        th = round(max(-10.0, min(10.0, th)), 1)
        wr = ctx.value('r1_walk', forward_m=0.0, turn_deg=th); turns += 1
        P(step='turn', cmd=th, turned=wr.get('turn_deg'), walked=wr.get('walked_m'), pelvis=wr.get('pelvis'), tilt=wr.get('tilt_deg'))
        time.sleep(1.0)
        continue
    cmd = round(min(CMD_MAX, max(CMD_MIN, (x - X_GOAL) * K)), 3)
    wr = ctx.value('r1_walk', forward_m=cmd)
    P(step='walk', cmd=cmd, walked=wr.get('walked_m'), turned=wr.get('turned_deg'), pelvis=wr.get('pelvis'), tilt=wr.get('tilt_deg'))
    time.sleep(1.0)
if last is None:
    P(step='STOP', why='no paw at all'); sys.exit(2)
ok = GATE['x'][0] <= last[0] <= GATE['x'][1] and GATE['y'][0] <= last[1] <= GATE['y'][1]
P(step='stance', paw=last, in_gate=ok, gate=GATE)
sys.exit(0 if ok else 3)
