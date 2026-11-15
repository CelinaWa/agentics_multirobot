"""Check g1_hands_walk through the tool path: pose the arms with g1_step (palms facing, in front of the chest), enter
the teleop hold, squeeze a box the check sets between the palms (press_m), then walk back, left, right, forward and
turn with it. PASS: the box stays within 0.10 m of its height in the hands and the robot stays up.
  CHECK_DIR=<dir> PYTHONPATH=../rax ~/envs/rax-g1/bin/python hands_walk_check.py"""
import json, math, os, subprocess, sys, threading, time, types
from pathlib import Path
import numpy as np, zmq
from services import arming
from services.team.backend import G1R1Backend
from services.g1sonic import robot as R

HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000)
    sub.connect('tcp://127.0.0.1:5792'); q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
def yaw(qw):
    w, x, y, z = qw; return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

OUT = Path(os.environ['CHECK_DIR'])
root = OUT / f'walk_commanded_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
oplog = OUT / f'{root.name}_operator.log'
opts = json.loads((HERE.parent / 'rax/scripts/g1r1_sim_open_lowlevel_options.json').read_text())
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options=opts)
op = subprocess.Popen([sys.executable, str(HERE.parent / 'rax/scripts/g1r1_sim_operator.py'), '--session', str(root),
                       '--minutes', '30', '--max-resets', '5'], stdout=open(oplog, 'w'), stderr=subprocess.STDOUT,
                      env={**os.environ, 'PYTHONPATH': str(HERE.parent / 'rax')})
for _ in range(120):
    time.sleep(1)
    if 'band released' in oplog.read_text():
        break
time.sleep(3)
be = G1R1Backend(config); g1 = be.robots['g1'].backend
rows, ok = [], False
def fb(result):
    value = result[0] if isinstance(result, tuple) else result
    return (value.get('transition') or {}).get('feedback') or value.get('feedback') or {}
try:
    g1.call('g1_hold', {}); time.sleep(3)
    # the arm pose with g1_step: shoulders forward, elbows bent, arms spread (palms face each other)
    base = np.asarray(g1.last_row, dtype=float).copy()
    goal = base.copy()
    for side, sign in (('left', 1), ('right', -1)):
        for joint, value in ((f'{side}_shoulder_pitch_joint', -0.45), (f'{side}_elbow_joint', 0.9),
                             (f'{side}_shoulder_roll_joint', sign * 0.40)):
            goal[9 + R.JOINT_NAMES.index(joint)] = value
    for a0, a1 in ((0.0, 0.5), (0.5, 1.0)):     # two calls, each ending in a pose to hold
        end = base * (1 - a1) + goal * a1
        g1.call('g1_step', {'actions': [(base * (1 - a) + goal * a).tolist() for a in np.linspace(a0, a1, 80)] + [end.tolist()] * 8})
    time.sleep(1.0)
    hp = g1.hand_points(g1.state.latest().q)
    left, right = hp['left'][0].tolist(), hp['right'][0].tolist()
    mid = (np.asarray(left) + np.asarray(right)) / 2
    op.terminate(); time.sleep(1.5)                            # the band is released; this check places the box now
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    placing = True
    def place_loop():
        while placing:
            q = qpos(); yw = yaw(q[3:7]); c, sn = math.cos(yw), math.sin(yw)
            bx, by = q[0] + c * mid[0] - sn * mid[1], q[1] + sn * mid[0] + c * mid[1]
            keys.send_string(f'place=box,{bx},{by},{q[2] + mid[2] - 0.13},{math.cos(yw / 2)},0,0,{math.sin(yw / 2)}')
            time.sleep(0.1)
    th = threading.Thread(target=place_loop, daemon=True); th.start(); time.sleep(0.5)
    if os.environ.get('SWITCH') == '1':
        # as the agent does: squeeze with g1_step (shoulder rolls inward past contact), box free, THEN enter the hold
        sq = goal.copy()
        for side, sign in (('left', 1), ('right', -1)):
            sq[9 + R.JOINT_NAMES.index(f'{side}_shoulder_roll_joint')] = sign * float(os.environ.get('ROLL_SQ', '-0.10'))
        g1.call('g1_step', {'actions': [(goal * (1 - a) + sq * a).tolist() for a in np.linspace(0, 1, 90)] + [sq.tolist()] * 8})
        placing = False; th.join(); time.sleep(3)
        q = qpos(); log('g1_step squeeze, box free 3 s: box z minus pelvis', round(float(q[52] - q[2]), 3))
        f = {'hand_target_error_m': None, 'hand_separation_m': None}
    else:
        f = fb(g1.call('g1_hands_walk', {}))                   # enter the hold at the measured hand points
        contact = 0.205
        f = fb(g1.call('g1_hands_walk', {'left_hand': [left[0], contact, left[2]], 'right_hand': [right[0], -contact, right[2]],
                                         'press_m': 0.12}))
        placing = False; th.join()
    log('squeezed:', {k: f[k] for k in ('hand_target_error_m', 'hand_separation_m')})
    time.sleep(4)
    def box():
        q = qpos(); return round(float(q[52] - q[2]), 3), round(float(q[52]), 3)
    z0 = box()[0]; log('box held standing: base z minus pelvis', z0)
    for label, args in (('backward 0.5', {'forward_m': -0.5}), ('left 0.5', {'left_m': 0.5}), ('right 0.5', {'left_m': -0.5}),
                        ('forward 0.8', {'forward_m': 0.8}), ('turn 40', {'turn_deg': 40}), ('turn -40', {'turn_deg': -40}),
                        ('stand 5 s', {})):
        qa = qpos()
        if not args:
            time.sleep(5); f = {}
        else:
            f = fb(g1.call('g1_walk', {**args, 'carry': True, 'carry_commanded': True}))
        qb = qpos(); y0 = yaw(qa[3:7]); dx, dy = qb[:2] - qa[:2]
        row = {'move': label, 'truth_forward_left_m': [round(float(math.cos(y0) * dx + math.sin(y0) * dy), 3),
                                                       round(float(-math.sin(y0) * dx + math.cos(y0) * dy), 3)],
               'truth_turn_deg': round(math.degrees(yaw(qb[3:7]) - y0), 1), 'odometry': f.get('odometry_m', {}).get('forward'),
               'odometry_left': f.get('odometry_m', {}).get('left'), 'measured_turn': f.get('measured_turn_deg'),
               'max_tilt': f.get('max_tilt_deg'), 'box_z_minus_pelvis': box()[0], 'box_z': box()[1]}
        rows.append(row); log(row)
    ok = all(abs(r['box_z_minus_pelvis'] - z0) < 0.10 for r in rows) and all((r['max_tilt'] or 0) < 20 for r in rows)
    # let go: open the hands, then back to the streamed hold
    log('before the hold: box z', box()[1])
    g1.call('g1_hold', {}); time.sleep(3)
    s = g1.state.latest(); log('after g1_hold: control', g1.control_mode, 'tilt', round(s.tilt_deg(), 1))
    ok = ok and g1.control_mode == 'streamed' and s.tilt_deg() < 10
finally:
    try:
        be.close()
    finally:
        op.terminate()
(OUT / f'{root.name}.json').write_text(json.dumps(rows, indent=1))
log('RESULT', 'PASS' if ok else 'FAIL')
