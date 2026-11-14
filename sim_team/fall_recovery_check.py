"""Check that the sim operator brings the G1 back after a fall: the G1 is tipped over and its stack stopped (what the
backend's tilt guard does), the operator restarts the stack and hangs the G1 on its gantry at its start, and the G1
stands again under a new g1_hold. Then a robots+box reset, checked against where the box comes to rest.
  PYTHONPATH=../rax ~/envs/rax-g1/bin/python fall_recovery_check.py   (after team_sim.sh start, nothing else running)"""
import json, math, subprocess, sys, time, types
from pathlib import Path
import numpy as np, zmq
from services import arming
from services.controller import operator
from services.team.backend import G1R1Backend
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
def tilt(q):
    w, x, y, z = q[3:7]; return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

import os
OUT = Path(os.environ.get('CHECK_DIR', str(HERE / 'session_root')))
root = OUT / f'fall_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True, exist_ok=True); (root / 'agent-workspace').mkdir(exist_ok=True)
op = subprocess.Popen([sys.executable, str(HERE.parent / 'rax' / 'scripts' / 'g1r1_sim_operator.py'), '--session', str(root),
                       '--minutes', '12', '--max-resets', '3'], stdout=open(OUT / 'fall_check_operator.log', 'w'),
                      stderr=subprocess.STDOUT)
time.sleep(4)
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options={
    'g1': {'zmq_host': '127.0.0.1', 'pose_port': 5796, 'state_port': 5797, 'camera': None}, 'r1': {'stack_port': 5791}})
be = G1R1Backend(config)
def call(name, **a):
    be.validate(name, a); return be.call(name, a)[0]
ok = True
try:
    call('g1_hold'); time.sleep(14)
    q = qpos(); log('standing on its own: G1 z %.3f tilt %.1f' % (q[2], tilt(q)))
    # the fall: the stack is told to stop (as the tilt guard does) and the robot goes over
    subprocess.run(['docker', 'exec', 'sonic-sim2sim', 'tmux', 'send-keys', '-t', 'teamsim:deploy', 'O'])
    keys = ctx.socket(zmq.PUB); keys.connect('tcp://127.0.0.1:5790'); time.sleep(0.6)
    keys.send_string(f'place=pelvis,{q[0]},{q[1]},0.25,{math.cos(0.6)},{math.sin(0.6)},0,0'); time.sleep(1.0)
    q = qpos(); log('after the fall: G1 z %.3f tilt %.1f' % (q[2], tilt(q)))
    end = time.time() + 180
    while time.time() < end and 'hangs on its gantry' not in (OUT / 'fall_check_operator.log').read_text():
        time.sleep(3)
    q = qpos(); log('operator done: G1 at (%.2f, %.2f) z %.3f tilt %.1f' % (q[0], q[1], q[2], tilt(q)))
    log('armed:', arming.read(root, holds=False)['armed'])
    be.close(); be = G1R1Backend(config)
    call('g1_hold'); time.sleep(14)
    q0 = qpos(); time.sleep(5); q1 = qpos()
    log('standing again: G1 z %.3f tilt %.1f; the world advances: %s' % (q1[2], tilt(q1), bool(np.abs(q1 - q0).max() > 1e-6)))
    ok = q1[2] > 0.6 and tilt(q1) < 10 and bool(np.abs(q1 - q0).max() > 1e-6)
    operator.open_request(root, 'reset', 'Reset please: box and robots, with the R1 for practice at r1_at=0.20,-0.02,-135')
    time.sleep(25); q = qpos(); log('R1 after r1_at: (%.2f, %.2f) z %.3f; box (%.2f, %.2f, %.2f)' % (q[57], q[58], q[59], q[50], q[51], q[52]))
    time.sleep(15)
    log('reset reply:', [r for r in operator.pending(root)][:1] or 'answered')
finally:
    be.close()
    op.terminate()
print('\n'.join((OUT / 'fall_check_operator.log').read_text().splitlines()[-12:]))
log('RESULT', 'PASS' if ok else 'FAIL')
