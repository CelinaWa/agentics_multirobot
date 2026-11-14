"""Reproduce S28-1/2/4: the G1 has stepped back and the box lies on the floor where the G1's start is; then the
operator resets the box and the robots. Passes when no guard trips and the G1 stands at its start afterwards.
The operator binds the sim's key port, so the scene is set up first (this script binds it), then the operator starts.
  OPERATOR=<operator script> CHECK_DIR=<dir> PYTHONPATH=../rax ~/envs/rax-g1/bin/python reset_box_at_feet_check.py"""
import json, math, os, subprocess, sys, time, types
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
OUT = Path(os.environ['CHECK_DIR']); script = os.environ['OPERATOR']
root = OUT / f'reset_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
oplog = OUT / f'{root.name}_operator.log'
opts = json.loads((HERE.parent / 'rax/scripts/g1r1_sim_open_lowlevel_options.json').read_text())
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options=opts)
arming.write(root, True, minutes=10, by='check')
be = G1R1Backend(config); g1 = be.robots['g1'].backend
op, ok = None, False
try:
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    g1.call('g1_hold', {}); time.sleep(10)
    q = qpos(); log('G1 at (%.2f, %.2f) tilt %.1f' % (q[0], q[1], tilt(q)))
    # the G1 stepped back (as after a carry): carried 0.28 m back the way the operator carries it
    keys.send_string('place=pelvis,-0.85,-0.35,%.4f' % q[2]); time.sleep(4); g1.call('g1_hold', {}); time.sleep(4)
    q = qpos(); log('G1 stepped back to (%.2f, %.2f) tilt %.1f' % (q[0], q[1], tilt(q)))
    keys.send_string('place=box,-0.45,-0.35,0.005,1,0,0,0'); time.sleep(3)
    q = qpos(); log('box on the floor at (%.2f, %.2f, %.2f); G1 tilt %.1f' % (q[50], q[51], q[52], tilt(q)))
    keys.close(linger=0); time.sleep(0.5)
    op = subprocess.Popen([sys.executable, script, '--session', str(root), '--minutes', '10', '--max-resets', '5'],
                          stdout=open(oplog, 'w'), stderr=subprocess.STDOUT, env={**os.environ, 'PYTHONPATH': str(HERE.parent / 'rax')})
    time.sleep(6)
    t_req = time.time()
    reply = operator.wait(root, operator.open_request(root, 'reset', 'Reset please: the box and both robots back at their starts.'), 120).get('reply')
    time.sleep(10)
    q = qpos(); raw = json.loads(arming.arm_file(root).read_text())
    log('after: G1 at (%.2f, %.2f) z %.3f tilt %.1f; box (%.2f, %.2f, %.2f); arming by %s armed %s reason %s'
        % (q[0], q[1], q[2], tilt(q), q[50], q[51], q[52], raw.get('by'), raw.get('armed'), raw.get('reason')))
    log('reply:', reply)
    tripped = 'tripped' in oplog.read_text() or (raw.get('armed') is not True)
    ok = q[2] > 0.6 and tilt(q) < 10 and abs(q[0] + 0.57) < 0.05 and not tripped
finally:
    be.close()
    if op: op.terminate()
print('\n'.join(oplog.read_text().splitlines()[-6:]) if oplog.exists() else '')
log('RESULT', 'PASS' if ok else 'FAIL')
