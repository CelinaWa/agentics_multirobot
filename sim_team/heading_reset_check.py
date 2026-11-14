"""Check the operator's G1 heading reset: turn the standing G1 off its start heading (as attempts do), then reset
the robots; the G1 should end at its start heading and stay there (no turning back, no steps) under a new g1_hold.
  CHECK_DIR=<dir> YAW=8 PYTHONPATH=../rax ~/envs/rax-g1/bin/python heading_reset_check.py"""
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
def yaw(q):
    w, x, y, z = q[3:7]; return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
def tilt(q):
    w, x, y, z = q[3:7]; return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
def watch(label, secs):
    q0 = qpos(); ys = []
    for _ in range(int(secs)):
        time.sleep(1); q = qpos(); ys.append(yaw(q))
    log('%s: yaw %.1f -> %.1f (min %.1f max %.1f), moved %.3f m, tilt %.1f' % (label, yaw(q0), ys[-1], min(ys), max(ys),
        float(np.hypot(*(q[:2] - q0[:2]))), tilt(q)))
    return q
OUT = Path(os.environ['CHECK_DIR']); turn = float(os.environ.get('YAW', '8'))
root = OUT / f'heading_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
oplog = OUT / f'{root.name}_operator.log'
opts = json.loads((HERE.parent / 'rax/scripts/g1r1_sim_open_lowlevel_options.json').read_text())
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options=opts)
arming.write(root, True, minutes=10, by='check')
be = G1R1Backend(config); g1 = be.robots['g1'].backend
op, ok = None, False
try:
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    g1.call('g1_hold', {}); time.sleep(5)
    q = qpos(); h = math.radians(turn) / 2
    keys.send_string(f'place=pelvis,{q[0]},{q[1]},{q[2]},{math.cos(h)},0,0,{math.sin(h)}'); time.sleep(1)
    g1.call('g1_hold', {})
    watch(f'turned to {turn:g} deg, new g1_hold', 12)
    keys.close(linger=0); time.sleep(0.5)
    op = subprocess.Popen([sys.executable, str(HERE.parent / 'rax/scripts/g1r1_sim_operator.py'), '--session', str(root),
                           '--minutes', '10', '--max-resets', '5'], stdout=open(oplog, 'w'), stderr=subprocess.STDOUT,
                          env={**os.environ, 'PYTHONPATH': str(HERE.parent / 'rax')})
    time.sleep(6)
    reply = operator.wait(root, operator.open_request(root, 'reset', 'Reset please: the box and both robots back at their starts.'), 120).get('reply')
    log('reply:', reply[-420:])
    watch('after the reset, before any call (keepalive)', 8)
    log('backend events:', [e['what'] for e in g1.events if 're-anchored' in e['what']])
    g1.call('g1_hold', {})
    q = watch('after a new g1_hold', 15)
    raw = json.loads(arming.arm_file(root).read_text())
    log('arming', raw.get('armed'), raw.get('reason'))
    ok = abs(yaw(q)) < 1.5 and tilt(q) < 10 and abs(q[0] + 0.57) < 0.04 and raw.get('armed') is True
finally:
    be.close()
    if op: op.terminate()
log('RESULT', 'PASS' if ok else 'FAIL')
