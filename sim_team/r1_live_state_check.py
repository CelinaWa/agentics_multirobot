"""Does the R1 service answer r1_status while an r1_walk call is running, and does the status carry its base pose?
  CHECK_DIR=<dir> PYTHONPATH=../rax ~/envs/rax-g1/bin/python r1_live_state_check.py"""
import json, os, subprocess, sys, threading, time, types
from pathlib import Path
import numpy as np, zmq
from services.team.backend import G1R1Backend
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000)
    sub.connect('tcp://127.0.0.1:5792'); q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
OUT = Path(os.environ['CHECK_DIR'])
root = OUT / f'r1_live_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
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
be = G1R1Backend(config); r1 = be.robots['r1'].backend
try:
    s = r1.call('r1_status', {}); s = s[0] if isinstance(s, tuple) else s
    print('status keys:', sorted(s)[:40]); print(json.dumps(s)[:900])
    done = []
    t = threading.Thread(target=lambda: done.append(r1.call('r1_walk', {'forward_m': -0.25}))); t0 = time.time(); t.start()
    samples = []
    while t.is_alive():
        a = time.time()
        try:
            s = r1.call('r1_status', {}); s = s[0] if isinstance(s, tuple) else s
            samples.append((round(a - t0, 2), round(time.time() - a, 3), round(float(qpos()[57]), 3)))
        except Exception as error:
            samples.append((round(a - t0, 2), 'ERR ' + str(error)[:80]))
        time.sleep(0.1)
    t.join(); print('walk took', round(time.time() - t0, 1), 's; status calls during it:', len(samples)); print(samples[:25])
finally:
    be.close(); op.terminate()
