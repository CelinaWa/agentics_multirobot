"""Do the two robots stay together when both walk continuously and the R1 follows by speed? No box, the low table
aside. The G1 walks backward under g1_hands_walk; the R1 walks forward with r1_gait in 0.5 s pieces, its speed set
from the gap to the G1 (ground truth here, standing in for the hand-point error a shared object would give).
  CHECK_DIR=<dir> [DIST=1.0] [GAIN=2.0] [FOLLOW=1] PYTHONPATH=../rax ~/envs/rax-g1/bin/python pair_follow_check.py"""
import json, math, os, subprocess, sys, threading, time, types
from pathlib import Path
import numpy as np, zmq
from services.team.backend import G1R1Backend
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000)
    sub.connect('tcp://127.0.0.1:5792'); q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
OUT = Path(os.environ['CHECK_DIR']); DIST = float(os.environ.get('DIST', '1.0')); GAIN = float(os.environ.get('GAIN', '2.0'))
FOLLOW = os.environ.get('FOLLOW', '1') == '1'
root = OUT / f'pair_follow_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
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
be = G1R1Backend(config); g1 = be.robots['g1'].backend; r1 = be.robots['r1'].backend
try:
    g1.call('g1_hold', {}); time.sleep(3)
    op.terminate(); time.sleep(1.5)
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    keys.send_string('place=box,-1.6,-3.2,0.15,1,0,0,0'); time.sleep(0.5)
    keys.send_string('place=low_table,-1.6,-2.6,0.0,1,0,0,0'); time.sleep(1.5)
    g1.call('g1_hands_walk', {})
    for leg in range(2):
        q0 = qpos(); gap0 = q0[57] - q0[0]; done = {}
        def g1_walk():
            try:
                done['g1'] = g1.call('g1_hands_walk', {'forward_m': -DIST, 'speed_mps': 0.3})
            except Exception as error:
                done['g1_error'] = str(error)[:200]
        th = threading.Thread(target=g1_walk); th.start()
        series, t0 = [], time.time()
        while th.is_alive() or (series and abs(series[-1][1]) > 0.04 and time.time() - t0 < 14):
            q = qpos(); gap = float((q[57] - q[0]) - gap0)        # > 0: the R1 is behind (both move toward -x)
            v = float(np.clip(0.30 + GAIN * gap, 0.22, 0.45)) if FOLLOW else 0.30
            if FOLLOW and gap < -0.03 and not th.is_alive():
                break                                                 # ahead of a standing G1: stop
            if gap < -0.06:                                           # well ahead: coast (let the gait ramp out) instead of stepping
                time.sleep(0.5)
            else:
                r1.call('r1_gait', {'seconds': 0.5, 'speed_mps': v, 'settle': False})
            series.append((round(time.time() - t0, 1), round(gap, 3), round(v, 2)))
        out = r1.call('r1_gait', {'seconds': 0.2, 'speed_mps': 0.22, 'settle': True})[0]
        time.sleep(1.0); q1 = qpos()
        gaps = [g for _, g, _ in series]
        log(f'leg {leg + 1}: G1 moved', round(float(q1[0] - q0[0]), 3), 'R1 moved', round(float(q1[57] - q0[57]), 3), '| gap during: min', min(gaps), 'max', max(gaps),
            '| final gap', round(float((q1[57] - q1[0]) - gap0), 3), '| lateral g1', round(float(q1[1] - q0[1]), 3), 'r1', round(float(q1[58] - q0[58]), 3), done.get('g1_error', ''))
        log('   series (t, gap, r1 speed):', series)
finally:
    try:
        g1.call('g1_hold', {})
    except Exception as error:
        log('hold:', error)
    be.close(); op.terminate()
