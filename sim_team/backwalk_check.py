"""Check that the G1 walks backward (walk_any_direction) in the office sim.
  PYTHONPATH=../rax:../rax/scripts ~/envs/rax-g1/bin/python backwalk_check.py   (after team_sim.sh start)"""
import time, types
from pathlib import Path
import numpy as np, zmq
from services import arming
from services.team.backend import G1R1Backend
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
root = HERE / 'session_root'; (root / 'operator').mkdir(parents=True, exist_ok=True); (root / 'agent-workspace').mkdir(exist_ok=True)
keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(0.6)
arming.write(root, True, minutes=30, by='backwalk_check')
for key in ('band=-0.10', 'band=-0.20'):
    keys.send_string(key); time.sleep(1.0)
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options={
    'g1': {'zmq_host': '127.0.0.1', 'pose_port': 5796, 'state_port': 5797, 'camera': None, 'walk_any_direction': True, 'runaway_m': 0.4},
    'r1': {'stack_port': 5791}})
be = G1R1Backend(config)
def call(name, **a):
    be.validate(name, a); return be.call(name, a)[0]
try:
    call('g1_hold'); time.sleep(8); keys.send_string('release'); time.sleep(6)
    for arguments in ({'forward_m': -0.5, 'speed_mps': 0.45}, {'left_m': 0.6, 'speed_mps': 0.45}, {'left_m': -0.6, 'speed_mps': 0.45}, {'left_m': 0.8, 'speed_mps': 0.4}, {'left_m': -0.8, 'speed_mps': 0.4}):
        a = qpos()[:3].copy(); out = call('g1_walk', **arguments); time.sleep(2); b = qpos()[:3]
        fb = (out.get('transition') or {}).get('feedback') or {}
        log(arguments, '-> moved x %+.3f y %+.3f, pelvis z %.3f, tilt %.1f' % (b[0] - a[0], b[1] - a[1], b[2], call('g1_status')['tilt_deg']), '| warm-up' if fb.get('warmup') else '')
finally:
    be.close()
