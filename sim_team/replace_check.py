"""Check that the sim operator can carry both standing robots back to their starting places without a fall.
  PYTHONPATH=../rax:../rax/scripts ~/envs/rax-g1/bin/python replace_check.py   (after team_sim.sh start)"""
import json, time, types
from pathlib import Path
import numpy as np, zmq
from services import arming
from services.team.backend import G1R1Backend
import g1r1_sim_operator as OP
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
layout = json.loads((HERE / 'scene' / 'layout.json').read_text())
root = HERE / 'session_root'; (root / 'operator').mkdir(parents=True, exist_ok=True); (root / 'agent-workspace').mkdir(exist_ok=True)
keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(0.6)
arming.write(root, True, minutes=30, by='replace_check')
for key in ('band=-0.10', 'band=-0.20'):
    keys.send_string(key); time.sleep(1.0)
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options={
    'g1': {'zmq_host': '127.0.0.1', 'pose_port': 5796, 'state_port': 5797, 'camera': None}, 'r1': {'stack_port': 5791}})
be = G1R1Backend(config)
def call(name, **a):
    be.validate(name, a); return be.call(name, a)[0]
try:
    call('g1_hold'); time.sleep(8); keys.send_string('release'); time.sleep(6)
    out = call('r1_walk', forward_m=-0.25, turn_deg=0.0); log('r1 walked back', out.get('walked_m'))
    out = call('r1_walk', forward_m=0.0, turn_deg=40.0); log('r1 turned', out.get('turned_deg'), 'walked', out.get('walked_m'))
    q = qpos(); log('before: G1 xy', np.round(q[0:2], 3), 'R1 xy', np.round(q[57:59], 3), 'R1 quat', np.round(q[60:64], 2))
    log('standing check:', OP.standing(ctx))
    OP.replace_robots(ctx, keys, layout); time.sleep(5)
    q = qpos(); r = call('r1_status'); g = call('g1_status')
    log('after:  G1 xy', np.round(q[0:2], 3), 'z %.3f tilt %.1f' % (q[2], g['tilt_deg']), '| R1 xy', np.round(q[57:59], 3), 'z %.3f tilt %.1f fallen %s' % (q[59], r['tilt_deg'], r.get('fallen')))
    out = call('r1_move_joints', joints={'right_elbow_joint': 0.6}, secs=1.5); log('r1 moves after the re-place: tilt', out.get('tilt_deg'), 'steps', out.get('steps'))
    time.sleep(4); q = qpos(); log('5 s later: R1 xy', np.round(q[57:59], 3), 'G1 z %.3f R1 z %.3f' % (q[2], q[59]))
finally:
    be.close()
