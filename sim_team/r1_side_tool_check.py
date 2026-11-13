"""End-to-end check of r1_walk(left_m) through the team backend in the office sim.
  PYTHONPATH=../rax:../rax/scripts ~/envs/rax-g1/bin/python r1_side_tool_check.py   (after team_sim.sh start)"""
import time, types
from pathlib import Path
import zmq
from services import arming
from services.team.backend import G1R1Backend
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
root = HERE / 'session_root'; (root / 'operator').mkdir(parents=True, exist_ok=True); (root / 'agent-workspace').mkdir(exist_ok=True)
keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(0.6)
arming.write(root, True, minutes=30, by='r1_side_tool_check')
for key in ('band=-0.10', 'band=-0.20'):
    keys.send_string(key); time.sleep(1.0)
config = types.SimpleNamespace(root=root, task='t', observation_profile='rgbd', backend_options={
    'g1': {'zmq_host': '127.0.0.1', 'pose_port': 5796, 'state_port': 5797, 'camera': None}, 'r1': {'stack_port': 5791}})
be = G1R1Backend(config)
def call(name, **a):
    be.validate(name, a); return be.call(name, a)[0]
try:
    call('g1_hold'); time.sleep(8); keys.send_string('release'); time.sleep(6)
    for left in (-0.1, -0.1, 0.1, 0.1, -0.2, 0.2):
        out = call('r1_walk', left_m=left)
        print(time.strftime('%H:%M:%S'), 'r1_walk left_m', left, '->', {k: out.get(k) for k in ('lateral_m', 'walked_m', 'turned_deg', 'tilt_deg', 'fallen')}, flush=True)
    print('team frame r1:', call('team_status').get('frame', {}).get('r1'))
finally:
    be.close()
