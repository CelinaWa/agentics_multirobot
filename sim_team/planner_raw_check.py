"""Check the raw g1_planner tool: stand, hand points held (teleop encoder), a slow walk forward and back with them,
a turn by facing, then g1_hold hands back. PASS: it walks both ways, the hands stay within 0.08 m of their targets,
and the streamed hold takes over.
  CHECK_DIR=<dir> PYTHONPATH=../rax ~/envs/rax-g1/bin/python planner_raw_check.py"""
import json, math, os, subprocess, sys, time, types
from pathlib import Path
import numpy as np, zmq
from services.team.backend import G1R1Backend

HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000)
    sub.connect('tcp://127.0.0.1:5792'); q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
def yaw(qw):
    w, x, y, z = qw; return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

OUT = Path(os.environ['CHECK_DIR'])
root = OUT / f'planner_raw_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
oplog = OUT / f'{root.name}_operator.log'
opts = json.loads((HERE.parent / 'rax/scripts/g1r1_sim_raw_options.json').read_text())
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
ok, rows = False, []
def fb(result):
    value = result[0] if isinstance(result, tuple) else result
    return (value.get('transition') or {}).get('feedback') or {}
try:
    names = [t['name'] for t in g1.tools()]
    log('G1 tools offered:', names)
    g1.call('g1_hold', {}); time.sleep(3)
    f = fb(g1.call('g1_planner', {'seconds': 1.0}))                      # plain messages first (clears the stack's flag)
    start = f['hand_points_pelvis']
    target = {'left': [0.40, 0.20, 0.10], 'right': [0.40, -0.20, 0.10]}
    f = fb(g1.call('g1_planner', {'seconds': 4.0, 'hand_points': {k: start[k] for k in ('left', 'right')}, 'hand_points_end': target}))
    def step(label, **args):
        qa = qpos(); f = fb(g1.call('g1_planner', {'hand_points': target, **args})); qb = qpos()
        y0 = yaw(qa[3:7]); dx, dy = qb[:2] - qa[:2]
        err = max(float(np.linalg.norm(np.asarray(f['hand_points_pelvis'][k]) - np.asarray(target[k]))) for k in ('left', 'right'))
        row = {'move': label, 'truth_forward_left_m': [round(float(math.cos(y0) * dx + math.sin(y0) * dy), 3),
                                                       round(float(-math.sin(y0) * dx + math.cos(y0) * dy), 3)],
               'truth_turn_deg': round(math.degrees(yaw(qb[3:7]) - y0), 1), 'odometry': f['odometry_m'],
               'heading_in_planner_frame_deg': f['heading_in_planner_frame_deg'], 'max_tilt': f['max_tilt_deg'], 'hand_err_m': round(err, 3)}
        rows.append(row); log(row); return row
    step('hold 3 s', seconds=3.0)
    fwd = step('slow walk forward 5 s at 0.3', seconds=5.0, mode='slow_walk', movement=[1, 0], speed_mps=0.3)
    step('idle 3 s', seconds=3.0)
    back = step('slow walk backward 5 s at 0.3', seconds=5.0, mode='slow_walk', movement=[-1, 0], speed_mps=0.3)
    step('idle 3 s ', seconds=3.0)
    turn = step('face +30 deg, 5 s', seconds=5.0, facing_deg=30.0)
    g1.call('g1_hold', {}); time.sleep(3)
    s = g1.state.latest(); log('after g1_hold: control', g1.control_mode, 'tilt', round(s.tilt_deg(), 1))
    ok = ('g1_walk' not in names and 'g1_hands_walk' not in names and fwd['truth_forward_left_m'][0] > 0.3
          and back['truth_forward_left_m'][0] < -0.3 and turn['truth_turn_deg'] > 15
          and all(r['hand_err_m'] < 0.08 and r['max_tilt'] < 20 for r in rows) and g1.control_mode == 'streamed')
finally:
    try:
        be.close()
    finally:
        op.terminate()
log('RESULT', 'PASS' if ok else 'FAIL')
