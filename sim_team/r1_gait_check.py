"""Check r1_gait (the R1's gait by velocity): it answers state while walking, covers more ground at more speed, and
can change speed without stopping. The low table and the box are set aside.
  CHECK_DIR=<dir> PYTHONPATH=../rax ~/envs/rax-g1/bin/python r1_gait_check.py"""
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
OUT = Path(os.environ['CHECK_DIR'])
root = OUT / f'r1_gait_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
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
rows, ok = [], False
try:
    log('R1 tools:', [t['name'] for t in r1.tools() if 'gait' in t['name'] or 'walk' in t['name']])
    op.terminate(); time.sleep(1.5)
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    keys.send_string('place=box,-1.6,-3.2,0.15,1,0,0,0'); time.sleep(0.5)
    keys.send_string('place=low_table,-1.6,-2.6,0.0,1,0,0,0'); time.sleep(1.5)
    def run(label, calls):
        qa = qpos(); out = None; n = 0; t0 = time.time()
        for args in calls:
            out = r1.call('r1_gait', args)[0]; n += len(out['samples'])
        qb = qpos(); d = qb[57:59] - qa[57:59]
        row = {'move': label, 'truth_dx': round(float(d[0]), 3), 'truth_dy': round(float(d[1]), 3), 'walked_m': out.get('walked_m'),
               'lateral_m': out.get('lateral_m'), 'turned_deg': out.get('turned_deg'), 'tilt': out.get('tilt_deg'),
               'samples': n, 'secs': round(time.time() - t0, 1), 'hand_error_last': out['samples'][-1]['left']['hand_error_m']}
        rows.append(row); log(row); return row
    a = run('forward 2 s at 0.42', [{'seconds': 2.0, 'speed_mps': 0.42}])
    run('backward 2 s at 0.42', [{'seconds': 2.0, 'speed_mps': 0.42, 'backward': True}])
    b = run('forward 2 s at 0.30', [{'seconds': 2.0, 'speed_mps': 0.30}])
    run('backward 2 s at 0.30', [{'seconds': 2.0, 'speed_mps': 0.30, 'backward': True}])
    c = run('forward 2 s at 0.24', [{'seconds': 2.0, 'speed_mps': 0.24}])
    run('backward 2 s at 0.24', [{'seconds': 2.0, 'speed_mps': 0.24, 'backward': True}])
    e = run('forward 0.24/0.40/0.24 without stopping', [{'seconds': 1.0, 'speed_mps': 0.24, 'settle': False},
                                                        {'seconds': 1.0, 'speed_mps': 0.40, 'settle': False},
                                                        {'seconds': 1.0, 'speed_mps': 0.24}])
    run('backward 3 s at 0.30 turning 8 deg/s', [{'seconds': 3.0, 'speed_mps': 0.30, 'backward': True, 'turn_deg_s': 8.0}])
    s = r1.call('r1_status', {})[0]
    ok = (all(r['samples'] >= 8 for r in rows) and min(a['walked_m'], b['walked_m']) > c['walked_m'] > 0.3 and e['walked_m'] > 0.7
          and all((r['tilt'] or 0) < 15 for r in rows) and not s.get('fault'))
finally:
    be.close(); op.terminate()
(OUT / f'{root.name}.json').write_text(json.dumps(rows, indent=1))
log('RESULT', 'PASS' if ok else 'FAIL')
