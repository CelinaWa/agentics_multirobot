"""Calibrate the R1 gait: distance against the stride scale (hip) and the reference speed. Talks to the stack directly."""
import json, os, subprocess, sys, time, types
from pathlib import Path
import numpy as np, zmq
from services.team.backend import G1R1Backend
HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
OUT = Path(os.environ['CHECK_DIR'])
root = OUT / f'r1_gait_calib_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
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
    op.terminate(); time.sleep(1.5)
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    keys.send_string('place=box,-1.6,-3.2,0.15,1,0,0,0'); time.sleep(0.5)
    keys.send_string('place=low_table,-1.6,-2.6,0.0,1,0,0,0'); time.sleep(1.5)
    forward = True
    for hip, speed in ((0.9, 0.40), (0.8, 0.36), (0.7, 0.31), (0.6, 0.27), (0.5, 0.22), (0.7, 0.15), (0.6, 0.10), (0.5, 0.10), (0.8, 0.20)):
        r1.stack.request('gait', forward=forward, speed_mps=speed, seconds=2.3, hip=hip)
        time.sleep(2.0)
        out = r1.stack.gait_settle()
        print(f"hip {hip} ref {speed} m/s {'fwd' if forward else 'back'}: walked {out['walked_m']} in 2 s -> {abs(out['walked_m']) / 2:.2f} m/s, lateral {out['lateral_m']}, tilt {out['tilt_deg']}", flush=True)
        forward = not forward
finally:
    be.close(); op.terminate()
