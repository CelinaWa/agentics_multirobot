"""Ground-truth check of the office_box success condition from the running simulation's qpos stream.
  ~/envs/rax-g1/bin/python success_check.py"""
import json, math
from pathlib import Path
import numpy as np, zmq
HERE = Path(__file__).resolve().parent
layout = json.loads((HERE / 'scene' / 'layout.json').read_text())
sub = zmq.Context.instance().socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 5000); sub.connect('tcp://127.0.0.1:5792')
q = np.frombuffer(sub.recv(), dtype=np.float64)
def tilt(w, x, y, z): return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
box, bq = q[50:53], q[53:57]
goal = [v for k, v in layout.items() if 'goal' in k]
print('goal entries:', {k: v for k, v in layout.items() if 'goal' in k or 'bin' in k})
print('box xyz', np.round(box, 3), 'tilt %.1f deg' % tilt(*bq))
print('G1 pelvis z %.3f tilt %.1f | R1 pelvis z %.3f tilt %.1f' % (q[2], tilt(*q[3:7]), q[59], tilt(*q[60:64])))
