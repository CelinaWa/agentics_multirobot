"""Check the R1 side step in the office sim: sends walk requests straight to the r1 stack port.
  ~/envs/rax-g1/bin/python r1_side_check.py '[[left_m, forward_m], ...]'   (after team_sim.sh start)"""
import json, sys, time
import numpy as np, zmq
ctx = zmq.Context.instance()
def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q
sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.RCVTIMEO, 60000); sock.connect('tcp://127.0.0.1:5791')
def op(**req):
    sock.send(json.dumps(req).encode()); return json.loads(sock.recv_multipart()[0])
time.sleep(3)
for left, forward in json.loads(sys.argv[1]):
    a = qpos()[57:60].copy()
    out = op(op='walk', forward_m=forward, turn_deg=0.0, left_m=left)
    b = qpos()[57:60]
    print(f"left {left:+.2f} fwd {forward:+.2f} -> reply lateral {out.get('lateral_m')} walked {out.get('walked_m')} turned {out.get('turned_deg')} "
          f"passes {out.get('passes')} tilt {out.get('tilt_deg')} fallen {out.get('fallen')} | world dx {b[0]-a[0]:+.3f} dy {b[1]-a[1]:+.3f} z {b[2]:.3f}", flush=True)
    if out.get('fallen') or not out.get('ok', True):
        print(out); break
