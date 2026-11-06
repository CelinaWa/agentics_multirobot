"""Bridge test against r1/deploy_sim/run.sh (start the bridge first: python -m services.r1sonic.bridge_real --port 5594).
End-to-end: RAX R1SonicBackend -> bridge_real -> r1_sim_cpu (real R1 deploy) -> lab R1 MuJoCo DDS sim."""
from pathlib import Path
import json, sys, tempfile, time, types, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'rax'))
from services.r1sonic.backend import R1SonicBackend
from services import arming
PORT = 5594
root = Path(tempfile.mkdtemp(prefix='r1_bridge_e2e_'))
(root / 'operator').mkdir(parents=True, exist_ok=True)
arming.write(root, True, minutes=30, by='bridge e2e test')
# head_pitch 0: this deploy build blocks head motion (tracking disabled), so do not command 0.5
be = R1SonicBackend(types.SimpleNamespace(backend_options={'stack_port': PORT, 'head_pitch': 0.0}, root=root, observation_profile='rgb'))
LOG = []
_orig = be.stack.stream
T0 = time.time(); MARK = []; TILT = []
def rec(rows):
    MARK.append((time.time() - T0, len(rows))); LOG.append([np.asarray(r) for r in rows]); return _orig(rows)
import threading, zmq as _zmq
def sampler():
    c = _zmq.Context.instance().socket(_zmq.REQ); c.setsockopt(_zmq.RCVTIMEO, 2000); c.connect(f'tcp://127.0.0.1:{PORT}')
    while not STOP.is_set():
        try:
            c.send(json.dumps({'op': 'state'}).encode()); r = json.loads(c.recv())
            TILT.append((time.time() - T0, r.get('tilt_deg'), r.get('gyro')))
        except Exception:
            break
        time.sleep(0.05)
STOP = threading.Event(); threading.Thread(target=sampler, daemon=True).start()
be.stack.stream = rec
def dump():
    for i, rows in enumerate(LOG):
        a, b = rows[0], rows[-1]
        print(f'  stream {i}: n={len(rows)} Rw {np.round(a[7:10],3)} -> {np.round(b[7:10],3)}  torso {np.round(a[14:17],3)} -> {np.round(b[14:17],3)}  Lw {np.round(b[0:3],3)}')
    print('  correction', {k: np.round(v, 3).tolist() for k, v in be.correction.items()})
    STOP.set()
    print('  marks', [(round(t, 2), n) for t, n in MARK])
    bad = next((t for t, a, g in TILT if a and a > 5), None)
    if bad:
        for t, a, g in TILT:
            if bad - 3.0 <= t <= bad + 0.5:
                print(f'   t={t:6.2f} tilt={a:5.1f} gyro={np.round(g, 2)}')
s = be._state()
print('state ok: tilt', s['tilt_deg'], 'joints', len(s['q']), 'R grasp point', np.round(be._grasp_points(s['qd'])['right'], 3))
try:
  for goal in ([0.30, -0.20, 0.00], [0.33, -0.22, -0.02], [0.28, -0.16, 0.05]):
    r = be._move_hand('right', goal, approach=[1, 0, 0], fingers=[0, 1, 0], secs=2.5)
    time.sleep(1.0)
    s = be._state(); h = np.asarray(be._grasp_points(s['qd'])['right'])
    print('goal', goal, 'reachable', r.get('reachable'), 'reported err', r.get('grasp_error_m'),
          'measured', np.round(h, 3), 'err cm', round(100 * float(np.linalg.norm(h - goal)), 1), 'tilt', s['tilt_deg'])
except Exception as e:
    print('MOVE FAILED', e)
finally:
    dump()
try:
    print('walk ->', be.call('r1_walk', {'forward_m': 0.07}))
except Exception as e:
    print('walk raised', type(e).__name__, e)
try:
    print('observe ->', str(be.call('r1_observe', {}))[:200])
except Exception as e:
    print('observe raised', type(e).__name__, str(e)[:200])
