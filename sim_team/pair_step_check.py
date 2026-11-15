"""Can the G1 and the R1 step together at a matched pace? No box, the low table set aside: both robots take STEPS
steps of STEP m in the same world direction at the same time (the G1 backward under g1_hands_walk, the R1 forward
with r1_walk; they face each other), and the ground truth gives each robot's travel per step and how the gap
between them changes. A shared box tolerated about 6 cm of mismatch and lost its squeeze at 13 cm (S36).
  CHECK_DIR=<dir> [STEP=0.15] [STEPS=6] [CATCHUP=1] PYTHONPATH=../rax ~/envs/rax-g1/bin/python pair_step_check.py"""
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

OUT = Path(os.environ['CHECK_DIR']); STEP = float(os.environ.get('STEP', '0.15')); STEPS = int(os.environ.get('STEPS', '6'))
CATCHUP = os.environ.get('CATCHUP') == '1'
root = OUT / f'pair_step_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
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
rows = []
try:
    g1.call('g1_hold', {}); time.sleep(3)
    op.terminate(); time.sleep(1.5)
    keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
    keys.send_string('place=box,-1.6,-3.2,0.15,1,0,0,0'); time.sleep(0.5)
    keys.send_string('place=low_table,-1.6,-2.6,0.0,1,0,0,0'); time.sleep(1.5)
    g1.call('g1_hands_walk', {})                  # enter the hold (hands where they are)
    def both(g_m, r_m):
        out = {}
        def run(name, backend, tool, args):
            t0 = time.time()
            try:
                backend.call(tool, args)
            except Exception as error:
                out[name + '_error'] = str(error)[:200]
            out[name + '_s'] = round(time.time() - t0, 1)
        threads = []
        if abs(g_m) > 1e-6:
            threads.append(threading.Thread(target=run, args=('g1', g1, 'g1_hands_walk', {'forward_m': g_m})))
        if abs(r_m) > 1e-6:
            threads.append(threading.Thread(target=run, args=('r1', r1, 'r1_walk', {'forward_m': r_m})))
        for t in threads: t.start()
        for t in threads: t.join()
        return out
    q0 = qpos(); g_start, r_start = q0[0], q0[57]
    owe_g = owe_r = 0.0
    for k in range(STEPS):
        qa = qpos()
        # both move toward -x: the G1 (facing +x) backward, the R1 (facing -x) forward
        out = both(-(float(os.environ.get('G_STEP', STEP)) + owe_g), float(os.environ.get('R_STEP', STEP)) + owe_r)
        time.sleep(0.5); qb = qpos()
        dg, dr = float(qb[0] - qa[0]), float(qb[57] - qa[57])
        gap = float((qb[57] - qb[0]) - (q0[57] - q0[0]))
        row = {'step': k + 1, 'g1_dx': round(dg, 3), 'r1_dx': round(dr, 3), 'mismatch_this_step': round(dr - dg, 3),
               'gap_change_total': round(gap, 3), 'g1_dy': round(float(qb[1] - qa[1]), 3), 'r1_dy': round(float(qb[58] - qa[58]), 3), **out}
        rows.append(row); log(row)
        if CATCHUP:      # the robot that is behind owes the gap on its next step (capped)
            owe_g = max(-0.08, min(0.08, -gap)) if gap < 0 else 0.0      # gap < 0: the G1 lags (it is ahead in +x)
            owe_r = max(-0.08, min(0.08, gap)) if gap > 0 else 0.0       # gap > 0: the R1 lags
    worst = max(abs(r['gap_change_total']) for r in rows)
    log('SUMMARY step', STEP, 'catch-up', CATCHUP, '| worst gap change', round(worst, 3), '| per-step mismatch',
        [r['mismatch_this_step'] for r in rows], '| total travel g1', round(float(qpos()[0] - g_start), 3), 'r1', round(float(qpos()[57] - r_start), 3))
finally:
    try:
        g1.call('g1_hold', {})
    except Exception as error:
        log('hold:', error)
    be.close(); op.terminate()
(OUT / f'{root.name}.json').write_text(json.dumps(rows, indent=1))
