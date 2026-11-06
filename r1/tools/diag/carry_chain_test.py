"""Lift/carry reliability trials through the RAX backend against a private sim.
argv: PORT TRIALS DEPTH_M ARC(0/1) WALKTEST(0/1). Each trial: reset scene+robot, stance, overhang, pivot pinch
(grasp site DEPTH_M up the shaft from the paw ball), lift (straight or arc about the hip), optional walks."""
import json, os, sys, threading, time, types
from pathlib import Path
import numpy as np, zmq
from scipy.spatial.transform import Rotation
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax')
from services.r1sonic.backend import R1SonicBackend
from services import arming

PORT = int(sys.argv[1]); TRIALS = int(sys.argv[2]); DEPTH = float(sys.argv[3]); ARC = int(sys.argv[4]); WALKTEST = int(sys.argv[5])
ctx = zmq.Context.instance()
def req(op, timeout=130000, **f):
    s = ctx.socket(zmq.REQ); s.setsockopt(zmq.LINGER, 0); s.setsockopt(zmq.RCVTIMEO, timeout); s.connect(f'tcp://127.0.0.1:{PORT}')
    s.send(json.dumps({'op': op, **f}).encode()); r = json.loads(s.recv_multipart()[0]); s.close(); return r
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
MATE = {0: 1, 1: 0, 2: 3, 3: 2}
def place_overhang(cm=8.0):
    g = req('gt'); edge_x = g['table']['pos'][0] - 0.300; front = min(p[0] for p in g['dog_paws'])
    dx = max(-0.25, min(0.0, (edge_x - cm / 100) - front)); d = g['dog']
    req('place', body='dog', pos=[d['pos'][0] + dx, d['pos'][1], d['pos'][2]], quat=d['quat']); time.sleep(1.5); return edge_x
def separate_pair():
    p = req('gt')['dog_paws']; front = min(range(4), key=lambda n: p[n][0]); mate = MATE[front]
    def arrange(sw, sp):
        req('dog_joint', name=f'dog_limb_{mate}_swing', value=sw); req('dog_joint', name=f'dog_limb_{mate}_splay', value=sp)
        time.sleep(1.5); place_overhang(); time.sleep(2.0)
        q = req('gt')['dog_paws']; return (q[mate][0] - q[front][0]), (min(range(4), key=lambda n: q[n][0]) == front)
    best = (0.0, 0.0, 0.0)
    for sw, sp in ((1.1, 1.1), (1.1, -1.1)):
        gap, ok = arrange(sw, sp)
        if ok and gap > best[0]: best = (gap, sw, sp)
    arrange(best[1], best[2]) if best[0] >= 0.035 else arrange(0.0, 0.0)
def pel_frame(g):
    q = np.asarray(g['pelvis']['quat'])[[1, 2, 3, 0]]; yaw = Rotation.from_quat(q).as_euler('zyx')[0]
    R = Rotation.from_euler('z', yaw).as_matrix(); return np.asarray(g['pelvis']['pos']), R
def to_pel(g, p):
    o, R = pel_frame(g); return R.T @ (np.asarray(p) - o)
def puppy(g):
    top = g['table_top_z']; d = g['dog']['pos']; box = g['box']['pos']
    return 'dogz-top %+.3f dogx-boxfront %+.3f handz-top %+.3f' % (d[2] - top, d[0] - (box[0] - 0.137), g['right_grasp_world'][2] - top)
def width(): return req('contacts')['right_width']

for _ in range(40):
    try:
        if req('state', timeout=3000).get('ok'): break
    except Exception: pass
    time.sleep(1)
root = Path('/tmp/claude-1001/-home-junjieye-workspace-multirobot/4346b64e-d4d8-4650-9c6b-73005fc9b1fb/scratchpad/exp_root')
(root / 'operator').mkdir(parents=True, exist_ok=True)
arming.write(root, True, minutes=120, by='carry experiment')
cfg = types.SimpleNamespace(backend_options={'stack_port': PORT, 'head_pitch': 0.5}, root=root, observation_profile='rgb')

def walk_logged(fwd, label):
    stop = threading.Event(); rows = []
    def poll():
        while not stop.is_set():
            try: rows.append((width(), puppy(req('gt', timeout=2000))))
            except Exception: pass
            time.sleep(0.1)
    t = threading.Thread(target=poll, daemon=True); t.start()
    out = req('walk', forward_m=fwd); stop.set(); t.join(1)
    ws = [r[0] for r in rows]; first_drop = next((i for i, x in enumerate(ws) if x < 0.012), None)
    log('%s: walked %.3f, width min %.4f, drop at sample %s/%d' % (label, out.get('walked_m', float('nan')), min(ws) if ws else -1, first_drop, len(rows)))
    if first_drop is not None:
        for i in range(max(0, first_drop - 3), min(len(rows), first_drop + 1)): log('   sample %d: w %.4f | %s' % (i, rows[i][0], rows[i][1]))
    g = req('gt'); log('   after: width %.4f | %s | tilt %.1f' % (width(), puppy(g), g['tilt_deg']))
    return width() >= 0.012

results = []
for trial in range(TRIALS):
    log(f'===== trial {trial + 1}/{TRIALS} depth {DEPTH} arc {ARC}')
    if trial > 0:
        req('reset_scene'); req('reset'); time.sleep(3.5)
    be = R1SonicBackend(cfg); be._head(0.5, 1.0)
    g = req('gt')
    while g['table_edge_ahead_m'] > 0.55:
        req('walk', forward_m=min(0.25, g['table_edge_ahead_m'] - 0.40)); g = req('gt')
    place_overhang(); separate_pair()
    g = req('gt'); front = min(range(4), key=lambda n: g['dog_paws'][n][0]); paw = to_pel(g, g['dog_paws'][front])
    target = np.array([0.31, -0.21]); o, R = pel_frame(g); d = R @ np.array([*(target - paw[:2]), 0.0])
    req('slide_table', dx=float(d[0]), dy=float(d[1])); time.sleep(3.0)
    g = req('gt'); paw = to_pel(g, g['dog_paws'][front])
    # leg axis: from the paw ball toward the hip (limb body origin)
    hip = to_pel(g, g['dog_hips'][front])
    axis = (hip - paw); axis /= np.linalg.norm(axis)
    gp = paw + DEPTH * axis
    TOP = os.environ.get('TOPGRASP') == '1'
    log('paw %s hip %s grasp point %s (edge ahead %.3f) top=%s' % (np.round(paw, 3), np.round(hip, 3), np.round(gp, 3), g['table_edge_ahead_m'], TOP))
    try:
        if TOP:   # from above: fingers open across the leg, pads straddle the shaft behind the paw bulge
            fing = [-axis[1], axis[0], 0.0]
            r = be._move_hand('right', [gp[0], gp[1], gp[2] + 0.10], approach=[0.3, 0, -1], fingers=fing, gripper_width_m=0.07, secs=2.0)
            r = be._move_hand('right', [gp[0], gp[1], gp[2] + 0.005], approach=[0.3, 0, -1], fingers=fing, secs=1.5)
        else:     # approach along the measured leg axis (paw -> hip), fingers across it
            ax = np.array([axis[0], axis[1], 0.0]); ax /= np.linalg.norm(ax); fing = [-ax[1], ax[0], 0.0]
            r = be._move_hand('right', list(gp - 0.08 * ax), approach=list(ax), fingers=fing, gripper_width_m=0.07, secs=2.0)
            r = be._move_hand('right', list(gp), approach=list(ax), fingers=fing, secs=1.5)
        log('grasp ik %.3f servo %s' % (r['plan']['ik_error_m'], r.get('servo_errors_m')))
        r = be._gripper('right', 0.0, 1.0); w = r['gripper']['measured_m']; log('close -> width %.4f' % w)
    except Exception as e:
        log('grasp failed: %s' % str(e)[:120]); results.append('grasp-fail'); continue
    if w < 0.015: log('NOT HELD'); results.append('miss'); continue
    h = np.asarray(be._grasp_points(be._state()['qd'])['right'])
    if ARC:   # follow the leg's arc about the hip (7 cm leg), then rise with the torso tipping up: x advances with z
        plan = [(0.015, 0.04, 0.0), (0.035, 0.07, 0.2), (0.055, 0.10, 0.3), (0.07, 0.14, 0.4), (0.08, 0.18, 0.5), (0.08, 0.22, 0.5), (0.08, 0.265, 0.5)]
    else:
        plan = [(0.0, 0.04, 0.0), (0.0, 0.08, 0.0), (0.0, 0.12, 0.0), (0.0, 0.16, 0.3), (0.0, 0.20, 0.5), (0.0, 0.24, 0.5), (0.0, 0.265, 0.5)]
    ok = True
    for dx, z, tilt in plan:
        ax = np.array([axis[0], axis[1], 0.0]); ax /= np.linalg.norm(ax); fing = [-ax[1], ax[0], 0.0]
        goal = [h[0] + dx * ax[0], h[1] + dx * ax[1], z]; appr = None
        for t_ in (tilt, tilt + 0.2, tilt + 0.4, max(0.0, tilt - 0.2)):
            pv = be._move_hand('right', goal, approach=[ax[0], ax[1], t_], fingers=fing, preview_only=True)
            if pv['reachable']: appr = [ax[0], ax[1], t_]; break
        if appr is None: log('lift to %s unreachable' % np.round(goal, 3)); ok = False; break
        try: be._move_hand('right', goal, approach=appr, fingers=fing, secs=1.5)
        except Exception as e: log('lift step failed: %s' % str(e)[:100]); ok = False; break
        time.sleep(0.4); g = req('gt'); w = width()
        log('lift (%+.3f, z %.3f) appr %s -> width %.4f | %s' % (dx, z, appr, w, puppy(g)))
        if w < 0.012: log('SLIPPED during lift'); ok = False; break
    if not ok: results.append('lift-slip'); continue
    g = req('gt'); hang = g['dog']['pos'][2] - g['table_top_z']
    log('LIFTED: puppy torso %+.3f vs table top (hanging=%s)' % (hang, hang < 0.02))
    if WALKTEST == 2:   # hold still and watch for creep
        t0 = time.time(); w0 = width(); z0 = req('gt')['dog']['pos'][2]
        while time.time() - t0 < 150:
            time.sleep(10); g = req('gt'); w = width()
            log('   hold %3.0f s: width %.4f (d %+.4f) dog z %+.4f' % (time.time() - t0, w, w - w0, g['dog']['pos'][2] - z0))
            if w < 0.012: log('CREPT OUT'); break
        results.append('held-150s' if width() >= 0.012 else 'creep-drop'); continue
    if WALKTEST == 3:   # the carry as the agent does it: short forward steps, width after each; release over the box
        ws = []; over = False
        for k in range(5):
            out = req('walk', forward_m=0.07); w = width(); ws.append(round(w, 4)); g = req('gt')
            dxf = g['dog']['pos'][0] - (g['box']['pos'][0] - 0.137)
            log('   step %d: walked %.3f width %.4f | %s' % (k + 1, out.get('walked_m', -1), w, puppy(g)))
            if w < 0.012: break
            if dxf >= 0.04: over = True; break
        if over:
            be._gripper('right', 0.07, 1.0); time.sleep(2.5); g = req('gt')
            d = np.array(g['dog']['pos']); b = np.array(g['box']['pos']); rel = d - b
            inside = abs(rel[0]) < 0.134 and abs(rel[1]) < 0.189 and 0.0 < rel[2] < 0.20
            log('   RELEASED: dog rel box (%+.3f, %+.3f, %+.3f) inside=%s box tilt q=%s' % (rel[0], rel[1], rel[2], inside, np.round(g['box_quat'], 3)))
            results.append('PLACED' if inside else 'released-outside'); continue
        results.append('steps-held=%d widths=%s' % (sum(1 for w in ws if w >= 0.012), ws)); continue
    if not WALKTEST: results.append('lifted'); continue
    held = walk_logged(-0.15, 'walk BACK -0.15 (free space)')
    if held: held = walk_logged(0.15, 'walk FWD +0.15 (hand before the edge)')
    if held:
        h2 = np.asarray(be._grasp_points(be._state()['qd'])['right']); edge = req('gt')['table_edge_ahead_m']
        log('hand now at %s, edge ahead %.3f' % (np.round(h2, 3), edge))
        held = walk_logged(0.15, 'walk FWD +0.15 (hand passes over the edge, puppy meets the table)')
    results.append('carried' if held else 'walk-drop')
log('RESULTS: %s' % results)
req('shutdown')
