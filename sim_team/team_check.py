"""First run of the g1r1 team backend against the two-robot office simulation (team_sim.sh start): the gantry puts the
G1 on its feet, then both robots are driven through the team tools. Run on the host:
  PYTHONPATH=../rax ~/envs/rax-g1/bin/python team_check.py"""
import json, sys, threading, time, types
from pathlib import Path
import numpy as np, zmq
from services import arming
from services.team.backend import G1R1Backend

HERE = Path(__file__).resolve().parent
KEYS, STATE = 5790, 5797
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)

def gantry():
    """Lower the hanging G1 until its feet touch, keep the band until control has settled, then release (as gantry.py)."""
    pub = ctx.socket(zmq.PUB); pub.bind(f'tcp://127.0.0.1:{KEYS}'); time.sleep(0.6)
    for key in ('band=-0.10', 'band=-0.20'):
        pub.send_string(key); time.sleep(1.0)
    sub = ctx.socket(zmq.SUB); sub.connect(f'tcp://127.0.0.1:{STATE}'); sub.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
    sub.recv(); log('[gantry] G1 control is running; releasing the band in 8 s')
    time.sleep(8.0); pub.send_string('release'); time.sleep(0.3); pub.close(0); log('[gantry] released')

root = HERE / 'session_root'; (root / 'operator').mkdir(parents=True, exist_ok=True); (root / 'agent-workspace').mkdir(exist_ok=True)
layout = json.loads((HERE / 'scene' / 'layout.json').read_text())
config = types.SimpleNamespace(root=root, task='office_box', observation_profile='rgbd', backend_options={
    'g1': {'zmq_host': '127.0.0.1', 'pose_port': 5796, 'state_port': STATE, 'camera': 'network', 'camera_host': '127.0.0.1', 'camera_port': 7796},
    'r1': {'stack_host': '127.0.0.1', 'stack_port': 5791},
    'team': {'start_poses': {k: [v['pos'][0], v['pos'][1], v['yaw_deg']] for k, v in layout['robots'].items()}}})
arming.write(root, True, minutes=30, by='team_check')
threading.Thread(target=gantry, daemon=True).start()
time.sleep(3.0)
be = G1R1Backend(config)
def call(name, **arguments):
    be.validate(name, arguments); return be.call(name, arguments)[0]
try:
    names = [t['name'] for t in be.tools()]
    log('tools:', len(names), 'of them team:', [n for n in names if n.startswith('team_')])
    hold = call('g1_hold'); log('g1_hold ->', {k: hold.get(k) for k in ('held', 'control', 'tilt_deg', 'pelvis_height_est_m', 'start_error')})
    time.sleep(12.0)
    g = call('g1_status'); r = call('r1_status')
    log('G1: tilt %.1f deg, pelvis %.2f m, mode %s | R1: tilt %.1f deg, fallen %s' % (
        g['tilt_deg'], g['proprio']['pelvis_height_est_m'], g['stack']['mode'], r.get('tilt_deg', -1), r.get('fallen')))
    be.validate('team_observe', {}); obs, images = be.call('team_observe', {})
    log('team_observe: images', obs['image_order'], '| G1 keys', sorted(obs['robots']['g1'])[:6], '| R1 keys', sorted(obs['robots']['r1'])[:6])
    import base64
    for k, (name, image) in enumerate(zip(obs['image_order'], images)):
        (root / f"observe_{k}_{name}.{image['mimeType'].split('/')[1]}").write_bytes(base64.b64decode(image['data']))
    st = call('team_status'); log('team frame gaps:', st['team_frame']['gap_m'], '| armed:', st['motion']['armed'])
    # both robots move one joint at the same time
    q = np.array(g['proprio']['joint_positions_rad']); row0 = np.array(g['command']['last_row'])
    idx = g['joint_names_canonical29'].index('left_elbow_joint')
    rows = []
    for k in range(60):
        row = row0.copy(); row[9 + idx] = row0[9 + idx] - 0.3 * min(1.0, (k + 1) / 50); rows.append(row.tolist())
    t0 = time.time()
    out = call('team_parallel', calls=[{'tool': 'g1_step', 'arguments': {'actions': rows}},
                                       {'tool': 'r1_move_joints', 'arguments': {'joints': {'right_elbow_joint': 0.5}, 'secs': 1.5}}])
    log('team_parallel took %.1f s ->' % (time.time() - t0), [(j['robot'], j['tool'], j['state'], j['seconds']) for j in out['jobs']])
    for j in out['jobs']:
        if 'error' in j: log('   error:', j['error']['reason'][:300])
    g2 = call('g1_status'); r2 = call('r1_status')
    log('after: G1 elbow %.2f -> %.2f rad, tilt %.1f | R1 tilt %.1f, fallen %s' % (
        q[idx], g2['proprio']['joint_positions_rad'][idx], g2['tilt_deg'], r2.get('tilt_deg', -1), r2.get('fallen')))
finally:
    be.close()
