"""Does SONIC's teleop encoder (planner legs + VR 3-point hand targets) keep the hands where they are told while the
G1 walks? Phase A (no box): hold the hands at a carry pose, then walk forward, back, sideways and turn, and measure
how far the hands drift from their targets (forward kinematics of the measured joints, in the pelvis frame). Phase B
(BOX=1): the operator puts the box between the hands, the targets squeeze it, then the same walks; the box's height
and its place relative to the pelvis are measured from the simulation's ground truth.
  CHECK_DIR=<dir> [BOX=1] PYTHONPATH=../rax ~/envs/rax-g1/bin/python teleop_mode_check.py"""
import json, math, os, subprocess, sys, time, types
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np, zmq
from services import arming
from services.controller import operator
from services.team.backend import G1R1Backend
from services.g1sonic import robot as R
from services.g1sonic.zmq_protocol import IDLE, SLOW_WALK

HERE = Path(__file__).resolve().parent
ctx = zmq.Context.instance()
def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)

# ---- forward kinematics of the G1 from the simulation's own model (meshes dropped) ----
tree = ET.parse(HERE / 'scene' / 'office_g1_r1.xml')
pelvis = next(b for b in tree.getroot().iter('body') if b.get('name') == 'pelvis')
for parent in list(pelvis.iter()):
    for geom in list(parent.findall('geom')):
        parent.remove(geom)
for element in pelvis.iter():
    element.attrib.pop('class', None)
    element.attrib.pop('childclass', None)
for parent in list(pelvis.iter()):
    for site in list(parent.findall('site')) + list(parent.findall('camera')):
        parent.remove(site)
pelvis.set('pos', '0 0 0')
fk_xml = ('<mujoco><compiler angle="radian"/><worldbody>' + ET.tostring(pelvis, encoding='unicode')
          + '</worldbody></mujoco>')
M = mujoco.MjModel.from_xml_string(fk_xml); D = mujoco.MjData(M)
QADR = {M.joint(j).name: M.jnt_qposadr[j] for j in range(M.njnt)}
BODY = {n: M.body(n).id for n in ('left_wrist_yaw_link', 'right_wrist_yaw_link', 'torso_link')}
OFFS = {'left_wrist_yaw_link': np.array([0.18, -0.025, 0.0]), 'right_wrist_yaw_link': np.array([0.18, 0.025, 0.0]),
        'torso_link': np.array([0.0, 0.0, 0.35])}

def fk(q_isaac):
    """{body: (point = body pos + offset rotated, quat wxyz)} in the pelvis frame for joint angles in IsaacLab order."""
    D.qpos[:] = 0; D.qpos[3] = 1.0
    for name, v in zip(R.JOINT_NAMES, q_isaac):
        D.qpos[QADR[name]] = v
    mujoco.mj_kinematics(M, D)
    out = {}
    for n, b in BODY.items():
        rot = D.xmat[b].reshape(3, 3)
        out[n] = (D.xpos[b] + rot @ OFFS[n], D.xquat[b].copy())
    return out

def vr_of(points):
    order = ('left_wrist_yaw_link', 'right_wrist_yaw_link', 'torso_link')
    return (np.concatenate([points[n][0] for n in order]), np.concatenate([points[n][1] for n in order]))

def qpos():
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000)
    sub.connect('tcp://127.0.0.1:5792'); q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close(); return q

def yaw(qw):
    w, x, y, z = qw; return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

OUT = Path(os.environ['CHECK_DIR']); BOX = os.environ.get('BOX') == '1'
root = OUT / f'teleop_check_{int(time.time())}'; (root / 'operator').mkdir(parents=True); (root / 'agent-workspace').mkdir()
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
be = G1R1Backend(config); g1 = be.robots['g1'].backend
results = {}
try:
    g1.call('g1_hold', {}); time.sleep(3)
    snap = g1.state.latest(); q0 = np.asarray(snap.q, dtype=float)
    start = fk(q0)
    log('measured start points (pelvis frame):', {k: np.round(v[0], 3).tolist() for k, v in start.items()})
    # the carry pose: hands in front of the chest, palms facing (orientations kept from the measured pose)
    sep = float(os.environ.get('SEP', '0.40'))
    target = {k: (v[0].copy(), v[1].copy()) for k, v in start.items()}
    target['left_wrist_yaw_link'] = (np.array([0.42, sep / 2, 0.12]), start['left_wrist_yaw_link'][1])
    target['right_wrist_yaw_link'] = (np.array([0.42, -sep / 2, 0.12]), start['right_wrist_yaw_link'][1])

    with g1.lock:
        g1._planner_keepalive = None; g1.streamer.keepalive = None; g1.streamer.mirror_upper = False
        g1.streamer.hold(); g1.streamer.command(start=True, planner=True); g1.control_mode = 'planner'
    # the deploy enters the teleop encoder only when its VR flag turns on: clear it first with plain planner messages
    for _ in range(10):
        g1.streamer.planner(IDLE, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], -1.0); time.sleep(0.05)
    trace = []
    def drive(label, seconds, mode=IDLE, move=(0, 0, 0), facing_deg=0.0, speed=-1.0, points=None, blend=None):
        """Send planner messages with VR targets at 20 Hz; blend: (from_points, to_points) over the segment."""
        t0 = time.time(); errs, tilts, boxz, last_box = [], [], [], 0.0
        qa = qpos(); p0 = qa[:2].copy(); y0 = yaw(qa[3:7])
        while time.time() - t0 < seconds:
            a = min(1.0, (time.time() - t0) / seconds)
            pts = points
            if blend:
                pts = {k: (blend[0][k][0] * (1 - a) + blend[1][k][0] * a, blend[1][k][1]) for k in blend[1]}
            pos, quat = vr_of(pts)
            f = math.radians(facing_deg)
            g1.streamer.planner(mode, list(move), [math.cos(f), math.sin(f), 0.0], speed, vr_position=pos, vr_orientation=quat)
            s = g1.state.latest()
            if s is not None:
                m = fk(np.asarray(s.q, dtype=float))
                errs.append([float(np.linalg.norm(m[k][0] - pts[k][0])) for k in ('left_wrist_yaw_link', 'right_wrist_yaw_link')]
                            + [float(np.linalg.norm(m['left_wrist_yaw_link'][0] - m['right_wrist_yaw_link'][0]))])
                tilts.append(s.tilt_deg())
            if BOX and time.time() - last_box > 0.5:
                last_box = time.time(); qq = qpos(); boxz.append(round(float(qq[52] - qq[2]), 3))
            time.sleep(0.05)
        qb = qpos(); e = np.array(errs[len(errs) // 3:]) if errs else np.zeros((1, 3))
        row = {'hand_err_mean_m': round(float(e[:, :2].mean()), 3), 'hand_err_max_m': round(float(e[:, :2].max()), 3),
               'sep_min_m': round(float(e[:, 2].min()), 3), 'sep_max_m': round(float(e[:, 2].max()), 3),
               'tilt_max_deg': round(max(tilts) if tilts else -1, 1),
               'moved_m': np.round(qb[:2] - p0, 3).tolist(), 'turned_deg': round(math.degrees(yaw(qb[3:7]) - y0), 1)}
        dx, dy = qb[:2] - p0
        row['moved_robot_frame_m'] = [round(float(math.cos(y0) * dx + math.sin(y0) * dy), 3),
                                      round(float(-math.sin(y0) * dx + math.cos(y0) * dy), 3)]
        if s is not None:
            row['hands_measured'] = [np.round(m[k][0], 3).tolist() for k in ('left_wrist_yaw_link', 'right_wrist_yaw_link')]
        if BOX:
            row['box_z'] = round(float(qb[52]), 3)
            row['box_z_minus_pelvis_trace'] = boxz
            c, sn = math.cos(yaw(qb[3:7])), math.sin(yaw(qb[3:7])); d = qb[50:52] - qb[:2]
            row['box_in_pelvis_xy'] = [round(float(c * d[0] + sn * d[1]), 3), round(float(-sn * d[0] + c * d[1]), 3)]
        results[label] = row
        log(label, row)
        return pts

    pts = drive('raise hands to the carry pose', 4.0, blend=(start, target))
    pts = drive('hold the carry pose (standing)', 4.0, points=target)
    def with_sep(width, z=0.12, x=0.42):
        t = {k: (v[0].copy(), v[1]) for k, v in target.items()}
        t['left_wrist_yaw_link'] = (np.array([x, width / 2, z]), start['left_wrist_yaw_link'][1])
        t['right_wrist_yaw_link'] = (np.array([x, -width / 2, z]), start['right_wrist_yaw_link'][1])
        return t
    hold = target
    def arm_pose(roll, pitch=-0.45, elbow=0.9):
        q = np.array(R.DEFAULT_STANDING, dtype=float)
        for side, sign in (('left', 1), ('right', -1)):
            for j, v in ((f'{side}_shoulder_pitch_joint', pitch), (f'{side}_elbow_joint', elbow), (f'{side}_shoulder_roll_joint', sign * roll)):
                q[R.JOINT_NAMES.index(j)] = v
        pts = fk(q)
        pts['torso_link'] = target['torso_link']
        return pts
    if BOX and os.environ.get('POSE') == 'arm':
        wide, tight = arm_pose(float(os.environ.get('ROLL_OPEN', '0.40'))), arm_pose(float(os.environ.get('ROLL_TIGHT', '-0.05')))
        log('arm-pose targets: open sep', round(float(wide['left_wrist_yaw_link'][0][1] - wide['right_wrist_yaw_link'][0][1]), 3),
            'tight sep', round(float(tight['left_wrist_yaw_link'][0][1] - tight['right_wrist_yaw_link'][0][1]), 3),
            'left point', np.round(tight['left_wrist_yaw_link'][0], 3).tolist())
        op.terminate(); time.sleep(1.5)
        keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
        drive('to the open carry pose', 4.0, blend=(target, wide))
        drive('hold open', 2.0, points=wide)
        mid = (wide['left_wrist_yaw_link'][0] + wide['right_wrist_yaw_link'][0]) / 2
        import threading
        placing = True
        def place_loop():
            while placing:
                q = qpos(); yw = yaw(q[3:7]); c, sn = math.cos(yw), math.sin(yw)
                bx, by = q[0] + c * mid[0] - sn * mid[1], q[1] + sn * mid[0] + c * mid[1]
                keys.send_string(f'place=box,{bx},{by},{q[2] + mid[2] - 0.13},{math.cos(yw / 2)},0,0,{math.sin(yw / 2)}')
                time.sleep(0.1)
        th = threading.Thread(target=place_loop, daemon=True); th.start()
        drive('close on the box (box held in place by the check)', 4.0, blend=(wide, tight))
        placing = False; th.join()
        drive('squeeze, box free (standing)', 6.0, points=tight)
        hold = tight
    elif BOX:
        op.terminate(); time.sleep(1.5)       # the operator released the band; now this check places the box itself
        keys = ctx.socket(zmq.PUB); keys.bind('tcp://127.0.0.1:5790'); time.sleep(1.0)
        wide, tight = with_sep(float(os.environ.get('WIDE', '0.56'))), with_sep(float(os.environ.get('TIGHT', '0.30')))
        drive('open the hands wide', 3.0, blend=(target, wide))
        import threading
        placing = True
        def place_loop():
            while placing:
                q = qpos(); yw = yaw(q[3:7]); c, sn = math.cos(yw), math.sin(yw)
                bx, by = q[0] + c * 0.45, q[1] + sn * 0.45
                keys.send_string(f'place=box,{bx},{by},{q[2] + 0.12 - 0.13},{math.cos(yw / 2)},0,0,{math.sin(yw / 2)}')
                time.sleep(0.1)
        th = threading.Thread(target=place_loop, daemon=True); th.start()
        drive('close on the box (box held in place by the check)', 4.0, blend=(wide, tight))
        placing = False; th.join()
        drive('squeeze, box free (standing)', 5.0, points=tight)
        hold = tight
    for label, mv, sp in (('walk backward', (-1, 0, 0), 0.3), ('walk left', (0, 1, 0), 0.3), ('walk right', (0, -1, 0), 0.3),
                          ('walk forward', (1, 0, 0), 0.3)):
        drive(label, 7.0, SLOW_WALK, mv, 0.0, sp, points=hold)
        drive('stand after ' + label, 3.0, points=hold)
    drive('turn 30 deg in place', 4.0, IDLE, (0, 0, 0), 30.0, -1.0, points=hold)
    drive('stand after the turn', 3.0, IDLE, (0, 0, 0), 30.0, -1.0, points=hold)
    target = hold
    g1._planner_keepalive = lambda: g1.streamer.planner(IDLE, [0.0, 0.0, 0.0], [math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0],
                                                        -1.0, vr_position=vr_of(target)[0], vr_orientation=vr_of(target)[1])
    g1.streamer.keepalive = g1._planner_keepalive
    log('deploy log VR lines:', [l for l in (HERE / 'logs' / 'deploy.log').read_text().splitlines() if 'VR 3-point' in l][-4:])
finally:
    try:
        g1.call('g1_hold', {})
    except Exception as error:
        log('hold after the check:', error)
    be.close(); op.terminate()
(OUT / f'{root.name}.json').write_text(json.dumps(results, indent=1))
log('RESULTS', json.dumps(results))
