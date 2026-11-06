"""Real-robot bridge: serves the r1 stack protocol that the RAX r1sonic backend speaks (JSON over ZMQ REQ/REP,
see stack.py) on top of the real SONIC_R1 deployment (gear_sonic_deploy / g1_deploy_onnx_ref, ZMQManager input,
ZMQOutputHandler telemetry).

STATUS 2026-11-06 (tested): run against the REAL R1 deployment in simulation - r1_sim_cpu (gear_sonic_deploy, the
CPU build of r1_deploy_onnx_ref) + the lab's R1 MuJoCo DDS simulator (MA-Teleop teleop/clients/sim/r1_sonic_sim.py,
DDS domain 42 on lo). Standing holds (tilt < 2 deg) and an IK-consistent right-hand reach to (0.30, -0.20, 0.0) in
the pelvis frame tracks to < 1 cm; through the RAX backend three table-front reaches pass 15/15 with the simulator in
--lockstep (0.3-1.8 cm). Free-running, 5/7 runs fell during a reach: the policy is sensitive to ~10-20 ms of
observation latency - check 50 Hz timing on the robot PC. Test harness: r1/deploy_sim/ (run.sh, bridge_e2e.py).
NOT working yet: walking (see walk()), camera, PiPER grippers, head pitch.

What the R1 deploy stack expects (src/g1/g1_deploy_onnx_ref, include/input_interface/zmq_endpoint_interface.hpp):
  * It CONNECTS a SUB socket to the sender at zmq_host:zmq_port (lab: 6556); the sender BINDS a PUB.
    Messages are  topic bytes + 1280-byte zero-padded JSON header + packed little-endian payload.
  * topic "command": {start u8, stop u8, planner u8}. R1 has NO planner (deploy.sh --robot r1 defaults to
    --no-planner), so start selects STREAMED MOTION, which reads only the "pose" topic.
  * topic "pose", protocol version 1: joint_pos f32[N,26] and joint_vel f32[N,26] in IsaacLab order, body_quat
    f32[N,4] (wxyz, reference root), frame_index i64[N]; plus vr_position f32[9] (left wrist, right wrist, torso) and
    vr_orientation f32[12] (wxyz). With vr_position present the deploy selects encoder mode 1 (teleop): the
    lower-body joints of the stream + the three points, root-local with the link offsets applied - exactly the RAX
    stream row (rows 0-20), so a row maps 1:1. Without vr_position it is mode 0 (whole-body joint reference).
  * telemetry: ZMQOutputHandler BINDS a PUB on zmq_out_port (lab: 6557), topic "g1_debug", msgpack map every tick.
    body_q / body_q_measured are in MuJoCo joint order (MUJOCO_JOINTS below; verified against the deploy's own
    target), base_quat is wxyz.
  * Grippers: the PiPER grippers are not part of the policy I/O. --piper-right runs MA-Teleop's
    tools/r1/piper_can_bridge.py --serve (the lab's verified right-gripper path, locally or over ssh) and speaks its
    JSON-lines protocol: 'start' only while the trigger is released, then a fresh 'pose' every tick; the commanded
    right width maps to its binary trigger (< 0.035 m closes to 2 mm, otherwise opens to 70 mm) and its feedback
    is the measured width. Any PiPER error stops the session (state() raises). Without --piper-right the gripper
    is not driven and its width reads 0.07. Left gripper: no verified adapter serial yet.
  * Head: this deploy build starts with head tracking disabled (pitch limit +/-0.35 rad), the RAX task assumes 0.5.
  * Odometry: the telemetry carries no base position (TODO, only needed once walking works).

Run on the robot PC:  python -m services.r1sonic.bridge_real --port 5591 --stack-in 6556 --stack-out 6557
then point the RAX backend options at it: {"stack_host": "<robot pc>", "stack_port": 5591}.
"""
from __future__ import annotations

import argparse
import json
import queue
import shlex
import struct
import subprocess
import threading
import time

import numpy as np
import zmq

try:
    import msgpack
except ImportError:  # the telemetry decoder needs it on the robot PC
    msgpack = None

HEADER_SIZE = 1280                      # ZMQPackedMessageSubscriber::HEADER_SIZE
CONTROL_HZ = 50.0
# SONIC_R1 validation/r1_deployment_contract.json: mujoco_joint_order (telemetry order) and the IsaacLab order with
# the nominal standing pose (stream order)
MUJOCO_JOINTS = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint',
                 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint',
                 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
                 'waist_roll_joint', 'waist_yaw_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'right_shoulder_pitch_joint',
                 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
                 'head_pitch_joint', 'head_yaw_joint']
ISAAC_JOINTS = ['left_hip_pitch_joint', 'right_hip_pitch_joint', 'waist_roll_joint', 'left_hip_roll_joint',
                'right_hip_roll_joint', 'waist_yaw_joint', 'left_hip_yaw_joint', 'right_hip_yaw_joint', 'head_pitch_joint',
                'left_shoulder_pitch_joint', 'right_shoulder_pitch_joint', 'left_knee_joint', 'right_knee_joint',
                'head_yaw_joint', 'left_shoulder_roll_joint', 'right_shoulder_roll_joint', 'left_ankle_pitch_joint',
                'right_ankle_pitch_joint', 'left_shoulder_yaw_joint', 'right_shoulder_yaw_joint', 'left_ankle_roll_joint',
                'right_ankle_roll_joint', 'left_elbow_joint', 'right_elbow_joint', 'left_wrist_roll_joint',
                'right_wrist_roll_joint']
ISAAC_NOMINAL = np.array([-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35, 0.35, 0.3, 0.3, 0.0, 0.18, -0.18,
                          -0.2, -0.2, 0.0, 0.0, 0.0, 0.0, 0.87, 0.87, 0.0, 0.0])


def packed_message(topic: bytes, fields: list[tuple[str, str, list[int], bytes]]) -> bytes:
    """topic + padded JSON header + concatenated little-endian payloads (the deploy stack's wire format)."""
    header = {'v': 1, 'endian': 'le', 'count': 1,
              'fields': [{'name': n, 'dtype': d, 'shape': s} for n, d, s, _ in fields]}
    hb = json.dumps(header).encode()
    if len(hb) > HEADER_SIZE:
        raise ValueError('header too long')
    return topic + hb + b'\x00' * (HEADER_SIZE - len(hb)) + b''.join(p for _, _, _, p in fields)


def f32(values) -> bytes:
    return np.asarray(values, dtype='<f4').ravel().tobytes()


class PiperClient:
    """One PiPER gripper through MA-Teleop's piper_can_bridge.py --serve (JSON lines on stdin/stdout)."""
    CLOSE_BELOW_M = 0.035

    def __init__(self, command: str):
        self.proc = subprocess.Popen(shlex.split(command), stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.events: queue.SimpleQueue = queue.SimpleQueue()
        self.ready = self.started = False
        self.error = None
        self.width_m = None                     # measured width from the gripper feedback
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in self.proc.stdout:
            try:
                self.events.put(json.loads(line))
            except ValueError:
                pass
        self.events.put({'event': 'error', 'reason': 'PiPER bridge process ended'})

    def _send(self, event: dict) -> None:
        try:
            self.proc.stdin.write((json.dumps(event) + '\n').encode())
        except (BrokenPipeError, OSError) as exc:
            self.error = self.error or f'PiPER bridge pipe: {exc}'

    def tick(self, width_m: float) -> None:
        while not self.events.empty():
            ev = self.events.get()
            kind = ev.get('event')
            if kind == 'ready':
                self.ready = True
            elif kind == 'error':
                self.error = self.error or ev.get('reason', 'PiPER error')
            if isinstance(ev.get('feedback'), dict) and 'position_mm' in ev['feedback']:
                self.width_m = float(ev['feedback']['position_mm']) / 1000.0
        if not self.ready or self.error:
            return
        trigger = 1.0 if width_m < self.CLOSE_BELOW_M else 0.0
        if not self.started:
            if trigger > 0.0:
                return                          # the bridge enables only with the trigger released
            self._send({'event': 'start'}); self.started = True
        self._send({'event': 'pose', 'trigger': trigger, 'age_s': 0.0})

    def stop(self) -> None:
        self._send({'event': 'stop'})


class RealStack:
    """Adapter onto the deploy stack over its two ZMQ ports."""

    def __init__(self, stack_in_port: int, stack_out_port: int, out_topic: str, bind_host: str = '*',
                 piper_right: str | None = None, veer_deg: float = 0.0):
        self.ctx = zmq.Context.instance()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind(f'tcp://{bind_host}:{stack_in_port}')
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, out_topic)
        self.sub.connect(f'tcp://127.0.0.1:{stack_out_port}')   # VERIFY: host of the deploy process
        self.out_topic = out_topic.encode()
        self.lock = threading.Lock()
        self.row = None                         # last reference row (24 floats, see stack.py); FK of the measured pose at start
        self.queue: list[np.ndarray] = []
        self.resets = 0
        self.heading0 = None                    # reference heading (yaw at start), set on the first telemetry
        self.odom = np.zeros(3)                 # pelvis position in the reference frame (no odometry source yet)
        self.last = None                        # last decoded telemetry
        self.joint_names = MUJOCO_JOINTS        # telemetry order
        self.started = False
        self.frame = 0                          # pose-topic frame_index
        self.piper_right = PiperClient(piper_right) if piper_right else None
        from services.r1sonic.kin import Kinematics
        from services.r1sonic.backend import MODEL
        self.kin = Kinematics(MODEL)
        self.okin = Kinematics(MODEL)           # odometry FK, used only by the tick thread
        self.veer_deg = veer_deg
        self.ref_yaw = 0.0                      # streamed reference heading (relative to the stream's start heading)
        self.walk_plan = None                   # {'q': [n+1, 26] gait frames, 'yaw': [n+1], 'i': next frame}
        self.odo_xy = np.zeros(2)               # leg-kinematics odometry: pelvis xy in the start-heading frame
        self.odo_stance = None; self.odo_foot = None; self.odo_index = None

    # ---- deploy-stack commands ----
    def send_command(self, start: bool, stop: bool) -> None:
        self.pub.send(packed_message(b'command', [('start', 'u8', [1], struct.pack('B', int(start))),
                                                   ('stop', 'u8', [1], struct.pack('B', int(stop))),
                                                   ('planner', 'u8', [1], struct.pack('B', 0))]))

    def send_pose(self, row: np.ndarray, jp: np.ndarray, jv: np.ndarray, yaw: np.ndarray) -> None:
        """One protocol-1 pose message: LOOK lower-body frames from this tick on (IsaacLab order) + the row's three
        points (encoder mode 1, teleop). The deploy samples 10 frames 5 apart from its current frame."""
        pos = np.concatenate([row[0:3], row[7:10], row[14:17]])          # L wrist, R wrist, torso (reference frame)
        quat = np.concatenate([row[3:7], row[10:14], row[17:21]])        # wxyz, as the deploy expects
        n = len(jp)
        bq = np.stack([np.cos(yaw / 2), np.zeros(n), np.zeros(n), np.sin(yaw / 2)], axis=1)
        fields = [('joint_pos', 'f32', [n, 26], f32(jp)),
                  ('joint_vel', 'f32', [n, 26], f32(jv)),
                  ('body_quat', 'f32', [n, 4], f32(bq)),
                  ('frame_index', 'i64', [n], np.arange(self.frame, self.frame + n, dtype='<i8').tobytes()),
                  ('vr_position', 'f32', [9], f32(pos)),
                  ('vr_orientation', 'f32', [12], f32(quat))]
        self.pub.send(packed_message(b'pose', fields))
        self.frame += 1

    def measured_row(self, q: dict | None = None) -> np.ndarray:
        """The three points of the measured joints (FK, pelvis frame) as a stream row; grippers open, head 0."""
        if q is None:
            t = self.last
            q = dict(zip(self.joint_names, np.asarray(t.get('body_q_measured', t['body_q']), dtype=float)))
        row = np.zeros(24)
        for k, (p, qw) in enumerate(self.kin.points(q)):
            row[7 * k:7 * k + 3] = p; row[7 * k + 3:7 * k + 7] = qw
        row[21] = row[22] = 0.07
        return row

    # ---- telemetry ----
    def poll_telemetry(self) -> None:
        if msgpack is None:
            return
        try:
            while True:
                msg = self.sub.recv(flags=zmq.NOBLOCK)
                self.last = msgpack.unpackb(msg[len(self.out_topic):], raw=False)
        except zmq.Again:
            pass

    def state(self) -> dict:
        self.poll_telemetry()
        t = self.last
        if t is None:
            raise RuntimeError('no telemetry from the deploy stack yet (is it running with --output zmq?)')
        q = np.asarray(t['body_q'], dtype=float); dq = np.asarray(t['body_dq'], dtype=float)
        base = np.asarray(t['base_quat'], dtype=float)                  # VERIFY order: assumed wxyz
        yaw = _yaw_wxyz(base)
        if self.heading0 is None:
            self.heading0 = yaw
        rel = _quat_mul_wxyz(_yaw_quat_wxyz(-self.heading0), base)      # orientation relative to the reference heading
        names = self.joint_names or [f'joint_{i}' for i in range(len(q))]
        return {'t': time.time(), 'tick': int(t.get('index', 0)), 'joint_names': names,
                'q': q.tolist(), 'dq': dq.tolist(), 'base_quat_wxyz': rel.tolist(),
                'gyro': list(t.get('base_ang_vel', [0, 0, 0])),
                'gripper_width_m': {'left': 0.07, 'right': self._right_width()},
                'command_row': np.round(self.row if self.row is not None else np.zeros(24), 5).tolist(),
                'queued': len(self.queue), 'measured_points_row': np.round(self.measured_row()[:21], 5).tolist(),
                'fallen': bool(_tilt_deg(rel) > 60.0), 'tilt_deg': round(_tilt_deg(rel), 2),
                'odometry_xy_m': np.round(self.odo_xy, 3).tolist(), 'walking': self.walk_plan is not None}

    def _right_width(self) -> float:
        if self.piper_right is None:
            return 0.07
        if self.piper_right.error:
            raise RuntimeError(f'right PiPER gripper stopped: {self.piper_right.error}')
        if self.piper_right.width_m is None:
            raise RuntimeError('no feedback from the right PiPER gripper yet')
        return self.piper_right.width_m

    # ---- reference streaming at 50 Hz ----
    def stream(self, rows: list[np.ndarray]) -> None:
        with self.lock:
            self.queue = rows

    def hold(self) -> None:
        with self.lock:
            self.queue = []

    def tick(self) -> None:
        self.poll_telemetry()
        with self.lock:
            if self.row is None:
                # the deploy publishes telemetry only after start; its INIT has put the robot in the nominal standing
                # pose, so the first reference is the three points of that pose (then the arms stay where they are)
                self.row = self.measured_row(dict(zip(ISAAC_JOINTS, ISAAC_NOMINAL)))
            if self.queue:
                self.row = self.queue.pop(0)
            row = self.row.copy()
            jp, jv, yaw = self._lower_frames()
        self._odometry()
        if not self.started or self.frame % 50 == 0:
            self.send_command(start=True, stop=False); self.started = True
        self.send_pose(row, jp, jv, yaw)
        if self.piper_right is not None:
            self.piper_right.tick(float(row[22]))
        # TODO: left gripper (no verified adapter); head pitch from row[23]

    # ---- locomotion: a lower-body gait reference (no planner on R1) ----
    # The RAX sim's parametric gait (r1/sim/r1_sim_server.py _gait_pose), streamed with LOOK frames of look-ahead.
    # Deploy-stack sim 2026-11-06 (after fixing the lab sim's gyro frame): walks 2.1-2.2 m for a 1.35 m reference,
    # 3/3 free-running, max tilt 6-8 deg.
    G_PERIOD, G_RAMP, G_SPEED, LOOK = 52, 16, 0.009, 50
    A_HIP, A_KNEE, A_ANK = 0.40, 0.55, 0.20

    def _lower_frames(self):
        """(joint_pos [LOOK,26], joint_vel [LOOK,26], yaw [LOOK]) from this tick: the gait while walking, else stance."""
        p = self.walk_plan
        if p is None:
            return np.tile(ISAAC_NOMINAL, (self.LOOK, 1)), np.zeros((self.LOOK, 26)), np.full(self.LOOK, self.ref_yaw)
        last = len(p['q']) - 1
        idx = np.minimum(np.arange(p['i'], p['i'] + self.LOOK), last)
        prev = np.clip(np.arange(p['i'] - 1, p['i'] + self.LOOK - 1), 0, last)
        jp, yaw = p['q'][idx], p['yaw'][idx]
        jv = (jp - p['q'][prev]) * CONTROL_HZ
        p['i'] += 1
        if p['i'] > last:
            self.ref_yaw = float(p['yaw'][last]); self.walk_plan = None
        return jp, jv, yaw

    def _gait_plan(self, forward_m: float, turn_deg: float) -> dict:
        sgn = 1.0 if forward_m >= 0 else -1.0
        advance = abs(forward_m) > 0.02
        hipscale = 1.0 if advance else 0.0                           # pure turn: step in place, no hip swing (0.3 drifted 0.25-0.38 m)
        n = max(1, max(int(round(abs(forward_m) / self.G_SPEED)) + 12, int(round(abs(turn_deg) / 0.8)) + 2 * self.G_RAMP))
        nom = np.asarray(ISAAC_NOMINAL, dtype=float); ix = {nm: ISAAC_JOINTS.index(nm) for nm in ISAAC_JOINTS}
        dph = 2 * np.pi / self.G_PERIOD; turn = np.radians(turn_deg) / n
        veer = np.radians(self.veer_deg) if (sgn > 0 and advance) else 0.0
        qs, yaws, yaw = [], [], self.ref_yaw + veer
        for j in range(n + 1):
            r = float(np.clip(min(j / self.G_RAMP, (n - j) / self.G_RAMP), 0.0, 1.0))
            q = nom.copy()
            for ph, side in ((j * dph, 'left'), (j * dph + np.pi, 'right')):
                q[ix[f'{side}_hip_pitch_joint']] -= r * self.A_HIP * hipscale * np.cos(ph) * sgn
                q[ix[f'{side}_knee_joint']] += r * self.A_KNEE * max(0.0, -np.sin(ph))
                q[ix[f'{side}_ankle_pitch_joint']] -= r * self.A_ANK * np.sin(ph)
            qs.append(q); yaws.append(yaw); yaw += turn * r
        return {'q': np.array(qs), 'yaw': np.array(yaws), 'i': 0}

    def _odometry(self) -> None:
        """Leg-kinematics odometry: the pelvis moves opposite to the stance (lower) foot, both expressed in the
        start-heading frame through the IMU orientation. An estimate (foot slip and the stance switch are not modelled)."""
        t = self.last
        if t is None or t.get('index') == self.odo_index:
            return
        self.odo_index = t.get('index')
        base = np.asarray(t['base_quat'], dtype=float)
        if self.heading0 is None:
            self.heading0 = _yaw_wxyz(base)
        R = _rot_wxyz(_quat_mul_wxyz(_yaw_quat_wxyz(-self.heading0), base))
        self.okin.set(dict(zip(self.joint_names, np.asarray(t.get('body_q_measured', t['body_q']), dtype=float))))
        feet = {}
        for side in ('left', 'right'):                                   # a sole point 6 cm below the ankle (best of a sweep
            pa, Ra = self.okin.body(f'{side}_ankle_roll_link')           # against sim truth: rms 0.16 m vs 0.22 m at the ankle)
            feet[side] = R @ (pa + Ra @ np.array([0.0, 0.0, -0.06]))
        low = min(feet, key=lambda k: feet[k][2])
        stance = self.odo_stance if (self.odo_stance and feet[self.odo_stance][2] < feet[low][2] + 0.01) else low
        if stance == self.odo_stance and self.odo_foot is not None:
            self.odo_xy -= (feet[stance] - self.odo_foot)[:2]
        self.odo_stance, self.odo_foot = stance, feet[stance]

    def _pose2d(self):
        self.poll_telemetry()
        base = np.asarray(self.last['base_quat'], dtype=float)
        return self.odo_xy.copy(), _yaw_wxyz(base) - (self.heading0 or 0.0)

    def _walk_once(self, forward_m: float, turn_deg: float) -> dict:
        start, _ = self._pose2d()
        with self.lock:
            plan = self._gait_plan(forward_m, turn_deg); self.walk_plan = plan; self.queue = []
        t0 = time.monotonic(); fallen = False
        while self.walk_plan is plan and time.monotonic() - t0 < 25:
            time.sleep(0.02)
            if self.state()['fallen']:
                fallen = True; break
        with self.lock:
            self.walk_plan = None
        t1 = time.monotonic(); still = None; prev = self.odo_xy.copy()      # settle: the policy shuffles ~1 s after the gait
        while not fallen and time.monotonic() - t1 < 5.0:
            time.sleep(0.1)
            s = self.state(); v = float(np.linalg.norm(self.odo_xy - prev)) / 0.1; prev = self.odo_xy.copy()
            if v < 0.02 and float(np.linalg.norm(s['gyro'])) < 0.08:
                still = still or time.monotonic()
                if time.monotonic() - still >= 0.5:
                    break
            else:
                still = None
        p, yaw = self._pose2d()
        with self.lock:
            self.ref_yaw = float(yaw)                                    # snap the reference heading onto the settled robot
            keep = self.row[21:24].copy() if self.row is not None else np.array([0.07, 0.07, 0.0])
            self.row = self.measured_row(); self.row[21:24] = keep       # keep gripper + head commands (a carry must not drop)
        s = self.state()
        return {'disp': p - start, 'fallen': fallen or s['fallen'], 'tilt_deg': s['tilt_deg']}

    def walk(self, forward_m: float, turn_deg: float) -> dict:
        """Walk forward_m along the current heading (negative = backward) with an optional turn; blocks until still.
        Same pass logic as the RAX sim (one forward correction for a residual >= 6 cm, short passes lengthened to
        12 cm, at most one turn correction), closed on the leg-odometry ESTIMATE: re-observe afterwards."""
        if self.last is None:
            raise RuntimeError('no telemetry from the deploy stack yet')
        p0, yaw0 = self._pose2d(); head = np.array([np.cos(yaw0), np.sin(yaw0)])
        translating = abs(forward_m) > 0.02
        fwd_left, turn_left = float(forward_m), float(turn_deg)
        for i in range(3):
            out = self._walk_once(fwd_left, turn_left)
            p, yaw = self._pose2d()
            walked = float(np.dot(p - p0, head)); lateral = float(np.dot(p - p0, [-head[1], head[0]]))
            turned = float(np.degrees((yaw - yaw0 + np.pi) % (2 * np.pi) - np.pi))
            fwd_left = (float(forward_m) - walked) if translating else 0.0
            turn_left = float(turn_deg) - turned
            turn_tol = 6.0 if i == 0 else 10.0
            if out['fallen'] or (abs(fwd_left) < 0.03 and abs(turn_left) < turn_tol):
                break
            if abs(fwd_left) < 0.06 or i >= 1:
                fwd_left = 0.0
            elif abs(fwd_left) < 0.12:
                fwd_left = float(np.sign(fwd_left)) * 0.12
            if abs(turn_left) < turn_tol or i >= 1:
                turn_left = 0.0
            if fwd_left == 0.0 and turn_left == 0.0:
                break
        return {'requested_m': round(float(forward_m), 3), 'walked_m': round(walked, 3), 'lateral_m': round(lateral, 3),
                'turn_deg': float(turn_deg), 'turned_deg': round(turned, 1), 'passes': i + 1, 'fallen': out['fallen'],
                'tilt_deg': out['tilt_deg'], 'odometry': 'leg-kinematics estimate (no ground truth on the robot)'}


# ---- small quaternion helpers (wxyz) ----
def _yaw_wxyz(q):
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _yaw_quat_wxyz(yaw):
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def _quat_mul_wxyz(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _rot_wxyz(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _tilt_deg(q):
    w, x, y, z = q
    zz = 1 - 2 * (x * x + y * y)                 # third row, third column of the rotation matrix
    return float(np.degrees(np.arccos(np.clip(zz, -1, 1))))


def serve(stack: RealStack, port: int) -> None:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.bind(f'tcp://127.0.0.1:{port}')
    stop = threading.Event()

    def loop():
        period = 1.0 / CONTROL_HZ
        while not stop.is_set():
            t0 = time.monotonic()
            try:
                stack.tick()
            except Exception as exc:  # noqa: BLE001
                print('tick error:', exc, flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    threading.Thread(target=loop, daemon=True).start()
    print(json.dumps({'ready': True, 'port': port, 'bridge': 'real'}), flush=True)
    while not stop.is_set():
        req = json.loads(sock.recv())
        op = req.get('op')
        try:
            if op == 'state':
                out = {**stack.state(), 'resets': stack.resets, 'mode': 'teleop'}
            elif op == 'stream':
                stack.stream([np.asarray(r, dtype=float) for r in req['rows']]); out = {}
            elif op == 'hold':
                stack.hold(); out = {'held': True}
            elif op == 'walk':
                out = stack.walk(float(req['forward_m']), float(req.get('turn_deg', 0.0)))
            elif op == 'head':
                with stack.lock:
                    stack.row[23] = float(req['pitch'])
                out = {'head_pitch': float(req['pitch'])}
            elif op in ('grasp_paw', 'place_in_box'):
                out = {'refused': 'not available on the real robot: use r1_leg_lift / r1_carry_release', 'held': False, 'in_box': False}
            elif op == 'shutdown':
                stack.send_command(start=False, stop=True); stop.set(); out = {}
                if stack.piper_right is not None:
                    stack.piper_right.stop()
            elif op == 'camera':
                raise NotImplementedError('the R1 head camera is not wired into the real bridge yet')
            else:
                raise ValueError(f'unknown op {op!r}')
            sock.send(json.dumps({'ok': True, **out}).encode())
        except Exception as exc:  # noqa: BLE001
            sock.send(json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'}).encode())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=5591, help='port the RAX backend connects to')
    ap.add_argument('--stack-in', type=int, default=6556, help='the deploy stack\'s zmq_port (it connects here; lab 6556)')
    ap.add_argument('--stack-out', type=int, default=6557, help='the deploy stack\'s zmq_out_port (telemetry PUB; lab 6557)')
    ap.add_argument('--out-topic', default='g1_debug', help='the deploy stack\'s zmq_out_topic')
    ap.add_argument('--piper-right', default=None,
                    help='command that runs MA-Teleop tools/r1/piper_can_bridge.py --serve for the right gripper, e.g. '
                         '"ssh -o BatchMode=yes unitree@192.168.123.164 python3 -u <MA-Teleop>/tools/r1/piper_can_bridge.py --serve"')
    ap.add_argument('--veer-deg', type=float, default=0.0,
                    help='left heading bias for forward walks if the robot veers (the RAX sim needed 20; the deploy sim 0)')
    a = ap.parse_args()
    serve(RealStack(a.stack_in, a.stack_out, a.out_topic, piper_right=a.piper_right, veer_deg=a.veer_deg), a.port)


if __name__ == '__main__':
    main()
