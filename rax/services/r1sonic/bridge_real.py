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
    return struct.pack('<%df' % len(values), *[float(v) for v in values])


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
                 piper_right: str | None = None):
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

    # ---- deploy-stack commands ----
    def send_command(self, start: bool, stop: bool) -> None:
        self.pub.send(packed_message(b'command', [('start', 'u8', [1], struct.pack('B', int(start))),
                                                   ('stop', 'u8', [1], struct.pack('B', int(stop))),
                                                   ('planner', 'u8', [1], struct.pack('B', 0))]))

    def send_pose(self, row: np.ndarray) -> None:
        """One protocol-1 pose frame: the standing lower body + the row's three points (encoder mode 1, teleop)."""
        pos = np.concatenate([row[0:3], row[7:10], row[14:17]])          # L wrist, R wrist, torso (reference frame)
        quat = np.concatenate([row[3:7], row[10:14], row[17:21]])        # wxyz, as the deploy expects
        fields = [('joint_pos', 'f32', [1, 26], f32(ISAAC_NOMINAL)),
                  ('joint_vel', 'f32', [1, 26], f32(np.zeros(26))),
                  ('body_quat', 'f32', [1, 4], f32([1.0, 0.0, 0.0, 0.0])),
                  ('frame_index', 'i64', [1], struct.pack('<q', self.frame)),
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
                'fallen': bool(_tilt_deg(rel) > 60.0), 'tilt_deg': round(_tilt_deg(rel), 2)}

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
        if not self.started or self.frame % 50 == 0:
            self.send_command(start=True, stop=False); self.started = True
        self.send_pose(row)
        if self.piper_right is not None:
            self.piper_right.tick(float(row[22]))
        # TODO: left gripper (no verified adapter); head pitch from row[23]

    # ---- locomotion ----
    def walk(self, forward_m: float, turn_deg: float) -> dict:
        """Not available yet. R1 has no planner, so walking means streaming a lower-body walking reference. Tested in
        the deploy-stack simulation 2026-11-06: the RAX sim's parametric gait fell within ~1 s (full and half
        amplitude, with and without look-ahead frames), and the deploy's own recorded clip
        reference/r1_example/Neutral_walk_forward_002__A057 walked ~1.4 m and then fell, in both encoder modes."""
        raise NotImplementedError('walking is not available on the real R1 bridge yet: no walking reference has been '
                                  'shown to keep the robot up (stand where the target is in reach instead)')


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
    a = ap.parse_args()
    serve(RealStack(a.stack_in, a.stack_out, a.out_topic, piper_right=a.piper_right), a.port)


if __name__ == '__main__':
    main()
