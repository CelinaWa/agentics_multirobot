"""Real-robot bridge: serves the r1 stack protocol that the RAX r1sonic backend speaks (JSON over ZMQ REQ/REP,
see stack.py) on top of the real SONIC_R1 deployment (gear_sonic_deploy / g1_deploy_onnx_ref, ZMQManager input,
ZMQOutputHandler telemetry).

STATUS 2026-11-06: written from the deploy-stack sources, NOT yet run against the stack. Every item marked TODO/VERIFY
is robot-day work. Validate with r1/tools/diag/carry_chain_test.py pointed at this bridge's --port before any RAX
session (it drives the backend exactly as the agent does).

What the deploy stack expects (src/g1/g1_deploy_onnx_ref, include/input_interface/zmq_manager.hpp):
  * It CONNECTS a SUB socket to the sender at zmq_host:zmq_port (default localhost:5556); the sender BINDS a PUB.
    Messages are  topic bytes + 1280-byte zero-padded JSON header + packed little-endian payload.
    Header: {"v": 1, "endian": "le", "count": 1, "fields": [{"name", "dtype", "shape"}, ...]}.
  * topic "command": {start u8, stop u8, planner u8, delta_heading? f32}. planner=1 selects PLANNER mode, which is
    the mode that reads the "planner" topic below (it carries the VR 3-point reference as optional fields).
  * topic "planner" (every tick, timeout 1 s): mode i32 (LocomotionMode: IDLE=0, SLOW_WALK=1, WALK=2 ...),
    movement f32[3] (unit direction), facing f32[3] (unit), speed f32 (-1 = default), height f32 (-1 = default),
    optional vr_position f32[9] (left wrist, right wrist, chest xyz), vr_orientation f32[12] (3 quaternions),
    vr_compliance f32[3]. The stack copies vr_position straight into the policy observation: "the sender has
    already applied offsets and root normalization" (g1_deploy_onnx_ref.cpp GatherVR3PointPosition). The RAX
    stream rows are exactly that: positions and quaternions relative to the reference root (origin = reference
    root position, axes = reference heading), the same transform the sim's teleop_input applies. So a row maps
    1:1 onto vr_position + vr_orientation. VERIFY on the robot: quaternion order (the sim uses wxyz) and the
    chest point convention (sim: waist_yaw_link, IsaacLab body 6; vr_3point_index = {25, 26, 6}).
  * telemetry: ZMQOutputHandler BINDS a PUB on zmq_out_port (default 5557), topic zmq_out_topic (default
    "g1_debug"), msgpack map every tick with body_q, body_dq, base_quat, base_ang_vel, body_torso_quat,
    left_hand_q, right_hand_q, last_action, index, ... VERIFY the joint order (Isaac order expected by the backend:
    SONIC_R1_100k/validation/r1_deployment_contract.json) and the quaternion order.
  * Grippers: the R1's PiPER grippers are not part of the policy I/O; command them through the PiPER SDK (TODO).
  * Odometry: the telemetry carries no base position. walk() below dead-reckons from the commanded speed and
    duration and the measured heading; replace by a better source when one exists (TODO).

Run on the robot PC:  python -m services.r1sonic.bridge_real --port 5591 --stack-in 5556 --stack-out 5557
then point the RAX backend options at it: {"stack_host": "<robot pc>", "stack_port": 5591}.
"""
from __future__ import annotations

import argparse
import json
import struct
import threading
import time

import numpy as np
import zmq

try:
    import msgpack
except ImportError:  # the telemetry decoder needs it on the robot PC
    msgpack = None

HEADER_SIZE = 1280                      # ZMQPackedMessageSubscriber::HEADER_SIZE
IDLE, SLOW_WALK = 0, 1                  # LocomotionMode (localmotion_kplanner.hpp)
CONTROL_HZ = 50.0
TELEOP_COMPLIANCE = (0.5, 0.5, 0.0)     # initial_vr_3point_compliance_ in the deploy source


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


class RealStack:
    """Adapter onto the deploy stack over its two ZMQ ports."""

    def __init__(self, stack_in_port: int, stack_out_port: int, out_topic: str, bind_host: str = '*'):
        self.ctx = zmq.Context.instance()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind(f'tcp://{bind_host}:{stack_in_port}')
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, out_topic)
        self.sub.connect(f'tcp://127.0.0.1:{stack_out_port}')   # VERIFY: host of the deploy process
        self.out_topic = out_topic.encode()
        self.lock = threading.Lock()
        self.row = np.zeros(24)                 # last reference row (24 floats, see stack.py)
        self.queue: list[np.ndarray] = []
        self.resets = 0
        self.walking = None                     # (direction unit xy, speed, t_end) while a walk is in progress
        self.heading0 = None                    # reference heading (yaw at start), set on the first telemetry
        self.odom = np.zeros(3)                 # dead-reckoned pelvis position in the reference frame (TODO)
        self.last = None                        # last decoded telemetry
        self.joint_names = None                 # TODO: 26 names in Isaac order from the deployment contract
        self.started = False

    # ---- deploy-stack commands ----
    def send_command(self, start: bool, stop: bool, planner: bool = True) -> None:
        self.pub.send(packed_message(b'command', [('start', 'u8', [1], struct.pack('B', int(start))),
                                                   ('stop', 'u8', [1], struct.pack('B', int(stop))),
                                                   ('planner', 'u8', [1], struct.pack('B', int(planner)))]))

    def send_planner(self, mode: int, movement, facing, speed: float, row: np.ndarray) -> None:
        pos = np.concatenate([row[0:3], row[7:10], row[14:17]])          # L wrist, R wrist, chest (reference frame)
        quat = np.concatenate([row[3:7], row[10:14], row[17:21]])        # VERIFY wxyz vs xyzw on the robot
        fields = [('mode', 'i32', [1], struct.pack('<i', int(mode))),
                  ('movement', 'f32', [3], f32(movement)),
                  ('facing', 'f32', [3], f32(facing)),
                  ('speed', 'f32', [1], f32([speed])),
                  ('height', 'f32', [1], f32([-1.0])),
                  ('vr_position', 'f32', [9], f32(pos)),
                  ('vr_orientation', 'f32', [12], f32(quat)),
                  ('vr_compliance', 'f32', [3], f32(TELEOP_COMPLIANCE))]
        self.pub.send(packed_message(b'planner', fields))

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
                'gripper_width_m': {'left': _gripper_width(t.get('left_hand_q')), 'right': _gripper_width(t.get('right_hand_q'))},
                'command_row': np.round(self.row, 5).tolist(), 'queued': len(self.queue),
                'measured_points_row': np.round(self.row[:21], 5).tolist(),      # TODO: FK of the measured joints
                'fallen': bool(_tilt_deg(rel) > 60.0), 'tilt_deg': round(_tilt_deg(rel), 2)}

    # ---- reference streaming at 50 Hz ----
    def stream(self, rows: list[np.ndarray]) -> None:
        with self.lock:
            self.queue = rows

    def hold(self) -> None:
        with self.lock:
            self.queue = []

    def tick(self) -> None:
        with self.lock:
            if self.queue:
                self.row = self.queue.pop(0)
            walking = self.walking
            row = self.row.copy()
        if not self.started:
            self.send_command(start=True, stop=False, planner=True); self.started = True
        if walking is not None and time.monotonic() < walking[2]:
            d = walking[0]
            self.send_planner(SLOW_WALK, [d[0], d[1], 0.0], [d[0], d[1], 0.0], walking[1], row)
            self.odom[:2] += np.asarray(d) * walking[1] / CONTROL_HZ        # dead reckoning (TODO: real odometry)
        else:
            self.send_planner(IDLE, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], -1.0, row)
        # TODO: PiPER grippers from row[21] (left width) and row[22] (right width); head pitch from row[23]

    # ---- locomotion ----
    def walk(self, forward_m: float, turn_deg: float) -> dict:
        """Timed SLOW_WALK along the current heading (dead-reckoned), then wait until the gyro is quiet.
        VERIFY the movement vector's frame (robot heading vs world) and the achieved speed; turning is TODO
        (facing vector or the command topic's delta_heading)."""
        speed = 0.15                                                        # m/s, SLOW_WALK range 0.1-0.8
        s = self.state(); yaw = _yaw_wxyz(np.asarray(s['base_quat_wxyz']))
        direction = np.array([np.cos(yaw), np.sin(yaw)]) * (1.0 if forward_m >= 0 else -1.0)
        p0 = self.odom.copy()
        with self.lock:
            self.walking = (direction, speed, time.monotonic() + abs(forward_m) / speed)
        time.sleep(abs(forward_m) / speed + 0.2)
        with self.lock:
            self.walking = None
        t1 = time.monotonic(); still = None
        while time.monotonic() - t1 < 5.0:
            time.sleep(0.05)
            s = self.state()
            if float(np.linalg.norm(s['gyro'])) < 0.1:
                still = still or time.monotonic()
                if time.monotonic() - still > 0.5:
                    break
            else:
                still = None
        walked = float(np.dot(self.odom[:2] - p0[:2], direction)) * (1.0 if forward_m >= 0 else -1.0)
        return {'requested_m': forward_m, 'walked_m': round(walked, 3), 'turned_deg': 0.0, 'lateral_m': 0.0,
                'pelvis': self.odom.round(3).tolist(), 'tilt_deg': s['tilt_deg'], 'fallen': s['fallen'], 'passes': 1}


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


def _gripper_width(hand_q):
    """PiPER: width = 2 * finger slide + 0.004 in the sim model. TODO: the real SDK reports the width directly."""
    if not hand_q:
        return 0.07
    return float(2 * abs(hand_q[0]) + 0.004)


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
            else:
                raise ValueError(f'unknown op {op!r}')
            sock.send(json.dumps({'ok': True, **out}).encode())
        except Exception as exc:  # noqa: BLE001
            sock.send(json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'}).encode())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=5591, help='port the RAX backend connects to')
    ap.add_argument('--stack-in', type=int, default=5556, help='the deploy stack\'s zmq_port (it connects here)')
    ap.add_argument('--stack-out', type=int, default=5557, help='the deploy stack\'s zmq_out_port (telemetry PUB)')
    ap.add_argument('--out-topic', default='g1_debug', help='the deploy stack\'s zmq_out_topic')
    a = ap.parse_args()
    serve(RealStack(a.stack_in, a.stack_out, a.out_topic), a.port)


if __name__ == '__main__':
    main()
