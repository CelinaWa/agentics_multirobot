"""From semantic40 frames to the deploy stack's reference stream.

`ReferenceConverter` does what HumanoidArena's action provider does to a frame before SONIC sees
it: the rot6d root rotation becomes a quaternion, left-multiplied by the heading the robot had when
the session's first motion was sent (so `[3:9]` is relative to that heading), a jump of more than
`max_root_delta_deg` from the previous target is slerped down to that limit, the joint velocities
are the tick-to-tick difference of the joint references, and each hand's 0/1 selects its open or
closed Dex3 pose.

`Streamer` owns the 50 Hz wire: it sends one reference per tick, the next queued row while a call
is executing and otherwise the last reference again with zero velocities (a hold). The stack keeps
the last frame it received, so a client that stops sending leaves the robot in its last pose; the
continuous stream also keeps the stack's sliding window fed. `execute` blocks until its rows are
on the wire or a tick hook stops it; `hold` drops the rows still queued.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import robot as R
from .zmq_protocol import (POSE_TOPIC, UPPER_BODY_INDICES, build_command_message, build_planner_message,
                           pack_pose_message)


# ----------------------------------------------------------------------------- quaternions (wxyz)
def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_multiply(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conjugate(q):
    return np.asarray(q, dtype=float) * [1.0, -1.0, -1.0, -1.0]


def matrix_from_quat(q):
    w, x, y, z = quat_normalize(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_from_matrix(m):
    m = np.asarray(m, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def rot6d_to_matrix(six):
    """The rotation whose first two columns, row-major, are SIX (Gram-Schmidt, like the provider)."""
    six = np.asarray(six, dtype=float)
    a = six[0::2]
    b = six[1::2]
    c1 = a / max(np.linalg.norm(a), 1e-9)
    b = b - np.dot(c1, b) * c1
    c2 = b / max(np.linalg.norm(b), 1e-9)
    c3 = np.cross(c1, c2)
    return np.column_stack([c1, c2, c3])


def rot6d_from_quat(q):
    return matrix_from_quat(q)[:, :2].reshape(-1)


def yaw_of(q):
    w, x, y, z = quat_normalize(q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def heading_quat(q):
    """The yaw-only part of Q: the robot's heading as a rotation about world z."""
    yaw = yaw_of(q)
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def angle_between(a, b):
    d = abs(float(np.dot(quat_normalize(a), quat_normalize(b))))
    return 2.0 * math.acos(min(1.0, d))


def slerp(a, b, t):
    a, b = quat_normalize(a), quat_normalize(b)
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        return quat_normalize(a + t * (b - a))
    theta = math.acos(d)
    return (math.sin((1 - t) * theta) * a + math.sin(t * theta) * b) / math.sin(theta)


def tilt_deg(q):
    """How far the body's z axis leans from vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, matrix_from_quat(q)[2, 2]))))


# ----------------------------------------------------------------------------- references
@dataclass
class Reference:
    """One tick on the wire."""
    joint_pos: np.ndarray          # 29, IsaacLab order, rad
    joint_vel: np.ndarray          # 29, rad/s
    body_quat: np.ndarray          # wxyz, world heading applied
    left_hand: np.ndarray          # 7 Dex3 targets
    right_hand: np.ndarray
    frame_index: int = 0
    projection_deg: float = 0.0    # the requested root jump this tick, before clamping
    hold: bool = False

    def fields(self, hands=True):
        fields = {"joint_pos": self.joint_pos.reshape(1, 29).astype(np.float32),
                  "joint_vel": self.joint_vel.reshape(1, 29).astype(np.float32),
                  "body_quat": self.body_quat.reshape(1, 4).astype(np.float32),
                  "frame_index": np.array([self.frame_index], dtype=np.int64)}
        if hands:
            fields["left_hand_joints"] = self.left_hand.reshape(1, 7).astype(np.float32)
            fields["right_hand_joints"] = self.right_hand.reshape(1, 7).astype(np.float32)
        return fields


class ReferenceConverter:
    def __init__(self, *, max_root_delta_deg=26.0, control_dt=R.CONTROL_DT):
        self.max_root_delta = math.radians(max_root_delta_deg)
        self.dt = control_dt
        self.anchor = None
        self.prev_target = None
        self.prev_joints = None
        self.last = None
        self.hand_shapes = {"left": None, "right": None}  # (open, closed) Dex3 targets; None: R.HAND_POSES

    def hand(self, side, closed):
        """The 7 Dex3 targets of SIDE for a closing fraction (or 7 explicit targets), on its current shape."""
        if np.ndim(closed) == 1 and np.size(closed) == 7:
            return np.asarray(closed, dtype=np.float32).copy()
        shape = self.hand_shapes.get(side)
        if shape is None:
            return R.hand_target(side, closed)
        fraction = float(np.clip(float(closed), 0.0, 1.0))
        return (shape[0] + fraction * (shape[1] - shape[0])).astype(np.float32)

    def hand_fraction(self, side, targets):
        """The closing fraction a held hand target encodes on SIDE's current shape (0 open .. 1 closed)."""
        open_, close = self.hand(side, 0.0), self.hand(side, 1.0)
        span = close - open_
        denominator = float(np.dot(span, span))
        if denominator < 1e-9:
            return 0.0
        return float(np.clip(np.dot(np.asarray(targets) - open_, span) / denominator, 0.0, 1.0))

    @property
    def anchored(self):
        return self.anchor is not None

    def clear(self):
        """Forget the anchor and the hold: the next prime starts from a fresh measured state."""
        self.anchor = None
        self.prev_target = None
        self.prev_joints = None
        self.last = None

    def reset(self, measured_root_quat):
        """Anchor the frames' heading to the robot's current heading; forget the previous target."""
        self.anchor = heading_quat(measured_root_quat)
        self.prev_target = quat_normalize(measured_root_quat)
        self.prev_joints = None
        self.last = None

    def prime(self, joints, measured_root_quat, frame_index=0, hands=(False, False)):
        """A hold reference built from the measured state, for the wire before any command: the measured
        joints at the measured heading, body upright (a measured lean is not held: it would keep the robot
        tilted)."""
        if self.anchor is None:
            self.reset(measured_root_quat)
        joints = np.asarray(joints, dtype=float).reshape(29)
        self.prev_joints = joints.copy()
        self.prev_target = heading_quat(measured_root_quat)
        self.last = Reference(joints, np.zeros(29), self.prev_target.copy(), self.hand("left", hands[0]),
                              self.hand("right", hands[1]), frame_index, 0.0, True)
        return self.last

    def convert(self, row, frame_index):
        if self.anchor is None:
            raise RuntimeError("the heading is not anchored: reset from a measured root orientation first")
        row = np.asarray(row, dtype=float).reshape(R.ACTION_DIM)
        relative = quat_from_matrix(rot6d_to_matrix(row[R.ROT6D]))
        target = quat_multiply(self.anchor, relative)
        projection = 0.0
        if self.prev_target is not None:
            delta = angle_between(self.prev_target, target)
            projection = math.degrees(delta)
            if delta > self.max_root_delta:
                target = slerp(self.prev_target, target, self.max_root_delta / delta)
        joints = row[R.JOINTS].copy()
        velocity = np.zeros(29) if self.prev_joints is None else (joints - self.prev_joints) / self.dt
        self.prev_target, self.prev_joints = target, joints
        self.last = Reference(joints, velocity, target, self.hand("left", row[38]),
                              self.hand("right", row[39]), frame_index, projection, False)
        return self.last

    def hold(self, frame_index):
        last = self.last
        if last is None:
            return None
        self.last = Reference(last.joint_pos.copy(), np.zeros(29), last.body_quat.copy(), last.left_hand.copy(),
                              last.right_hand.copy(), frame_index, 0.0, True)
        return self.last

    def relative_rot6d(self, measured_root_quat):
        """The [3:9] block that commands the measured orientation (for frames built from the state)."""
        if self.anchor is None:
            return np.array(R.IDENTITY_ROT6D)
        return rot6d_from_quat(quat_multiply(quat_conjugate(self.anchor), quat_normalize(measured_root_quat)))


# ----------------------------------------------------------------------------- the wire
class ZmqPublisher:
    """The client side of the stack's pose socket: PUB bound here, the stack's SUB connects."""

    def __init__(self, host="127.0.0.1", port=5556, context=None):
        import zmq
        self.context = context or zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        if port:
            self.socket.bind(f"tcp://{host}:{int(port)}")
            self.port = int(port)
        else:
            self.port = self.socket.bind_to_random_port(f"tcp://{host}")
        self.host = host
        self.lock = threading.Lock()

    def send(self, message):
        with self.lock:
            self.socket.send(message)

    def close(self):
        self.socket.close(0)


class Streamer:
    def __init__(self, send, converter, *, hz=R.CONTROL_HZ, clock=time.monotonic, sleep=time.sleep,
                 topic=POSE_TOPIC, version=1, hands=True, threaded=True):
        self.send, self.converter = send, converter
        self.dt, self.clock, self.sleep = 1.0 / hz, clock, sleep
        self.topic, self.version, self.hands = topic, version, hands
        self.threaded = threaded
        self.frame_index = 0
        self.sent = self.held = 0
        self.last = None
        self.mirror_upper = False  # see tick()
        # while the robot is left under the walking planner between calls: a callable sending its idle planner message,
        # called every 5th tick (the deploy resets a planner silent for 1 s to its reference motion's frame 0: a lunge)
        self.keepalive = None
        self._pending = deque()
        self._executed = []
        self._hook = None
        self._error = None
        self._done = threading.Event()
        self._done.set()
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    @property
    def executing(self):
        return not self._done.is_set()

    @property
    def executed(self):
        """The references of the current or last call that reached the wire."""
        with self._lock:
            return list(self._executed)

    @property
    def primed(self):
        return self.converter.last is not None

    def start(self):
        if self.threaded and self._thread is None:
            self._running = True
            self._thread = threading.Thread(target=self._run, name="g1sonic-stream", daemon=True)
            self._thread.start()
        return self

    def _run(self):
        n = 0
        while self._running:
            started = self.clock()
            self.tick()
            keepalive = self.keepalive
            if keepalive is not None and n % 5 == 0:
                try:
                    keepalive()
                except Exception:
                    pass
            n += 1
            self.sleep(max(0.0, self.dt - (self.clock() - started)))

    def prime(self, joints, measured_root_quat, hands=(False, False)):
        with self._lock:
            self.converter.prime(joints, measured_root_quat, self.frame_index, hands)

    def tick(self):
        """One period: the next queued row, else the last reference held."""
        hook = None
        with self._lock:
            row = self._pending.popleft() if self._pending else None
            if row is None:
                reference = self.converter.hold(self.frame_index)
                if reference is None:
                    return None
                self.held += 1
            else:
                reference = self.converter.convert(row, self.frame_index)
            self.send(pack_pose_message(reference.fields(self.hands), self.topic, self.version))
            if self.mirror_upper:
                # after a carrying walk the deploy keeps taking waist and arm targets from planner messages even in
                # streamed motion (has_upper_body_control_ stays set): mirror the streamed upper body there
                self.send(build_planner_message(0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], -1.0,
                                                upper_body=np.asarray(reference.joint_pos, float)[list(UPPER_BODY_INDICES)]))
            self.frame_index += 1
            self.sent += 1
            self.last = reference
            if row is not None:
                self._executed.append(reference)
                hook = self._hook
            finished = row is not None and not self._pending
        if hook is not None:
            try:
                hook(reference)
            except Exception as error:  # a tick hook stops the call: the remaining rows are dropped
                with self._lock:
                    self._pending.clear()
                    self._error = error
                    self._hook = None
                self._done.set()
                return reference
        if finished:
            with self._lock:
                self._hook = None
            self._done.set()
        return reference

    def execute(self, rows, on_tick=None, timeout=None):
        """Stream ROWS one per tick; the references sent. ON_TICK runs after each sent row and may
        raise to stop the call; its exception is re-raised here after the stream holds."""
        rows = [np.asarray(row, dtype=float) for row in rows]
        if not rows:
            return []
        with self._lock:
            if not self._done.is_set():
                raise RuntimeError("another motion is still streaming")
            self._pending = deque(rows)
            self._executed, self._error, self._hook = [], None, on_tick
            self._done.clear()
        if not self.threaded:
            while not self._done.is_set():
                self.tick()
        elif not self._done.wait(len(rows) * self.dt + (5.0 if timeout is None else timeout)):
            with self._lock:
                self._pending.clear()
                self._hook = None
            self._done.set()
            raise RuntimeError("the reference stream stalled: no tick for the queued rows")
        with self._lock:
            error, executed = self._error, list(self._executed)
        if error is not None:
            raise error
        return executed

    def hold(self):
        """Drop the rows still queued; the last reference keeps going out."""
        with self._lock:
            dropped = len(self._pending)
            self._pending.clear()
            self._hook = None
        self._done.set()
        return dropped

    def command(self, *, start=False, stop=False, planner=False):
        self.send(build_command_message(start=start, stop=stop, planner=planner))

    def planner(self, mode, movement, facing, speed=-1.0, height=-1.0, upper_body=None, left_hand=None, right_hand=None):
        self.send(build_planner_message(mode, movement, facing, speed, height, upper_body, left_hand, right_hand))

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
