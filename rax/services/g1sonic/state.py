"""The deploy stack's state stream: the msgpack map it publishes every control tick on its ZMQ
state port (`--zmq-out-port`, topic `g1_debug`). Joints arrive in MuJoCo order and are turned
into the canonical IsaacLab order here; `last_action` is the decoder's joint target, what the
tracking error is measured against."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from . import robot as R
from .bridge import tilt_deg, yaw_of
from .zmq_protocol import STATE_TOPIC, unpack_state


@dataclass
class Snapshot:
    received: float
    index: int
    base_quat: np.ndarray
    base_ang_vel: np.ndarray
    q: np.ndarray                      # 29, IsaacLab order
    dq: np.ndarray
    last_action: np.ndarray | None     # 29, IsaacLab order; the policy's joint targets
    left_hand_q: np.ndarray
    right_hand_q: np.ndarray
    motor_temperature: np.ndarray | None
    raw: dict = field(default_factory=dict, repr=False)

    def tilt_deg(self):
        return tilt_deg(self.base_quat)

    def yaw_rad(self):
        return yaw_of(self.base_quat)


def _vector(state, key, size, default=None):
    value = state.get(key)
    if value is None or len(value) != size:
        return default
    return np.asarray(value, dtype=float)


def snapshot_from_state(state, received, joint_order="mujoco"):
    reorder = R.to_isaaclab if joint_order == "mujoco" else (lambda v: np.asarray(v, dtype=float))
    q = _vector(state, "body_q", 29)
    if q is None:
        raise ValueError("state message without body_q")
    action = _vector(state, "last_action", 29)
    temperature = state.get("motor_temperature")
    return Snapshot(
        received=received, index=int(state.get("index", 0)),
        base_quat=_vector(state, "base_quat", 4, np.array([1.0, 0.0, 0.0, 0.0])),
        base_ang_vel=_vector(state, "base_ang_vel", 3, np.zeros(3)),
        q=reorder(q), dq=reorder(_vector(state, "body_dq", 29, np.zeros(29))),
        last_action=None if action is None else reorder(action),
        left_hand_q=_vector(state, "left_hand_q", 7, np.zeros(7)),
        right_hand_q=_vector(state, "right_hand_q", 7, np.zeros(7)),
        motor_temperature=None if temperature is None else np.asarray(temperature, dtype=float),
        raw=state)


class StateSubscriber:
    def __init__(self, host="127.0.0.1", port=5557, topic=STATE_TOPIC, *, joint_order="mujoco",
                 clock=time.monotonic, context=None):
        self.host, self.port, self.topic, self.joint_order, self.clock = host, int(port), topic, joint_order, clock
        self.context = context
        self._latest = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._running = False
        self._thread = None
        self._socket = None
        self.received = 0
        self.errors = 0

    def start(self):
        import zmq
        self.context = self.context or zmq.Context.instance()
        self._socket = self.context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.connect(f"tcp://{self.host}:{self.port}")
        self._socket.setsockopt(zmq.SUBSCRIBE, self.topic.encode())
        self._running = True
        self._thread = threading.Thread(target=self._receive, name="g1sonic-state", daemon=True)
        self._thread.start()
        return self

    def _receive(self):
        import zmq
        poller = zmq.Poller()
        poller.register(self._socket, zmq.POLLIN)
        while self._running:
            if not dict(poller.poll(100)):
                continue
            try:
                raw = self._socket.recv(zmq.NOBLOCK)
                snapshot = snapshot_from_state(unpack_state(raw, self.topic), self.clock(), self.joint_order)
            except Exception:
                self.errors += 1
                continue
            with self._lock:
                self._latest = snapshot
                self.received += 1
            self._event.set()

    def push(self, state):
        """A state delivered by hand (tests)."""
        with self._lock:
            self._latest = snapshot_from_state(state, self.clock(), self.joint_order)
            self.received += 1
        self._event.set()

    def latest(self):
        with self._lock:
            return self._latest

    def age(self):
        snapshot = self.latest()
        return math.inf if snapshot is None else self.clock() - snapshot.received

    def wait(self, timeout):
        if self.latest() is None:
            self._event.wait(timeout)
        return self.latest()

    def wait_after(self, received, timeout):
        """The first snapshot received after the clock stamp `received`, or None after `timeout`."""
        deadline = self.clock() + timeout
        while True:
            snapshot = self.latest()
            if snapshot is not None and snapshot.received > received:
                return snapshot
            if self.clock() >= deadline:
                return None
            time.sleep(0.02)

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._socket is not None:
            self._socket.close(0)
            self._socket = None
