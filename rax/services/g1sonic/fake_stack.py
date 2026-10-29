"""A stand-in for `g1_deploy_onnx_ref` on the wire, for tests and dry runs without a robot: it
subscribes to the pose socket, publishes a state map in the stack's format every tick, tracks the
joint references with a first-order lag and turns its heading toward the reference quaternion.
It moves nothing and knows no physics; its tracking error is the lag, nothing else."""

from __future__ import annotations

import threading
import time

import numpy as np

from . import robot as R
from .bridge import quat_normalize, slerp
from .zmq_protocol import POSE_TOPIC, STATE_TOPIC, pack_state, parse_message


class FakeSonicStack:
    def __init__(self, *, host="127.0.0.1", state_port=0, hz=R.CONTROL_HZ, tau_s=0.08, joints=None,
                 base_quat=(1.0, 0.0, 0.0, 0.0), context=None):
        import zmq
        self.context = context or zmq.Context.instance()
        self.host, self.dt, self.tau = host, 1.0 / hz, tau_s
        self.state_socket = self.context.socket(zmq.PUB)
        self.state_socket.setsockopt(zmq.LINGER, 0)
        self.state_port = int(state_port) or self.state_socket.bind_to_random_port(f"tcp://{host}")
        if state_port:
            self.state_socket.bind(f"tcp://{host}:{int(state_port)}")
        self.pose_socket = None
        self.q = np.zeros(29) if joints is None else np.asarray(joints, dtype=float).copy()
        self.dq = np.zeros(29)
        self.base_quat = quat_normalize(base_quat)
        self.target = None           # the latest reference (joint_pos, body_quat, hands)
        self.planner_mode = True     # the deploy starts in planner mode; command planner=False streams poses
        self.planners = []           # planner messages: (mode, movement, facing, speed)
        self.planner_extras = []     # their optional upper-body and hand fields, by name
        self.planner_yaw0 = 0.0      # heading when the planner took over (its frame's x axis)
        self.facing_yaw = None       # the planner's commanded facing, as a world yaw
        self.hands = {"left": np.zeros(7), "right": np.zeros(7)}
        self.received = []           # (topic, fields) in arrival order
        self.commands = []
        self.control_active = False
        self.damped = False
        self.tilt_override_deg = None  # tests: report this tilt instead of the heading-only quaternion
        self.index = 0
        self.published = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def connect_pose(self, host, port, topics=(POSE_TOPIC, "command", "planner")):
        import zmq
        self.pose_socket = self.context.socket(zmq.SUB)
        self.pose_socket.setsockopt(zmq.LINGER, 0)
        self.pose_socket.connect(f"tcp://{host}:{int(port)}")
        for topic in topics:
            self.pose_socket.setsockopt(zmq.SUBSCRIBE, topic.encode())
        return self

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name="fake-sonic-stack", daemon=True)
        self._thread.start()
        return self

    def _drain(self):
        import zmq
        if self.pose_socket is None:
            return
        while True:
            try:
                raw = self.pose_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                return
            topic, _header, fields = parse_message(raw)
            with self._lock:
                self.received.append((topic, fields))
                if topic == "command":
                    command = {name: bool(fields[name].reshape(-1)[0]) for name in ("start", "stop", "planner")
                               if name in fields}
                    self.commands.append(command)
                    if command.get("start"):
                        self.control_active, self.damped = True, False
                    if command.get("stop"):
                        self.control_active, self.damped = False, True
                    if "planner" in command and command["planner"] != self.planner_mode:
                        self.planner_mode = command["planner"]
                        w, x, y, z = self.base_quat
                        self.planner_yaw0 = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
                        self.facing_yaw = None
                elif topic == "planner":
                    movement, facing = (np.asarray(fields[k], dtype=float).reshape(3) for k in ("movement", "facing"))
                    self.planners.append((int(fields["mode"].reshape(-1)[0]), movement, facing,
                                          float(fields["speed"].reshape(-1)[0])))
                    self.planner_extras.append({k: np.asarray(fields[k], dtype=float).reshape(-1) for k in (
                        "upper_body_position", "left_hand_joints", "right_hand_joints") if k in fields})
                    self.facing_yaw = self.planner_yaw0 + float(np.arctan2(facing[1], facing[0]))
                elif "joint_pos" in fields:
                    quat = fields.get("body_quat", fields.get("body_quat_w"))
                    self.target = (np.asarray(fields["joint_pos"], dtype=float).reshape(-1)[:29],
                                   None if quat is None else quat_normalize(np.asarray(quat, dtype=float).reshape(-1)[:4]))
                    for side in ("left", "right"):
                        key = f"{side}_hand_joints"
                        if key in fields:
                            self.hands[side] = np.asarray(fields[key], dtype=float).reshape(-1)[:7]

    def restart(self, joints=None):
        """The deploy relaunched: INIT's pose, no control, index from 0, nothing published until a start."""
        with self._lock:
            self.control_active, self.damped = False, False
            self.target = None
            self.index = 0
            if joints is not None:
                self.q = np.asarray(joints, dtype=float).copy()
            self.dq = np.zeros(29)

    def step(self):
        """One tick of the fake: drain the wire, track, publish (state flows only while control runs)."""
        self._drain()
        with self._lock:
            if not self.control_active and not self.damped:
                return
            if self.planner_mode and self.facing_yaw is not None and not self.damped:  # the planner turns the body
                w, x, y, z = self.base_quat
                yaw = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
                error = float(np.arctan2(np.sin(self.facing_yaw - yaw), np.cos(self.facing_yaw - yaw)))
                yaw += float(np.clip(error, -0.05, 0.05))
                self.base_quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
            if self.target is not None and not self.damped and not self.planner_mode:
                alpha = min(1.0, self.dt / self.tau)
                joints, quat = self.target
                previous = self.q.copy()
                self.q = self.q + alpha * (joints - self.q)
                self.dq = (self.q - previous) / self.dt
                if quat is not None:
                    self.base_quat = slerp(self.base_quat, quat, alpha)
            base_quat = self.base_quat
            if self.tilt_override_deg is not None:
                half = np.radians(self.tilt_override_deg) / 2
                base_quat = quat_normalize([np.cos(half), 0.0, np.sin(half), 0.0])
            target = self.q if self.target is None else self.target[0]
            state = {"control_loop_type": "fake", "index": self.index, "ros_timestamp": time.time(),
                     "base_quat": base_quat.tolist(), "base_ang_vel": [0.0, 0.0, 0.0],
                     "body_torso_quat": base_quat.tolist(), "body_torso_ang_vel": [0.0, 0.0, 0.0],
                     "body_q": R.to_mujoco(self.q).tolist(), "body_dq": R.to_mujoco(self.dq).tolist(),
                     "left_hand_q": self.hands["left"].tolist(), "left_hand_dq": [0.0] * 7,
                     "right_hand_q": self.hands["right"].tolist(), "right_hand_dq": [0.0] * 7,
                     "last_action": R.to_mujoco(target).tolist(),
                     "last_left_hand_action": self.hands["left"].tolist(),
                     "last_right_hand_action": self.hands["right"].tolist(),
                     "token_state": [], "motor_temperature": [35.0] * 29,
                     "fake": {"control_active": self.control_active, "damped": self.damped}}
            self.index += 1
        self.state_socket.send(pack_state(state, STATE_TOPIC))
        self.published += 1

    def _run(self):
        while self._running:
            started = time.monotonic()
            self.step()
            time.sleep(max(0.0, self.dt - (time.monotonic() - started)))

    def poses(self):
        with self._lock:
            return [fields for topic, fields in self.received if topic == POSE_TOPIC]

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for socket in (self.pose_socket, self.state_socket):
            if socket is not None:
                socket.close(0)
