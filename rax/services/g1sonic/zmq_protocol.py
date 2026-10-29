"""The deploy stack's ZMQ wire formats, written out here so the robot service needs nothing from
the GR00T-WholeBodyControl checkout.

Pose and command messages (`g1_deploy_onnx_ref --input-type zmq`, PUB on the client, SUB on the
stack): `topic bytes + a 1280-byte JSON header padded with NUL + the fields' little-endian
binary data`. The header is `{"v": version, "endian": "le", "count": 1, "fields": [{"name",
"dtype", "shape"}, ...]}`. Protocol version 1 requires `joint_pos`, `joint_vel` (29, IsaacLab
order), `body_quat` (wxyz) and `frame_index`; `left_hand_joints` and `right_hand_joints` (7 each)
are optional. A command message has the topic `command` and u8 fields `start`, `stop`, `planner`.

State messages (PUB on the stack, SUB on the client): `topic bytes + one msgpack map`.
"""

from __future__ import annotations

import json

import numpy as np

HEADER_SIZE = 1280
POSE_TOPIC = "pose"
COMMAND_TOPIC = "command"
PLANNER_TOPIC = "planner"
IDLE, SLOW_WALK = 0, 1  # SONIC planner locomotion modes (localmotion_kplanner.hpp LocomotionMode)
STATE_TOPIC = "g1_debug"

_DTYPE_NAMES = {np.dtype("float32"): "f32", np.dtype("float64"): "f64", np.dtype("int32"): "i32",
                np.dtype("int64"): "i64", np.dtype("uint8"): "u8", np.dtype("bool"): "bool"}
_NAME_DTYPES = {name: dtype for dtype, name in _DTYPE_NAMES.items()}


def _header(fields, version, count=1):
    text = json.dumps({"v": int(version), "endian": "le", "count": int(count), "fields": fields},
                      separators=(",", ":")).encode()
    if len(text) > HEADER_SIZE:
        raise ValueError(f"ZMQ header too large: {len(text)} > {HEADER_SIZE}")
    return text.ljust(HEADER_SIZE, b"\x00")


def pack_fields(fields, *, topic, version, count=1):
    """One message of named arrays, in dictionary order."""
    described, blobs = [], []
    for name, value in fields.items():
        array = np.ascontiguousarray(value)
        if array.dtype not in _DTYPE_NAMES:
            array = np.ascontiguousarray(array, dtype=np.float32)
        if array.dtype.byteorder == ">":
            array = array.astype(array.dtype.newbyteorder("<"))
        described.append({"name": name, "dtype": _DTYPE_NAMES[array.dtype], "shape": list(array.shape)})
        blobs.append(array.tobytes())
    return topic.encode() + _header(described, version, count) + b"".join(blobs)


def pack_pose_message(fields, topic=POSE_TOPIC, version=1):
    return pack_fields(fields, topic=topic, version=version)


def build_command_message(start=False, stop=False, planner=False, delta_heading=None):
    """The stack's control command: start the control loop, stop it (it then damps every joint), or
    switch between the planner and the streamed motion."""
    fields = {"start": np.array([1 if start else 0], dtype=np.uint8),
              "stop": np.array([1 if stop else 0], dtype=np.uint8),
              "planner": np.array([1 if planner else 0], dtype=np.uint8)}
    if delta_heading is not None:
        fields["delta_heading"] = np.array([float(delta_heading)], dtype=np.float32)
    return pack_fields(fields, topic=COMMAND_TOPIC, version=1)


# The 17 upper-body joints a planner message may override (waist, then the arms interleaved left/right), as
# indices into the canonical29 (IsaacLab) order: policy_parameters.hpp upper_body_joint_isaaclab_order_in_isaaclab_index.
UPPER_BODY_INDICES = (2, 5, 8, 11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28)


def build_planner_message(mode, movement, facing, speed=-1.0, height=-1.0, upper_body=None, left_hand=None,
                          right_hand=None):
    """A planner command for the stack's locomotion planner (planner mode): locomotion `mode`, the movement
    and facing directions as 3-vectors in the planner's frame (x along the heading the robot had when the
    planner took over, y left), speed in m/s (-1: the mode's default) and height (-1: default).
    `upper_body` (17 joint positions in UPPER_BODY_INDICES order) replaces the planner's waist and arm motion,
    and `left_hand` / `right_hand` (7 Dex3 targets each) its hand targets: the robot walks with the arms and
    hands where they are (carrying)."""
    fields = {"mode": np.array([int(mode)], dtype=np.int32),
              "movement": np.asarray(movement, dtype=np.float32).reshape(3),
              "facing": np.asarray(facing, dtype=np.float32).reshape(3),
              "speed": np.array([float(speed)], dtype=np.float32),
              "height": np.array([float(height)], dtype=np.float32)}
    if upper_body is not None:
        fields["upper_body_position"] = np.asarray(upper_body, dtype=np.float32).reshape(17)
        fields["upper_body_velocity"] = np.zeros(17, dtype=np.float32)
    if left_hand is not None:
        fields["left_hand_joints"] = np.asarray(left_hand, dtype=np.float32).reshape(7)
    if right_hand is not None:
        fields["right_hand_joints"] = np.asarray(right_hand, dtype=np.float32).reshape(7)
    return pack_fields(fields, topic=PLANNER_TOPIC, version=1)


def parse_message(raw):
    """(topic, header, {name: array}) of a pose or command message."""
    start = raw.index(b"{")
    topic = raw[:start].decode()
    header = json.loads(raw[start:start + HEADER_SIZE].rstrip(b"\x00").decode())
    payload = raw[start + HEADER_SIZE:]
    fields, offset = {}, 0
    for field in header.get("fields", []):
        dtype = _NAME_DTYPES[field["dtype"]]
        count = int(np.prod(field["shape"])) if field["shape"] else 1
        size = count * dtype.itemsize
        fields[field["name"]] = np.frombuffer(payload[offset:offset + size], dtype=dtype).reshape(field["shape"]).copy()
        offset += size
    return topic, header, fields


def pack_state(state, topic=STATE_TOPIC):
    import msgpack
    return topic.encode() + msgpack.packb(state, use_bin_type=True)


def unpack_state(raw, topic=STATE_TOPIC):
    import msgpack
    return msgpack.unpackb(raw[len(topic):], raw=False)
