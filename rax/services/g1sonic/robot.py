"""G1 + Dex3 semantic40 action contract for the real robot (dependency-light: numpy only).

The action is HumanoidArena's canonical 40-value whole-body frame at 50 Hz, tracked by the frozen
SONIC whole-body controller running in NVIDIA's C++ deploy stack. Layout and semantics follow
HumanoidArena's action provider (`isaaclab_twist2_g1/action_provider`): the controller receives
the 29 joint angles [9:38], their tick-to-tick change as joint velocities, and the root rotation
[3:9] relative to the heading the robot had when the session's first motion was sent. Root XY
[0:2] and height [2] are part of the layout but do not move the robot: it translates only by
stepping, through the leg joint references.

Joint orders: semantic40 and the reference stream use the IsaacLab (canonical29) order; the
deploy stack publishes measured joints in MuJoCo order, which `to_isaaclab` undoes.
"""

from __future__ import annotations

import math

import numpy as np

ROBOT = "g1sonic"
CONTROL_HZ = 50
CONTROL_DT = 1.0 / CONTROL_HZ
ACTION_DIM = 40
# Rows per motion call: 2 s at 50 Hz.
MAX_ACTION_ROWS = 100

ACTION_SLICES = {
    "root_xy_delta_m": slice(0, 2),
    "root_height_m": slice(2, 3),
    "root_rot6d": slice(3, 9),
    "joints_canonical29_rad": slice(9, 38),
    "hands_closed": slice(38, 40),
}
JOINTS = ACTION_SLICES["joints_canonical29_rad"]
HANDS = ACTION_SLICES["hands_closed"]
ROT6D = ACTION_SLICES["root_rot6d"]
# Yaw-free identity rotation: the first two columns of the 3x3 identity, row-major.
IDENTITY_ROT6D = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)

# IsaacLab articulation order of the 29 body joints (HumanoidArena's canonical29).
JOINT_NAMES = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
)
assert len(JOINT_NAMES) == JOINTS.stop - JOINTS.start

# MuJoCo (motor) order of the same joints: left leg, right leg, waist, left arm, right arm.
MUJOCO_JOINT_NAMES = tuple(
    [f"left_{n}_joint" for n in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
    + [f"right_{n}_joint" for n in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
    + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
    + [f"left_{n}_joint" for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                    "wrist_roll", "wrist_pitch", "wrist_yaw")]
    + [f"right_{n}_joint" for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                     "wrist_roll", "wrist_pitch", "wrist_yaw")])
# isaaclab[i] = mujoco[MUJOCO_TO_ISAACLAB[i]]; the same table as the lab's g1_sonic_client.py.
MUJOCO_TO_ISAACLAB = tuple(MUJOCO_JOINT_NAMES.index(name) for name in JOINT_NAMES)
assert MUJOCO_TO_ISAACLAB == (0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25,
                              19, 26, 20, 27, 21, 28)


def to_isaaclab(vector_mujoco):
    """A 29-vector in MuJoCo order as the same joints in IsaacLab order."""
    return np.asarray(vector_mujoco, dtype=float)[list(MUJOCO_TO_ISAACLAB)]


def to_mujoco(vector_isaaclab):
    out = np.empty(29)
    out[list(MUJOCO_TO_ISAACLAB)] = np.asarray(vector_isaaclab, dtype=float)
    return out


# Dex3 joints, in the deploy stack's hand order (thumb, middle, index), and the two poses the binary
# command maps to (HumanoidArena's DEFAULT_HAND_POSE for unitree_g1).
HAND_JOINT_NAMES = {
    side: tuple(f"{side}_hand_{name}_joint" for name in (
        "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"))
    for side in ("left", "right")}
HAND_POSES = {
    "left": {"open": (0.0,) * 7, "close": (0.0, 1.0, 1.74, -1.57, -1.74, -1.57, -1.74)},
    "right": {"open": (0.0,) * 7, "close": (0.0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74)},
}


# Dex3 joint ranges (rad) per side, in HAND_JOINT_NAMES order (g1_29dof_with_hand.xml).
HAND_LIMITS = {
    "left": ((-1.0472, 1.0472), (-0.724312, 1.0472), (0.0, 1.74533), (-1.5708, 0.0), (-1.74533, 0.0),
             (-1.5708, 0.0), (-1.74533, 0.0)),
    "right": ((-1.0472, 1.0472), (-1.0472, 0.724312), (-1.74533, 0.0), (0.0, 1.5708), (0.0, 1.74533),
              (0.0, 1.5708), (0.0, 1.74533)),
}


def hand_target(side, closed):
    """The 7 Dex3 joint targets of SIDE: `closed` 0 is open, 1 the fist, a fraction in between interpolates."""
    fraction = float(np.clip(float(closed), 0.0, 1.0))
    open_, close = (np.array(HAND_POSES[side][key], dtype=np.float32) for key in ("open", "close"))
    return (open_ + fraction * (close - open_)).astype(np.float32)


# Agent-facing camera: the ZED on the head, served by the robot computer.
CAMERAS = ("cam_front",)


class RejectedBeforeAction(ValueError):
    """A request refused before anything moves; the agent sees it as a validation failure."""


def reject_unless(condition, message):
    if not condition:
        raise RejectedBeforeAction(message)


def action_layout():
    return {name: [s.start, s.stop] for name, s in ACTION_SLICES.items()}


def validate_actions(actions):
    """Shape and value checks of semantic40 rows; raises RejectedBeforeAction."""
    reject_unless(isinstance(actions, list) and 1 <= len(actions) <= MAX_ACTION_ROWS,
                  f"actions must be a list of 1..{MAX_ACTION_ROWS} semantic40 rows")
    for index, row in enumerate(actions):
        reject_unless(isinstance(row, list) and len(row) == ACTION_DIM,
                      f"actions[{index}] must be a list of {ACTION_DIM} numbers")
        for value in row:
            reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
                          f"actions[{index}] must contain finite numbers")
        reject_unless(all(0 <= row[i] <= 1 for i in range(HANDS.start, HANDS.stop)),
                      f"actions[{index}][38:40] (hand commands) must be between 0 (open) and 1 (closed)")


def rot6d_from_yaw(yaw_rad):
    """The [3:9] block for a pure heading change, relative to the session's anchored heading."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [c, -s, s, c, 0.0, 0.0]


# SONIC's default standing pose (gear_sonic_deploy policy_parameters.hpp `default_angles`, MuJoCo order):
# the pose the deploy stack ramps the robot to in INIT and holds until Start.
SONIC_DEFAULT_ANGLES_MUJOCO = (-0.312, 0.0, 0.0, 0.669, -0.363, 0.0, -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                               0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, 0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0)
DEFAULT_STANDING = to_isaaclab(np.array(SONIC_DEFAULT_ANGLES_MUJOCO, dtype=float))
