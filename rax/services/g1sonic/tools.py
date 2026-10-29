"""MCP tool schemas and request validation for the real G1 under SONIC: HumanoidArena's tools under
the `g1_` prefix, plus `g1_hold`. Pure Python + numpy; validation runs before anything reaches
the wire."""

from __future__ import annotations

import math

import numpy as np

from . import robot as R
from .robot import RejectedBeforeAction, reject_unless

PREFIX = "g1_"
STATUS, OBSERVE, STEP = "g1_status", "g1_observe", "g1_step"
FREE_SPACE_MOVE, EXECUTE_PLAN = "g1_free_space_move", "g1_execute_motion_plan"
PIXEL_TO_POSITION, HOLD = "g1_pixel_to_position", "g1_hold"
WALK = "g1_walk"
HAND_SHAPE = "g1_hand_shape"

ROBOT_TOOLS = (STATUS, OBSERVE, STEP, FREE_SPACE_MOVE, EXECUTE_PLAN, PIXEL_TO_POSITION, HOLD, WALK, HAND_SHAPE)
GEOMETRY_TOOLS = frozenset({PIXEL_TO_POSITION})
MUTATING = frozenset({STEP, FREE_SPACE_MOVE, EXECUTE_PLAN, WALK})
WALK_LIMITS = {"forward_m": (0.0, 3.0), "turn_deg": (-45.0, 45.0), "speed_mps": (0.1, 0.6)}
WALK_FLAGS = ("carry",)
READ_TOOLS = (STATUS, OBSERVE, PIXEL_TO_POSITION)
BODY_FRAMES = ("pelvis", "torso")

_number = {"type": "number"}
_vector = lambda n: {"type": "array", "items": _number, "minItems": n, "maxItems": n}  # noqa: E731


def tool_definitions(geometry=True):
    layout = ", ".join(f"[{s.start}:{s.stop}] {name}" for name, s in R.ACTION_SLICES.items())
    tools = [
        {"name": STATUS, "description": (
            "Arming, the link to the SONIC deploy stack, the measured joints, root orientation and tilt, the "
            "semantic40 action layout, joint names and limits, the limits this backend enforces and the last "
            "motion's feedback. Moves nothing."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": OBSERVE, "description": (
            "A fresh front-camera RGB" + (" and depth" if geometry else "") + " frame with the camera's pose in the "
            "pelvis frame, the measured root orientation, the 29 joint positions and velocities in canonical "
            "order and the Dex3 finger state. No world position and no object state. Moves nothing."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": STEP, "description": (
            f"Stream 1..{R.MAX_ACTION_ROWS} semantic40 reference frames to the robot, one per {R.CONTROL_HZ} Hz tick "
            f"(at most {R.MAX_ACTION_ROWS / R.CONTROL_HZ:g} s): {layout}. The frozen SONIC whole-body controller "
            "tracks only the 29 joint angles (canonical order, radians), their tick-to-tick change and the root "
            "rotation [3:9] (first two rotation-matrix columns, row-major, relative to the heading the robot had "
            "when the session's first motion was sent; identity [1,0,0,1,0,0]). Root XY [0:2] and height [2] are "
            "required by the layout but do not move the robot: it translates only by stepping. Hands [38:40] are "
            "0 (open) to 1 (closed), a fraction closing every finger that far. References are not motor targets: read the measured feedback. After the "
            "last row the robot holds that row until the next call, so end every call in a pose to hold (the "
            "last rows identical). Needs operator arming; a disarm, a stale link or a large tilt stops the call."),
         "inputSchema": {"type": "object", "required": ["actions"], "additionalProperties": False, "properties": {
             "actions": {"type": "array", "minItems": 1, "maxItems": R.MAX_ACTION_ROWS, "items": _vector(R.ACTION_DIM)}}}},
        {"name": FREE_SPACE_MOVE, "description": (
            "cuRobo motion plan of one arm (use_waist=true: the three waist joints too) to a hand pose in a body "
            "frame (pelvis or torso; metres, quaternion wxyz, or position only when the quaternion is omitted). "
            "end_effector selects the palm, the hand centre or the wrist. Avoids self-collision only: scene objects "
            "are unknown to the planner. Executes the plan as 50 Hz semantic40 frames with the other joints, the "
            "hands and the root rotation held at their last commanded values; legs do not step. preview_only=true "
            "returns the plan and a motion_plan_id without moving. Needs operator arming to execute."),
         "inputSchema": {"type": "object", "required": ["arm", "position"], "additionalProperties": False,
             "properties": {
                 "arm": {"type": "string", "enum": ["left", "right"]},
                 "position": _vector(3), "quaternion_wxyz": _vector(4),
                 "frame": {"type": "string", "enum": list(BODY_FRAMES), "default": "pelvis"},
                 "end_effector": {"type": "string", "enum": ["palm", "wrist", "hand_center"], "default": "palm"},
                 "use_waist": {"type": "boolean", "default": False},
                 "time_scale": {"type": "number", "minimum": 1.0, "maximum": 4.0, "default": 1.0},
                 "preview_only": {"type": "boolean", "default": False}}}},
        {"name": EXECUTE_PLAN, "description": "Execute a previewed motion plan if the robot has not moved since planning.",
         "inputSchema": {"type": "object", "required": ["motion_plan_id"], "additionalProperties": False,
             "properties": {"motion_plan_id": {"type": "string", "minLength": 1, "maxLength": 64}}}},
        {"name": WALK, "description": (
            "Walk with SONIC's own locomotion planner: real stepping, as a teleoperator's joystick drives the robot. "
            "Moves forward_m along the current heading (forward only: the planner does not step backward or "
            "sideways; to go elsewhere, turn first, then walk forward) and turns turn_deg (positive = left, at most "
            "45 per call: turn further in several calls) while walking; a turn alone steps slowly forward about 0.1-0.2 m, "
            "so leave room ahead. Speed speed_mps (slow walk, "
            "0.1-0.6 m/s, default 0.3), then stops and "
            "stands. Distances are commanded, not measured: the robot has no odometry, so re-localize against the "
            "scene afterwards. Without carry, arms and hands follow the planner during the walk (they swing; "
            "anything held is dropped). With carry=true the waist, both arms and both hands stay at the last "
            "commanded frame for the whole walk and afterwards (carrying a held object). The last frame is held "
            "again afterwards from the measured pose (g1_step continues from there). Needs operator arming; a "
            "disarm, a stale link or a large tilt stops the walk and the robot stands."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "forward_m": {"type": "number", "minimum": 0.0, "maximum": 3.0},
             "turn_deg": {"type": "number", "minimum": -45, "maximum": 45},
             "speed_mps": {"type": "number", "minimum": 0.1, "maximum": 0.6},
             "carry": {"type": "boolean", "default": False}}}},
        {"name": HAND_SHAPE, "description": (
            "Set the Dex3 joint targets a hand's closing fraction ([38] left, [39] right in g1_step rows) "
            "interpolates between: `open` (fraction 0) and `closed` (fraction 1), 7 values each in hand_state "
            "joint order (thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1; radians, within each "
            "joint's range; the left hand mirrors the right's signs). Omit both to restore the default open/fist "
            "shapes. Moves nothing: the held hand keeps its targets until the next g1_step. Shapes persist for "
            "the session and g1_status reports them. Example right-hand pre-curled pinch: open [0, 0.4, 0, 0.5, 0, "
            "0.5, 0], closed [0, -1.0, -1.74, 1.57, 1.74, 1.57, 1.74]."),
         "inputSchema": {"type": "object", "required": ["side"], "additionalProperties": False, "properties": {
             "side": {"type": "string", "enum": ["left", "right"]},
             "open": _vector(7), "closed": _vector(7)}}},
        {"name": HOLD, "description": (
            "Hold: drop any motion still streaming and keep the robot at its last reference. Always allowed, "
            "armed or not. It is not an emergency stop: the operator's stop (keyboard O or the gamepad's Select "
            "at the robot) damps the joints and the robot sits down."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    ]
    if geometry:
        tools.append({"name": PIXEL_TO_POSITION, "description": (
            "Convert front-camera depth pixels of the current observation into 3-D points in the pelvis frame and in "
            "the level frame (origin at the pelvis, z up, x along the measured heading). No world position. Pass "
            "the observation's step_id; stale selections are rejected."),
         "inputSchema": {"type": "object", "required": ["observation_step_id", "selections"], "additionalProperties": False,
             "properties": {
                 "observation_step_id": {"type": "integer", "minimum": 0},
                 "selections": {"type": "array", "minItems": 1, "maxItems": 32, "items": {
                     "type": "object", "required": ["u", "v"], "additionalProperties": False, "properties": {
                         "u": {"type": "number"}, "v": {"type": "number"}}}},
                 "depth_window_radius": {"type": "integer", "minimum": 0, "maximum": 5, "default": 1}}}})
    return tools


def _finite(value, name):
    reject_unless(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
                  f"{name} must be a finite number")


def _finite_vector(value, n, name):
    reject_unless(isinstance(value, list) and len(value) == n, f"{name} must be a list of {n} numbers")
    for item in value:
        _finite(item, name)


def _only(arguments, allowed):
    unknown = set(arguments) - set(allowed)
    reject_unless(not unknown, f"Unknown arguments: {sorted(unknown)}")


def validate_request(name, arguments):
    """Complete schema/shape validation before dispatch; raises RejectedBeforeAction."""
    reject_unless(isinstance(arguments, dict), "arguments must be an object")
    if name in (STATUS, OBSERVE, HOLD):
        _only(arguments, ())
    elif name == STEP:
        _only(arguments, ("actions",))
        R.validate_actions(arguments.get("actions"))
    elif name == FREE_SPACE_MOVE:
        _only(arguments, ("arm", "position", "quaternion_wxyz", "frame", "end_effector", "use_waist",
                          "time_scale", "preview_only"))
        reject_unless(arguments.get("arm") in ("left", "right"), "arm must be left or right")
        _finite_vector(arguments.get("position"), 3, "position")
        if "quaternion_wxyz" in arguments:
            _finite_vector(arguments["quaternion_wxyz"], 4, "quaternion_wxyz")
            norm = math.sqrt(sum(float(v) ** 2 for v in arguments["quaternion_wxyz"]))
            reject_unless(abs(norm - 1.0) < 1e-3, "quaternion_wxyz must be unit length")
        reject_unless(arguments.get("frame", "pelvis") in BODY_FRAMES, f"frame must be one of {list(BODY_FRAMES)}")
        reject_unless(arguments.get("end_effector", "palm") in ("palm", "wrist", "hand_center"),
                      "end_effector must be palm, wrist or hand_center")
        for key in ("use_waist", "preview_only"):
            reject_unless(type(arguments.get(key, False)) is bool, f"{key} must be a boolean")
        scale = arguments.get("time_scale", 1.0)
        _finite(scale, "time_scale")
        reject_unless(1.0 <= scale <= 4.0, "time_scale must be in [1, 4]")
    elif name == HAND_SHAPE:
        _only(arguments, ("side", "open", "closed"))
        side = arguments.get("side")
        reject_unless(side in ("left", "right"), "side must be left or right")
        reject_unless(("open" in arguments) == ("closed" in arguments), "give both open and closed, or neither")
        for key in ("open", "closed"):
            if key in arguments:
                _finite_vector(arguments[key], 7, key)
                for value, (low, high), joint in zip(arguments[key], R.HAND_LIMITS[side], R.HAND_JOINT_NAMES[side]):
                    reject_unless(low - 1e-6 <= float(value) <= high + 1e-6,
                                  f"{key}: {joint} {float(value):.3f} is outside [{low:.3f}, {high:.3f}]")
    elif name == WALK:
        _only(arguments, tuple(WALK_LIMITS) + WALK_FLAGS)
        reject_unless(type(arguments.get("carry", False)) is bool, "carry must be a boolean")
        for key, (low, high) in WALK_LIMITS.items():
            if key in arguments:
                _finite(arguments[key], key)
                reject_unless(low <= float(arguments[key]) <= high, f"{key} must be in [{low:g}, {high:g}]")
        reject_unless(any(abs(float(arguments.get(k, 0.0))) > 1e-6 for k in ("forward_m", "turn_deg")),
                      "g1_walk needs a nonzero forward_m or turn_deg")
    elif name == EXECUTE_PLAN:
        _only(arguments, ("motion_plan_id",))
        plan = arguments.get("motion_plan_id")
        reject_unless(isinstance(plan, str) and 0 < len(plan) <= 64, "motion_plan_id must be a string")
    elif name == PIXEL_TO_POSITION:
        _only(arguments, ("observation_step_id", "selections", "depth_window_radius"))
        reject_unless(type(arguments.get("observation_step_id")) is int, "observation_step_id must be an integer")
        selections = arguments.get("selections")
        reject_unless(isinstance(selections, list) and 1 <= len(selections) <= 32, "selections must have 1..32 items")
        for item in selections:
            reject_unless(isinstance(item, dict) and set(item) == {"u", "v"}, "each selection needs exactly u and v")
            _finite_vector([item["u"], item["v"]], 2, "pixel")
        radius = arguments.get("depth_window_radius", 1)
        reject_unless(type(radius) is int and 0 <= radius <= 5, "depth_window_radius must be 0..5")
    else:
        raise RejectedBeforeAction(f"Unknown tool: {name}")


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v - x*i - y*j - z*k, w*i + x*v + y*k - z*j, w*j - x*k + y*v + z*i, w*k + x*j - y*i + z*v])


def _rotate(q, p):
    conjugate = np.asarray(q, dtype=np.float64) * [1, -1, -1, -1]
    return _multiply(_multiply(q, np.r_[0.0, p]), conjugate)[1:]


def level_from_pelvis(point, root_orientation_wxyz):
    """Pelvis-frame point -> level frame (origin at the pelvis, z up, x along the measured heading)."""
    q = np.asarray(root_orientation_wxyz, dtype=np.float64)
    w, x, y, z = q / np.linalg.norm(q)
    heading = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    world_axes = _rotate((w, x, y, z), np.asarray(point, dtype=np.float64))
    return _rotate((math.cos(heading / 2), 0.0, 0.0, -math.sin(heading / 2)), world_axes)


def pixel_to_pelvis(depth_m, intrinsics, pose, u, v, radius=1):
    """Back-project one front-camera pixel into the pelvis frame. Depth is distance along the optical
    axis; the camera pose has X forward along the optical axis, Y left, Z up. Uses the median valid
    depth in a (2r+1)^2 window."""
    depth = np.asarray(depth_m, dtype=np.float64)
    height, width = depth.shape[:2]
    reject_unless(0 <= u < width and 0 <= v < height, f"pixel ({u}, {v}) outside {width}x{height}")
    iu, iv = int(round(u)), int(round(v))
    window = depth[max(0, iv - radius):iv + radius + 1, max(0, iu - radius):iu + radius + 1]
    valid = window[np.isfinite(window) & (window > 0)]
    if valid.size == 0:
        return {"valid": False, "reason": "no valid depth near the pixel"}
    distance = float(np.median(valid))
    K = np.asarray(intrinsics["matrix"], dtype=np.float64)
    x_cv = (u - K[0, 2]) / K[0, 0] * distance
    y_cv = (v - K[1, 2]) / K[1, 1] * distance
    point_camera = np.array([distance, -x_cv, -y_cv])
    point = _rotate(pose["quaternion_wxyz"], point_camera) + np.asarray(pose["position_m"], dtype=np.float64)
    return {"valid": True, "position_pelvis_m": point.tolist(), "depth_m": distance,
            "depth_spread_m": float(valid.max() - valid.min())}
