"""MCP tool schemas and request validation for the Unitree R1 + PiPER grippers under SONIC_R1 (teleop mode)."""
from __future__ import annotations

import math

PREFIX = "r1_"
STATUS, OBSERVE, PIXEL_TO_POSITION = "r1_status", "r1_observe", "r1_pixel_to_position"
MOVE_HAND, MOVE_JOINTS, GRIPPER, HEAD, HOLD = "r1_move_hand", "r1_move_joints", "r1_gripper", "r1_head", "r1_hold"
WALK = "r1_walk"
GRASP_PAW, PLACE_IN_BOX = "r1_grasp_paw", "r1_place_in_box"
ROBOT_TOOLS = (STATUS, OBSERVE, PIXEL_TO_POSITION, MOVE_HAND, MOVE_JOINTS, GRIPPER, HEAD, HOLD, WALK, GRASP_PAW, PLACE_IN_BOX)
READ_TOOLS = (STATUS, OBSERVE, PIXEL_TO_POSITION)
MUTATING = frozenset({MOVE_HAND, MOVE_JOINTS, GRIPPER, HEAD, WALK, GRASP_PAW, PLACE_IN_BOX})

ARMS = ("left", "right")
# joints the agent commands (the legs stand on SONIC's default stance; the head yaw stays 0)
UPPER = tuple(f"{s}_{j}_joint" for s in ARMS for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                                         "wrist_roll")) + ("waist_roll_joint", "waist_yaw_joint")
GRIPPER_MAX_M = 0.07
HEAD_PITCH = (-0.6, 0.6)
SECS = (0.5, 8.0)


class RejectedBeforeAction(ValueError):
    """A request refused before anything moved."""


def reject_unless(ok, message):
    if not ok:
        raise RejectedBeforeAction(message)


_number = {"type": "number"}
_vector = lambda n: {"type": "array", "items": _number, "minItems": n, "maxItems": n}  # noqa: E731


def tool_definitions(geometry=True):
    tools = [
        {"name": STATUS, "description": (
            "Arming, the link to the r1 stack, the measured joints (26, SONIC order) and limits, IMU orientation and tilt, "
            "both grippers' measured opening, both grasp points in the pelvis frame, what the robot is commanded to hold, "
            "the limits this backend enforces and the last motion's feedback. Moves nothing."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": OBSERVE, "description": (
            "A fresh head-camera RGB" + (" and depth" if geometry else "") + " frame with the camera pose in the pelvis "
            "frame (from the measured head and waist joints), the measured joints, IMU orientation, tilt, both grippers' "
            "opening and both grasp points (pelvis frame). No world position and no object state. Moves nothing."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": MOVE_HAND, "description": (
            "Move one gripper's grasp point (between the finger pads, 2 cm in from the fingertips) to `position` in the "
            "pelvis frame (metres; x forward, y left, z up from the pelvis). `approach` (unit vector, pelvis frame) is the "
            "direction the gripper points (from the wrist to the fingertips); `fingers` is the axis the two fingers "
            "open and close along (sign does not matter). Both are soft preferences: the arm has 5 joints (shoulder "
            "pitch/roll/yaw, elbow, wrist roll; no wrist pitch or yaw), so the gripper always points along the forearm "
            "and not every orientation exists at every position. The backend solves the inverse kinematics (use_waist "
            "lets waist roll/yaw help), refuses a goal the arm cannot reach within max_ik_error_m (the reply says how "
            "far off the best solution is), moves the joints smoothly over `secs`, then corrects the remaining error "
            "from the measured joints (servo). The whole-body controller tracks hand targets to a few cm and shifts the "
            "pelvis while the arm is out: positions you measured before the move are off by that much afterwards; "
            "re-observe. `gripper_width_m` sets the opening during the move. preview_only=true solves and reports "
            "without moving. Needs operator arming."),
         "inputSchema": {"type": "object", "required": ["arm", "position"], "additionalProperties": False, "properties": {
             "arm": {"type": "string", "enum": list(ARMS)},
             "position": _vector(3), "approach": _vector(3), "fingers": _vector(3),
             "use_waist": {"type": "boolean", "default": False},
             "secs": {"type": "number", "minimum": SECS[0], "maximum": SECS[1], "default": 2.0},
             "gripper_width_m": {"type": "number", "minimum": 0.0, "maximum": GRIPPER_MAX_M},
             "servo": {"type": "boolean", "default": True},
             "preview_only": {"type": "boolean", "default": False}}}},
        {"name": MOVE_JOINTS, "description": (
            "Move arm and waist joints to absolute angles (rad) over `secs`: {joint name: angle} for any of "
            + ", ".join(UPPER) + ". Joints not named keep their commanded angle. The controller is given the resulting "
            "wrist and torso poses (it tracks them, not the joints themselves, to a few degrees). Use it for tucked or "
            "home poses, wrist roll, and waist turns. Needs operator arming."),
         "inputSchema": {"type": "object", "required": ["joints"], "additionalProperties": False, "properties": {
             "joints": {"type": "object", "minProperties": 1, "additionalProperties": _number,
                        "propertyNames": {"enum": list(UPPER)}},
             "secs": {"type": "number", "minimum": SECS[0], "maximum": SECS[1], "default": 2.0}}}},
        {"name": GRIPPER, "description": (
            "Open or close one PiPER parallel gripper to `width_m` between the finger pads (0 closed .. 0.07 open) "
            "over `secs`, arm held. The reply's measured width: wider than commanded means something is between the "
            "fingers (about 12 N per finger). Needs operator arming."),
         "inputSchema": {"type": "object", "required": ["arm", "width_m"], "additionalProperties": False, "properties": {
             "arm": {"type": "string", "enum": list(ARMS)},
             "width_m": {"type": "number", "minimum": 0.0, "maximum": GRIPPER_MAX_M},
             "secs": {"type": "number", "minimum": 0.3, "maximum": 3.0, "default": 1.0}}}},
        {"name": HEAD, "description": (
            f"Tilt the head (and its camera): pitch {HEAD_PITCH[0]}..{HEAD_PITCH[1]} rad, positive looks down. The head "
            "yaw stays 0 (turn the waist to look sideways). Needs operator arming."),
         "inputSchema": {"type": "object", "required": ["pitch"], "additionalProperties": False, "properties": {
             "pitch": {"type": "number", "minimum": HEAD_PITCH[0], "maximum": HEAD_PITCH[1]},
             "secs": {"type": "number", "minimum": 0.3, "maximum": 3.0, "default": 1.0}}}},
        {"name": HOLD, "description": (
            "Drop any motion still streaming and keep the last command. Always allowed, armed or not. It is not an "
            "emergency stop."),
         "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
        {"name": WALK, "description": (
            "Walk on your legs to approach or reposition: forward_m metres along your current heading (negative = "
            "backward), with an optional turn_deg (left positive, right negative). Your legs step with a built-in gait "
            "while your arms hold their last commanded pose; you translate only by stepping. Blocks until you finish "
            "and are standing again, then reports the actual distance walked (walked_m), the new pelvis position and "
            "tilt. The commanded distance is approximate (0.2-0.5 m per call works best; larger overshoots), so "
            "re-observe and walk again to fine-tune. You move, so measure the scene again afterwards. Needs operator arming."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "forward_m": {"type": "number", "minimum": -1.0, "maximum": 1.0, "default": 0.0},
             "turn_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0, "default": 0.0}}}},
        {"name": GRASP_PAW, "description": (
            "Grasp the plush puppy by its nearest hind paw and lift it partially. One blocking call that locates the "
            "nearest hind leg, picks the arm on that side, servos the gripper onto the leg (width-feedback) and clamps, "
            "then lifts in small steps. Stand within reach of the puppy first (walk up so a near hind leg is ~0.25-0.35 m "
            "ahead of your pelvis). Reports which paw/arm (side), whether it is held, the clamped width, and the lift. "
            "If held is false, re-observe and reposition (a small walk/turn to present a hind leg cleanly) and try again. "
            "This is a FLAT table-top grasp of a paw resting on the table. It is NOT for a leg hanging past the front edge: "
            "for that, r1_move_hand in front of the hanging paw with the fingers vertical (0,0,1), approach ~(1,0,0.15), "
            "then r1_gripper width 0. It refuses (held=false, 'refused') if the nearest paw is more than 0.35 m ahead: "
            "leaning further has tipped the robot over. Needs operator arming."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {}}},
        {"name": PLACE_IN_BOX, "description": (
            "With a paw held (after r1_grasp_paw), lower the puppy over the box opening and release so it drops in. One "
            "blocking call. The box must be under your holding hand first: ask the operator to slide the table so the "
            "box comes under your hand (the puppy stays in your gripper), then call this. Reports in_box and the final "
            "puppy/box positions. Needs operator arming."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "side": {"type": "string", "enum": ["auto", "left", "right"], "default": "auto"}}}},
    ]
    if geometry:
        tools.append({"name": PIXEL_TO_POSITION, "description": (
            "Convert head-camera depth pixels of the current observation into 3-D points in the pelvis frame and in the "
            "level frame (origin at the pelvis, z up, x along the measured heading). Pass the observation's step_id; "
            "stale selections are rejected."),
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


def _vec(value, n, name, unit=False):
    reject_unless(isinstance(value, list) and len(value) == n, f"{name} must be a list of {n} numbers")
    for v in value:
        _finite(v, name)
    if unit:
        reject_unless(math.sqrt(sum(float(v) ** 2 for v in value)) > 1e-6, f"{name} must not be zero")


def _only(arguments, allowed):
    unknown = set(arguments) - set(allowed)
    reject_unless(not unknown, f"Unknown arguments: {sorted(unknown)}")


def _range(arguments, key, low, high, default=None):
    if key in arguments:
        _finite(arguments[key], key)
        reject_unless(low <= float(arguments[key]) <= high, f"{key} must be in [{low:g}, {high:g}]")


def validate_request(name, arguments):
    reject_unless(isinstance(arguments, dict), "arguments must be an object")
    if name in (STATUS, OBSERVE, HOLD):
        _only(arguments, ())
    elif name == MOVE_HAND:
        _only(arguments, ("arm", "position", "approach", "fingers", "use_waist", "secs", "gripper_width_m", "servo",
                          "preview_only"))
        reject_unless(arguments.get("arm") in ARMS, "arm must be left or right")
        _vec(arguments.get("position"), 3, "position")
        for key in ("approach", "fingers"):
            if key in arguments:
                _vec(arguments[key], 3, key, unit=True)
        for key in ("use_waist", "servo", "preview_only"):
            reject_unless(type(arguments.get(key, False)) is bool, f"{key} must be a boolean")
        _range(arguments, "secs", *SECS)
        _range(arguments, "gripper_width_m", 0.0, GRIPPER_MAX_M)
    elif name == MOVE_JOINTS:
        _only(arguments, ("joints", "secs"))
        joints = arguments.get("joints")
        reject_unless(isinstance(joints, dict) and joints, "joints must be a non-empty object")
        for k, v in joints.items():
            reject_unless(k in UPPER, f"{k} is not a commandable joint ({', '.join(UPPER)})")
            _finite(v, k)
        _range(arguments, "secs", *SECS)
    elif name == GRIPPER:
        _only(arguments, ("arm", "width_m", "secs"))
        reject_unless(arguments.get("arm") in ARMS, "arm must be left or right")
        reject_unless("width_m" in arguments, "width_m is required")
        _range(arguments, "width_m", 0.0, GRIPPER_MAX_M)
        _range(arguments, "secs", 0.3, 3.0)
    elif name == HEAD:
        _only(arguments, ("pitch", "secs"))
        reject_unless("pitch" in arguments, "pitch is required")
        _range(arguments, "pitch", *HEAD_PITCH)
        _range(arguments, "secs", 0.3, 3.0)
    elif name == PIXEL_TO_POSITION:
        _only(arguments, ("observation_step_id", "selections", "depth_window_radius"))
        reject_unless(type(arguments.get("observation_step_id")) is int, "observation_step_id must be an integer")
        selections = arguments.get("selections")
        reject_unless(isinstance(selections, list) and 1 <= len(selections) <= 32, "selections must have 1..32 items")
        for item in selections:
            reject_unless(isinstance(item, dict) and set(item) == {"u", "v"}, "each selection needs exactly u and v")
            _vec([item["u"], item["v"]], 2, "pixel")
        radius = arguments.get("depth_window_radius", 1)
        reject_unless(type(radius) is int and 0 <= radius <= 5, "depth_window_radius must be 0..5")
    elif name == WALK:
        _only(arguments, ("forward_m", "turn_deg"))
        _range(arguments, "forward_m", -1.0, 1.0)
        _range(arguments, "turn_deg", -180.0, 180.0)
    elif name == GRASP_PAW:
        _only(arguments, ())
    elif name == PLACE_IN_BOX:
        _only(arguments, ("side",))
        if "side" in arguments and arguments["side"] not in ("auto", "left", "right"):
            raise ValueError("side must be auto, left, or right")
    else:
        raise RejectedBeforeAction(f"Unknown tool: {name}")
