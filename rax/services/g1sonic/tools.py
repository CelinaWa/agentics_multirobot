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
HANDS_WALK = "g1_hands_walk"
PLANNER = "g1_planner"
HAND_SHAPE = "g1_hand_shape"
CROUCH = "g1_crouch"

ROBOT_TOOLS = (STATUS, OBSERVE, STEP, FREE_SPACE_MOVE, EXECUTE_PLAN, PIXEL_TO_POSITION, HOLD, WALK, HAND_SHAPE, CROUCH,
               HANDS_WALK, PLANNER)
GEOMETRY_TOOLS = frozenset({PIXEL_TO_POSITION})
MUTATING = frozenset({STEP, FREE_SPACE_MOVE, EXECUTE_PLAN, WALK, CROUCH, HANDS_WALK, PLANNER})
# (backward and sideways only where the backend runs with walk_any_direction: the backend refuses them otherwise)
WALK_LIMITS = {"forward_m": (-1.0, 3.0), "left_m": (-1.0, 1.0), "turn_deg": (-45.0, 45.0), "speed_mps": (0.1, 0.6),
               "carry_press_rad": (0.0, 0.4)}
WALK_FLAGS = ("carry",)
HANDS_WALK_LIMITS = {"forward_m": (-1.5, 3.0), "left_m": (-1.5, 1.5), "turn_deg": (-60.0, 60.0), "speed_mps": (0.25, 0.6),
                     "press_m": (0.0, 0.25)}
HAND_POINT_BOX = ((-0.1, 0.7), (-0.6, 0.6), (-0.5, 0.6))   # pelvis frame x, y, z a hand point target may lie in
CARRY_OFFSET_JOINTS = frozenset(f"{side}_{name}_joint" for side in ("left", "right") for name in (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"))
# g1_crouch: pelvis drop below the standing height -> knee flexion added to the default stance (rad), with the
# ankle pitched down by half of it (to its -0.5 rad margin) and the hip taking the rest, so the torso stays
# upright. Measured in simulation (3/eng/squat_depth.py): the pelvis sat down beyond +2.0 rad.
CROUCH_TABLE = ((0.0, 0.0), (0.074, 0.52), (0.19, 1.2), (0.304, 1.8), (0.326, 2.0))
CROUCH_LIMITS = {"pelvis_drop_m": (0.0, 0.32), "lean_rad": (0.0, 0.5), "forward_rad": (-0.2, 0.3), "secs": (2.0, 10.0)}
CROUCH_FLAGS = ("arms_home",)
STANDING_PELVIS_M = 0.77          # pelvis height above the floor standing under SONIC (sim ground truth after a gantry
                                  # release; the kinematic estimate built on it was within 1.5 cm of ground truth)
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
            "stands. The reply's feedback.odometry_m is the leg odometry (planted-foot kinematics + IMU) of the walk, good to a few cm "
            "forward (sim); when the backend runs with walk_odometry the walk itself stops on it, otherwise distances are "
            "only commanded. Short walks are not a positioning tool: in sim (12 walks of 0.12 m from standing, arms tucked, "
            "carry) a short command either stepped in place without advancing (8 of 12, every one at 0.2 m/s) or, once "
            "striding, went on 0.25-0.36 m before stopping, odometry stop included (the planner needs a stride or two to "
            "stop). Walk toward a table or an obstacle only when it is at least 0.6 m ahead of the pelvis; closer, gain "
            "reach with the waist or ask the operator to bring the robot or the table closer. Either way re-localize against the "
            "scene afterwards. The first walk after a start begins with a planner warm-up (a 15 deg turn left, then "
            "right, about 10 s) before the commanded motion; the reply says so. Without carry, arms and hands follow "
            "the planner during the walk (they swing; "
            "anything held is dropped). With carry=true the waist, both arms and both hands stay at the last "
            "commanded frame for the whole walk and afterwards (carrying a held object). The last frame is held "
            "again afterwards from the measured pose (g1_step continues from there). The arms are held at their MEASURED "
            "joint angles: a press past contact (an object squeezed between the two hands) is not in them, and such an "
            "object drops as the walk begins. carry_press_rad adds that press back: both shoulder rolls are held this "
            "much further inward than measured (start from the press your squeeze commanded past contact). carry_offsets_rad does the same for any arm joint, by name (e.g. {\"left_shoulder_pitch_joint\": -0.24, \"left_elbow_joint\": -0.2}): each is held that far from its measured angle, signed, on top of carry_press_rad. Under load the measured arm sits below its command; the difference commanded minus measured just before the walk is the effort that holds the load, and without it the load sinks as the walk begins. If the robot is still stepping "
            "a few seconds after the walk, the planner keeps balancing it (safe) and the reply's feedback.control says so: "
            "call g1_hold, which tries for about 20 s to bring it to rest and take over (then re-observe: the robot "
            "moved); if it reports the robot still stepping, ask the operator instead of polling. Backward (forward_m < 0) and sideways (left_m) "
            "movement, facing kept, works only where the operator enabled it for the session; elsewhere it is refused. Needs operator "
            "arming; a disarm, a stale link or a large tilt stops the walk and the robot stands."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "forward_m": {"type": "number", "minimum": -1.0, "maximum": 3.0},
             "left_m": {"type": "number", "minimum": -1.0, "maximum": 1.0},
             "turn_deg": {"type": "number", "minimum": -45, "maximum": 45},
             "speed_mps": {"type": "number", "minimum": 0.1, "maximum": 0.6},
             "carry": {"type": "boolean", "default": False},
             "carry_press_rad": {"type": "number", "minimum": 0.0, "maximum": 0.4, "default": 0.0},
             "carry_offsets_rad": {"type": "object", "additionalProperties": {"type": "number", "minimum": -0.5, "maximum": 0.5}}}}},
        {"name": HANDS_WALK, "description": (
            "Walk, turn or stand with the two hands held where you put them: SONIC's own teleoperation mode, in which "
            "its planner moves the legs (real stepping, any direction: forward_m, left_m, both signed; turn_deg, "
            "positive = left) and its policy keeps each hand point at a target fixed in the pelvis frame (x forward, y "
            "left, z up, metres; a hand point is 0.18 m along the wrist link's x axis from the wrist; status and "
            "every reply give the measured ones as hand_points_pelvis). left_hand / right_hand set new targets ([x, y, "
            "z]; omitted: unchanged, and on the first call the measured points); the hands move there at about 8 cm/s "
            "(entering while already holding something: give press_m in that first call and no hand targets; the "
            "arms keep their last commanded joints through the switch and the press is on from the first moment) "
            "before any walking, keeping the wrist orientation they had when this hold began. press_m moves both "
            "targets that far toward each other past where you set them: against an object between the palms the "
            "hands cannot follow, and the difference is the squeeze that holds it (reply: hand_target_error_m; press_m is "
            "per hand, and is refused if the two targets would come within 0.10 m of each other). The planner does not "
            "step below about 0.3 m/s (0.1-0.2 m/s: no movement in 6 calls of 6), so speed_mps starts at 0.25; a "
            "walk that runs out of time before its leg odometry covers the distance says stalled: true. The "
            "walk may be given in the entering call itself (press_m with forward_m / left_m / turn_deg): a hold left "
            "standing sank and was lost within 8-43 s in S36, before any walk began. With "
            "no distance and no turn the call only moves the hands (squeeze, release, raise, lower). Measured in "
            "simulation: a 1.2 kg box (0.31 m between the palms, targets 0.15 m inside its walls) stayed held over "
            "0.8 m backward, 1.2 m left, 1.4 m right, 1.2 m forward and a 20 deg turn in one go, slipping 4 cm; "
            "hands alone stay within 2-5 cm of their targets while walking; a walk covers its distance by leg "
            "odometry (0.5 m asked: 0.45-0.54 m; 0.8 m forward: 0.71-0.81 m), a sideways walk also drifts up to 0.3 m "
            "backward (re-localize), and a 40 deg turn reached 36-38 deg. After the call the robot stays in this hold (the planner balances "
            "it and may shift its feet); the next g1_hands_walk continues from the same targets. g1_hold or any other "
            "G1 motion tool hands back to the streamed hold of the measured pose: the press is gone then, so let go "
            "of or set down what you hold first. Needs operator arming; a disarm, a stale link or a large tilt stops it."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "forward_m": {"type": "number", "minimum": -1.5, "maximum": 3.0},
             "left_m": {"type": "number", "minimum": -1.5, "maximum": 1.5},
             "turn_deg": {"type": "number", "minimum": -60, "maximum": 60},
             "speed_mps": {"type": "number", "minimum": 0.25, "maximum": 0.6, "default": 0.3},
             "left_hand": _vector(3), "right_hand": _vector(3),
             "press_m": {"type": "number", "minimum": 0.0, "maximum": 0.25}}}},
        {"name": PLANNER, "description": (
            "SONIC's locomotion planner, raw: this message is sent to the deploy stack 20 times a second for `seconds` "
            "(at most 10), with nothing added (no distance, no odometry stop, no ramp). Fields, as the stack defines "
            "them: mode 'idle' (stand, turn in place toward facing) or 'slow_walk' (step along movement); movement "
            "[x, y], a direction in the planner frame (x along the heading the robot had when the planner took over "
            "from the streamed hold, y left; the reply gives heading_in_planner_frame_deg); facing_deg, the heading to "
            "face in that frame; speed_mps (-1: the mode's default; slow walk 0.1-0.6); height_m (-1: default). "
            "Optional upper-body control, one of two: (a) hand_points {left, right, head: [x, y, z]} in the pelvis "
            "frame (x forward, y left, z up; a hand point is 0.18 m along the wrist link's x axis, the head point 0.35 m "
            "above the torso link; omitted keys: the measured ones) with hand_quats {left, right, head: [w, x, y, z]} "
            "(omitted: the measured ones): this is the policy's teleoperation encoder (VR 3-point), the planner moves "
            "the legs and the policy brings the three points to those poses; hand_points_end moves them linearly "
            "there over the call. (b) upper_body: 17 joint angles (waist yaw, roll, pitch, then shoulder pitch, "
            "shoulder roll, shoulder yaw, elbow, wrist roll, wrist pitch, wrist yaw, each left then right; g1_observe "
            "lists them as planner_upper_body_joint_names) held literally in the full-body encoder. With neither, the planner swings the "
            "arms. Known facts about the stack: it enters the teleoperation encoder only when hand points FIRST appear "
            "after messages without them (after an earlier use, send at least 0.2 s without hand_points first, or the "
            "hands follow the planner's arms); without a message for 1 s it resets, so after the call the last "
            "message repeats with mode idle until your next g1_planner call; short slow walks from standing may "
            "step in place for a stride or two before advancing. g1_hold or g1_step hands back to the streamed hold "
            "of the measured pose once the robot stands still. Needs operator arming; a disarm, a stale link or a "
            "large tilt stops it."),
         "inputSchema": {"type": "object", "required": ["seconds"], "additionalProperties": False, "properties": {
             "seconds": {"type": "number", "minimum": 0.1, "maximum": 10.0},
             "mode": {"type": "string", "enum": ["idle", "slow_walk"], "default": "idle"},
             "movement": _vector(2), "facing_deg": {"type": "number", "minimum": -180, "maximum": 180},
             "speed_mps": {"type": "number", "minimum": -1.0, "maximum": 0.6},
             "height_m": {"type": "number", "minimum": -1.0, "maximum": 0.9},
             "hand_points": {"type": "object", "additionalProperties": False,
                             "properties": {k: _vector(3) for k in ("left", "right", "head")}},
             "hand_points_end": {"type": "object", "additionalProperties": False,
                                 "properties": {k: _vector(3) for k in ("left", "right", "head")}},
             "hand_quats": {"type": "object", "additionalProperties": False,
                            "properties": {k: _vector(4) for k in ("left", "right", "head")}},
             "upper_body": _vector(17)}}},
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
        {"name": CROUCH, "description": (
            "Crouch with the legs, or stand back up: lowers the pelvis by pelvis_drop_m below SONIC's standing height "
            "(about 0.77 m; at most 0.32 m lower, from simulation: 0.07 m bends the knees +0.5 rad, 0.19 m +1.2, 0.30 m "
            "+1.8; deeper the robot sits down) by bending knees, ankles and hips together so the torso stays upright, "
            "and leans the torso forward by lean_rad (waist pitch, 0-0.5 rad). forward_rad shifts the pelvis forward over "
            "the feet (about 6 cm per 0.1 rad). Measured in simulation: commands of 0.1 / 0.2 / 0.3 m dropped the pelvis "
            "0.08 / 0.21 / 0.29 m, the body slid 3 / 7 / 19 cm backward on the way down and stayed about 9 cm back after "
            "standing up, tilt stayed under 7 deg. The move takes secs "
            "(2-10 s; use 3 s or more for deep crouches). pelvis_drop_m 0 and lean_rad 0 stands back up. Arms, hands and "
            "waist yaw keep the last frame, except that when rising with arms_home (default true) the arms first return "
            "to the standing pose over 2 s (standing up quickly from a lean with an arm out fell in simulation). The "
            "hands reach down to about 0.35 m above the floor at the deepest crouch: the floor itself is out of reach; "
            "crouch for targets at knee to hip height. Builds the frames from the held one and streams them as g1_step "
            "does (same limits, feedback and stops), so call g1_hold first; the reply's transition.crouch has the knee "
            "flexion used, joints clipped to their limits and the pelvis height estimated from the leg kinematics. "
            "Needs operator arming."),
         "inputSchema": {"type": "object", "additionalProperties": False, "properties": {
             "pelvis_drop_m": {"type": "number", "minimum": 0.0, "maximum": 0.32, "default": 0.0},
             "lean_rad": {"type": "number", "minimum": 0.0, "maximum": 0.5, "default": 0.0},
             "forward_rad": {"type": "number", "minimum": -0.2, "maximum": 0.3, "default": 0.0},
             "secs": {"type": "number", "minimum": 2.0, "maximum": 10.0, "default": 4.0},
             "arms_home": {"type": "boolean", "default": True}}}},
        {"name": HOLD, "description": (
            "Hold: drop any motion still streaming and keep the robot at its last reference. Always allowed, "
            "armed or not. It is not an emergency stop: the operator's stop (keyboard O or the gamepad's Select "
            "at the robot) damps the joints and the robot sits down. With reanchor (armed only): take the measured "
            "pose and heading as the new reference and as the heading the frames' root rotation is relative to - "
            "after someone moved or turned the robot (an operator reset), the old reference would turn it back."),
         "inputSchema": {"type": "object", "properties": {"reanchor": {"type": "boolean", "default": False}},
                         "additionalProperties": False}},
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
    if name in (STATUS, OBSERVE):
        _only(arguments, ())
    elif name == HOLD:
        _only(arguments, ("reanchor",))
        reject_unless(isinstance(arguments.get("reanchor", False), bool), "reanchor must be true or false")
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
        _only(arguments, tuple(WALK_LIMITS) + WALK_FLAGS + ("carry_offsets_rad",))
        offsets = arguments.get("carry_offsets_rad", {})
        reject_unless(isinstance(offsets, dict), "carry_offsets_rad must be an object of joint name -> rad")
        for joint, value in offsets.items():
            reject_unless(joint in CARRY_OFFSET_JOINTS, f"carry_offsets_rad: {joint!r} is not an arm joint")
            _finite(value, f"carry_offsets_rad.{joint}")
            reject_unless(abs(float(value)) <= 0.5, f"carry_offsets_rad.{joint} must be within 0.5 rad")
        reject_unless(type(arguments.get("carry", False)) is bool, "carry must be a boolean")
        for key, (low, high) in WALK_LIMITS.items():
            if key in arguments:
                _finite(arguments[key], key)
                reject_unless(low <= float(arguments[key]) <= high, f"{key} must be in [{low:g}, {high:g}]")
        reject_unless(any(abs(float(arguments.get(k, 0.0))) > 1e-6 for k in ("forward_m", "left_m", "turn_deg")),
                      "g1_walk needs a nonzero forward_m, left_m or turn_deg")
    elif name == PLANNER:
        _only(arguments, ("seconds", "mode", "movement", "facing_deg", "speed_mps", "height_m", "hand_points",
                          "hand_points_end", "hand_quats", "upper_body"))
        _finite(arguments.get("seconds"), "seconds")
        reject_unless(0.1 <= float(arguments["seconds"]) <= 10.0, "seconds must be in [0.1, 10]")
        reject_unless(arguments.get("mode", "idle") in ("idle", "slow_walk"), "mode must be 'idle' or 'slow_walk'")
        if "movement" in arguments:
            _finite_vector(arguments["movement"], 2, "movement")
        for key, (low, high) in (("facing_deg", (-180.0, 180.0)), ("speed_mps", (-1.0, 0.6)), ("height_m", (-1.0, 0.9))):
            if key in arguments:
                _finite(arguments[key], key)
                reject_unless(low <= float(arguments[key]) <= high, f"{key} must be in [{low:g}, {high:g}]")
        reject_unless(not ("hand_points" in arguments and "upper_body" in arguments),
                      "give hand_points or upper_body, not both")
        reject_unless("hand_points" in arguments or not ("hand_points_end" in arguments or "hand_quats" in arguments),
                      "hand_points_end and hand_quats need hand_points")
        for group, size in (("hand_points", 3), ("hand_points_end", 3), ("hand_quats", 4)):
            value = arguments.get(group, {})
            reject_unless(isinstance(value, dict) and set(value) <= {"left", "right", "head"},
                          f"{group} must be an object with keys among left, right, head")
            for key, vector in value.items():
                _finite_vector(vector, size, f"{group}.{key}")
                if size == 3:
                    reject_unless(all(abs(float(v)) <= 1.0 for v in vector), f"{group}.{key} must be within 1 m of the pelvis")
        if "upper_body" in arguments:
            _finite_vector(arguments["upper_body"], 17, "upper_body")
    elif name == HANDS_WALK:
        _only(arguments, tuple(HANDS_WALK_LIMITS) + ("left_hand", "right_hand"))
        for key, (low, high) in HANDS_WALK_LIMITS.items():
            if key in arguments:
                _finite(arguments[key], key)
                reject_unless(low <= float(arguments[key]) <= high, f"{key} must be in [{low:g}, {high:g}]")
        for key in ("left_hand", "right_hand"):
            if key in arguments:
                _finite_vector(arguments[key], 3, key)
                for value, (low, high), axis in zip(arguments[key], HAND_POINT_BOX, "xyz"):
                    reject_unless(low <= float(value) <= high, f"{key}: {axis} {float(value):.3f} is outside [{low:g}, {high:g}] (pelvis frame)")
    elif name == CROUCH:
        _only(arguments, tuple(CROUCH_LIMITS) + CROUCH_FLAGS)
        reject_unless(type(arguments.get("arms_home", True)) is bool, "arms_home must be a boolean")
        for key, (low, high) in CROUCH_LIMITS.items():
            if key in arguments:
                _finite(arguments[key], key)
                reject_unless(low <= float(arguments[key]) <= high, f"{key} must be in [{low:g}, {high:g}]")
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
