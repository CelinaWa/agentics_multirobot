---
name: robot-interface
description: The real Unitree G1 humanoid with two Dex3 hands under the frozen SONIC whole-body controller - its semantic40 reference frames, observations, feedback, frames, arming, hold and stop, the limits the backend enforces and which tool to use for which objective. Read before composing any robot action.
---

# Robot interface: G1 + Dex3 (SONIC, semantic40)

The robot is a real Unitree G1 humanoid with 29 body joints and two three-finger Dex3 hands,
balanced by the frozen SONIC whole-body controller running in NVIDIA's deploy stack on the robot's
network. You do not command motors: you send whole-body **reference frames**, the stack turns
them into motor commands and keeps the robot balanced. One camera, `cam_front`, sits on the head,
looking forward and down. `{{ robot.status }}` reports the live action layout, joint names and
limits, the limits this backend enforces, arming and the link to the stack; this skill explains
them. **Every command is physical** and reaches the robot only through these tools, with a
person at the emergency stop.

## One reference frame

Every `g1_step` row is one frame for one 50 Hz control tick (40 numbers); a call carries 1 to 100
rows (up to 2 s):

| Slice | Meaning |
|---|---|
| `[0:2]` | Reference root displacement for this tick. Required by the layout; **the controller ignores it**. |
| `[2]` | Reference pelvis height (metres). Required by the layout; **ignored** as well. |
| `[3:9]` | Reference root rotation relative to the heading the robot had when the session's first motion was sent: the first two columns of the rotation matrix, row-major. Identity is `[1, 0, 0, 1, 0, 0]`; a pure turn by `psi` is `[cos psi, -sin psi, sin psi, cos psi, 0, 0]`. A jump of more than 26 degrees in one tick is slerped down to 26 (`max_root_orientation_projection_deg` reports what you asked for). |
| `[9:38]` | Absolute angles (rad) of the 29 body joints, in the order of `proprio.joint_names_canonical29`. |
| `[38:40]` | Left and right hand: `0` open, `1` closed; a value in between closes every finger joint that fraction of the way. |

The controller receives the 29 joint angles, their change from tick to tick (as joint
velocities) and the root rotation. It receives no root position: **the robot moves through the
room only by stepping**, through the leg joint references. Walking therefore means sending
stepping leg references ([controller-field-guide](../controller-field-guide/SKILL.md)).

- **Frames are references, not motor targets.** On an empty floor and without contact the
  controller tracks the joints to a few degrees, and over a walk the root drifts from the
  commanded motion by tens of centimetres. A replayed motion does not reproduce its recorded
  displacement, a part of a motion may do something quite different from the whole, and a very
  short slice may hardly move the robot ([controller-tracking](../controller-tracking/SKILL.md)).
- **The robot holds your last row.** After the last row of a call the backend keeps sending that
  row, 50 times a second, until your next call. End every call in a pose to hold: the last rows
  identical, both feet on the floor. A call longer than a few rows whose last rows differ is refused.
- **Holding still is not a row of zeros.** Zeros drive every joint toward zero and the root toward
  the anchored heading. Your first frame starts from the measured `proprio.joint_positions_rad`
  and the measured heading; later frames continue from the frame you last sent, changing only
  what you intend to move. The backend refuses a first row more than 0.35 rad from the held pose
  and a tick-to-tick step above 0.15 rad on any joint, and keeps every reference 0.05 rad inside
  the joint limits (`{{ robot.status }}` -> `limits`).

## Observations

`{{ robot.observe }}` and every motion reply carry:

- `proprio.root_orientation_world_wxyz`: the measured pelvis orientation from the IMU. Its yaw is
  your heading; there is **no world position** of the robot or of any object.
- `proprio.joint_names_canonical29`, `joint_positions_rad`, `joint_velocities_rad_s`: the
  measured body joints.
- `proprio.hand_state`: the 14 Dex3 finger joints' measured positions (no torque on this robot:
  fingers that stop short of the closed pose are touching something).
- `cam_front`: the front camera image, scaled to 640 pixels wide; the full frame is saved under
  `runtime/frames/<frame_id>/` with `meta.json` (`observe` -> `camera`).
{% if camera_geometry %}
- Depth of `cam_front` (distance along the optical axis, uint16 PNG in millimetres, `0`
  invalid, at the same scale as the image), `camera_intrinsics` and `camera_poses.cam_front`:
  the camera pose in the pelvis frame (camera x along the optical axis, y left, z up), computed
  from the measured joints and the robot model's camera mount. The camera sits on the head, so
  it turns with the waist joints.
{% endif %}

A motion reply's `transition.feedback` has the ticks executed, `joint_tracking_rmse_rad` and
`max_joint_tracking_error_rad` of the last tick (measured joints against the controller's own
targets), `reference_error_rmse_rad` (against your reference), `max_root_orientation_projection_deg`,
`tilt_deg` and `fallen`. `transition.steps[*].executed_action` holds every executed frame, paired
with the preceding `observation_step_id`; `g1_step` can send them again from a matching state.
Observations are what the robot is now: the stack's state is refused when older than 0.2 s and
the camera frame when older than 1 s.

## Arming, hold and stop

- **Motion needs arming.** `g1_step`, `g1_free_space_move` and `g1_execute_motion_plan` refuse
  until the operator has armed the session; if they are not armed, say what you will do with
  `request_operator` and wait. The operator can disarm at any time: a running call stops within
  one tick and the robot **holds its last reference**; it does not sit down.
- **The controller starts on the first armed motion.** Until then `{{ robot.status }}` has no `proprio`
  and `{{ robot.observe }}` refuses. Once the operator has armed, call `g1_hold` first: the backend
  starts SONIC's control (the robot stands on its measured pose) and streams a hold of that pose.
  Then read `proprio` and build your first frames from it.
- **`g1_walk`** travels with SONIC's own planner: forward_m (forward only, 0-3 m) and turn_deg (at most
  ±45 per call; turn further in several calls), speed_mps 0.1-0.6, then it holds the measured pose again; the
  next `g1_step` starts from `proprio`, and the step id moves on, so observe again. It cannot step backward or
  sideways: to go anywhere else, turn toward it first, then walk forward. A turn steps slowly forward about
  0.1-0.2 m, so keep 0.3 m clear ahead and to the sides while turning. If the robot is still stepping when the
  walk ends, the planner keeps it balanced and the next armed call (g1_hold) hands over once it stands still. During the walk the planner moves the arms too: walk with empty hands or with an object you can
  afford to drop, and re-check the grip after.
- **`g1_hold`** is always allowed, armed or not: it drops the rows still queued and keeps the robot
  on its last frame. Use it when anything looks wrong, then observe.
- **A stop is the operator's.** The operator's stop at the robot (keyboard `O`, the gamepad's
  Select) damps every joint and the robot sits down; the backend sends the same stop and disarms
  the session when the measured tilt passes 60 degrees. After any stop or fault, make no motion
  and plan no recovery of your own: report with `request_operator` (kind `help`) and wait. After
  the operator arms again, call `{{ robot.status }}` and `{{ robot.observe }}` first and start from
  the measured pose in small steps.
- A lost link to the stack (no state for 0.2 s) refuses or stops motion the same way; the robot
  keeps whatever reference it had.

## Frames

- **pelvis**: the robot's root body; x forward, y left, z up. It moves and tilts with the robot.
- **level**: the pelvis origin, but gravity-aligned: z straight up, x the pelvis forward
  direction projected onto the floor. It needs the measured root orientation; heights in it are
  relative to the pelvis.
- **torso**: `torso_link`, which carries the arms and the camera; the waist joints move it
  relative to the pelvis.

Every pose any tool returns or takes is in one of these body frames. A point in them moves when
you walk or turn; re-measure after moving.

## The body you have

- **Hands close by a fraction.** `0` is open: every finger joint at 0 (index and middle straight,
  the thumb spread about 90 degrees from them); `1` is a fixed fist; a value in between moves every
  joint that fraction of the way, all fingers together (there is no per-finger command). Ramp the
  value over several rows to close slowly, and stop at the fraction where the fingers touch the
  object. Fully closed, the fingertips curl back to a few centimetres in front of the palm. An
  object is held only if it lies between the thumb and the two fingers as they close, or if open
  fingers and thumb clamp it from outside.
- **Measured joints tell where the body is**; the angles you commanded do not. Compute hand and
  foot positions from measured joints.
- **The camera looks down and forward**: objects near your feet or behind you are out of view,
  and your own hands, arms, knees and feet appear in the lower image.
{% if camera_geometry %}

## Robot model

`scripts/g1_model.py` (with its `robot.json`) is the robot's kinematic model, exported from the
simulated G1 of the same URDF. It describes the robot only, not the scene. Save an observation's
JSON (the first text block of a `{{ robot.observe }}` or motion reply) to a file, then:

```bash
python3 .agents/skills/robot-interface/scripts/g1_model.py pose --observation obs.json --png /tmp/pose.png
python3 .agents/skills/robot-interface/scripts/g1_model.py project 0.45 -0.2 -0.1 --observation obs.json
python3 .agents/skills/robot-interface/scripts/g1_model.py deproject 320 300 --observation obs.json --depth-png frame_depth.png
python3 .agents/skills/robot-interface/scripts/g1_model.py semantic40
python3 .agents/skills/robot-interface/scripts/g1_model.py hand
```

- `pose` prints the key points (both hand centres, palms, thumb, index and middle tips, both
  soles, the head and the camera) in the level frame, with heights above your lowest sole, and
  draws side, rear and top views; open the picture after any posture change you did not plan.
- `project` puts a planned 3-D point into the image, so you can check that it lands where you
  mean before you move; `deproject` turns a pixel and its depth into a 3-D point.
- `semantic40` names all 40 dimensions; from Python, `g1_model.semantic40_frame(joints29,
  hands=(False, False), ...)` builds a frame and `describe_semantic40(frame)` names every value of
  one you are about to send. Left and right are easy to swap by index; names are not.
- `hand` lists the Dex3 joints, the open and closed poses, the fingertip positions in the palm
  frame and the thumb-to-finger gap.
{% endif %}

## Choose the tool to match the objective

- **Whole-body motion, walking, turning, posture, hands:** `g1_step` frames built from the
  measured state, then from your last frame. The [controller-field-guide](../controller-field-guide/SKILL.md)
  describes walking, turning and contact.
- **A hand to a pose in open space:** `g1_free_space_move` plans one arm (optionally the
  waist) around the robot itself only, and keeps the legs and everything else at your last
  frame ([free-space-move](../free-space-move/SKILL.md)). It needs the planner's GPU on the robot
  service's host; if the reply says the planner is unavailable, build the arm motion from frames.
{% if camera_geometry %}
- **Metric points from the image:** `g1_pixel_to_position`
  ([object-localization](../object-localization/SKILL.md)).
{% else %}
- **Locating objects without depth:** [object-localization](../object-localization/SKILL.md).
{% endif %}
- **Anything unexpected:** `g1_hold`, then `{{ robot.observe }}`.

## Reading errors

A failed tool returns its whole error, `{type, operation, stage, reason}` plus the measured
`robot_state` when it failed during a motion.

| You see | Meaning | Do |
|---|---|---|
| `stage: validation` (not armed, a row outside a limit, a stale observation id) | nothing moved | fix the cause named in the reason |
| `MotionStopped` during a motion (disarm, lost link, tilt) | the robot holds its last reference at the measured state | observe, find the cause, plan from the measured state; never resend the same rows |
| any other error during a motion | the backend held the robot where it was | read the message, check `{{ robot.status }}` and a fresh observation; tell the operator if it repeats |

## From code

Python reaches the same tools through `api.runtime.Context`:
`ctx.call("g1_step", actions=rows)`. Calls are serialized with your direct calls; check every
reply and stop on the first unexpected one.

## Related skills

- [controller-tracking](../controller-tracking/SKILL.md): how the references are tracked, and
  how to measure and correct.
- [controller-field-guide](../controller-field-guide/SKILL.md): practical motion limits and
  feedback checks for this controller.
- [references/pose-records.md](references/pose-records.md) for how poses and executed frames are
  recorded for reuse.
