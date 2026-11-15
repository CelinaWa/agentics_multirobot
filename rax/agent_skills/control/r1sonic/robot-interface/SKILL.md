---
name: robot-interface
description: The Unitree R1 humanoid with two PiPER parallel grippers under the frozen SONIC_R1 whole-body controller in teleop mode - its tools, frames, the 5-joint arm's reach and orientations, the gripper, observations, arming and hold, and the limits the backend enforces. Read before composing any robot action.
---

# Robot interface: R1 + PiPER grippers (SONIC_R1, teleop mode)

{% if simulated %}The robot is a simulated Unitree R1 (MuJoCo) driven by the same controller and backend as the real
robot{% else %}The robot is a real Unitree R1{% endif %}: 26 body joints (legs, waist roll and yaw, two 5-joint arms,
head pitch and yaw) and a PiPER parallel gripper on each wrist. The frozen SONIC_R1 whole-body controller keeps it
balanced. You do not command motors or joints directly: every 50 Hz tick the controller is given **three point
targets** (left wrist, right wrist, torso) and the default standing legs, and it moves the whole body to put the
points there. The backend computes those targets from arm and waist joint angles (forward kinematics), so every target
is a pose some arm configuration has. The grippers are position-commanded beside the controller. One RGB-D camera,
`cam_head`, sits in the head; `r1_head` tilts and turns it. **The robot does not walk on command**: it stands where it is, and
the operator moves the table or the robot when you ask (`request_operator`).

## Tools

| Tool | What it does |
|---|---|
| `r1_status` | arming, the stack link, measured joints, IMU, tilt, gripper widths, both grasp points, the command held, limits, last feedback |
| `r1_observe` | head RGB (+ depth) with the camera pose in the pelvis frame, joints, IMU, grippers, grasp points |
| `r1_pixel_to_position` | depth pixels of the current observation -> pelvis and level frame points |
| `r1_move_hand` | one grasp point to a position (pelvis frame), with a preferred `approach` and `fingers` axis; IK, smooth move, servo |
| `r1_move_joints` | arm/waist joints to angles (home and tucked poses, wrist roll, waist turns) |
| `r1_gripper` | open/close one gripper to a width (0..0.07 m) |
| `r1_head` | head pitch (-0.6..0.6 rad, positive looks down) and yaw (-1.9..1.9 rad, positive looks left) |
| `r1_hold` | drop any streaming motion, keep the last command |
| `r1_step` | the controller's own interface: reference rows of arm, waist and LEG joint angles, one per 50 Hz tick |

From your own code: `ctx.value("r1_move_hand", arm="right", position=[x, y, z], approach=[1, 0, -0.3])`.

## Frames

- **Pelvis frame**: origin at the pelvis, x forward, y left, z up, moving with the pelvis. Every position the tools
  take and return is in it. The pelvis is about 0.73 m above the floor standing; a table top at 0.70-0.77 m is at
  z of about -0.03..+0.04.
- **Level frame**: the same origin, z up against gravity, x along the measured heading (`r1_pixel_to_position` gives
  both). The IMU orientation (`proprio.imu_orientation_wxyz`) is relative to the heading the robot had at the start.
- There is no world position of the robot or of any object. Positions measured before a motion are in the pelvis
  frame of that moment.

## The arm, the gripper and what reaches where

- **5 arm joints, no wrist pitch or yaw.** The gripper points along the forearm; the wrist only rolls it about that
  axis. So the gripper's direction is set by where the hand is: reaching forward at table height, the gripper points
  forward and at most about 30 degrees down. A goal with a steeper `approach` (straight down) has no arm configuration
  and `r1_move_hand` either refuses it or solves with a large `approach_error_deg` (the reply says which). Preview
  with `preview_only=true`.
- **The grasp point** is between the finger pads, 2 cm in from the fingertips (`grasp_points_pelvis_m`). The fingers
  are plates 7 cm long (along the approach) and 5.6 cm wide; the opening is 0 to 0.07 m between the pads.
- **The gripper housing is 14.5 cm long along the finger axis** (and 7 cm the other way) and sits 6-8 cm behind the
  grasp point. Near a surface, fingers that open perpendicular to it put the housing into it; fingers opening parallel
  to the table keep it clear.
- **Measured in simulation** (table pushed away, IK-consistent targets): the controller put the grasp point 1-5 cm
  from the goal after the move; the backend's servo on the measured joints then brought most goals within 1.5 cm.
  Comfortable right-hand region: x 0.3-0.5 m, y -0.3..0.0 m, z -0.05..+0.2 m; across the body (y > 0) and close to
  the chest the error grows. `use_waist=true` lets the waist roll and yaw help (reaching across the body or far out).
- **The pelvis moves when the arm works.** Reaching forward, and much more pressing on anything, makes the controller
  shift the pelvis (sim: 5-8 cm back reaching, up to 16 cm back when a finger pressed on the table, and sideways). The
  servo corrects the hand relative to the pelvis, not to the world: a target measured before the move is off by the
  shift afterwards. Re-observe the target with the arm already out, then make the last short move.
- **Contact makes it step.** A gripper or arm pressing on the table or an object pushes the robot; it steps to keep
  balance and its hand misses. The backend stops and holds a motion after more than 2 steps and reports `stopped`.

## Gripper

`r1_gripper(arm, width_m)`: about 12 N per finger. The reply has `measured_m` and `blocked_m`: a measured width
wider than commanded means something is between the fingers. `r1_move_hand(..., gripper_width_m=...)` sets the
opening during the move.

## Observations

- `cam_head` image (and depth: distance along the optical axis, mm, 0 invalid), full frames saved under
  `runtime/frames/<frame_id>/` with `meta.json` (intrinsics, camera pose, joints at capture).
- `camera_poses.cam_head`: the camera pose in the pelvis frame from the measured head and waist joints. The head pitch
  starts at 0.5 rad (looking down at a table in front); the waist turns the camera with it.
- `scripts/r1_kin.py` with `scripts/r1_piper_kin.xml` is the backend's own kinematics model (FK of the grasp points,
  the camera and the teleop points, and the IK `r1_move_hand` uses) for your code.

## Arming, hold and faults

Motion tools need operator arming. `r1_hold` is always allowed. The backend disarms with a fault when the robot tilts
past 30 degrees (falling) or steps 6 times in 6 s by itself; after that, call `request_operator` (kind `help`) and
wait. Every move ends holding its last command: the robot keeps that pose until the next call.

## Limits this backend enforces

`r1_status` -> `limits`: joints stay 0.05 rad inside their limits; `r1_move_hand` refuses goals whose best IK solution
misses by more than 3 cm; the servo adds at most 5 cm; a motion stops after more than 2 steps; tilt above 30 degrees
holds and disarms.

## The controller's own interface: `r1_step`

The controller takes two inputs every tick: the **three points** (both wrists and the torso) and a **lower-body
joint reference** (the twelve leg joints: hip pitch, roll and yaw, knee, ankle pitch and roll on each side). The
other tools set only the points and leave the legs on the default stance (hip pitch -0.1, knee 0.3, ankle pitch
-0.2 rad; the rest 0). `r1_step` lets you give both, as joint angles, row by row:

```python
ctx.value("r1_step", joints=["left_knee_joint", "right_knee_joint", ...], rows=[[...], [...], ...])   # 50 rows = 1 s
```

- Arm and waist angles are sent as the three points they produce (forward kinematics), as with `r1_move_joints`.
- Leg angles are sent as the lower-body reference. They are a reference, not motor targets: the controller keeps
  its balance first and follows them as far as it can. Read `measured_joints_rad` and `tilt_deg` in the reply.
- The legs keep the last reference you gave, also while you use the other tools afterwards: a crouch stays a crouch
  during `r1_move_hand`. `r1_status` -> `command` shows it (`legs`, `legs_reference`).
- Lowering only the torso point does not lower the robot: the legs are still told to stand, the controller cannot
  reach the point, steps away and falls{% if simulated %} (measured in this simulation: nothing down to 20 cm, a 1.6 m walk-off at 25 cm,
  a fall at 30 cm){% endif %}. Change the leg reference; the points are relative to the pelvis and come down with it.
- Nothing here is a ready-made motion. A crouch, a weight shift or a step is yours to build from the reference and
  to measure: ramp slowly (the backend caps the change per row), try a small amount first, watch the tilt and the
  feet, and stand back up the way you went down. Without `allow_steps` a motion in which the feet step stops and holds.
{% if simulated %}

One measurement to start from (this simulation, empty hands; both legs: knee +k, ankle pitch -min(k/2, 0.6), hip
pitch the rest so the torso stays upright; ramped over 3 s per 0.8 of k): k = 0.8 lowered the pelvis 7 cm (tilt
1.4 deg), k = 1.6 lowered it 18 cm (tilt 2.5 deg, no steps). Crouched at k = 1.6, `r1_move_hand` with the waist put
the right grasp point 0.26 m above the floor (tilt 5 deg), and the robot stood back up along the same ramp. The
ankle pitch limit (-0.82 rad with the margin) is reached near k = 1.25: the hip takes what the ankle cannot. Deeper
references were only tried past the joint limits, outside this tool: 25 cm at 8 deg of tilt, and a fall at the knee
limit. These are numbers of the simulation: re-measure on the robot.
{% endif %}
