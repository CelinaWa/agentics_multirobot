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
`cam_head`, sits in the head; `r1_head` tilts it. **The robot does not walk on command**: it stands where it is, and
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
| `r1_head` | head pitch (-0.6..0.6 rad, positive looks down); yaw stays 0 |
| `r1_hold` | drop any streaming motion, keep the last command |

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
