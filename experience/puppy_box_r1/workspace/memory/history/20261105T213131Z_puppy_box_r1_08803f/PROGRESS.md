# PROGRESS — puppy_box_r1 (session 20261105T213131Z_puppy_box_r1_08803f, sim)

NOTE: agent workspace is READ-ONLY (Bash: Read-only file system; Read/Write tools denied). Working copy of
code/memory/handoff in /tmp/claude-1001/ws_08803f. Frames are readable under
agent-workspace/runtime/frames/<id>/ (cam_head.jpg, cam_head_depth.png, meta.json). Read tool on images denied:
only images returned by r1_observe can be seen.

## Completion checklist (TASK.md)
- [ ] puppy body inside the clear box
- [ ] robot standing still, arms near body, grippers open, not touching table/box
- [ ] box upright on table; nothing on floor

## State
- Start 21:31Z, 7200 s, $10 Gemini, 5 resets. Armed at start by "scripted operator (sim)".
- Backend commits since last session: scripted operator, grasp_paw reach guard (refuses paw > 0.35 ahead),
  walk keeps grip/head, backward walking fixed (unverified).
- t0 frame 20261105133207c26d3e (head 0.5): table z -0.047, edge x 0.627 yaw 0.2deg (square).
  Puppy at front edge, head RIGHT, legs toward me. Paw surface pts: hind (0.621,-0.012),(0.607,-0.062);
  front (0.607,-0.186),(0.621,-0.238). Box (depth at rim height): x 0.84..1.13, y -0.33..+0.069 (39 cm along y),
  rim at table+0.145; box near wall = edge+0.21.
- code: code/puppy_box/scene.py <frame_id> (table, paws, box) in the ws copy.

## Attempt 1 (21:34Z-) layout: puppy at front edge head RIGHT, box BEHIND (front wall edge+0.23..0.25)
- help "position hind leg to overhang" -> answered in 3 s; operator pulled the puppy ~6 cm toward me: ALL 4 paws
  overhang the edge by ~5-7 cm, legs horizontal at table+5 cm (paw centres ~z table+0.05).
- Walk facts (this backend): fwd 0.2 -> 0.045; 0.3 -> 0.10; 0.3 -> 0.25 (+0.12 RIGHT drift); BACKWARD -0.2 -> 0.228 back
  (works now). turn 12 -> ~6.6deg, 15 -> ~3.8deg (+0.10 right drift), 30 -> ~21deg (+0.12 fwd, -0.075 right).
  Walk keeps head pitch now. r1_walk 'pelvis' odometry is CONSISTENT with perception (edge world x 0.75 in
  every frame) -> use a world frame from odometry + heading from the box/edge geometry. IMU yaw does NOT track turns.
- Reach (preview, no waist): approach (1,0,0.15) fingers vertical OK for y -0.15..-0.2, x .25-.35 (ik .02);
  y>-0.05 needs a diagonal approach (0.89,0.45,0.05): (0.36,0.015,0.013) ik .018. x<=0.50 at z .2-.3 (0.55 -> .035).
  use_waist solutions use waist yaw 0.5-1.5 rad: avoid.
- After the 30deg turn (frame 202611051342530bb2d0, step 108): pelvis world (0.408,-0.236), heading ~24deg left;
  upper front paw ~(0.347,0.010,0.013) centre; lower front paw (0.358,0.067,-0.003); box top corners (pelvis):
  FL(0.718,0.159) BL(0.951,0.033) BR(0.779,-0.312) FR(0.557,-0.197), rim z ~0.10; centre (0.748,-0.077).
  Plan: grip upper front paw with approach (0.89,0.45,0.05), lift, walk fwd ~0.17 holding, hand to ~(0.49,-0.16).
- Reaching the paw at y>+0.05 (pelvis) chased: each reach yawed the waist (measured 0.18->0.35->0.45 with command 0)
  and shifted the pelvis so the paw moved +1.2..+1.6 cm left per move; servo saturated (0.0487/0.05), hand 2.3 cm short.
- INCIDENT 1 (21:46Z): r1_move_joints HOME from hand (0.27,-0.02,0.04) while heading 24deg left of the table: the hand
  swung down-right into the table's front edge (closer on the right when turned left): tilt 7.9, 1 step, recovered by
  itself. Rule: with an oblique stance, retract the hand along the edge normal >=0.1 before lowering it.
- Balance step after incident 1 pushed me back/rotated: upper front paw then at (0.36,-0.085) -> comfortable.
  Pre-grasp (0.29,-0.092,0.01) appr (1,0,0.1): body yawed again, paw re-measured (0.39,-0.16); intermediate
  (0.345,-0.16,0.012); final (0.415,-0.185,0.006) appr (1,0,-0.1) -> hand (0.411,-0.194); nudge to (0.418,-0.179,0)
  -> hand (0.424,-0.182,-0.004); r1_gripper 0 -> measured 0.0346 = HELD (front paw ball, fingers vertical).
  Frame 20261105134841411503.
- Lift with approach (1,0,-0.1)/(1,0,0): z .05 (w .034), .10 (w .037), .15 (w .035 -> .029 at observe). Puppy front
  rose, body rolled. Box rim nearest part then at x ~0.73 (pelvis shifted back during the reach).
- SLIP: move_hand to (0.38,-0.18,0.27) WITHOUT an approach -> IK picked a different arm config (shoulder roll -0.6->-1.4,
  yaw 0.3->1.1, wrist roll -0.6->+0.1) -> width 0.0044, puppy dropped and fell OFF the table (not visible on table).
  Rule: while holding, always pass the same approach, preview, and reject waypoints whose joints jump (>0.4 rad).
- Arm home via (0.15,-0.26,0.1) then HOME joints: clean (tilt 0.6). Requesting reset #1 (21:50Z).

## Attempt 2 (after reset #1, 21:51Z): same layout (head right, box behind, front paws in front of right shoulder)
- Straight approach: walk 0.2 -> 0.073, 0.2 -> 0.142 (no lateral drift). Edge 0.416, lower front paw (0.35,-0.18).
- Pre-grasp (0.27,-0.18,0.005) appr (1,0,0.1) -> hand within 0.5 cm; final (0.367,-0.177,.003) appr (1,0,-0.1),
  nudges to (0.381,-0.161,0.013); close -> 0.0326 HELD. Body shift this time small (~1.3 cm).
- LOCAL IK: installed mujoco into /tmp/claude-1001/ws_08803f/pylib; code/tools/contik.py plan_b() = continuous IK
  seeded by the previous solution. Backend grasp point = local 'right_grasp' site - 0.107*approach (calibrated).
- Lift by r1_move_joints along the continuous plan (1.2 s/step): z .045 w.034, .07 w.037, .10 w.039, .17 w.040,
  .20 w.037 (held; puppy hanging diagonally, rear still on table), then step to .29 -> w .0044 SLIPPED again.
  Puppy fell back on the table (not off), now on its side in front of the box, hind legs pointing LEFT (+y).
- Diagnosis: with the approach along the leg, the paw ball is the thing pinched between horizontal pads; when the
  puppy hangs fully the leg exits over the fingertip edge and pulls the ball out (friction-only). Need a STOPPER grasp:
  pads clamp the LEG with the paw ball outside the pads (approach roughly perpendicular to the leg).
  Side approach (0,1,0)/(0.5,0.87,0) at table height is NOT reachable (ik .04-.08).

## Attempt 3 (after reset #2, 22:02Z): same layout
- Walk 0.2 -> .093, 0.2 -> .119; upper front paw (0.34,-0.181); pre-grasp (0.27,-0.186,.008) -> final (0.372,-0.179,.013)
  -> nudge (0.378,-0.177,.013) -> hand (0.385,-0.178,0.022); close -> 0.0316 HELD.
- Continuous lift 2 s/step: z .06 w.034, .09 w.035, .135 w.032, .19 -> w .0044 SLIPPED (lower than attempt 2's .20 hold).
  Puppy fell OFF the table.
- Funnel so far: grasp 3/3 (front paw ball, fingers vertical, w .032-.035); lift >= z .19: 0/3 (slip). Same method
  failed 3x -> CHANGE METHOD. Next: backend primitive r1_grasp_paw (leg clamp, guard <=0.35) + walk + r1_place_in_box.
- Time 1958 s used at 22:04Z; resets used 2/5.

## Attempt 4 (after reset #3, 22:05Z): r1_grasp_paw pipeline
- Walk 0.2 -> .071, 0.2 -> .107, 0.15 -> .12 (edge ~0.33, hind paws ~0.27 ahead, y -0.01/-0.06).
- r1_grasp_paw -> {"paw":2,"side":"right","held":true,"clamped_width_m":0.0571,"lifted_m":0.062}; hand (0.398,-0.245,0.221),
  fingers HORIZONTAL (axis ~y), approach (0.97,0.07,0.22); width settled 0.031. Frame 202611051406052f337b.
- r1_walk 0.15 WITH the puppy held -> walked 0.203, width 0.030 (HELD). Box front rim then at x 0.37-0.41, interior
  y +0.07..-0.21, rim z ~0.105; hand at (0.392,-0.244,0.22) = outside the front-right corner.
- BUG (mine): r1_move_joints after r1_grasp_paw re-sent the backend's STORED gripper command (0.07, never updated by
  r1_grasp_paw) -> gripper OPENED, puppy dropped in front of the box (hind legs vertical). FIX: call
  r1_gripper(right, 0) immediately after r1_grasp_paw; pass gripper_width_m=0 to every r1_move_hand while holding.
- INCIDENT 2: HOME joints from hand (0.43,-0.21,0.235) with the edge only ~0.13 ahead: hand swept down through the
  puppy/table (tilt 7.3, 1 step) and knocked the puppy OFF the table. Rule: retreat = hand straight back (x<0.15) at
  height, THEN down/home; never sweep the arm down near the table or the puppy.
- Funnel: r1_grasp_paw 1/1 held; carry-walk 1/1 held. Place 0 (own bug). Resets used 3/5 -> requesting #4.

## Attempt 5 (after reset #4, 22:08Z): r1_grasp_paw again
- Walk 0.2 -> .066, 0.2 -> .142, 0.12 -> .065: edge ~0.355, hind paws ~0.29-0.30 ahead (y -0.02/-0.07).
- r1_grasp_paw -> held FALSE, clamped .0041, tilt 12.0 (near-fall), waist left yawed 0.28 after; puppy knocked OFF the table.
- r1_grasp_paw record (both sessions): held 2/5 (paw ~.27 here, ~.37 prior); fall 1 (.38-.39, before guard);
  knocked puppy off 2 (~.25 prior, ~.30 here). Tool leaves the backend's stored gripper command and waist command stale.
- Resets used 4/5. Using the last reset for one more r1_grasp_paw pipeline run (stance paws ~0.27).

## Attempt 6 (after reset #5, 22:10Z): r1_grasp_paw -> ROBOT FELL (tilt 62.6, disarmed), measured waist_yaw 1.52 after
- Stance: pelvis world (0.432,0.199), edge ~0.31, hind paws ~0.26 ahead (attempt-4 success was at (0.408,0.197)).
- r1_hold + help request; operator restored + re-armed ("reset 6/5"). Frame 20261105141213b7b9ba.
- DECISION: r1_grasp_paw is a FALL HAZARD (sessions: 2 held, 2 falls, 3 knock-offs in 7 calls) -> never call it again.
## Attempt 7 (22:13Z): own pinch + slow lift to ~0.16 + walk holding + raise over rim + drag in + release + push.

## FINAL (22:25Z, ~54 min of 120 used)
- TASK NOT ACHIEVED. Final state verified (r1_status step 288): standing, tilt 0.07, arms HOME, both grippers 0.07,
  0.26 m back from the table; puppy on the table in front of the box; box upright; nothing on the floor
  (frame 20261105141627ff7de3).
- Resets: 5/5 + 1 operator restore after the r1_grasp_paw fall. Falls: 1 (r1_grasp_paw).
- Deliverables (all in /tmp/claude-1001/ws_08803f; workspace + skill-library read-only):
  code/puppy_box/controller.py (step-wise; perception + planning offline-tested on 5 frames; not run end-to-end),
  scene.py, code/tools/contik.py; memory/{TIPS,RUNS,INCIDENTS,PROGRESS}.md, PARAMS.json; handoff/README.md;
  skill_draft/skills/r1-paw-pinch-grasp (draft). skill_library_merge -> merged:false "no committed changes".
- Next concrete step: a stopper grasp (pads across a leg, paw ball outside) before any lift above z 0.12; see handoff.
