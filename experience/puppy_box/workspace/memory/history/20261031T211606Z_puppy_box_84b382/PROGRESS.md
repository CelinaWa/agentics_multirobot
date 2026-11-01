# PROGRESS — puppy_box (session 20261031T211606Z)

STATUS (21:47, ~31 min into 60): task NOT completed. Two top-down right-hand grasps failed (rule limit
reached, operator informed and approved trying a different grasp). One reset used, one left.
Robot standing, arms home (L/R shoulder pitch 0.5/0.4, elbows 0.6, wrist roll 0), hands open, box untouched.
Puppy on the table ~0.56 m from pelvis (operator), rotated (head left, legs toward robot).

Shell sandbox unavailable (bwrap fails) -> no Python/code/git/skill-library commit this session.

## Scene measurements (start, pelvis/level frame, pelvis ~0.78 m above floor)
- Table top z ~ -0.054; table front edge x ~ 0.22.
- Puppy start: body middle (0.38..0.41, -0.06, top z 0.085); head (0.47, 0.13); tail (0.39, -0.20).
- After reset 1 the puppy came back differently: head (0.38,0.10), torso centre ~(0.42..0.45,-0.10..-0.15), top z ~0.08.
- Box (colour-image rim lines ray-cast to z=+0.09): right wall y~0.13 (x 0.40..0.68), near rim (0.40,0.12)-(0.32,0.39),
  far rim (0.68,0.14)-(0.62,0.55); centre ~(0.51,0.33). Box not touched all session.

## Arm model that matched the camera (to ~3 cm)
- Right shoulder centre S=(0.004,-0.138,0.278) (+ waist pitch p: x+=0.234 sin p). Upper arm 0.18, elbow->hand back 0.27.
- Upper-arm angle a=p+q1 (q1 = shoulder pitch, negative = forward). Elbow 0 = forearm perpendicular; forearm angle
  below horizontal phi = a + q4. Shoulder yaw + swings forearm inward (+y).
- Joint index in row: WY 11, WP 17, LSP 20, RSP 21, RSR 25, RSY 29, RE 31, RWR 33, right hand 39.

## Tracking (cmd - measured), free arm
- RSP ~ -0.1; elbow +0.26..+0.38 (much larger, up to +0.9, when the hand touches anything);
  right shoulder yaw -0.12..-0.43 (non-linear, poor); shoulder roll ~+0.15; waist pitch tracks ~90%;
  WAIST YAW DOES NOT TRACK (cmd -0.2 -> meas +0.08) and drifts +0.1..+0.19 when leaning with the arm out.
- Each g1_step reply arrives before the arm settles: always g1_observe again before measuring.

## Key failure causes (both grasps)
1. Palm-down (wrist roll -1.3) puts the Dex3 thumb pointing STRAIGHT DOWN ~10 cm below the hand back. It touches first
   and rolls/pushes the plush (puppy moved up to 15 cm, out of reach). Any lateral move while the thumb is low drags it.
2. Closing from above: index finger always goes to full fist (misses), middle/thumb stall briefly, on lift everything
   slips -> plush is ~20 cm across, too wide for a top pinch.
3. Leaning (waist pitch 0.3) pushed the HOME left hand onto the table edge (left wrist pitch deflected -0.9):
   move left shoulder pitch to 0.5 BEFORE leaning.

## Next to try (if time / next session)
- Grasp a narrower part: neck/front legs bundle, or the torso from the near side with the hand vertical
  (wrist roll ~0, palm facing +y, fingers forward/down) so thumb and fingers straddle ~10 cm.
- Or g1_hand_shape: open pose with thumb rotated/curled so it does not stick down; pre-curled fingers.
- Keep the puppy within x<=0.45 (do not touch it before the final descent); lift hand >=5 cm before any lateral move.

## Log
- Steps 0-195: retract (RSP +0.45), forearm up (elbow -0.5), swing up (RSP -0.5), extend; palm-down over puppy.
- Grasp #1 FAILED (step 236-301) closed 5 cm too high. Puppy rolled back.
- Lean 0.3 + further reach: thumb pushed puppy to x~0.56 (beyond reach). Reset 1 used (step 566).
- After reset: lean 0.22, positioning fought poor yaw tracking (~10 min). Grasp #2 FAILED (step 804-823).
- Arm homed safely (step 943). Operator informed; operator approved trying a different grasp, but the puppy was at
  0.56 m (out of reach) and no credible different grasp was ready, so I stopped manipulation.
- Air test (step 944-1051): right g1_hand_shape open [0,0.4,0,0.5,0,0.5,0] applied (measured thumb_1 0.36, middle_0/index_0
  ~0.5). With forearm 45 deg up and wrist roll -1.3 the thumb still sticks out sideways/down, similar to the default
  shape (frame 202610312149078ae5f4) -> this shape alone does not fix the down-pointing thumb. Default shape restored.
- Coming home the right hand rested on the table front edge (elbow stuck 0.28 vs 0.6, wrist pitch -0.38); fixed by
  right shoulder pitch 0.65. FINAL HOME POSE: L/R shoulder pitch 0.5/0.65, elbows 0.6, wrist roll 0, hands open, tilt ~2 deg
  (frame 2026103121503811f320: no contact, puppy on table, box untouched).
