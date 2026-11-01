# PROGRESS — puppy_box session 20261101T004902Z (sim)

STATUS (end): NOT solved. Box fell off the table (robot slid back during leans while the box rested over the
left hand); the scripted reset restored only the puppy, so the place was impossible. Leg-wrap grasp failed 3x.
Robot left standing, arms home (right shoulder pitch 0.35), hands open, puppy on the table.
Library: palm-pose-servo v1 (draft) merged (main 3306c45). Code: code/pbox/ (scene.py, robot.py, ik.py, fk.py).
Next session: see handoff/README.md.

## Scene (this layout, measured)
- Box LEFT of the puppy, in view: approx x 0.16-0.45, y 0.03-0.42 (pelvis), first frame; depth over the box returns a
  sloped surface up to z 0.16 (not the table). Puppy RIGHT, head toward the box (left), legs toward robot.
- First frame (just after start) placed everything ~14 cm too close; after the first motion: torso x 0.42-0.52,
  y -0.40..-0.10; legs x 0.30-0.44, y -0.38..-0.19; table z ~ -0.085.. -0.09 (pelvis frame drifts ~1 cm per frame).
- Puppy is articulated RIGID capsules (legs ~7 cm thick cylinders, torso ellipsoid) — top-down torso grasp is
  hopeless with Dex3; new approach = wrap grasp around a LEG.

## Tools that work here
- g1_free_space_move (cuRobo) WORKS this session. Palm frame: x = fingers, y = palm normal (right hand: inward at
  roll 0), thumb along +y. Palm-down yaw -45 deg (fingers forward-right): quat wxyz [0.6533,-0.6533,0.2706,-0.2706].
  Yaw +-90 / +45 at low z fail to plan without waist. Planner refuses palm z < 0.05 here -> own IK (code/pbox/ik.py)
  + g1_step ramp (code/pbox/robot.py ramp).
- Unfold right arm: shoulder pitch 0.5, then sp 0.0 + elbow -0.3, then plan.

## Attempts
1. Leg wrap, yaw -45, palm z 0.035, thumb at y -0.16: close stalled (thumb_2 -0.54, index_0 0.0) but lift 4 cm
   -> full fist: nothing held. Image 20261101005822492a2c: front leg was LEFT of the hand (larger y) -> thumb
   on/over the leg. Fix: localize the leg in the frame right before descending (hand raised), from image pixels.

## INCIDENT: box fell off the table (between frames 20261101005944d8fe4a and 202611010100107efa1c)
- At start (frame 20261101004938c428c9) the box stood at the table FRONT EDGE directly above/over the robot's LEFT
  hand (left hand visible through the box floor; depth over the box = a sloped surface z 0.06-0.16 = the box tilted on
  the hand). The robot then drifted BACKWARD ~19 cm over the session (operator: dog 0.48 m -> 0.67 m) during waist
  leans 0.21/0.25/0.30/0.42 with an arm forward; the box slid off the edge with the left hand and fell to the floor.
- request_operator reset restored ONLY the puppy (scripted); help requests get "approved, go ahead" — the box stays
  on the floor. Task cannot succeed for the rest of this session.
- LESSON: before any motion, check whether the box/object overhangs the table edge onto a hand (hand seen through
  the clear box floor, sloped depth above the rim). If so, ask the operator to move it / move the hand DOWN and back
  slowly first. Avoid repeated waist leans: each drifts the pelvis back several cm (cumulative 19 cm).

## Attempt 2 (leg wrap, lean 0.42) — FAIL
- Measured palm drifted 3-5 cm toward -y vs command under lean (waist yaw drifts +0.15); compensated by commanding
  y +6 cm. Closed with palm z 0.03: thumb + index went full, middle_0 stalled at 0 -> lift 3 cm = full fist.
- Robot ended standing, arms home, hands open (frame 202611010102554f2dd6).
