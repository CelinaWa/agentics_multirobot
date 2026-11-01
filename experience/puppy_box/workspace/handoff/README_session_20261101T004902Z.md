# Handoff — puppy_box (G1 + Dex3 under SONIC), session 20261101T004902Z (sim rehearsal, 4th session)

Result: NOT solved. Layout this time: box LEFT of the puppy and in view; puppy on the right, head toward the box,
legs toward the robot. The box was lost to the floor (see 2.); three leg-wrap grasps failed.
Robot left standing, arms home (right shoulder pitch 0.35), hands open, puppy on the table, box on the floor.
Earlier handoffs: README_session_20261101T002411Z.md, README_session_20261031T234111Z.md, README_session_20261031T211606Z.md.
Library: NEW `palm-pose-servo` (draft, merged): palm IK on the measured waist + measured-FK servo, unfold/retract.
Session code: code/pbox/ (scene.py depth cloud, robot.py ramp/plan_exec, ik.py, fk.py), output/{hm,px,grid,to,servo,lift}.py.

## Order of work in a new layout
1. Start: `g1_status` shows "console start failed" -> request_operator(help) -> armed -> g1_hold. The FIRST
   observation was ~14 cm off (everything looked closer): make one small motion and re-observe before trusting it.
2. BEFORE ANY MOTION, check the box against the robot: if a hand shows through the clear box floor, or depth over
   the box is a sloped surface 6-16 cm above the rim height, the box is resting at the table edge on that hand.
   Do not lean, and do not swing that arm back: ask the operator to move the box first. This session it slid off
   the edge with the left hand and fell to the floor. A scripted `reset` restores ONLY the puppy, so a fallen box
   ends the task.
3. Leaning is expensive: every waist lean with an arm forward slides the pelvis back a few cm (19 cm cumulative here:
   operator distance 0.48 -> 0.67 m) and drifts waist yaw +0.15 (palm lands 3-5 cm toward -y). Plan ONE lean at most,
   then re-localize everything in the frame taken after it.
4. Localize with `code/pbox/scene.py` cloud + `output/hm.py` (top-down height map, cm above the table) and
   `output/px.py` (deproject chosen pixels). Pick pixels on a zoomed grid image (`output/grid.py`). The forearm
   hides what is under the hand: localize with the hand moved aside, then aim.
5. Place reach: the box here was within reach (x 0.16-0.45, y 0.03-0.42) of the right hand crossing left, untested.
   Use g1_free_space_move (it worked this session) or palm-pose-servo.

## Grasp (still UNSOLVED after 4 sessions; what we know now)
- The sim puppy is ARTICULATED RIGID CAPSULES: legs ~6.5 cm cylinders, torso a rigid ellipsoid ~20 x 12 cm,
  tail a thin capsule sticking up at the back. Top-down torso grasps cannot work with a Dex3 (sessions 1-3).
- Leg wrap (palm-down, fingers yawed -45 deg across the leg, thumb on the free-table side) failed 3x here, each for
  a geometric reason rather than for the idea itself:
  (a) thumb landed on the leg (leg localized from a stale frame);
  (b) lean drift put the palm 3-5 cm off;
  (c) the leg was under the fingertips (9 cm along the fingers) with the palm 6 cm above the leg top.
  To do next: the leg must sit under the PALM centre (~6.5 cm along the fingers, just past the thumb at 2.3 cm)
  with the palm <= 2 cm above the leg top (thumb tip ~10 cm below the palm reaches the table). The right palm-down
  hand cannot get below z ~0.04 at x 0.30-0.40 without a lean (table top -0.072), so one lean of ~0.2 is needed
  (step 3 above). Check the hold by closing fully: fingers must stall short of the fist, and stay stalled on a
  3 cm lift.
- After a leg grasp the body hangs ~30-35 cm below the hand: the hand must reach ~0.35 m above the rim before
  swinging over the box. Check that with the planner (preview_only) BEFORE grasping.
- Alternative grasp point: the upright tail (thin, sticks up at the back). It is far (x ~0.6), so it needs a lean.
  A tail claw held 5 cm and slipped at 13 cm in session 2.

## End pose that works
palm-pose-servo `retract_home(ctx, 'right')`: open, neutral raised arm (sp -0.4, el 0.4), waist 0, home (sp 0.2,
el 0.6), then shoulder pitch 0.35 so the fingers stay behind the table edge (verified, tilt 3.3).
